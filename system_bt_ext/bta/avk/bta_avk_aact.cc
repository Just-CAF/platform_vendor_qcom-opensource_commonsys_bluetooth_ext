/******************************************************************************
 * Copyright (C) 2017, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted (subject to the limitations in the
 *  disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

 *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 *  GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 *  HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 *  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/******************************************************************************
 *
 *  Copyright (C) 2004-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  This file contains action functions for advanced audio/video stream
 *  state machine. these functions are shared by both audio and video
 *  streams.
 *
 ******************************************************************************/

#include "bt_target.h"

#include <base/logging.h>
#include <string.h>
#include <vector>

#include "a2dp_api.h"
#include "avdt_api.h"
#include "bt_utils.h"
#include "bta_avk_int.h"
#include "btif/include/btif_avk_co.h"
#include "btif/include/btif_storage.h"
#include "device/include/interop.h"
#include "l2c_api.h"
#include "l2cdefs.h"
#include "osi/include/osi.h"
#include "osi/include/properties.h"
#include "utl.h"
#include "btm_int.h"
#include "device/include/controller.h"
#include "a2dp_sbc.h"
#include "btif/include/btif_a2dp_source.h"
#include "btif/include/btif_avk.h"
#include "btif/include/btif_hf.h"
#include "btif/include/btif_config.h"
#if (BTA_AR_INCLUDED == TRUE)
#include "bta_ar_api.h"
#endif
#include "device/include/device_iot_config.h"
#include "osi/include/allocator.h"

/*****************************************************************************
 *  Constants
 ****************************************************************************/

/* the delay time in milliseconds to start service discovery on AVRCP */
#ifndef BTA_AVK_RC_DISC_TIME_VAL
#define BTA_AVK_RC_DISC_TIME_VAL 3500
#endif


#ifndef BTA_TIMEOUT_AVDT_CLOSE_MS
#define BTA_TIMEOUT_AVDT_CLOSE_MS 200
#endif
/* the timer in milliseconds to guard against link busy and AVDT_CloseReq failed
 * to be sent */
#ifndef BTA_AVK_CLOSE_REQ_TIME_VAL
#define BTA_AVK_CLOSE_REQ_TIME_VAL 4000
#endif

/* number to retry on reconfigure failure - some headsets requirs this number to
 * be more than 1 */
#ifndef BTA_AVK_RECONFIG_RETRY
#define BTA_AVK_RECONFIG_RETRY 6
#endif

/* ACL quota we are letting FW use for A2DP Offload Tx. */
#define BTA_AVK_A2DP_OFFLOAD_XMIT_QUOTA 4

#define MAX_2MBPS_AVDTP_MTU 663
#define BTIF_A2DP_MAX_BITPOOL_MQ 35
#define AVDTP_2DH3_MTU 360
static uint8_t last_sent_vsc_cmd = 0;
extern bool enc_update_in_progress;
extern bool tx_enc_update_initiated;
extern tBTIF_A2DP_SOURCE_VSC btif_a2dp_src_vsc;
extern void btif_media_send_reset_vendor_state();
extern bool btif_avk_device_in_sink_role();

static void bta_avk_st_rc_timer(tBTA_AVK_SCB* p_scb,
                               UNUSED_ATTR tBTA_AVK_DATA* p_data);

static void bta_avk_vendor_offload_select_codec(tBTA_AVK_SCB* p_scb);

//static uint8_t bta_avk_vendor_offload_convert_sample_rate(uint16_t sample_rate);

void bta_avk_vendor_offload_check_stop_start(tBTA_AVK_SCB* p_scb);
static void update_sub_band_info(uint8_t **param, int *param_len, uint8_t id, uint16_t data);
static void update_sub_band_info(uint8_t **param, int *param_len, uint8_t id, uint8_t *data,uint8_t size);
static void enc_mode_change_callback(tBTM_VSC_CMPL *param);

/* state machine states */
enum {
  BTA_AVK_INIT_SST,
  BTA_AVK_INCOMING_SST,
  BTA_AVK_OPENING_SST,
  BTA_AVK_OPEN_SST,
  BTA_AVK_RCFG_SST,
  BTA_AVK_CLOSING_SST
};

/* the call out functions for audio stream */
const tBTA_AVK_CO_FUNCTS bta_avk_a2dp_cos = {
    bta_avk_co_audio_init,          bta_avk_co_audio_disc_res,
    bta_avk_co_audio_getconfig,     bta_avk_co_audio_setconfig,
    bta_avk_co_audio_open,          bta_avk_co_audio_close,
    bta_avk_co_audio_start,         bta_avk_co_audio_stop,
    bta_avk_co_audio_src_data_path, bta_avk_co_audio_delay,
    bta_avk_co_audio_update_mtu,    bta_avk_co_cp_get_flag,
    bta_avk_co_cp_is_active};

/* ssm action functions for audio stream */
const tBTA_AVK_SACT bta_avk_a2dp_action[] = {
    bta_avk_do_disc_a2dp,    /* BTA_AVK_DO_DISC  */
    bta_avk_cleanup,         /* BTA_AVK_CLEANUP */
    bta_avk_free_sdb,        /* BTA_AVK_FREE_SDB */
    bta_avk_config_ind,      /* BTA_AVK_CONFIG_IND */
    bta_avk_disconnect_req,  /* BTA_AVK_DISCONNECT_REQ */
    bta_avk_security_req,    /* BTA_AVK_SECURITY_REQ */
    bta_avk_security_rsp,    /* BTA_AVK_SECURITY_RSP */
    bta_avk_setconfig_rsp,   /* BTA_AVK_SETCONFIG_RSP */
    bta_avk_st_rc_timer,     /* BTA_AVK_ST_RC_TIMER */
    bta_avk_str_opened,      /* BTA_AVK_STR_OPENED */
    bta_avk_security_ind,    /* BTA_AVK_SECURITY_IND */
    bta_avk_security_cfm,    /* BTA_AVK_SECURITY_CFM */
    bta_avk_do_close,        /* BTA_AVK_DO_CLOSE */
    bta_avk_connect_req,     /* BTA_AVK_CONNECT_REQ */
    bta_avk_sdp_failed,      /* BTA_AVK_SDP_FAILED */
    bta_avk_disc_results,    /* BTA_AVK_DISC_RESULTS */
    bta_avk_disc_res_as_acp, /* BTA_AVK_DISC_RES_AS_ACP */
    bta_avk_open_failed,     /* BTA_AVK_OPEN_FAILED */
    bta_avk_getcap_results,  /* BTA_AVK_GETCAP_RESULTS */
    bta_avk_setconfig_rej,   /* BTA_AVK_SETCONFIG_REJ */
    bta_avk_discover_req,    /* BTA_AVK_DISCOVER_REQ */
    bta_avk_conn_failed,     /* BTA_AVK_CONN_FAILED */
    bta_avk_do_start,        /* BTA_AVK_DO_START */
    bta_avk_str_stopped,     /* BTA_AVK_STR_STOPPED */
    bta_avk_reconfig,        /* BTA_AVK_RECONFIG */
    bta_avk_data_path,       /* BTA_AVK_DATA_PATH */
    bta_avk_start_ok,        /* BTA_AVK_START_OK */
    bta_avk_start_failed,    /* BTA_AVK_START_FAILED */
    bta_avk_str_closed,      /* BTA_AVK_STR_CLOSED */
    bta_avk_clr_cong,        /* BTA_AVK_CLR_CONG */
    bta_avk_suspend_cfm,     /* BTA_AVK_SUSPEND_CFM */
    bta_avk_rcfg_str_ok,     /* BTA_AVK_RCFG_STR_OK */
    bta_avk_rcfg_failed,     /* BTA_AVK_RCFG_FAILED */
    bta_avk_rcfg_connect,    /* BTA_AVK_RCFG_CONNECT */
    bta_avk_rcfg_discntd,    /* BTA_AVK_RCFG_DISCNTD */
    bta_avk_suspend_cont,    /* BTA_AVK_SUSPEND_CONT */
    bta_avk_rcfg_cfm,        /* BTA_AVK_RCFG_CFM */
    bta_avk_rcfg_open,       /* BTA_AVK_RCFG_OPEN */
    bta_avk_security_rej,    /* BTA_AVK_SECURITY_REJ */
    bta_avk_open_rc,         /* BTA_AVK_OPEN_RC */
    bta_avk_chk_2nd_start,   /* BTA_AVK_CHK_2ND_START */
    bta_avk_save_caps,       /* BTA_AVK_SAVE_CAPS */
    bta_avk_set_use_rc,      /* BTA_AVK_SET_USE_RC */
    bta_avk_cco_close,       /* BTA_AVK_CCO_CLOSE */
    bta_avk_switch_role,     /* BTA_AVK_SWITCH_ROLE */
    bta_avk_role_res,        /* BTA_AVK_ROLE_RES */
    bta_avk_delay_rpt,        /* BTA_AVK_DELAY_RPT */
    bta_avk_open_at_inc,     /* BTA_AVK_OPEN_AT_INC */
    bta_avk_offload_req,     /* BTA_AVK_OFFLOAD_REQ */
    bta_avk_offload_rsp,     /* BTA_AVK_OFFLOAD_RSP */
    bta_avk_disc_fail_as_acp,/* BTA_AVK_DISC_FAIL */
    NULL};

/* these tables translate AVDT events to SSM events */
static const uint16_t bta_avk_stream_evt_ok[] = {
    BTA_AVK_STR_DISC_OK_EVT,      /* AVDT_DISCOVER_CFM_EVT */
    BTA_AVK_STR_GETCAP_OK_EVT,    /* AVDT_GETCAP_CFM_EVT */
    BTA_AVK_STR_OPEN_OK_EVT,      /* AVDT_OPEN_CFM_EVT */
    BTA_AVK_STR_OPEN_OK_EVT,      /* AVDT_OPEN_IND_EVT */
    BTA_AVK_STR_CONFIG_IND_EVT,   /* AVDT_CONFIG_IND_EVT */
    BTA_AVK_STR_START_OK_EVT,     /* AVDT_START_CFM_EVT */
    BTA_AVK_STR_START_OK_EVT,     /* AVDT_START_IND_EVT */
    BTA_AVK_STR_SUSPEND_CFM_EVT,  /* AVDT_SUSPEND_CFM_EVT */
    BTA_AVK_STR_SUSPEND_CFM_EVT,  /* AVDT_SUSPEND_IND_EVT */
    BTA_AVK_STR_CLOSE_EVT,        /* AVDT_CLOSE_CFM_EVT */
    BTA_AVK_STR_CLOSE_EVT,        /* AVDT_CLOSE_IND_EVT */
    BTA_AVK_STR_RECONFIG_CFM_EVT, /* AVDT_RECONFIG_CFM_EVT */
    0,                           /* AVDT_RECONFIG_IND_EVT */
    BTA_AVK_STR_SECURITY_CFM_EVT, /* AVDT_SECURITY_CFM_EVT */
    BTA_AVK_STR_SECURITY_IND_EVT, /* AVDT_SECURITY_IND_EVT */
    BTA_AVK_STR_WRITE_CFM_EVT,    /* AVDT_WRITE_CFM_EVT */
    BTA_AVK_AVDT_CONNECT_EVT,     /* AVDT_CONNECT_IND_EVT */
    BTA_AVK_AVDT_DISCONNECT_EVT,  /* AVDT_DISCONNECT_IND_EVT */
#if (AVDT_REPORTING == TRUE)
    BTA_AVK_AVDT_RPT_CONN_EVT, /* AVDT_REPORT_CONN_EVT */
    BTA_AVK_AVDT_RPT_CONN_EVT, /* AVDT_REPORT_DISCONN_EVT */
#endif
    BTA_AVK_AVDT_DELAY_RPT_EVT, /* AVDT_DELAY_REPORT_EVT */
    0                          /* AVDT_DELAY_REPORT_CFM_EVT */
};

static const uint16_t bta_avk_stream_evt_fail[] = {
    BTA_AVK_STR_DISC_FAIL_EVT,    /* AVDT_DISCOVER_CFM_EVT */
    BTA_AVK_STR_GETCAP_FAIL_EVT,  /* AVDT_GETCAP_CFM_EVT */
    BTA_AVK_STR_OPEN_FAIL_EVT,    /* AVDT_OPEN_CFM_EVT */
    BTA_AVK_STR_OPEN_OK_EVT,      /* AVDT_OPEN_IND_EVT */
    BTA_AVK_STR_CONFIG_IND_EVT,   /* AVDT_CONFIG_IND_EVT */
    BTA_AVK_STR_START_FAIL_EVT,   /* AVDT_START_CFM_EVT */
    BTA_AVK_STR_START_OK_EVT,     /* AVDT_START_IND_EVT */
    BTA_AVK_STR_SUSPEND_CFM_EVT,  /* AVDT_SUSPEND_CFM_EVT */
    BTA_AVK_STR_SUSPEND_CFM_EVT,  /* AVDT_SUSPEND_IND_EVT */
    BTA_AVK_STR_CLOSE_EVT,        /* AVDT_CLOSE_CFM_EVT */
    BTA_AVK_STR_CLOSE_EVT,        /* AVDT_CLOSE_IND_EVT */
    BTA_AVK_STR_RECONFIG_CFM_EVT, /* AVDT_RECONFIG_CFM_EVT */
    0,                           /* AVDT_RECONFIG_IND_EVT */
    BTA_AVK_STR_SECURITY_CFM_EVT, /* AVDT_SECURITY_CFM_EVT */
    BTA_AVK_STR_SECURITY_IND_EVT, /* AVDT_SECURITY_IND_EVT */
    BTA_AVK_STR_WRITE_CFM_EVT,    /* AVDT_WRITE_CFM_EVT */
    BTA_AVK_AVDT_CONNECT_EVT,     /* AVDT_CONNECT_IND_EVT */
    BTA_AVK_AVDT_DISCONNECT_EVT,  /* AVDT_DISCONNECT_IND_EVT */
#if (AVDT_REPORTING == TRUE)
    BTA_AVK_AVDT_RPT_CONN_EVT, /* AVDT_REPORT_CONN_EVT */
    BTA_AVK_AVDT_RPT_CONN_EVT, /* AVDT_REPORT_DISCONN_EVT */
#endif
    BTA_AVK_AVDT_DELAY_RPT_EVT, /* AVDT_DELAY_REPORT_EVT */
    0                          /* AVDT_DELAY_REPORT_CFM_EVT */
};
void bta_avk_vendor_offload_start(tBTA_AVK_SCB *p_scb);
static void bta_avk_stream0_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data);
static void bta_avk_stream1_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data);
#if BTA_AVK_NUM_STRS > 2
static void bta_avk_stream2_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data);
#endif
#if BTA_AVK_NUM_STRS > 3
static void bta_avk_stream3_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data);
#endif
#if BTA_AVK_NUM_STRS > 4
static void bta_avk_stream4_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data);
#endif
#if BTA_AVK_NUM_STRS > 5
static void bta_avk_stream5_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data);
#endif
/* the array of callback functions to receive events from AVDT control channel
 */
tAVDT_CTRL_CBACK* const bta_avk_dt_cback[] = {bta_avk_stream0_cback,
                                             bta_avk_stream1_cback
#if BTA_AVK_NUM_STRS > 2
                                             ,
                                             bta_avk_stream2_cback
#endif
#if BTA_AVK_NUM_STRS > 3
                                             ,
                                             bta_avk_stream3_cback
#endif
#if BTA_AVK_NUM_STRS > 4
                                             ,
                                             bta_avk_stream4_cback
#endif
#if BTA_AVK_NUM_STRS > 5
                                             ,
                                             bta_avk_stream5_cback
#endif
};
/***********************************************
 *
 * Function         bta_get_scb_handle
 *
 * Description      gives the registered AVDT handle.by checking with sep_type.
 *
 *
 * Returns          void
 **********************************************/
static uint8_t bta_avk_get_scb_handle(tBTA_AVK_SCB* p_scb, uint8_t local_sep) {
  for (int i = 0; i < BTAV_A2DP_CODEC_INDEX_MAX; i++) {
    if ((p_scb->seps[i].tsep == local_sep) &&
        A2DP_CodecTypeEquals(p_scb->seps[i].codec_info,
                             p_scb->cfg.codec_info)) {
      return (p_scb->seps[i].av_handle);
    }
  }
  APPL_TRACE_DEBUG("%s: local sep_type %d not found", __func__, local_sep)
  return 0; /* return invalid handle */
}

/***********************************************
 *
 * Function         bta_avk_get_scb_sep_type
 *
 * Description      gives the sep type by cross-checking with AVDT handle
 *
 *
 * Returns          void
 **********************************************/
static uint8_t bta_avk_get_scb_sep_type(tBTA_AVK_SCB* p_scb,
                                       uint8_t tavdt_handle) {
  for (int i = 0; i < BTAV_A2DP_CODEC_INDEX_MAX; i++) {
    if (p_scb->seps[i].av_handle == tavdt_handle) return (p_scb->seps[i].tsep);
  }
  APPL_TRACE_DEBUG("%s: handle %d not found", __func__, tavdt_handle)
  return AVDT_TSEP_INVALID;
}

/*******************************************************************************
 *
 * Function         bta_avk_save_addr
 *
 * Description      copy the bd_addr and maybe reset the supported flags
 *
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_save_addr(tBTA_AVK_SCB* p_scb, const RawAddress& b) {
  APPL_TRACE_DEBUG("%s: r:%d, s:%d", __func__, p_scb->recfg_sup,
                   p_scb->suspend_sup);
  if (p_scb->peer_addr != b) {
    APPL_TRACE_ERROR("%s: reset flags", __func__);
    /* a new addr, reset the supported flags */
    p_scb->recfg_sup = true;
    p_scb->suspend_sup = true;
  }

  std::string addrstr = b.ToString();
  const char* bd_addr_str = addrstr.c_str();
  APPL_TRACE_DEBUG("%s: b[%s]", __func__, bd_addr_str);
  /* do this copy anyway, just in case the first addr matches
   * the control block one by accident */
  p_scb->peer_addr = b;
}

/*******************************************************************************
 *
 * Function         notify_start_failed
 *
 * Description      notify up-layer AVK start failed
 *
 *
 * Returns          void
 *
 ******************************************************************************/
static void notify_start_failed(tBTA_AVK_SCB* p_scb) {
  tBTA_AVK_START start;
  /* if start failed, clear role */
  p_scb->role &= ~BTA_AVK_ROLE_START_INT;
  start.chnl = p_scb->chnl;
  start.status = BTA_AVK_FAIL;
  start.initiator = true;
  start.hndl = p_scb->hndl;

  tBTA_AVK bta_avk_data;
  bta_avk_data.start = start;
  (*bta_avk_cb.p_cback)(BTA_AVK_START_EVT, &bta_avk_data);
}

static void bta_avk_update_flow_spec(tBTA_AVK_SCB* p_scb) {

  const char *codec_name;
  uint8_t *p_codec_info = (uint8_t*) p_scb->cfg.codec_info;

  tBT_FLOW_SPEC flow_spec;
  memset(&flow_spec, 0x00, sizeof(flow_spec));

  flow_spec.flow_direction = 0x00;     /* flow direction - out going */
  flow_spec.service_type = 0x02;       /* Guaranteed */
  flow_spec.token_rate = 0x00;         /* bytes/second - no token rate is specified*/
  flow_spec.token_bucket_size = 0x00;  /* bytes - no token bucket is needed*/
  flow_spec.latency = 0xFFFFFFFF;      /* microseconds - default value */

  codec_name = A2DP_CodecName(p_codec_info);
  if (strcmp(codec_name,"SBC") == 0) {
    flow_spec.peak_bandwidth = (345*1000)/8; /* bytes/second */

  } else if (strcmp(codec_name,"aptX") == 0)  {
    flow_spec.peak_bandwidth = (380*1000)/8; /* bytes/second */

  } else if (strcmp(codec_name,"aptX-HD") == 0) {
    flow_spec.peak_bandwidth = (660*1000)/8; /* bytes/second */

  } else if (strcmp(codec_name,"LDAC") == 0) {
    /* For ABR mode default peak bandwidth is 0 */
    flow_spec.peak_bandwidth = 0; /* bytes/second */

  } else if (strcmp(codec_name,"AAC") == 0) {
    flow_spec.peak_bandwidth = (320*1000)/8; /* bytes/second */
  }
  APPL_TRACE_DEBUG("codec_name %s peak_bandwidth %d",codec_name,
                                flow_spec.peak_bandwidth);
  //Sending Flow_spec has been taken care whenever event:BTIF_AVK_START_STREAM_REQ_EVT comes.
  //BTM_FlowSpec (p_scb->peer_addr, &flow_spec, NULL);
}

/*******************************************************************************
 *
 * Function         bta_avk_st_rc_timer
 *
 * Description      start the AVRC timer if no RC connection & CT is supported &
 *                  RC is used or
 *                  as ACP (we do not really know if we want AVRC)
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_st_rc_timer(tBTA_AVK_SCB* p_scb,
                               UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  BTIF_TRACE_DEBUG("%s: rc_handle:%d, use_rc: %d", __func__, p_scb->rc_handle,
                   p_scb->use_rc);
  /* for outgoing RC connection as INT/CT */
  if ((p_scb->rc_handle == BTA_AVK_RC_HANDLE_NONE) &&
      /* (bta_avk_cb.features & BTA_AVK_FEAT_RCCT) && */
      (p_scb->use_rc == true || (p_scb->role & BTA_AVK_ROLE_AD_ACP))) {
    if ((p_scb->wait & BTA_AVK_WAIT_ROLE_SW_BITS) == 0) {
      bta_sys_start_timer(p_scb->avrc_ct_timer, BTA_AVK_RC_DISC_TIME_VAL,
                          BTA_AVK_AVRC_TIMER_EVT, p_scb->hndl);
    } else {
      p_scb->wait |= BTA_AVK_WAIT_CHECK_RC;
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_next_getcap
 *
 * Description      The function gets the capabilities of the next available
 *                  stream found in the discovery results.
 *
 * Returns          true if we sent request to AVDT, false otherwise.
 *
 ******************************************************************************/
static bool bta_avk_next_getcap(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  int i;
  tAVDT_GETCAP_REQ* p_req;
  bool sent_cmd = false;
  uint16_t uuid_int = p_scb->uuid_int;
  uint8_t sep_requested = AVDT_TSEP_SNK;

  if (uuid_int == UUID_SERVCLASS_AUDIO_SOURCE)
    sep_requested = AVDT_TSEP_SNK;
  else if (uuid_int == UUID_SERVCLASS_AUDIO_SINK)
    sep_requested = AVDT_TSEP_SRC;

  APPL_TRACE_DEBUG("%s: sep_info_idx: %d num_seps = %d", __func__,
                              p_scb->sep_info_idx, p_scb->num_seps);
  for (i = p_scb->sep_info_idx; i < p_scb->num_seps; i++) {
    /* steam not in use, is a sink, and is the right media type (audio/video) */
    if ((p_scb->sep_info[i].in_use == false) &&
        (p_scb->sep_info[i].tsep == sep_requested) &&
        (p_scb->sep_info[i].media_type == p_scb->media_type)) {
      p_scb->sep_info_idx = i;

      /* we got a stream; get its capabilities */
      if (p_scb->p_cap == NULL)
        p_scb->p_cap = (tAVDT_CFG*)osi_malloc(sizeof(tAVDT_CFG));
      if ((p_scb->avdt_version >= AVDT_VERSION_SYNC) &&
          (A2DP_GetAvdtpVersion() >= AVDT_VERSION_SYNC)) {
        p_req = AVDT_GetAllCapReq;
      } else {
        p_req = AVDT_GetCapReq;
      }
      if ((*p_req)(p_scb->peer_addr,
                     p_scb->sep_info[i].seid,
                     p_scb->p_cap, bta_avk_dt_cback[p_scb->hdi]) == AVDT_SUCCESS) {
        sent_cmd = TRUE;
        break;
      } else
        APPL_TRACE_ERROR("%s: command could not be sent because of resource constraint",
                         __func__);
    }
  }

  /* if no streams available then stream open fails */
  if (!sent_cmd) {
    APPL_TRACE_ERROR("%s: BTA_AVK_STR_GETCAP_FAIL_EVT: peer_addr=%s", __func__,
                     p_scb->peer_addr.ToString().c_str());
    bta_avk_ssm_execute(p_scb, BTA_AVK_STR_GETCAP_FAIL_EVT, p_data);
  }

  return sent_cmd;
}

/*******************************************************************************
 *
 * Function         bta_avk_proc_stream_evt
 *
 * Description      Utility function to compose stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_proc_stream_evt(uint8_t handle, const RawAddress* bd_addr,
                                   uint8_t event, tAVDT_CTRL* p_data,
                                   int index) {
  uint16_t sec_len = 0;
  tBTA_AVK_SCB* p_scb = bta_avk_cb.p_scb[index];
  APPL_TRACE_VERBOSE("%s(): on the index : %d", __func__, index);

  if (p_data) {
    if (event == AVDT_SECURITY_IND_EVT) {
      sec_len = (p_data->security_ind.len < BTA_AVK_SECURITY_MAX_LEN)
                    ? p_data->security_ind.len
                    : BTA_AVK_SECURITY_MAX_LEN;
    } else if (event == AVDT_SECURITY_CFM_EVT && p_data->hdr.err_code == 0) {
      sec_len = (p_data->security_cfm.len < BTA_AVK_SECURITY_MAX_LEN)
                    ? p_data->security_cfm.len
                    : BTA_AVK_SECURITY_MAX_LEN;
    }
  }

  if (p_scb) {
    tBTA_AVK_STR_MSG* p_msg =
        (tBTA_AVK_STR_MSG*)osi_malloc(sizeof(tBTA_AVK_STR_MSG) + sec_len);

    /* copy event data, bd addr, and handle to event message buffer */
    p_msg->hdr.offset = 0;

    if (bd_addr != NULL) {
      p_msg->bd_addr = *bd_addr;
      VLOG(1) << __func__ << ": bd_addr:" << bd_addr;
    }

    if (p_data != NULL) {
      memcpy(&p_msg->msg, p_data, sizeof(tAVDT_CTRL));
      /* copy config params to event message buffer */
      switch (event) {
        case AVDT_RECONFIG_CFM_EVT:
          if (p_msg->msg.hdr.err_code == 0) {
            APPL_TRACE_DEBUG(
                "%s: reconfig cfm event codec info = 0x%06x-%06x-%06x-%02x",
                __func__,
                (p_msg->msg.reconfig_cfm.p_cfg->codec_info[0] << 16) +
                    (p_msg->msg.reconfig_cfm.p_cfg->codec_info[1] << 8) +
                    p_msg->msg.reconfig_cfm.p_cfg->codec_info[2],
                (p_msg->msg.reconfig_cfm.p_cfg->codec_info[3] << 16) +
                    (p_msg->msg.reconfig_cfm.p_cfg->codec_info[4] << 8) +
                    p_msg->msg.reconfig_cfm.p_cfg->codec_info[5],
                (p_msg->msg.reconfig_cfm.p_cfg->codec_info[6] << 16) +
                    (p_msg->msg.reconfig_cfm.p_cfg->codec_info[7] << 8) +
                    p_msg->msg.reconfig_cfm.p_cfg->codec_info[8],
                p_msg->msg.reconfig_cfm.p_cfg->codec_info[9]);
          }
          break;

        case AVDT_CONFIG_IND_EVT:
          /* We might have 2 SEP signallings(A2DP + VDP) with one peer device on
           * one L2CAP.
           * If we already have a signalling connection with the bd_addr and the
           * streaming
           * SST is at INIT state, change it to INCOMING state to handle the
           * signalling
           * from the 2nd SEP. */
          if ((bd_addr != NULL) &&
              (bta_avk_find_lcb(*bd_addr, BTA_AVK_LCB_FIND) != NULL) &&
              (bta_avk_is_scb_init(p_scb))) {
            bta_avk_set_scb_sst_incoming(p_scb);

            /* When ACP_CONNECT_EVT was received, we put first available scb to
             * incoming state.
             * Later when we receive AVDT_CONFIG_IND_EVT, we use a new p_scb and
             * set its state to
             * incoming which we do it above.
             * We also have to set the old p_scb state to init to be used later
             */
            for (int i = 0; i < BTA_AVK_NUM_STRS; i++) {
              if ((bta_avk_cb.p_scb[i]) && (i != index)) {
                if (bta_avk_cb.p_scb[i]->state == BTA_AVK_INCOMING_SST) {
                  bta_avk_cb.p_scb[i]->state = BTA_AVK_INIT_SST;
                  bta_avk_cb.p_scb[i]->coll_mask = 0;
                  break;
                }
              }
            }
          }

          memcpy(&p_msg->cfg, p_data->config_ind.p_cfg, sizeof(tAVDT_CFG));
          break;

        case AVDT_DELAY_REPORT_CFM_EVT:
          APPL_TRACE_DEBUG("%s: AVDT_DELAY_REPORT_CFM_EVT", __func__);
          return;

        case AVDT_SECURITY_IND_EVT:
          p_msg->msg.security_ind.p_data = (uint8_t*)(p_msg + 1);
          memcpy(p_msg->msg.security_ind.p_data, p_data->security_ind.p_data,
                 sec_len);
          break;

        case AVDT_SECURITY_CFM_EVT:
          p_msg->msg.security_cfm.p_data = (uint8_t*)(p_msg + 1);
          if (p_data->hdr.err_code == 0) {
            memcpy(p_msg->msg.security_cfm.p_data, p_data->security_cfm.p_data,
                   sec_len);
          }
          break;

        case AVDT_SUSPEND_IND_EVT:
          p_msg->msg.hdr.err_code = 0;
          break;

        case AVDT_CONNECT_IND_EVT:
          p_scb->recfg_sup = true;
          p_scb->suspend_sup = true;
          break;

        default:
          break;
      }
    } else
      p_msg->msg.hdr.err_code = 0;

    /* look up application event */
    if ((p_data == NULL) || (p_data->hdr.err_code == 0)) {
      p_msg->hdr.event = bta_avk_stream_evt_ok[event];
    } else {
      p_msg->hdr.event = bta_avk_stream_evt_fail[event];
    }

    p_msg->initiator = false;
    if (event == AVDT_SUSPEND_CFM_EVT) p_msg->initiator = true;

    APPL_TRACE_VERBOSE("%s: hndl:x%x", __func__, p_scb->hndl);
    VLOG(1) << __func__ << "p_msg->bd_addr:" << bd_addr;
    p_msg->hdr.layer_specific = p_scb->hndl;
    p_msg->handle = handle;
    p_msg->avdt_event = event;
    bta_sys_sendmsg(p_msg);
  }

  if (p_data && bd_addr) {
    bta_avk_conn_cback(handle, bd_addr, event, p_data);
  } else if (!p_data) {
    APPL_TRACE_ERROR("%s: p_data is null", __func__);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_sink_data_cback
 *
 * Description      This is the AVDTP callback function for sink stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_sink_data_cback(uint8_t handle, BT_HDR* p_pkt, uint32_t time_stamp,
                            uint8_t m_pt) {
  int index = 0;
  tBTA_AVK_SCB* p_scb;
  APPL_TRACE_DEBUG(
      "%s: avdt_handle: %d pkt_len=0x%x  offset = 0x%x "
      "number of frames 0x%x sequence number 0x%x",
      __func__, handle, p_pkt->len, p_pkt->offset,
      *((uint8_t*)(p_pkt + 1) + p_pkt->offset), p_pkt->layer_specific);
  /* Get SCB and correct sep type */
  for (index = 0; index < BTA_AVK_NUM_STRS; index++) {
    p_scb = bta_avk_cb.p_scb[index];
    if ((p_scb->avdt_handle == handle) &&
        (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SNK)) {
      break;
    }
  }
  if (index == BTA_AVK_NUM_STRS) {
    /* cannot find correct handler */
    osi_free(p_pkt);
    return;
  }
  p_pkt->event = BTA_AVK_SINK_MEDIA_DATA_EVT;
  p_scb->seps[p_scb->sep_idx].p_app_sink_data_cback(BTA_AVK_SINK_MEDIA_DATA_EVT,
                                                    (tBTA_AVK_SINK_MEDIA*)p_pkt, p_scb->peer_addr);
  /* Free the buffer: a copy of the packet has been delivered */
  osi_free(p_pkt);
}

/*******************************************************************************
 *
 * Function         bta_avk_stream0_cback
 *
 * Description      This is the AVDTP callback function for stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_stream0_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data) {
  APPL_TRACE_VERBOSE("%s: avdt_handle: %d event=0x%x", __func__, handle, event);
  bta_avk_proc_stream_evt(handle, bd_addr, event, p_data, 0);
}

/*******************************************************************************
 *
 * Function         bta_avk_stream1_cback
 *
 * Description      This is the AVDTP callback function for stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_stream1_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data) {
  APPL_TRACE_EVENT("%s: avdt_handle: %d event=0x%x", __func__, handle, event);
  bta_avk_proc_stream_evt(handle, bd_addr, event, p_data, 1);
}

#if BTA_AVK_NUM_STRS > 2
/*******************************************************************************
 *
 * Function         bta_avk_stream2_cback
 *
 * Description      This is the AVDTP callback function for stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_stream2_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data) {
  APPL_TRACE_EVENT("%s: avdt_handle: %d event=0x%x", __func__, handle, event);
  bta_avk_proc_stream_evt(handle, bd_addr, event, p_data, 2);
}
#endif

#if BTA_AVK_NUM_STRS > 3
/*******************************************************************************
 *
 * Function         bta_avk_stream3_cback
 *
 * Description      This is the AVDTP callback function for stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_stream3_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data) {
  APPL_TRACE_EVENT("%s: avdt_handle: %d event=0x%x", __func__, handle, event);
  bta_avk_proc_stream_evt(handle, bd_addr, event, p_data, 3);
}
#endif

/*******************************************************************************
 *
 * Function         bta_avk_stream4_cback
 *
 * Description      This is the AVDTP callback function for stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
#if BTA_AVK_NUM_STRS > 4
static void bta_avk_stream4_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data) {
  APPL_TRACE_EVENT("%s: avdt_handle: %d event=0x%x", __func__, handle, event);
  bta_avk_proc_stream_evt(handle, bd_addr, event, p_data, 4);
}
#endif

/*******************************************************************************
 *
 * Function         bta_avk_stream5_cback
 *
 * Description      This is the AVDTP callback function for stream events.
 *
 * Returns          void
 *
 ******************************************************************************/
#if BTA_AVK_NUM_STRS > 5
static void bta_avk_stream5_cback(uint8_t handle, const RawAddress* bd_addr,
                                 uint8_t event, tAVDT_CTRL* p_data) {
  APPL_TRACE_EVENT("%s: avdt_handle: %d event=0x%x", __func__, handle, event);
  bta_avk_proc_stream_evt(handle, bd_addr, event, p_data, 5);
}
#endif

/*******************************************************************************
 *
 * Function         bta_avk_a2dp_sdp_cback
 *
 * Description      A2DP service discovery callback.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_a2dp_sdp_cback(bool found, tA2DP_Service* p_service) {
  tBTA_AVK_SCB* p_scb = bta_avk_hndl_to_scb(bta_avk_cb.handle);

  if (p_scb == NULL) {
    APPL_TRACE_ERROR("%s: no scb found for handle(0x%x)", __func__,
                     bta_avk_cb.handle);
    return;
  }

  tBTA_AVK_SDP_RES* p_msg =
      (tBTA_AVK_SDP_RES*)osi_malloc(sizeof(tBTA_AVK_SDP_RES));

  if (!found && (p_scb->skip_sdp == true)) {
    p_msg->hdr.event = BTA_AVK_SDP_DISC_OK_EVT;
    p_scb->avdt_version = AVDT_VERSION;
    p_scb->skip_sdp = false;
    APPL_TRACE_WARNING("%s: Continue AVDTP signaling process for incoming A2dp connection",
                      __func__);
  } else {
    p_msg->hdr.event =
        (found) ? BTA_AVK_SDP_DISC_OK_EVT : BTA_AVK_SDP_DISC_FAIL_EVT;
    if (found && (p_service != NULL)) {
      p_scb->avdt_version = p_service->avdt_version;
#if (BT_IOT_LOGGING_ENABLED == TRUE)
      device_iot_config_addr_set_hex_if_greater(p_scb->peer_addr,
              IOT_CONF_KEY_A2DP_VERSION, p_scb->avdt_version, IOT_CONF_BYTE_NUM_2);
#endif
    }
    else
      p_scb->avdt_version = 0x00;
  }
  p_msg->hdr.layer_specific = bta_avk_cb.handle;

  bta_sys_sendmsg(p_msg);
  if (!found)
    APPL_TRACE_ERROR ("bta_avk_a2dp_sdp_cback, SDP record not found");
  bta_sys_conn_close(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
}

/*******************************************************************************
 *
 * Function         bta_avk_a2dp_sdp_cback2
 *
 * Description      A2DP service discovery callback2.
 *
 * Returns          void
 *
 ******************************************************************************/
static void bta_avk_a2dp_sdp_cback2(bool found, tA2DP_Service* p_service, tBTA_AVK_SCB* p_scb) {
  if (p_scb == NULL) {
    APPL_TRACE_ERROR("%s: invalid p_scb", __func__);
    return;
  }

  tBTA_AVK_SDP_RES* p_msg =
      (tBTA_AVK_SDP_RES*)osi_malloc(sizeof(tBTA_AVK_SDP_RES));

  if (!found && (p_scb->skip_sdp == true)) {
    p_msg->hdr.event = BTA_AVK_SDP_DISC_OK_EVT;
    p_scb->avdt_version = AVDT_VERSION;
    p_scb->skip_sdp = false;
    APPL_TRACE_WARNING("%s: Continue AVDTP signaling process for incoming A2dp connection",
                      __func__);
  } else {
    p_msg->hdr.event =
        (found) ? BTA_AVK_SDP_DISC_OK_EVT : BTA_AVK_SDP_DISC_FAIL_EVT;
    if (found && (p_service != NULL))
      p_scb->avdt_version = p_service->avdt_version;
    else
      p_scb->avdt_version = 0x00;
  }
  p_msg->hdr.layer_specific = p_scb->hndl;

  bta_sys_sendmsg(p_msg);
  if (!found)
    APPL_TRACE_ERROR ("bta_avk_a2dp_sdp_cback2 SDP record not found");
  bta_sys_conn_close(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
}

/*******************************************************************************
 *
 * Function         bta_avk_adjust_seps_idx
 *
 * Description      adjust the sep_idx
 *
 * Returns
 *
 ******************************************************************************/
static void bta_avk_adjust_seps_idx(tBTA_AVK_SCB* p_scb, uint8_t avdt_handle) {
  APPL_TRACE_DEBUG("%s: codec: %s and codec_index = %d", __func__,
          A2DP_CodecName(p_scb->cfg.codec_info), A2DP_SourceCodecIndex(p_scb->cfg.codec_info));
  for (int i = 0; i < BTAV_A2DP_CODEC_INDEX_MAX; i++) {
    APPL_TRACE_DEBUG("%s: av_handle: %d codec: %s", __func__,
                     p_scb->seps[i].av_handle,
                     A2DP_CodecName(p_scb->seps[i].codec_info));
    if (p_scb->seps[i].av_handle && (p_scb->seps[i].av_handle == avdt_handle) &&
        A2DP_CodecTypeEquals(p_scb->seps[i].codec_info,
                             p_scb->cfg.codec_info)) {
      p_scb->sep_idx = i;
      p_scb->avdt_handle = p_scb->seps[i].av_handle;
      break;
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_switch_role
 *
 * Description      Switch role was not started and a timer was started.
 *                  another attempt to switch role now - still opening.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_switch_role(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  tBTA_AVK_RS_RES switch_res = BTA_AVK_RS_NONE;
  tBTA_AVK_API_OPEN* p_buf = &p_scb->q_info.open;

  APPL_TRACE_DEBUG("%s: wait:x%x", __func__, p_scb->wait);
  if (p_scb->wait & BTA_AVK_WAIT_ROLE_SW_RES_START)
    p_scb->wait |= BTA_AVK_WAIT_ROLE_SW_RETRY;

  /* clear the masks set when the timer is started */
  p_scb->wait &=
      ~(BTA_AVK_WAIT_ROLE_SW_RES_OPEN | BTA_AVK_WAIT_ROLE_SW_RES_START);

  if (p_scb->q_tag == BTA_AVK_Q_TAG_OPEN) {
    if (bta_avk_switch_if_needed(p_scb) ||
        !bta_avk_link_role_ok(p_scb, A2DP_SET_MULTL_BIT)) {
      APPL_TRACE_DEBUG("%s: Role switch request in progress", __func__);
      p_scb->wait |= BTA_AVK_WAIT_ROLE_SW_RES_OPEN;
    } else {
      /* this should not happen in theory. Just in case...
       * continue to do_disc_a2dp */

       if(!p_scb->num_disc_snks) {
          /* Only there is no discovered sink list, then do discovery */
          APPL_TRACE_DEBUG("%s: continue discovery request(hndl:0x%x)",
                             __func__, p_scb->hndl);
          switch_res = BTA_AVK_RS_DONE;
        }
    }
  } else {
    /* report failure on OPEN */
    APPL_TRACE_DEBUG("%s: Role switch request failed", __func__);
    switch_res = BTA_AVK_RS_FAIL;
  }

  if (switch_res != BTA_AVK_RS_NONE) {
    if (bta_avk_cb.rs_idx == (p_scb->hdi + 1)) {
      bta_avk_cb.rs_idx = 0;
    }
    APPL_TRACE_DEBUG("%s: Role switch request to be retried", __func__);
    p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_RETRY;
    p_scb->q_tag = 0;
    p_buf->switch_res = switch_res;
    bta_avk_do_disc_a2dp(p_scb, (tBTA_AVK_DATA*)p_buf);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_role_res
 *
 * Description      Handle the role changed event
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_role_res(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  bool initiator = false;
  tBTA_AVK_START start;
  tBTA_AVK_ROLE_CHANGED role_changed;
  uint8_t cur_role = BTM_ROLE_UNDEFINED;

  APPL_TRACE_DEBUG("%s: q_tag:%d, wait:x%x, role:x%x", __func__, p_scb->q_tag,
                   p_scb->wait, p_scb->role);
  if (p_scb->role & BTA_AVK_ROLE_START_INT) initiator = true;

  /* Multicast: update BTIF about role switch
   * If role switch succeeded, we need to update multicast state
   * from BTIF.
   */
  if (p_data->role_res.hci_status == HCI_SUCCESS) {
    APPL_TRACE_DEBUG("bta_avk_role_res: Master update upper layer");

    role_changed.bd_addr = p_scb->peer_addr;
    role_changed.hndl = p_scb->hndl;

    if (BTM_GetRole (p_scb->peer_addr, &cur_role) == BTM_SUCCESS)
      role_changed.new_role = cur_role;
    (*bta_avk_cb.p_cback)(BTA_AVK_ROLE_CHANGED_EVT, (tBTA_AVK *)&role_changed);
  }

  if (p_scb->q_tag == BTA_AVK_Q_TAG_START) {
    if (p_scb->wait & BTA_AVK_WAIT_ROLE_SW_STARTED) {
      p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_BITS;
      if (p_data->role_res.hci_status != HCI_SUCCESS) {
        p_scb->role &= ~BTA_AVK_ROLE_START_INT;
        bta_sys_idle(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
        /* start failed because of role switch. */
        start.chnl = p_scb->chnl;
        start.status = BTA_AVK_FAIL_ROLE;
        start.hndl = p_scb->hndl;
        start.initiator = initiator;
        tBTA_AVK bta_avk_data;
        bta_avk_data.start = start;
        (*bta_avk_cb.p_cback)(BTA_AVK_START_EVT, &bta_avk_data);
      } else {
        bta_avk_start_ok(p_scb, p_data);
      }
    } else if (p_scb->wait & BTA_AVK_WAIT_ROLE_SW_RES_START)
      p_scb->wait |= BTA_AVK_WAIT_ROLE_SW_FAILED;
  } else if (p_scb->q_tag == BTA_AVK_Q_TAG_OPEN) {
    if (p_scb->wait & BTA_AVK_WAIT_ROLE_SW_RES_OPEN) {
      p_scb->role &= ~BTA_AVK_ROLE_START_INT;
      p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_BITS;

      if (p_data->role_res.hci_status != HCI_SUCCESS) {
        /* Open failed because of role switch. */
        /*av_open.bd_addr = p_scb->peer_addr;
        av_open.chnl = p_scb->chnl;
        av_open.hndl = p_scb->hndl;*/
        /* update Master/Slave Role for open event */
        /*if (BTM_GetRole(p_scb->peer_addr, &cur_role) == BTM_SUCCESS)
          av_open.role = cur_role;
        av_open.status = BTA_AVK_FAIL_ROLE;
        if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SRC)
          av_open.sep = AVDT_TSEP_SNK;
         else if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SNK) {
          av_open.sep = AVDT_TSEP_SRC;
        }
        tBTA_AVK bta_avk_data;
        bta_avk_data.open = av_open;
        (*bta_avk_cb.p_cback)(BTA_AVK_OPEN_EVT, &bta_avk_data);*/
        p_scb->q_info.open.switch_res = BTA_AVK_RS_NONE;
        bta_avk_do_disc_a2dp(p_scb, (tBTA_AVK_DATA*)&(p_scb->q_info.open));
      } else {
        /* Continue av open process */
        p_scb->q_info.open.switch_res = BTA_AVK_RS_DONE;
        bta_avk_do_disc_a2dp(p_scb, (tBTA_AVK_DATA*)&(p_scb->q_info.open));
      }
    } else {
      APPL_TRACE_WARNING(
          "%s: unexpected role switch event: q_tag = %d wait = %d", __func__,
          p_scb->q_tag, p_scb->wait);
    }
  }

  APPL_TRACE_DEBUG("%s: wait:x%x, role:x%x", __func__, p_scb->wait,
                   p_scb->role);
}

/*******************************************************************************
 *
 * Function         bta_avk_delay_rpt
 *
 * Description      Handle the delay report event from SNK
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_delay_rpt(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  APPL_TRACE_DEBUG("%s: delay report value: %d", __func__, p_data->str_msg.msg.delay_rpt_cmd.delay);
  p_scb->p_cos->delay(p_scb->hndl, p_data->str_msg.msg.delay_rpt_cmd.delay);
}

/*******************************************************************************
 *
 * Function         bta_avk_do_disc_a2dp
 *
 * Description      Do service discovery for A2DP.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_do_disc_a2dp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  bool ok_continue = false;
  uint16_t avdtp_version = 0;
  tA2DP_SDP_DB_PARAMS db_params;
  uint16_t attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST,
                          ATTR_ID_PROTOCOL_DESC_LIST,
                          ATTR_ID_BT_PROFILE_DESC_LIST};
  uint16_t sdp_uuid = 0; /* UUID for which SDP has to be done */

  APPL_TRACE_DEBUG("%s: use_rc: %d rs:%d, oc:%d", __func__,
                   p_data->api_open.use_rc, p_data->api_open.switch_res,
                   bta_avk_cb.audio_open_cnt);

  memcpy(&(p_scb->open_api), &(p_data->api_open), sizeof(tBTA_AVK_API_OPEN));

  switch (p_data->api_open.switch_res) {
    case BTA_AVK_RS_NONE:
      if (bta_avk_switch_if_needed(p_scb) ||
          !bta_avk_link_role_ok(p_scb, A2DP_SET_MULTL_BIT)) {
        /* waiting for role switch result. save the api to control block */
        memcpy(&p_scb->q_info.open, &p_data->api_open,
               sizeof(tBTA_AVK_API_OPEN));
        p_scb->wait |= BTA_AVK_WAIT_ROLE_SW_RES_OPEN;
        p_scb->q_tag = BTA_AVK_Q_TAG_OPEN;
        APPL_TRACE_DEBUG("%s: AVK Role switch triggered", __func__);
      } else {
        ok_continue = true;
      }
      break;

    case BTA_AVK_RS_FAIL:
      /* report a new failure event  */
      p_scb->open_status = BTA_AVK_FAIL_ROLE;
      APPL_TRACE_ERROR("%s: BTA_AVK_SDP_DISC_FAIL_EVT: peer_addr=%s", __func__,
                       p_scb->peer_addr.ToString().c_str());
      bta_avk_ssm_execute(p_scb, BTA_AVK_SDP_DISC_FAIL_EVT, NULL);
      break;

    case BTA_AVK_RS_OK:
      p_data = (tBTA_AVK_DATA*)&p_scb->q_info.open;
      /* continue to open if link role is ok */
      if (bta_avk_link_role_ok(p_scb, A2DP_SET_MULTL_BIT)) {
        ok_continue = true;
      } else {
        APPL_TRACE_DEBUG("%s: Role not proper yet, wait", __func__);
        p_scb->wait |= BTA_AVK_WAIT_ROLE_SW_RES_OPEN;
      }
      break;

    case BTA_AVK_RS_DONE:
      ok_continue = true;
      break;
  }

  APPL_TRACE_DEBUG("%s: ok_continue: %d wait:x%x, q_tag: %d", __func__,
                   ok_continue, p_scb->wait, p_scb->q_tag);

  if (!ok_continue) {
    tBTA_AVK_SCB* p_scbi;
    int i, mask;
    uint8_t role;
    APPL_TRACE_DEBUG("%s:audio open cnt = %d",__func__, bta_avk_cb.audio_open_cnt);
    if (bta_avk_cb.audio_open_cnt > 0) {
      for (i = 0; i < BTA_AVK_NUM_STRS; i++) {
        mask = BTA_AVK_HNDL_TO_MSK(i);
        p_scbi = bta_avk_cb.p_scb[i];
        if (p_scbi && (p_scb->hdi != i) &&    /* not the original channel */
          ((bta_avk_cb.conn_audio & mask) || /* connected audio */
          (bta_avk_cb.conn_video & mask))) {  /* connected video */
          BTM_GetRole(p_scbi->peer_addr, &role);
          APPL_TRACE_DEBUG("%s:Current role for idx %d is %d",__func__, p_scb->hdi, role);
          if (BTM_ROLE_MASTER != role) {
            if (!interop_match_addr_or_name(INTEROP_DISABLE_ROLE_SWITCH,
                                          &p_scbi->peer_addr)) {
              APPL_TRACE_DEBUG("%s:RS disabled, returning",__func__);
#if (TWS_ENABLED == TRUE)
              if (p_scbi->tws_device)
                AVDT_UpdateServiceBusyState(false);
#endif
              return;
            }else {
              APPL_TRACE_DEBUG("%s: Other connected remote is blacklisted for RS",__func__);
              APPL_TRACE_DEBUG("%s: RS is not possible, continue avdtp signaling",__func__);
            }
          }
        }
      }
    } else {
      APPL_TRACE_DEBUG("%s:returning",__func__);
      return;
    }
  }

  /* clear the role switch bits */
  p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_BITS;

  if (p_scb->wait & BTA_AVK_WAIT_CHECK_RC) {
    APPL_TRACE_DEBUG("%s: Start RC Timer, wait:x%x",__func__, p_scb->wait);
    p_scb->wait &= ~BTA_AVK_WAIT_CHECK_RC;
    bta_sys_start_timer(p_scb->avrc_ct_timer, BTA_AVK_RC_DISC_TIME_VAL,
                        BTA_AVK_AVRC_TIMER_EVT, p_scb->hndl);
  }

  if (bta_avk_cb.features & BTA_AVK_FEAT_MASTER) {
    L2CA_SetDesireRole(L2CAP_ROLE_DISALLOW_SWITCH);

    if (bta_avk_cb.audio_open_cnt == 1) {
      /* there's already an A2DP connection. do not allow switch */
      bta_sys_clear_default_policy(BTA_ID_AVK, HCI_ENABLE_MASTER_SLAVE_SWITCH);
    }
  }
  /* store peer addr other parameters */
  bta_avk_save_addr(p_scb, p_data->api_open.bd_addr);
  p_scb->sec_mask = p_data->api_open.sec_mask;
  p_scb->use_rc = p_data->api_open.use_rc;

  bta_sys_conn_open(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);

  if (p_scb->skip_sdp == true && (btif_config_get_uint16(p_scb->peer_addr.ToString().c_str(), AVDTP_VERSION_CONFIG_KEY,
           (uint16_t*)&avdtp_version) || (a2dp_get_avdt_sdp_ver() < AVDT_VERSION_SYNC))) {
    tA2DP_Service a2dp_ser;
    APPL_TRACE_DEBUG("%s: Cached peer avdtp version: 0x%x",__func__, avdtp_version);
    APPL_TRACE_DEBUG("%s: a2dp_get_avdt_sdp_ver(): 0x%x",__func__, a2dp_get_avdt_sdp_ver());
    if (a2dp_get_avdt_sdp_ver() < AVDT_VERSION_SYNC) {
      a2dp_ser.avdt_version = AVDT_VERSION;
    } else {
      a2dp_ser.avdt_version = avdtp_version;
    }
    p_scb->skip_sdp = false;
    p_scb->uuid_int = p_data->api_open.uuid;
    APPL_TRACE_WARNING("%s: Skip Sdp for incoming A2dp connection", __func__);
    bta_avk_a2dp_sdp_cback2(true, &a2dp_ser, p_scb);
    return;
  } else {
    /* set up parameters */
    db_params.db_len = BTA_AVK_DISC_BUF_SIZE;
    db_params.num_attr = 3;
    db_params.p_attrs = attr_list;
    p_scb->uuid_int = p_data->api_open.uuid;
    p_scb->sdp_discovery_started = true;
    if (p_scb->uuid_int == UUID_SERVCLASS_AUDIO_SINK)
      sdp_uuid = UUID_SERVCLASS_AUDIO_SOURCE;
    else if (p_scb->uuid_int == UUID_SERVCLASS_AUDIO_SOURCE)
      sdp_uuid = UUID_SERVCLASS_AUDIO_SINK;

    APPL_TRACE_DEBUG("%s: uuid_int 0x%x, Doing SDP For 0x%x", __func__,
                    p_scb->uuid_int, sdp_uuid);
    if (A2DP_FindService(sdp_uuid, p_scb->peer_addr, &db_params,
                        bta_avk_a2dp_sdp_cback) == A2DP_SUCCESS) {
      APPL_TRACE_DEBUG("%s: A2DP find service return SUCCESS", __func__);
      /* only one A2D find service is active at a time */
      bta_avk_cb.handle = p_scb->hndl;
      return;
    }

    /* when the code reaches here, either the DB is NULL
     * or A2DP_FindService is not successful */
    bta_avk_a2dp_sdp_cback2(true, NULL, p_scb);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_cleanup
 *
 * Description      cleanup AVK stream control block.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_cleanup(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  tBTA_AVK_CONN_CHG msg;
  uint8_t role = BTA_AVK_ROLE_AD_INT;

  APPL_TRACE_DEBUG("%s", __func__);
  last_sent_vsc_cmd = 0;

  /* free any buffers */
  osi_free_and_reset((void**)&p_scb->p_cap);
  p_scb->sdp_discovery_started = false;
  p_scb->avdt_version = 0;

  /* initialize some control block variables */
  p_scb->open_status = BTA_AVK_SUCCESS;
  if (p_scb->tws_device) {
    /* Check if offload start is pending */
    bta_avk_vendor_offload_check_stop_start(p_scb);
  }
  /* if de-registering shut everything down */
  msg.hdr.layer_specific = p_scb->hndl;
  p_scb->started = false;
  p_scb->suspend_local_sent = FALSE;
#if (TWS_STATE_ENABLED == TRUE)
  p_scb->start_pending = false;
  p_scb->eb_state = TWSP_EB_STATE_UNKNOWN;
#endif
  p_scb->current_codec = nullptr;
  p_scb->cong = false;
  p_scb->role = role;
  p_scb->cur_psc_mask = 0;
  p_scb->wait = 0;
  p_scb->num_disc_snks = 0;
  p_scb->offload_supported = false;
  p_scb->offload_started = false;
  p_scb->vendor_start = false;
  alarm_cancel(p_scb->avrc_ct_timer);

  /* TODO(eisenbach): RE-IMPLEMENT USING VSC OR HAL EXTENSION
    vendor_get_interface()->send_command(
        (vendor_opcode_t)BT_VND_OP_A2DP_OFFLOAD_STOP, (void*)&p_scb->l2c_cid);
    if (p_scb->offload_start_pending) {
      tBTA_AVK_STATUS status = BTA_AVK_FAIL_STREAM;
      tBTA_AVK bta_avk_data;
      bta_avk_data.status = status;
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, &bta_avk_data);
    }
  */
  /*if (BTM_IS_QTI_CONTROLLER())
  {
    APPL_TRACE_ERROR("bta_avk_cleanup: Vendor Stop");
    bta_avk_vendor_offload_stop();
  }*/
  p_scb->offload_start_pending = false;
  p_scb->skip_sdp = false;
  p_scb->coll_mask = 0;

  p_scb->cfg.psc_mask = AVDT_PSC_TRANS;
  if (bta_avk_cb.features & BTA_AVK_FEAT_DELAY_RPT)
    p_scb->cfg.psc_mask |= (AVDT_PSC_DELAY_RPT);
#if (AVDT_REPORTING == TRUE)
  if (bta_avk_cb.features & BTA_AVK_FEAT_REPORT)
    p_scb->cfg.psc_mask |= (AVDT_PSC_REPORT);
#endif

  p_scb->skip_sdp = false;
  p_scb->cache_setconfig = NULL;
  if (p_scb->deregistring) {
    /* remove stream */
  for (int i = 0; i < BTAV_A2DP_CODEC_INDEX_MAX; i++) {
      if (p_scb->seps[i].av_handle) AVDT_RemoveStream(p_scb->seps[i].av_handle);
      p_scb->seps[i].av_handle = 0;
    }

    bta_avk_dereg_comp((tBTA_AVK_DATA*)&msg);
  } else {
    /* report stream closed to main SM */
    msg.is_up = false;
    msg.peer_addr = p_scb->peer_addr;
    bta_avk_conn_chg((tBTA_AVK_DATA*)&msg);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_free_sdb
 *
 * Description      Free service discovery db buffer.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_free_sdb(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  p_scb->sdp_discovery_started = false;
}

/*******************************************************************************
 *
 * Function         bta_avk_config_ind
 *
 * Description      Handle a stream configuration indication from the peer.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_config_ind(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_CI_SETCONFIG setconfig;
  tAVDT_SEP_INFO* p_info;
  tAVDT_CFG* p_evt_cfg = &p_data->str_msg.cfg;
  uint8_t psc_mask = (p_evt_cfg->psc_mask | p_scb->cfg.psc_mask);
  uint8_t
      local_sep; /* sep type of local handle on which connection was received */
  tBTA_AVK_STR_MSG* p_msg = (tBTA_AVK_STR_MSG*)p_data;

  local_sep = bta_avk_get_scb_sep_type(p_scb, p_msg->handle);
  p_scb->avdt_label = p_data->str_msg.msg.hdr.label;

  BTIF_TRACE_DEBUG("%s: local_sep = %d", __func__, local_sep);
  A2DP_DumpCodecInfo(p_evt_cfg->codec_info);

  memcpy(p_scb->cfg.codec_info, p_evt_cfg->codec_info, AVDT_CODEC_SIZE);
  bta_avk_save_addr(p_scb, p_data->str_msg.bd_addr);

  /* Clear collision mask */
  if (p_scb->coll_mask & BTA_AVK_COLL_API_CALLED) {
    APPL_TRACE_DEBUG(" bta_avk_config_ind ReSetting collision mask  ");
    /* Clear collision mask */
    p_scb->coll_mask = 0;
  } else {
    APPL_TRACE_WARNING(" bta_avk_config_ind config_ind called before Open");
    p_scb->coll_mask |= BTA_AVK_COLL_SETCONFIG_IND;
  }
  BTIF_TRACE_DEBUG(" bta_avk_config_ind p_scb->hdi = %d ", p_scb->hdi);
  alarm_cancel(bta_avk_cb.accept_signalling_timer[p_scb->hdi]);
  alarm_cancel(bta_avk_cb.avdt_close_timer[p_scb->hdi]);

  /* if no codec parameters in configuration, fail */
  if ((p_evt_cfg->num_codec == 0) ||
      /* or the peer requests for a service we do not support */
      ((psc_mask != p_scb->cfg.psc_mask) &&
       (psc_mask != (p_scb->cfg.psc_mask & ~AVDT_PSC_DELAY_RPT)))) {
    setconfig.hndl = p_scb->hndl; /* we may not need this */
    setconfig.err_code = AVDT_ERR_UNSUP_CFG;
    bta_avk_ssm_execute(p_scb, BTA_AVK_CI_SETCONFIG_FAIL_EVT,
                       (tBTA_AVK_DATA*)&setconfig);
  } else {
    p_info = &p_scb->sep_info[0];
    p_info->in_use = 0;
    p_info->media_type = p_scb->media_type;
    p_info->seid = p_data->str_msg.msg.config_ind.int_seid;

    /* Sep type of Peer will be oppsite role to our local sep */
    if (local_sep == AVDT_TSEP_SRC) {
      p_info->tsep = AVDT_TSEP_SNK;
#if (BT_IOT_LOGGING_ENABLED == TRUE)
      device_iot_config_addr_set_int(p_scb->peer_addr,
          IOT_CONF_KEY_A2DP_ROLE, IOT_CONF_VAL_A2DP_ROLE_SINK);
#endif
    } else if (local_sep == AVDT_TSEP_SNK) {
      p_info->tsep = AVDT_TSEP_SRC;
#if (BT_IOT_LOGGING_ENABLED == TRUE)
      device_iot_config_addr_set_int(p_scb->peer_addr,
          IOT_CONF_KEY_A2DP_ROLE, IOT_CONF_VAL_A2DP_ROLE_SOURCE);
#endif
    }

    p_scb->role |= BTA_AVK_ROLE_AD_ACP;
    p_scb->cur_psc_mask = p_evt_cfg->psc_mask;
    if (bta_avk_cb.features & BTA_AVK_FEAT_RCTG)
      p_scb->use_rc = true;
    else
      p_scb->use_rc = false;

    p_scb->num_seps = 1;
    p_scb->sep_info_idx = 0;
    BTIF_TRACE_DEBUG("%s: SEID: %d use_rc: %d cur_psc_mask:0x%x", __func__,
                     p_info->seid, p_scb->use_rc, p_scb->cur_psc_mask);
    /*  in case of A2DP SINK this is the first time peer data is being sent to
     * co functions */
    if (local_sep == AVDT_TSEP_SNK) {
      p_scb->p_cos->setcfg(p_scb->hndl, p_evt_cfg->codec_info, p_info->seid,
                           p_scb->peer_addr, p_evt_cfg->num_protect,
                           p_evt_cfg->protect_info, AVDT_TSEP_SNK,
                           p_msg->handle);
    } else {
      p_scb->p_cos->setcfg(p_scb->hndl, p_evt_cfg->codec_info, p_info->seid,
                           p_scb->peer_addr, p_evt_cfg->num_protect,
                           p_evt_cfg->protect_info, AVDT_TSEP_SRC,
                           p_msg->handle);
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_disconnect_req
 *
 * Description      Disconnect AVDTP connection.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_disconnect_req(tBTA_AVK_SCB* p_scb,
                           UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  tBTA_AVK_RCB* p_rcb;
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;

  APPL_TRACE_WARNING("%s: conn_lcb: 0x%x peer_addr: %s", __func__,
                     bta_avk_cb.conn_lcb, p_scb->peer_addr.ToString().c_str());

  bta_sys_set_policy(BTA_ID_AVK, policy, p_scb->peer_addr);

  alarm_cancel(bta_avk_cb.link_signalling_timer);
  alarm_cancel(p_scb->avrc_ct_timer);

  if (bta_avk_cb.conn_lcb) {
    p_rcb = bta_avk_get_rcb_by_shdl((uint8_t)(p_scb->hdi + 1));
    if (p_rcb) bta_avk_del_rc(p_rcb);
    AVDT_DisconnectReq(p_scb->peer_addr, bta_avk_dt_cback[p_scb->hdi]);
  } else {
    bta_avk_ssm_execute(p_scb, BTA_AVK_AVDT_DISCONNECT_EVT, NULL);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_security_req
 *
 * Description      Send an AVDTP security request.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_security_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  if (bta_avk_cb.features & BTA_AVK_FEAT_PROTECT) {
    AVDT_SecurityReq(p_scb->avdt_handle, p_data->api_protect_req.p_data,
                     p_data->api_protect_req.len);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_security_rsp
 *
 * Description      Send an AVDTP security response.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_security_rsp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  if (bta_avk_cb.features & BTA_AVK_FEAT_PROTECT) {
    AVDT_SecurityRsp(p_scb->avdt_handle, p_scb->avdt_label,
                     p_data->api_protect_rsp.error_code,
                     p_data->api_protect_rsp.p_data,
                     p_data->api_protect_rsp.len);
  } else {
    AVDT_SecurityRsp(p_scb->avdt_handle, p_scb->avdt_label, AVDT_ERR_NSC, NULL,
                     0);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_setconfig_rsp
 *
 * Description      setconfig is OK
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_setconfig_rsp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t num = p_data->ci_setconfig.num_seid + 1;
  uint8_t avdt_handle = p_data->ci_setconfig.avdt_handle;
  uint8_t local_sep;

  /* we like this codec_type. find the sep_idx */
  local_sep = bta_avk_get_scb_sep_type(p_scb, avdt_handle);
  bta_avk_adjust_seps_idx(p_scb, avdt_handle);
  APPL_TRACE_DEBUG("%s: sep_idx: %d cur_psc_mask:0x%x, num: %d", __func__,
                   p_scb->sep_idx, p_scb->cur_psc_mask, num);

  if ((AVDT_TSEP_SNK == local_sep) &&
      (p_data->ci_setconfig.err_code == AVDT_SUCCESS) &&
      (p_scb->seps[p_scb->sep_idx].p_app_sink_data_cback != NULL)) {
    tBTA_AVK_SINK_MEDIA av_sink_codec_info;
    av_sink_codec_info.avk_config.bd_addr = p_scb->peer_addr;
    av_sink_codec_info.avk_config.codec_info = p_scb->cfg.codec_info;
    p_scb->seps[p_scb->sep_idx].p_app_sink_data_cback(BTA_AVK_SINK_MEDIA_CFG_EVT,
                                                      &av_sink_codec_info, p_scb->peer_addr);
  }

  p_scb->cache_setconfig = (tBTA_AVK_DATA *)osi_malloc(sizeof(tBTA_AVK_DATA));
  memset(p_scb->cache_setconfig, 0, sizeof(tBTA_AVK_DATA));
  memcpy(p_scb->cache_setconfig, p_data, sizeof(tBTA_AVK_DATA));

  AVDT_ConfigRsp(p_scb->avdt_handle, p_scb->avdt_label,
                 p_data->ci_setconfig.err_code, p_data->ci_setconfig.category);

  alarm_cancel(bta_avk_cb.link_signalling_timer);

  if (p_data->ci_setconfig.err_code == AVDT_SUCCESS) {
      p_scb->wait = BTA_AVK_WAIT_ACP_CAPS_ON;
      if (p_data->ci_setconfig.recfg_needed)
          p_scb->role |= BTA_AVK_ROLE_SUSPEND_OPT;
      APPL_TRACE_DEBUG("%s: recfg_needed:%d role:x%x num:%d", __func__,
                     p_data->ci_setconfig.recfg_needed, p_scb->role, num);
    /* callout module tells BTA the number of "good" SEPs and their SEIDs.
     * getcap on these SEID */
    p_scb->num_seps = num;

    if (p_scb->cur_psc_mask & AVDT_PSC_DELAY_RPT)
      p_scb->avdt_version = AVDT_VERSION_SYNC;
    else
      p_scb->avdt_version = AVDT_VERSION;

    if (local_sep == AVDT_TSEP_SRC) {
      if (p_scb->uuid_int == 0) p_scb->uuid_int = UUID_SERVCLASS_AUDIO_SOURCE;
        /* we do not know the peer device and it is using non-SBC codec
         * we need to know all the SEPs on SNK */
      bta_avk_discover_req(p_scb, NULL);
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_set_tws_chn_mode
 *
 * Description      Set channel mode.
 *
 * Returns          void
 *
 ******************************************************************************/
#if (TWS_ENABLED == TRUE)
void bta_avk_set_tws_chn_mode(tBTA_AVK_SCB *p_scb, bool adjust) {
  int i;
  tBTA_AVK_SCB *p_scbi;
  RawAddress tws_pair_addr;
  bool is_tws_pair = false;
  APPL_TRACE_DEBUG("%s",__func__);
  for (i = 0; i < BTA_AVK_NUM_STRS; i++) {
    p_scbi = bta_avk_cb.p_scb[i];
    if (p_scbi == NULL || p_scbi == p_scb) continue;
    APPL_TRACE_DEBUG("%s:p_scbi is tws dev = %d",__func__,p_scbi->tws_device);
    if (p_scbi->tws_device) {
      if (BTM_SecGetTwsPlusPeerDev(p_scb->peer_addr,
                               tws_pair_addr) == true) {
        if (tws_pair_addr ==  p_scbi->peer_addr) {
          if (!adjust) {//set role on connection
            if (p_scbi->channel_mode == 0) {
              p_scb->channel_mode = 1;//Right
              APPL_TRACE_DEBUG("%s:setting channel mode to Right",__func__);
            } else {
              p_scb->channel_mode = 0;//Left
              APPL_TRACE_DEBUG("%s:setting channel mode to Left",__func__);
            }
          } else {//one of the earbuds role is set, adjust the role
            if (p_scb->channel_mode == 0) {
              p_scbi->channel_mode = 1;//Right
              APPL_TRACE_DEBUG("%s:setting channel mode to Right",__func__);
            } else {
              p_scbi->channel_mode = 0;//Left
              APPL_TRACE_DEBUG("%s:setting channel mode to Left",__func__);
            }
          }
          APPL_TRACE_ERROR("%s:tws pair found",__func__);
          is_tws_pair = true;
          break;
        }
      } else {
        APPL_TRACE_ERROR("%s:tws pair not found",__func__);
        is_tws_pair = false;
        continue;
      }
    }
  }
}
#endif

/*******************************************************************************
 *
 * Function         bta_avk_str_opened
 *
 * Description      Stream opened OK (incoming/outgoing).
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_str_opened(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_CONN_CHG msg;
  uint8_t* p;
  uint16_t mtu;
  uint8_t cur_role;

  last_sent_vsc_cmd = 0;
  msg.hdr.layer_specific = p_scb->hndl;
  msg.is_up = true;
  msg.peer_addr = p_scb->peer_addr;
  p_scb->l2c_cid = AVDT_GetL2CapChannel(p_scb->avdt_handle);
  bta_avk_conn_chg((tBTA_AVK_DATA*)&msg);
  /* set the congestion flag, so AVK would not send media packets by accident */
  p_scb->cong = true;
  p_scb->offload_start_pending = false;
  p_scb->suspend_local_sent = FALSE;

  p_scb->stream_mtu =
      p_data->str_msg.msg.open_ind.peer_mtu - AVDT_MEDIA_HDR_SIZE;
  mtu = bta_avk_chk_mtu(p_scb, p_scb->stream_mtu);
  APPL_TRACE_DEBUG("%s: l2c_cid: 0x%x stream_mtu: %d mtu: %d", __func__,
                   p_scb->l2c_cid, p_scb->stream_mtu, mtu);
  if (mtu == 0 || mtu > p_scb->stream_mtu) mtu = p_scb->stream_mtu;
  APPL_TRACE_DEBUG("%s:updated mtu: %d", __func__, mtu);
  /* Set the media channel as high priority */
  L2CA_SetTxPriority(p_scb->l2c_cid, L2CAP_CHNL_PRIORITY_HIGH);
  L2CA_SetChnlFlushability(p_scb->l2c_cid, true);

  bta_sys_conn_open(BTA_ID_AVK, bta_avk_cb.audio_open_cnt, p_scb->peer_addr);
  memset(&p_scb->q_info, 0, sizeof(tBTA_AVK_Q_INFO));

  p_scb->l2c_bufs = 0;
  p_scb->p_cos->open(p_scb->hndl, mtu);
#if (TWS_ENABLED == TRUE)
  p_scb->tws_device = BTM_SecIsTwsPlusDev(p_scb->peer_addr);
  if (p_scb->tws_device == true) {
    char *codec_name = (char *)A2DP_CodecName(p_scb->cfg.codec_info);
    if (strcmp(codec_name, "aptX-TWS") && strcmp(codec_name, "aptX-adaptive")) {
      APPL_TRACE_DEBUG("%s:TWSP device configured with Non-TWSP ocdec", __func__);
      p_scb->tws_device = false;
    }
  }
//  p_scb->channel_mode = 0;//TODO fetch the valuse from core stack
#endif

  bta_avk_update_flow_spec(p_scb);

  {
    /* TODO check if other audio channel is open.
     * If yes, check if reconfig is needed
     * Rigt now we do not do this kind of checking.
     * BTA-AVK is INT for 2nd audio connection.
     * The application needs to make sure the current codec_info is proper.
     * If one audio connection is open and another SNK attempts to connect to
     * AVK,
     * the connection will be rejected.
     */
    /* check if other audio channel is started. If yes, start */
    tBTA_AVK_OPEN open;
    open.bd_addr = p_scb->peer_addr;
    open.chnl = p_scb->chnl;
    open.hndl = p_scb->hndl;
    open.status = BTA_AVK_SUCCESS;
    open.starting = bta_avk_chk_start(p_scb);
    open.edr = 0;
#if (TWS_ENABLED == TRUE)
    open.tws_device = p_scb->tws_device;
#endif
    L2CA_SetMediaStreamChannel(p_scb->l2c_cid, true);
    // update Master/Slave Role for start
    if (BTM_GetRole (p_scb->peer_addr, &cur_role) == BTM_SUCCESS)
      open.role = cur_role;
    p = BTM_ReadRemoteFeatures(p_scb->peer_addr);
    if (p != NULL) {
      if (HCI_EDR_ACL_2MPS_SUPPORTED(p)) open.edr |= BTA_AVK_EDR_2MBPS;
      if (HCI_EDR_ACL_3MPS_SUPPORTED(p)) {
        if (!interop_match_addr_or_name(INTEROP_2MBPS_LINK_ONLY, &p_scb->peer_addr)) {
          open.edr |= BTA_AVK_EDR_3MBPS;
        }
      }
    }
#if (BTA_AR_INCLUDED == TRUE)
    bta_ar_avdt_conn(BTA_ID_AVK, open.bd_addr);
#endif
    if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SRC) {
      open.sep = AVDT_TSEP_SNK;
    } else if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SNK) {
      open.sep = AVDT_TSEP_SRC;
    }

    tBTA_AVK bta_avk_data;
    bta_avk_data.open = open;
    (*bta_avk_cb.p_cback)(BTA_AVK_OPEN_EVT, &bta_avk_data);
#if (TWS_ENABLED == TRUE)
    APPL_TRACE_DEBUG("%s:audio count  = %d ",__func__, bta_avk_cb.audio_open_cnt);
    if (p_scb->tws_device) {
      bool channel_set = false;
      RawAddress p_addr;
      if (bta_avk_cb.audio_open_cnt > 1 &&
       BTM_SecGetTwsPlusPeerDev(p_scb->peer_addr,p_addr) &&
       !p_addr.IsEmpty()) {
       for (int i = 0; i < BTA_AVK_NUM_STRS; i++) {
         if (bta_avk_cb.p_scb[i]->peer_addr == p_addr &&
          bta_avk_cb.p_scb[i]->state == BTA_AVK_OPEN_SST)
          APPL_TRACE_DEBUG("%s: 2nd TWS device, adjust channel mode",__func__);
          bta_avk_set_tws_chn_mode(p_scb, false);
          channel_set = true;
          break;
        }
      }
      if (!channel_set) {
        APPL_TRACE_DEBUG("%s: 1st TWS device, set default mode",__func__);
        if (open.bd_addr.address[5] % 2)
          p_scb->channel_mode = 0;//Left channel
        else
          p_scb->channel_mode = 1;//Right channe
      }
    }
    if (p_scb->tws_device && ((p_scb->role & BTA_AVK_ROLE_AD_ACP) == 0)) {
    //For outgoing TWS+ connection, initiate avrcp connection
      APPL_TRACE_DEBUG("%s:Initiating avrcp connection for TWS+ remote",__func__);
      bta_avk_open_rc(p_scb, p_data);
    }
#endif
    if (open.starting) {
      bta_avk_ssm_execute(p_scb, BTA_AVK_AP_START_EVT, NULL);
    }
  }

  // This code is used to pass PTS TC for AVDTP ABORT
  char value[PROPERTY_VALUE_MAX] = {0};
  if ((osi_property_get("bluetooth.pts.force_a2dp_abort", value, "false")) &&
      (!strcmp(value, "true"))) {
    APPL_TRACE_ERROR("%s: Calling AVDT_AbortReq", __func__);
    AVDT_AbortReq(p_scb->avdt_handle);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_security_ind
 *
 * Description      Handle an AVDTP security indication.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_security_ind(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  p_scb->avdt_label = p_data->str_msg.msg.hdr.label;

  if (bta_avk_cb.features & BTA_AVK_FEAT_PROTECT) {
    tBTA_AVK_PROTECT_REQ protect_req;
    protect_req.chnl = p_scb->chnl;
    protect_req.hndl = p_scb->hndl;
    protect_req.p_data = p_data->str_msg.msg.security_ind.p_data;
    protect_req.len = p_data->str_msg.msg.security_ind.len;

    tBTA_AVK bta_avk_data;
    bta_avk_data.protect_req = protect_req;
    (*bta_avk_cb.p_cback)(BTA_AVK_PROTECT_REQ_EVT, &bta_avk_data);
  }
  /* app doesn't support security indication; respond with failure */
  else {
    AVDT_SecurityRsp(p_scb->avdt_handle, p_scb->avdt_label, AVDT_ERR_NSC, NULL,
                     0);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_security_cfm
 *
 * Description      Handle an AVDTP security confirm.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_security_cfm(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  if (bta_avk_cb.features & BTA_AVK_FEAT_PROTECT) {
    tBTA_AVK_PROTECT_RSP protect_rsp;
    protect_rsp.chnl = p_scb->chnl;
    protect_rsp.hndl = p_scb->hndl;
    protect_rsp.p_data = p_data->str_msg.msg.security_cfm.p_data;
    protect_rsp.len = p_data->str_msg.msg.security_cfm.len;
    protect_rsp.err_code = p_data->str_msg.msg.hdr.err_code;

    tBTA_AVK bta_avk_data;
    bta_avk_data.protect_rsp = protect_rsp;
    (*bta_avk_cb.p_cback)(BTA_AVK_PROTECT_RSP_EVT, &bta_avk_data);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_do_close
 *
 * Description      Close stream.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_do_close(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  APPL_TRACE_DEBUG("%s: p_scb->co_started=%d", __func__, p_scb->co_started);

  last_sent_vsc_cmd = 0;
  /* stop stream if started */
  if (p_scb->co_started) {
    bta_avk_str_stopped(p_scb, NULL);
  }
  alarm_cancel(bta_avk_cb.link_signalling_timer);

  /* close stream */
  p_scb->started = false;
  p_scb->current_codec = nullptr;
  p_scb->suspend_local_sent = FALSE;

  /* drop the buffers queued in L2CAP */
  L2CA_FlushChannel(p_scb->l2c_cid, L2CAP_FLUSH_CHANS_ALL);

  AVDT_CloseReq(p_scb->avdt_handle);
  /* just in case that the link is congested, link is flow controled by peer or
   * for whatever reason the the close request can not be sent in time.
   * when this timer expires, AVDT_DisconnectReq will be called to disconnect
   * the link
   */
  bta_sys_start_timer(p_scb->avrc_ct_timer, BTA_AVK_CLOSE_REQ_TIME_VAL,
                      BTA_AVK_API_CLOSE_EVT, p_scb->hndl);
}

/*******************************************************************************
 *
 * Function         bta_avk_connect_req
 *
 * Description      Connect AVDTP connection.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_connect_req(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  uint16_t result;
  UNUSED(p_data);

  p_scb->sdp_discovery_started = false;
  if (p_scb->coll_mask & BTA_AVK_COLL_INC_TMR) {
    /* SNK initiated L2C connection while SRC was doing SDP.    */
    /* Wait until timeout to check if SNK starts signalling.    */
    APPL_TRACE_EVENT("%s: coll_mask = 0x%2X", __func__, p_scb->coll_mask);
    p_scb->coll_mask |= BTA_AVK_COLL_API_CALLED;
    APPL_TRACE_EVENT("%s: updated coll_mask = 0x%2X", __func__,
                     p_scb->coll_mask);
    return;
  }

  result = AVDT_ConnectReq(p_scb->peer_addr, p_scb->sec_mask,
                  bta_avk_dt_cback[p_scb->hdi]);
  if (result != AVDT_SUCCESS) {
    /* AVDT connect failed because of resource issue
     * trigger the SDP fail event to enable the cleanup
     * and set the stream to proper state.
     */
    p_scb->open_status = BTA_AVK_FAIL_RESOURCES;
    APPL_TRACE_ERROR("bta_avk_connect_req: AVDT_ConnectReq failed: %d", result);
    bta_avk_ssm_execute(p_scb, BTA_AVK_SDP_DISC_FAIL_EVT, NULL);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_sdp_failed
 *
 * Description      Service discovery failed.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_sdp_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  APPL_TRACE_ERROR("%s: peer_addr=%s open_status=%d", __func__,
                   p_scb->peer_addr.ToString().c_str(), p_scb->open_status);

  if (p_scb->open_status == BTA_AVK_SUCCESS) {
    p_scb->open_status = BTA_AVK_FAIL_SDP;
  }

  p_scb->sdp_discovery_started = false;
  bta_avk_str_closed(p_scb, p_data);
}

/*******************************************************************************
 *
 * Function         bta_avk_disc_results
 *
 * Description      Handle the AVDTP discover results.  Search through the
 *                  results and find the first available stream, and get
 *                  its capabilities.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_disc_results(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t num_snks = 0, num_srcs = 0, i;
  /* our uuid in case we initiate connection */
  uint16_t uuid_int = p_scb->uuid_int;

  /* store number of stream endpoints returned */
  p_scb->num_seps = p_data->str_msg.msg.discover_cfm.num_seps;
  APPL_TRACE_DEBUG("%s: initiator UUID 0x%x, num_seps = %d",
                 __func__, uuid_int, p_scb->num_seps);

  for (i = 0; i < p_scb->num_seps; i++) {
    /* steam not in use, is a sink, and is audio */
    if ((p_scb->sep_info[i].in_use == false) &&
        (p_scb->sep_info[i].media_type == p_scb->media_type)) {
      if ((p_scb->sep_info[i].tsep == AVDT_TSEP_SNK) &&
          (uuid_int == UUID_SERVCLASS_AUDIO_SOURCE))
        num_snks++;

      if ((p_scb->sep_info[i].tsep == AVDT_TSEP_SRC) &&
          (uuid_int == UUID_SERVCLASS_AUDIO_SINK))
        num_srcs++;
    }
  }

  p_scb->p_cos->disc_res(p_scb->hndl, p_scb->num_seps, num_snks, num_srcs,
                         p_scb->peer_addr, uuid_int);
  p_scb->num_disc_snks = num_snks;
  p_scb->num_disc_srcs = num_srcs;

  /* if we got any */
  if (p_scb->num_seps > 0) {
    /* initialize index into discovery results */
    p_scb->sep_info_idx = 0;

    /* get the capabilities of the first available stream */
    bta_avk_next_getcap(p_scb, p_data);
  }
  /* else we got discover response but with no streams; we're done */
  else {
    APPL_TRACE_ERROR("%s: BTA_AVK_STR_DISC_FAIL_EVT: peer_addr=%s", __func__,
                     p_scb->peer_addr.ToString().c_str());
    bta_avk_ssm_execute(p_scb, BTA_AVK_STR_DISC_FAIL_EVT, p_data);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_disc_res_as_acp
 *
 * Description      Handle the AVDTP discover results.  Search through the
 *                  results and find the first available stream, and get
 *                  its capabilities.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_disc_res_as_acp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t num_snks = 0, i;

  /* store number of stream endpoints returned */
  p_scb->num_seps = p_data->str_msg.msg.discover_cfm.num_seps;
  for (i = 0; i < p_scb->num_seps; i++) {
    /* steam is a sink, and is audio */
    if ((p_scb->sep_info[i].tsep == AVDT_TSEP_SNK) &&
        (p_scb->sep_info[i].media_type == p_scb->media_type)) {
      p_scb->sep_info[i].in_use = false;
      num_snks++;
    }
  }

  APPL_TRACE_DEBUG("%s: peer_addr: %s, num_seps = %d, num_snks = %d",
          __func__, p_scb->peer_addr.ToString().c_str(), p_scb->num_seps, num_snks);

  p_scb->p_cos->disc_res(p_scb->hndl, p_scb->num_seps, num_snks, 0,
                         p_scb->peer_addr, UUID_SERVCLASS_AUDIO_SOURCE);
  p_scb->num_disc_snks = num_snks;
  p_scb->num_disc_srcs = 0;

  /* if we got any */
  if (p_scb->num_seps > 0) {
    if (p_scb->cache_setconfig) {
      APPL_TRACE_DEBUG("%s: Got discover_res as ok from remote.", __func__);
      memset(p_scb->cache_setconfig, 0, sizeof(tBTA_AVK_DATA));
      osi_free(p_scb->cache_setconfig);
      p_scb->cache_setconfig = NULL;
    }
    /* initialize index into discovery results */
    p_scb->sep_info_idx = 0;

    /* get the capabilities of the first available stream */
    bta_avk_next_getcap(p_scb, p_data);
  }
  /* else we got discover response but with no streams,
     so, we will send the STR_DISC_FAIL event which has been handled
     through the API bta_avk_disc_fail_as_acp, where it would initiate
     get_caps for the SEP on which remote does set_cofig, so that connection
     won't drop. */
  else {
    APPL_TRACE_ERROR("%s: BTA_AVK_STR_DISC_FAIL_EVT: peer_addr=%s", __func__,
                     p_scb->peer_addr.ToString().c_str());
    bta_avk_ssm_execute(p_scb, BTA_AVK_STR_DISC_FAIL_EVT, p_data);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_save_caps
 *
 * Description      report the SNK SEP capabilities to application
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_save_caps(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tAVDT_CFG cfg;
  tAVDT_SEP_INFO* p_info = &p_scb->sep_info[p_scb->sep_info_idx];
  uint8_t old_wait = p_scb->wait;
  bool getcap_done = false;
  uint8_t media_type;
  tA2DP_CODEC_TYPE codec_type;

  APPL_TRACE_DEBUG("%s: num_seps:%d sep_info_idx:%d wait:x%x", __func__,
                   p_scb->num_seps, p_scb->sep_info_idx, p_scb->wait);

  media_type = A2DP_GetMediaType(p_scb->p_cap->codec_info);
  codec_type = A2DP_GetCodecType(p_scb->p_cap->codec_info);
  APPL_TRACE_DEBUG("%s: num_codec %d", __func__, p_scb->p_cap->num_codec);
  APPL_TRACE_DEBUG("%s: media type: x%x, x%x, codec_type: %x, min/max bitpool: %x/%x,",
                    __func__, media_type, p_scb->media_type, codec_type,
                    p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET],
                    p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]);
  if (codec_type ==A2DP_MEDIA_CT_SBC ) {
    //minbitpool < 2, then set minbitpool = 2
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]) < A2DP_SBC_IE_MIN_BITPOOL) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET] = A2DP_SBC_IE_MIN_BITPOOL;
      APPL_TRACE_DEBUG("%s: Set min bitpool: %x", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]);
    }

    //minbitpool > 250, then set minbitpool = 250
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]) > A2DP_SBC_IE_MAX_BITPOOL) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET] = A2DP_SBC_IE_MAX_BITPOOL;
      APPL_TRACE_DEBUG("%s: Set min bitpool: %x", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]);
    }

    //maxbitpool > 250, then set maxbitpool = 250
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]) > A2DP_SBC_IE_MAX_BITPOOL) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET] = A2DP_SBC_IE_MAX_BITPOOL;
      APPL_TRACE_DEBUG("%s: Set max bitpool: %x", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]);
    }

    //minbitpool > maxbitpool, then set maxbitpool = minbitpool
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]) >
        (p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET])) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET] =
                       p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET];
      APPL_TRACE_WARNING("%s min bitpool value received for SBC is more than DUT supported Max bitpool"
                          "Clamping the max bitpool configuration further from %d to %d.", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET],
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]);
    }
  }
  A2DP_DumpCodecInfo(p_scb->p_cap->codec_info);

  memcpy(&cfg, p_scb->p_cap, sizeof(tAVDT_CFG));
  /* let application know the capability of the SNK */
  p_scb->p_cos->getcfg(p_scb->hndl, cfg.codec_info, &p_scb->sep_info_idx,
                       p_info->seid, &cfg.num_protect, cfg.protect_info);

  p_scb->sep_info_idx++;
  APPL_TRACE_DEBUG("%s: result: sep_info_idx:%d", __func__,
                   p_scb->sep_info_idx);
  A2DP_DumpCodecInfo(cfg.codec_info);

  if (p_scb->num_seps > p_scb->sep_info_idx) {
    /* Some devices have seps at the end of the discover list, which is not */
    /* matching media type(video not audio).                                */
    /* In this case, we are done with getcap without sending another        */
    /* request to AVDT.                                                     */
    if (!bta_avk_next_getcap(p_scb, p_data)) getcap_done = true;
  } else
    getcap_done = true;

  if (getcap_done) {
    APPL_TRACE_DEBUG("%s: getcap_done: num_seps:%d sep_info_idx:%d wait:x%x",
                     __func__, p_scb->num_seps, p_scb->sep_info_idx,
                     p_scb->wait);
    p_scb->wait &= ~(BTA_AVK_WAIT_ACP_CAPS_ON | BTA_AVK_WAIT_ACP_CAPS_STARTED);
    if (old_wait & BTA_AVK_WAIT_ACP_CAPS_STARTED) {
      bta_avk_start_ok(p_scb, NULL);
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_set_use_rc
 *
 * Description      set to use AVRC for this stream control block.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_set_use_rc(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  p_scb->use_rc = true;
}

/*******************************************************************************
 *
 * Function         bta_avk_cco_close
 *
 * Description      call close call-out function.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_cco_close(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  uint16_t mtu;

  mtu = bta_avk_chk_mtu(p_scb, BTA_AVK_MAX_A2DP_MTU);

  p_scb->p_cos->close(p_scb->hndl);
}

/*******************************************************************************
 *
 * Function         bta_avk_open_failed
 *
 * Description      Failed to open an AVDT stream
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_open_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  bool is_av_opened = false;
  tBTA_AVK_SCB* p_opened_scb = NULL;
  uint8_t idx;
  tBTA_AVK_OPEN open;
  uint8_t cur_role;

  APPL_TRACE_ERROR("%s: peer_addr=%s", __func__,
                   p_scb->peer_addr.ToString().c_str());
  p_scb->open_status = BTA_AVK_FAIL_STREAM;
  bta_avk_cco_close(p_scb, p_data);

  /* check whether there is already an opened audio or video connection with the
   * same device */
  for (idx = 0; (idx < BTA_AVK_NUM_STRS) && (is_av_opened == false); idx++) {
    p_opened_scb = bta_avk_cb.p_scb[idx];
    if (p_opened_scb && (p_opened_scb->state == BTA_AVK_OPEN_SST) &&
        (p_opened_scb->peer_addr == p_scb->peer_addr))
      is_av_opened = true;
  }

  /* if there is already an active AVK connnection with the same bd_addr,
     don't send disconnect req, just report the open event with
     BTA_AVK_FAIL_GET_CAP status */
  if (is_av_opened == true) {
    open.bd_addr = p_scb->peer_addr;
    open.chnl = p_scb->chnl;
    open.hndl = p_scb->hndl;
    open.status = BTA_AVK_FAIL_GET_CAP;
    open.starting = bta_avk_chk_start(p_scb);
    open.edr = 0;
    /* update Master/Slave Role for open event */
    if (BTM_GetRole (p_scb->peer_addr, &cur_role) == BTM_SUCCESS)
      open.role = cur_role;
    /* set the state back to initial state */
    bta_avk_set_scb_sst_init(p_scb);

    if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SRC) {
      open.sep = AVDT_TSEP_SNK;
    } else if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SNK) {
      open.sep = AVDT_TSEP_SRC;
    }

    APPL_TRACE_ERROR(
        "%s: there is already an active connection: peer_addr=%s chnl=%d "
        "hndl=%d status=%d starting=%d edr=%d",
        __func__, open.bd_addr.ToString().c_str(), open.chnl, open.hndl,
        open.status, open.starting, open.edr);

    tBTA_AVK bta_avk_data;
    bta_avk_data.open = open;
    (*bta_avk_cb.p_cback)(BTA_AVK_OPEN_EVT, &bta_avk_data);
  } else {
    AVDT_DisconnectReq(p_scb->peer_addr, bta_avk_dt_cback[p_scb->hdi]);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_getcap_results
 *
 * Description      Handle the AVDTP get capabilities results.  Check the codec
 *                  type and see if it matches ours.  If it does not, get the
 *                  capabilities of the next stream, if any.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_getcap_results(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tAVDT_CFG cfg;
  uint8_t media_type;
  tAVDT_SEP_INFO* p_info = &p_scb->sep_info[p_scb->sep_info_idx];
  uint16_t uuid_int; /* UUID for which connection was initiatied */
  tA2DP_CODEC_TYPE codec_type;

  if (p_scb == NULL)
  {
    APPL_TRACE_ERROR("%s: no scb found for handle", __func__);
    return;
  }

  if (p_scb != NULL && p_scb->p_cap == NULL)
  {
    APPL_TRACE_ERROR("bta_avk_getcap_results: p_scb->p_cap is NULL");
    bta_avk_ssm_execute(p_scb, BTA_AVK_STR_GETCAP_FAIL_EVT, p_data);
    return;
  }


  media_type = A2DP_GetMediaType(p_scb->p_cap->codec_info);
  codec_type = A2DP_GetCodecType(p_scb->p_cap->codec_info);
  APPL_TRACE_DEBUG("%s: num_codec %d", __func__, p_scb->p_cap->num_codec);
  APPL_TRACE_DEBUG("%s: media type: x%x, x%x, codec_type: %x, min/max bitpool: %x/%x,",
                    __func__, media_type, p_scb->media_type, codec_type,
                    p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET],
                    p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]);
  if (codec_type == A2DP_MEDIA_CT_SBC ) {
    //minbitpool < 2, then set minbitpool = 2
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]) < A2DP_SBC_IE_MIN_BITPOOL) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET] = A2DP_SBC_IE_MIN_BITPOOL;
      APPL_TRACE_DEBUG("%s: Set min bitpool: %x", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]);
    }

    //minbitpool > 250, then set minbitpool = 250
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]) > A2DP_SBC_IE_MAX_BITPOOL) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET] = A2DP_SBC_IE_MAX_BITPOOL;
      APPL_TRACE_DEBUG("%s: Set min bitpool: %x", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]);
    }

    //maxbitpool > 250, then set maxbitpool = 250
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]) > A2DP_SBC_IE_MAX_BITPOOL) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET] = A2DP_SBC_IE_MAX_BITPOOL;
      APPL_TRACE_DEBUG("%s: Set max bitpool: %x", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]);
    }

    //minbitpool > maxbitpool, then set maxbitpool = minbitpool
    if ((p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]) >
        (p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET])) {
      p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET] =
                       p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET];
      APPL_TRACE_WARNING("%s min bitpool value received for SBC is more than DUT supported Max bitpool"
                          "Clamping the max bitpool configuration further from %d to %d.", __func__,
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET],
                           p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET]);
    }
  }

  memcpy(&cfg, &p_scb->cfg, sizeof(tAVDT_CFG));
  cfg.num_codec = 1;
  cfg.num_protect = p_scb->p_cap->num_protect;
  memcpy(cfg.codec_info, p_scb->p_cap->codec_info, AVDT_CODEC_SIZE);
  memcpy(cfg.protect_info, p_scb->p_cap->protect_info, AVDT_PROTECT_SIZE);

  APPL_TRACE_DEBUG("%s: min/max bitpool: %x/%x", __func__,
                     p_scb->p_cap->codec_info[A2DP_SBC_IE_MIN_BITPOOL_OFFSET],
                     p_scb->p_cap->codec_info[A2DP_SBC_IE_MAX_BITPOOL_OFFSET]);
  A2DP_DumpCodecInfo(p_scb->p_cap->codec_info);

  /* if codec present and we get a codec configuration */
  if ((p_scb->p_cap->num_codec != 0) && (media_type == p_scb->media_type) &&
      (p_scb->p_cos->getcfg(p_scb->hndl, cfg.codec_info, &p_scb->sep_info_idx,
                            p_info->seid, &cfg.num_protect,
                            cfg.protect_info) == A2DP_SUCCESS)) {
    /* save copy of codec configuration */
    memcpy(&p_scb->cfg, &cfg, sizeof(tAVDT_CFG));

    APPL_TRACE_DEBUG("%s: result: sep_info_idx=%d", __func__,
                     p_scb->sep_info_idx);
    A2DP_DumpCodecInfo(p_scb->p_cap->codec_info);

    uuid_int = p_scb->uuid_int;
    APPL_TRACE_DEBUG("%s: initiator UUID = 0x%x", __func__, uuid_int);
    if (uuid_int == UUID_SERVCLASS_AUDIO_SOURCE)
      bta_avk_adjust_seps_idx(p_scb,
                             bta_avk_get_scb_handle(p_scb, AVDT_TSEP_SRC));
    else if (uuid_int == UUID_SERVCLASS_AUDIO_SINK)
      bta_avk_adjust_seps_idx(p_scb,
                             bta_avk_get_scb_handle(p_scb, AVDT_TSEP_SNK));

    /* use only the services peer supports */
    cfg.psc_mask &= p_scb->p_cap->psc_mask;
    p_scb->cur_psc_mask = cfg.psc_mask;

    if (btif_avk_device_in_sink_role()) {
      if (p_scb->cur_psc_mask & AVDT_PSC_DELAY_RPT)
        p_scb->avdt_version = AVDT_VERSION_SYNC;
      else
        p_scb->avdt_version = AVDT_VERSION;
    }

    if ((uuid_int == UUID_SERVCLASS_AUDIO_SINK) &&
        (p_scb->seps[p_scb->sep_idx].p_app_sink_data_cback != NULL)) {
      APPL_TRACE_DEBUG("%s: configure decoder for Sink connection", __func__);
      tBTA_AVK_SINK_MEDIA av_sink_codec_info;
      av_sink_codec_info.avk_config.bd_addr = p_scb->peer_addr;
      av_sink_codec_info.avk_config.codec_info = p_scb->cfg.codec_info;
      p_scb->seps[p_scb->sep_idx].p_app_sink_data_cback(
          BTA_AVK_SINK_MEDIA_CFG_EVT, &av_sink_codec_info, p_scb->peer_addr);
    }

    if (uuid_int == UUID_SERVCLASS_AUDIO_SOURCE) {
      A2DP_AdjustCodec(cfg.codec_info);
    }
    /* open the stream */
    AVDT_OpenReq(p_scb->seps[p_scb->sep_idx].av_handle, p_scb->peer_addr,
                 p_scb->sep_info[p_scb->sep_info_idx].seid, &cfg);

    if (!bta_avk_is_rcfg_sst(p_scb)) {
      /* free capabilities buffer */
      osi_free_and_reset((void**)&p_scb->p_cap);
    }
  } else {
    /* try the next stream, if any */
    p_scb->sep_info_idx++;
    bta_avk_next_getcap(p_scb, p_data);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_setconfig_rej
 *
 * Description      Send AVDTP set config reject.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_setconfig_rej(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_REJECT reject;
  uint8_t avdt_handle = p_data->ci_setconfig.avdt_handle;

  bta_avk_adjust_seps_idx(p_scb, avdt_handle);
  APPL_TRACE_DEBUG("%s: sep_idx: %d", __func__, p_scb->sep_idx);
  AVDT_ConfigRsp(p_scb->avdt_handle, p_scb->avdt_label, AVDT_ERR_UNSUP_CFG, 0);

  APPL_TRACE_DEBUG("%s peer_address: %s, handle: %d", __func__, p_scb->peer_addr.ToString().c_str(), p_scb->hndl);
  //reject.bd_addr = p_data->str_msg.bd_addr;
  reject.bd_addr = p_scb->peer_addr;
  reject.hndl = p_scb->hndl;

  tBTA_AVK bta_avk_data;
  bta_avk_data.reject = reject;
  (*bta_avk_cb.p_cback)(BTA_AVK_REJECT_EVT, &bta_avk_data);
}

/*******************************************************************************
 *
 * Function         bta_avk_discover_req
 *
 * Description      Send an AVDTP discover request to the peer.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_discover_req(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  /* send avdtp discover request */

    /* send avdtp discover request */
    if (AVDT_DiscoverReq(p_scb->peer_addr, p_scb->sep_info, BTA_AVK_NUM_SEPS, bta_avk_dt_cback[p_scb->hdi]) != AVDT_SUCCESS)
    {
        APPL_TRACE_ERROR("bta_avk_discover_req command could not be sent because of resource constraint");
        bta_avk_ssm_execute(p_scb, BTA_AVK_STR_DISC_FAIL_EVT, p_data);
    }
}

/*******************************************************************************
 *
 * Function         bta_avk_conn_failed
 *
 * Description      AVDTP connection failed.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_conn_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  APPL_TRACE_ERROR("%s: peer_addr=%s open_status=%d", __func__,
                   p_scb->peer_addr.ToString().c_str(), p_scb->open_status);

  p_scb->open_status = BTA_AVK_FAIL_STREAM;
  bta_avk_str_closed(p_scb, p_data);
}

/*******************************************************************************
 *
 * Function         bta_avk_do_start
 *
 * Description      Start stream.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_do_start(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;
  uint8_t cur_role = BTM_ROLE_UNDEFINED;

  APPL_TRACE_DEBUG("%s: sco_occupied:%d, role:x%x, started:%d", __func__,
                   bta_avk_cb.sco_occupied, p_scb->role, p_scb->started);
  if (bta_avk_cb.sco_occupied) {
    bta_avk_start_failed(p_scb, p_data);
    return;
  }

  /* disallow role switch during streaming, only if we are the master role
   * i.e. allow role switch, if we are slave.
   * It would not hurt us, if the peer device wants us to be master */
  if ((BTM_GetRole(p_scb->peer_addr, &cur_role) == BTM_SUCCESS) &&
      (cur_role == BTM_ROLE_MASTER)) {
    policy |= HCI_ENABLE_MASTER_SLAVE_SWITCH;
  }

  bta_sys_clear_policy(BTA_ID_AVK, policy, p_scb->peer_addr);

  if (cur_role == BTM_ROLE_SLAVE) {
    BTM_SetA2dpStreamQoS(p_scb->peer_addr, NULL);
  }

  if ((p_scb->started == false) &&
      ((p_scb->role & BTA_AVK_ROLE_START_INT) == 0)) {
    p_scb->role |= BTA_AVK_ROLE_START_INT;
    bta_sys_busy(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);

    AVDT_StartReq(&p_scb->avdt_handle, 1);
#if (TWS_STATE_ENABLED == TRUE)
    p_scb->start_pending = true;
#endif
  } else if (p_scb->started) {
    p_scb->role |= BTA_AVK_ROLE_START_INT;
    if (p_scb->wait == 0) {
      if (p_scb->role & BTA_AVK_ROLE_SUSPEND) {
        notify_start_failed(p_scb);
      } else {
        bta_avk_start_ok(p_scb, NULL);
      }
    }
  }
  APPL_TRACE_DEBUG("%s: started %d role:x%x", __func__, p_scb->started,
                   p_scb->role);
}

/*******************************************************************************
 *
 * Function         bta_avk_str_stopped
 *
 * Description      Stream stopped.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_str_stopped(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_SUSPEND suspend_rsp;
  uint8_t start = p_scb->started;
  bool sus_evt = true;
  BT_HDR* p_buf;
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;
  char *codec_name = (char *)A2DP_CodecName(p_scb->cfg.codec_info);

  APPL_TRACE_ERROR("%s: audio_open_cnt=%d, p_data %p", __func__,
                   bta_avk_cb.audio_open_cnt, p_data);

  bta_sys_idle(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
  if ((bta_avk_cb.features & BTA_AVK_FEAT_MASTER) == 0 ||
      bta_avk_cb.audio_open_cnt == 1)
    policy |= HCI_ENABLE_MASTER_SLAVE_SWITCH;
  bta_sys_set_policy(BTA_ID_AVK, policy, p_scb->peer_addr);

  if (p_scb->co_started) {
    /* TODO(eisenbach): RE-IMPLEMENT USING VSC OR HAL EXTENSION
    vendor_get_interface()->send_command(
        (vendor_opcode_t)BT_VND_OP_A2DP_OFFLOAD_STOP, (void*)&p_scb->l2c_cid);
    if (p_scb->offload_start_pending) {
      tBTA_AVK_STATUS status = BTA_AVK_FAIL_STREAM;
      tBTA_AVK bta_avk_data;
      bta_avk_data.status = status;
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, &bta_avk_data);
    }
    p_scb->offload_start_pending = false;
    */
    if (BTM_IS_QTI_CONTROLLER() && p_scb->offload_supported) {
      bta_avk_vendor_offload_stop(p_scb);
      //p_scb->offload_supported = false;
    }
    if (p_scb->role & BTA_AVK_ROLE_START_INT) {
      p_scb->role &= ~BTA_AVK_ROLE_START_INT;
      APPL_TRACE_DEBUG(" %s: role:x%x, started:%d", __func__,
                       p_scb->role, p_scb->started);
    }
    bta_avk_stream_chg(p_scb, false);
    p_scb->co_started = false;

    p_scb->p_cos->stop(p_scb->hndl);
    L2CA_SetFlushTimeout(p_scb->peer_addr, L2CAP_DEFAULT_FLUSH_TO);
  }

  /* if q_info.a2dp_list is not empty, drop it now */
  if (BTA_AVK_CHNL_AUDIO == p_scb->chnl) {
    while (!list_is_empty(p_scb->a2dp_list)) {
      p_buf = (BT_HDR*)list_front(p_scb->a2dp_list);
      list_remove(p_scb->a2dp_list, p_buf);
      osi_free(p_buf);
    }

    /* drop the audio buffers queued in L2CAP */
    if (p_data && p_data->api_stop.flush)
      L2CA_FlushChannel(p_scb->l2c_cid, L2CAP_FLUSH_CHANS_ALL);
  }

  suspend_rsp.chnl = p_scb->chnl;
  suspend_rsp.hndl = p_scb->hndl;

  if (p_data && p_data->api_stop.suspend) {
      APPL_TRACE_DEBUG("suspending: %d, sup:%d, suspend_local_sent = %d",
                         start, p_scb->suspend_sup,p_scb->suspend_local_sent);
      if ((start)  && (p_scb->suspend_sup) && (!p_scb->suspend_local_sent)) {
      p_scb->suspend_local_sent = TRUE;
      sus_evt = false;
      p_scb->l2c_bufs = 0;
      AVDT_SuspendReq(&p_scb->avdt_handle, 1);
    }

    /* send SUSPEND_EVT event only if not in reconfiguring state and sus_evt is
     * true*/
      if ((sus_evt) && ((p_scb->suspend_local_sent) || (p_scb->state != BTA_AVK_RCFG_SST))) {
      suspend_rsp.status = BTA_AVK_SUCCESS;
      suspend_rsp.initiator = true;
      tBTA_AVK bta_avk_data;
      bta_avk_data.suspend = suspend_rsp;
      (*bta_avk_cb.p_cback)(BTA_AVK_SUSPEND_EVT, &bta_avk_data);
    }
  } else {
    suspend_rsp.status = BTA_AVK_SUCCESS;
    suspend_rsp.initiator = true;
    APPL_TRACE_EVENT("%s: status %d", __func__, suspend_rsp.status);

    // Send STOP_EVT event only if not in reconfiguring state.
    // However, we should send STOP_EVT if we are reconfiguring when taking
    // the Close->Configure->Open->Start path.
    if (p_scb->state != BTA_AVK_RCFG_SST ||
        (p_data && p_data->api_stop.reconfig_stop)) {
      tBTA_AVK bta_avk_data;
      bta_avk_data.suspend = suspend_rsp;
      (*bta_avk_cb.p_cback)(BTA_AVK_STOP_EVT, &bta_avk_data);
    }
  }
  if(strstr(codec_name, "aptX") != NULL) {
    APPL_TRACE_WARNING("%s: Removing packet type limit for aptX codec stream stop", __func__);
    AVDT_UpdateLinkPktType(p_scb->avdt_handle, btm_cb.btm_acl_pkt_types_supported);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_reconfig
 *
 * Description      process the reconfigure request.
 *                  save the parameter in control block and
 *                  suspend, reconfigure or close the stream
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_reconfig(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tAVDT_CFG* p_cfg;
  tBTA_AVK_API_STOP stop;
  tBTA_AVK_RECONFIG evt;
  tBTA_AVK_API_RCFG* p_rcfg = &p_data->api_reconfig;

  APPL_TRACE_DEBUG("%s: r:%d, s:%d idx: %d (o:%d)", __func__, p_scb->recfg_sup,
                   p_scb->suspend_sup, p_scb->rcfg_idx, p_scb->sep_info_idx);

  p_scb->num_recfg = 0;
  /* store the new configuration in control block */
  if (p_scb->p_cap == NULL)
    p_scb->p_cap = (tAVDT_CFG*)osi_malloc(sizeof(tAVDT_CFG));
  if ((p_cfg = p_scb->p_cap) == NULL) {
    /* report failure */
    evt.status = BTA_AVK_FAIL_RESOURCES;
    evt.chnl   = p_scb->chnl;
    evt.hndl   = p_scb->hndl;
    (*bta_avk_cb.p_cback)(BTA_AVK_RECONFIG_EVT, (tBTA_AVK *)&evt);

    /* this event is not possible in this state.
     * use it to bring the SSM back to open state
     */
    bta_avk_ssm_execute(p_scb, BTA_AVK_SDP_DISC_OK_EVT, NULL);
    return;
  }
  btav_a2dp_codec_index_t curr_codec_index = A2DP_SourceCodecIndex(p_scb->cfg.codec_info);
  p_cfg = &p_scb->cfg;

  alarm_cancel(p_scb->avrc_ct_timer);

  APPL_TRACE_DEBUG(
      "%s: p_scb->sep_info_idx=%d p_scb->rcfg_idx=%d p_rcfg->sep_info_idx=%d",
      __func__, p_scb->sep_info_idx, p_scb->rcfg_idx, p_rcfg->sep_info_idx);
  A2DP_DumpCodecInfo(p_scb->p_cap->codec_info);
  A2DP_DumpCodecInfo(p_scb->cfg.codec_info);
  A2DP_DumpCodecInfo(p_rcfg->codec_info);

  p_cfg->num_protect = p_rcfg->num_protect;
  memcpy(p_cfg->codec_info, p_rcfg->codec_info, AVDT_CODEC_SIZE);
  memcpy(p_cfg->protect_info, p_rcfg->p_protect_info, p_rcfg->num_protect);
  p_scb->rcfg_idx = p_rcfg->sep_info_idx;
  p_cfg->psc_mask = p_scb->cur_psc_mask;

  // If the requested SEP index is same as the current one, then we
  // can Suspend->Reconfigure->Start.
  // Otherwise, we have to Close->Configure->Open->Start or
  // Close->Configure->Open for streams that are / are not started.
   APPL_TRACE_DEBUG("rcfg_idx:%d,sep_info_idx:%d,suspend:%d,recfg_sup:%d,suspend_sup:%d",
                     p_scb->rcfg_idx,
                     p_scb->sep_info_idx,
                     p_rcfg->suspend,
                     p_scb->recfg_sup,
                     p_scb->suspend_sup);
  btav_a2dp_codec_index_t rcfg_codec_index = A2DP_SourceCodecIndex(p_cfg->codec_info);
  APPL_TRACE_DEBUG("curr_index: %d, rcfg_index: %d",curr_codec_index,rcfg_codec_index);
  // p_scb->sep_info_idx > p_scb->num_seps condition satified for remote initiated SetConfig
  if ((p_scb->rcfg_idx == p_scb->sep_info_idx ||
      (p_scb->sep_info_idx > p_scb->num_seps &&
      curr_codec_index == rcfg_codec_index)) &&
      p_rcfg->suspend && p_scb->recfg_sup && p_scb->suspend_sup) {
      APPL_TRACE_DEBUG("p_scb->started:%d", p_scb->started);
      if (p_scb->sep_info_idx > p_scb->num_seps) p_scb->sep_info_idx = p_scb->rcfg_idx;
    if (p_scb->started) {
      // Suspend->Reconfigure->Start
      stop.flush = false;
      stop.suspend = true;
      stop.reconfig_stop = false;
      bta_avk_str_stopped(p_scb, (tBTA_AVK_DATA*)&stop);
    } else {
      // Reconfigure
      APPL_TRACE_DEBUG("%s: reconfig", __func__);
      A2DP_DumpCodecInfo(p_scb->cfg.codec_info);
      AVDT_ReconfigReq(p_scb->avdt_handle, &p_scb->cfg);
      p_scb->cfg.psc_mask = p_scb->cur_psc_mask;
    }
  } else {
    // Close the stream first, and then Configure it
    APPL_TRACE_DEBUG("%s: Close/Open started: %d state: %d num_protect: %d",
                     __func__, p_scb->started, p_scb->state,
                     p_cfg->num_protect);
    if (p_scb->started) {
      // Close->Configure->Open->Start
      if ((p_scb->rcfg_idx != p_scb->sep_info_idx) && p_scb->recfg_sup) {
        // Make sure we trigger STOP_EVT when taking the longer road to
        // reconfiguration, otherwise we don't call Start.
        stop.flush = false;
        stop.suspend = false;
        stop.reconfig_stop = true;
        bta_avk_str_stopped(p_scb, (tBTA_AVK_DATA*)&stop);
      } else {
        bta_avk_str_stopped(p_scb, NULL);
      }
      p_scb->started = false;
    } else {
      // Close->Configure->Open
      bta_avk_str_stopped(p_scb, NULL);
    }
    // Drop the buffers queued in L2CAP
    L2CA_FlushChannel(p_scb->l2c_cid, L2CAP_FLUSH_CHANS_ALL);
    AVDT_CloseReq(p_scb->avdt_handle);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_update_enc_mode
 *
 * Description      Sends Vendor Specific Command to SoC
 *                  with new encoder mode.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_update_enc_mode(tBTA_AVK_DATA* p_data) {
  uint8_t param[48];
  uint8_t *p_param;
  uint8_t *num_sub_band;
  int param_len = 0;
  uint16_t enc_mode = p_data->encoder_mode.enc_mode;

  memset(param, 0, 48);
  p_param = param;

  *p_param++ = VS_QHCI_ENCODER_MODE_CHANGE;
  param_len++;
  num_sub_band = p_param++;
  param_len++;

  update_sub_band_info(&p_param, &param_len, BTA_AVK_ENCODER_MODE_CHANGE_ID, enc_mode);
  *num_sub_band += 1;

   BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE, param_len,
                                 param, NULL);
}
void bta_avk_update_aptx_data(tBTA_AVK_DATA* p_data) {
  uint8_t param[48];
  uint8_t *p_param;
  uint8_t *num_sub_band;
  int param_len = 0;
  uint8_t subband_id = p_data->aptx_data.type;
  uint8_t subband_data = p_data->aptx_data.data;
  memset(param, 0, 48);
  p_param = param;
  *p_param++ = VS_QHCI_ENCODER_MODE_CHANGE;
  param_len++;
  num_sub_band = p_param++;
  param_len++;
  update_sub_band_info(&p_param, &param_len, subband_id, &subband_data, 1);
  *num_sub_band += 1;
   BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE, param_len,
                                 param, enc_mode_change_callback);
}

static void update_sub_band_info(uint8_t **param, int *p_param_len, uint8_t id, uint16_t data)
{
  uint8_t *p_param = *param;
  *p_param++ = BTA_AVK_ENCODER_MODE_CHANGE_ID;
  *p_param_len += 1;

  *p_param++ = 2; /* size of uint16_t */
  *p_param_len += 1;

  UINT16_TO_STREAM(p_param, data);
  *p_param_len += 2;
  *param = p_param;
}

void update_sub_band_info(uint8_t **param, int *p_param_len, uint8_t id, uint8_t *data, uint8_t size)
{
  uint8_t *p_param = *param;
  *p_param++ = BTA_AVK_ENCODER_DATA_ID;
  *p_param_len += 1;
  *p_param++ = 2;
  *p_param_len += 1;
  *p_param++ = id;
  *p_param++ = *data;
  *p_param_len += 2;
  *param = p_param;
}

static void enc_mode_change_callback(tBTM_VSC_CMPL *param)
{
  APPL_TRACE_ERROR("Received Encoder mode changed Callback from Controller");
}

/*******************************************************************************
 *
 * Function         bta_avk_data_path
 *
 * Description      Handle stream data path.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_data_path(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  BT_HDR* p_buf = NULL;
  uint32_t timestamp;
  bool new_buf = false;
  uint8_t m_pt = 0x60;
  tAVDT_DATA_OPT_MASK opt;

  APPL_TRACE_DEBUG("%s: p_scb->cong: %d", __func__, p_scb->cong);
  if (p_scb->cong) return;

  // Always get the current number of bufs que'd up
  p_scb->l2c_bufs =
      (uint8_t)L2CA_FlushChannel(p_scb->l2c_cid, L2CAP_FLUSH_CHANS_GET);

  if (!list_is_empty(p_scb->a2dp_list)) {
    p_buf = (BT_HDR*)list_front(p_scb->a2dp_list);
    list_remove(p_scb->a2dp_list, p_buf);
    /* use q_info.a2dp data, read the timestamp */
    timestamp = *(uint32_t*)(p_buf + 1);
  } else {
    new_buf = true;
    /* A2DP_list empty, call co_data, dup data to other channels */
    p_buf = (BT_HDR*)p_scb->p_cos->data(p_scb->cfg.codec_info, &timestamp);

    if (p_buf) {
      APPL_TRACE_DEBUG("%s: p_buf is valid: %d", __func__);
      /* use the offset area for the time stamp */
      *(uint32_t*)(p_buf + 1) = timestamp;

      /* dup the data to other channels */
      bta_avk_dup_audio_buf(p_scb, p_buf);
    }
  }

  if (p_buf) {
    if (p_scb->l2c_bufs < (BTA_AVK_QUEUE_DATA_CHK_NUM)) {
      /* There's a buffer, just queue it to L2CAP.
       * There's no need to increment it here, it is always read from
       * L2CAP (see above).
       */

      /* opt is a bit mask, it could have several options set */
      opt = AVDT_DATA_OPT_NONE;
      if (p_scb->no_rtp_hdr) {
        opt |= AVDT_DATA_OPT_NO_RTP;
      }

      //
      // Fragment the payload if larger than the MTU.
      // NOTE: The fragmentation is RTP-compatibie.
      //
      size_t extra_fragments_n = 0;
      if (p_buf->len > 0) {
        extra_fragments_n = (p_buf->len / p_scb->stream_mtu) +
                            ((p_buf->len % p_scb->stream_mtu) ? 1 : 0) - 1;
      }
      std::vector<BT_HDR*> extra_fragments;
      extra_fragments.reserve(extra_fragments_n);

      uint8_t* data_begin = (uint8_t*)(p_buf + 1) + p_buf->offset;
      uint8_t* data_end = (uint8_t*)(p_buf + 1) + p_buf->offset + p_buf->len;
      while (extra_fragments_n-- > 0) {
        data_begin += p_scb->stream_mtu;
        size_t fragment_len = data_end - data_begin;
        if (fragment_len > p_scb->stream_mtu) fragment_len = p_scb->stream_mtu;

        BT_HDR* p_buf2 = (BT_HDR*)osi_malloc(BT_DEFAULT_BUFFER_SIZE);
        p_buf2->offset = p_buf->offset;
        p_buf2->len = 0;
        p_buf2->layer_specific = 0;
        uint8_t* packet2 =
            (uint8_t*)(p_buf2 + 1) + p_buf2->offset + p_buf2->len;
        memcpy(packet2, data_begin, fragment_len);
        p_buf2->len += fragment_len;
        extra_fragments.push_back(p_buf2);
        p_buf->len -= fragment_len;
      }

      if (p_scb->current_codec->useRtpHeaderMarkerBit()) {
        m_pt |= AVDT_MARKER_SET;
      }
      if (!extra_fragments.empty()) {
        // Reset the RTP Marker bit for all fragments except the last one
        m_pt &= ~AVDT_MARKER_SET;
      }
      AVDT_WriteReqOpt(p_scb->avdt_handle, p_buf, timestamp, m_pt, opt);
      for (size_t i = 0; i < extra_fragments.size(); i++) {
        if (i + 1 == extra_fragments.size()) {
          // Set the RTP Marker bit for the last fragment
          m_pt |= AVDT_MARKER_SET;
        }
        BT_HDR* p_buf2 = extra_fragments[i];
        AVDT_WriteReqOpt(p_scb->avdt_handle, p_buf2, timestamp, m_pt, opt);
      }
      p_scb->cong = true;
    } else {
      /* there's a buffer, but L2CAP does not seem to be moving data */
      if (new_buf) {
        /* just got this buffer from co_data,
         * put it in queue */
        list_append(p_scb->a2dp_list, p_buf);
      } else {
        /* just dequeue it from the a2dp_list */
        if (list_length(p_scb->a2dp_list) < 3) {
          /* put it back to the queue */
          list_prepend(p_scb->a2dp_list, p_buf);
        } else {
          /* too many buffers in a2dp_list, drop it. */
          bta_avk_co_audio_drop(p_scb->hndl);
          osi_free(p_buf);
        }
      }
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_start_ok
 *
 * Description      Stream started.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_start_ok(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_START start;
  bool initiator = false;
  bool suspend = false;
  uint16_t flush_to;
  uint8_t new_role = p_scb->role;
  BT_HDR hdr;
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;
  uint8_t cur_role = BTM_ROLE_UNDEFINED;
  char *codec_name = (char *)A2DP_CodecName(p_scb->cfg.codec_info);

  APPL_TRACE_DEBUG("%s: wait:x%x, role:x%x", __func__, p_scb->wait,
                   p_scb->role);

  p_scb->started = true;
#if (TWS_ENABLED == TRUE)
  p_scb->start_pending = false;
#endif
  p_scb->current_codec = bta_avk_get_a2dp_current_codec();

  if (p_scb->sco_suspend) {
    p_scb->sco_suspend = false;
  }

  if (new_role & BTA_AVK_ROLE_START_INT) initiator = true;

  /* for A2DP SINK we do not send get_caps */
  if ((p_scb->avdt_handle == p_scb->seps[p_scb->sep_idx].av_handle) &&
      (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SNK)) {
    p_scb->wait &= ~(BTA_AVK_WAIT_ACP_CAPS_ON);
    APPL_TRACE_DEBUG("%s: local SEP type is SNK new wait is 0x%x", __func__,
                     p_scb->wait);
  }
  if (p_scb->wait & BTA_AVK_WAIT_ROLE_SW_FAILED) {
    /* role switch has failed */
    p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_FAILED;
    p_data = (tBTA_AVK_DATA*)&hdr;
    hdr.offset = BTA_AVK_RS_FAIL;
  }
  APPL_TRACE_DEBUG("%s: wait:x%x", __func__, p_scb->wait);

  if (p_data && (p_data->hdr.offset != BTA_AVK_RS_NONE)) {
    p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_BITS;
    if (p_data->hdr.offset == BTA_AVK_RS_FAIL) {
      bta_sys_idle(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
      start.chnl = p_scb->chnl;
      start.status = BTA_AVK_FAIL_ROLE;
      start.hndl = p_scb->hndl;
      start.initiator = initiator;
      tBTA_AVK bta_avk_data;
      bta_avk_data.start = start;
      (*bta_avk_cb.p_cback)(BTA_AVK_START_EVT, &bta_avk_data);
      return;
    }
  }

  if(strstr(codec_name, "aptX") != NULL) {
    APPL_TRACE_WARNING("%s: Changing packet type for aptX family codec stream", __func__);
    AVDT_UpdateLinkPktType(p_scb->avdt_handle, (btm_cb.btm_acl_pkt_types_supported |
                HCI_PKT_TYPES_MASK_NO_3_DH1 | HCI_PKT_TYPES_MASK_NO_3_DH3 | HCI_PKT_TYPES_MASK_NO_3_DH5));
  }

  if (!bta_avk_link_role_ok(p_scb, A2DP_SET_ONE_BIT))
    p_scb->q_tag = BTA_AVK_Q_TAG_START;
  else {
    /* The wait flag may be set here while we are already master on the link */
    /* this could happen if a role switch complete event occurred during
     * reconfig */
    /* if we are now master on the link, there is no need to wait for the role
     * switch, */
    /* complete anymore so we can clear the wait for role switch flag */
    p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_BITS;
  }

  if (p_scb->wait &
      (BTA_AVK_WAIT_ROLE_SW_RES_OPEN | BTA_AVK_WAIT_ROLE_SW_RES_START)) {
    p_scb->wait |= BTA_AVK_WAIT_ROLE_SW_STARTED;
    p_scb->q_tag = BTA_AVK_Q_TAG_START;
  }

  if (p_scb->wait) {
    APPL_TRACE_ERROR("%s: wait:x%x q_tag:%d- not started", __func__,
                     p_scb->wait, p_scb->q_tag);
    /* Clear first bit of p_scb->wait and not to return from this point else
     * HAL layer gets blocked. And if there is delay in Get Capability response
     * as
     * first bit of p_scb->wait is cleared hence it ensures bt_av_start_ok is
     * not called
     * again from bta_avk_save_caps.
     */
    p_scb->wait &= ~BTA_AVK_WAIT_ACP_CAPS_ON;
  }

  /* tell role manager to check M/S role */
  bta_sys_conn_open(BTA_ID_AVK, bta_avk_cb.audio_open_cnt, p_scb->peer_addr);

  bta_sys_busy(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);

  if (p_scb->media_type == AVDT_MEDIA_TYPE_AUDIO) {
    /* in normal logic, conns should be bta_avk_cb.audio_count - 1,
     * However, bta_avk_stream_chg is not called to increase
     * bta_avk_cb.audio_count yet.
     * If the code were to be re-arranged for some reasons, this number may need
     * to be changed
     */
    p_scb->co_started = bta_avk_cb.audio_open_cnt;
    flush_to = p_bta_avk_cfg->p_audio_flush_to[p_scb->co_started - 1];
  } else {
    flush_to = p_bta_avk_cfg->video_flush_to;
  }
  L2CA_SetFlushTimeout(p_scb->peer_addr, flush_to);

  /* clear the congestion flag */
  p_scb->cong = false;

  if (new_role & BTA_AVK_ROLE_START_INT) {
    new_role &= ~BTA_AVK_ROLE_START_INT;
  } else if ((new_role & BTA_AVK_ROLE_AD_ACP) &&
             (new_role & BTA_AVK_ROLE_SUSPEND_OPT)) {
    if ((bta_avk_is_multicast_enabled() == true
#if (TWS_ENABLED == TRUE)
         || p_scb->tws_device
#endif
         ) &&
        (BTM_GetRole (p_scb->peer_addr, &cur_role) == BTM_SUCCESS) &&
        (cur_role == BTM_ROLE_MASTER)
        ) {
      /* If playing on other stream, dont suspend this. */
      if (bta_avk_chk_start(p_scb)) {
        suspend = FALSE;
        APPL_TRACE_DEBUG("cur_role: %d suspend: %d", cur_role, suspend);
      }
    } else {
      suspend = TRUE;
      APPL_TRACE_DEBUG("cur_role: %d suspend: %d", cur_role, suspend);
    }
  }

  if (!suspend) {
    p_scb->q_tag = BTA_AVK_Q_TAG_STREAM;
    bta_avk_stream_chg(p_scb, true);
  }

  {
    /* If sink starts stream, disable sniff mode here */
    if (!initiator) {
      /* If souce is the master role, disable role switch during streaming.
       * Otherwise allow role switch, if source is slave.
       * Because it would not hurt source, if the peer device wants source to be
       * master */
      if ((BTM_GetRole(p_scb->peer_addr, &cur_role) == BTM_SUCCESS) &&
          (cur_role == BTM_ROLE_MASTER)) {
        policy |= HCI_ENABLE_MASTER_SLAVE_SWITCH;
      }

      bta_sys_clear_policy(BTA_ID_AVK, policy, p_scb->peer_addr);
    }

    p_scb->role = new_role;
    p_scb->role &= ~BTA_AVK_ROLE_AD_ACP;
    p_scb->role &= ~BTA_AVK_ROLE_SUSPEND_OPT;

    p_scb->no_rtp_hdr = false;
    p_scb->p_cos->start(p_scb->hndl, p_scb->cfg.codec_info, &p_scb->no_rtp_hdr);
    p_scb->co_started = true;

    APPL_TRACE_DEBUG("%s: suspending: %d, role:x%x, init %d", __func__, suspend,
                     p_scb->role, initiator);

    tBTA_AVK_START start;
    start.suspending = suspend;
    start.initiator = initiator;
    start.chnl = p_scb->chnl;
    start.status = BTA_AVK_SUCCESS;
    start.hndl = p_scb->hndl;
    /* update Master/Slave Role for start event */
    if (BTM_GetRole (p_scb->peer_addr, &cur_role) == BTM_SUCCESS)
      start.role = cur_role;
    (*bta_avk_cb.p_cback)(BTA_AVK_START_EVT, (tBTA_AVK*)&start);

    if (suspend) {
      tBTA_AVK_API_STOP stop;
      p_scb->role |= BTA_AVK_ROLE_SUSPEND;
      p_scb->cong = true; /* do not allow the media data to go through */
      /* do not duplicate the media packets to this channel */
      p_scb->p_cos->stop(p_scb->hndl);
      p_scb->co_started = false;
      stop.flush = false;
      stop.suspend = true;
      stop.reconfig_stop = false;
      bta_avk_ssm_execute(p_scb, BTA_AVK_AP_STOP_EVT, (tBTA_AVK_DATA*)&stop);
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_start_failed
 *
 * Description      Stream start failed.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_start_failed(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  if (p_scb->started == false && p_scb->co_started == false) {
    bta_sys_idle(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
    notify_start_failed(p_scb);
  }

  bta_sys_set_policy(BTA_ID_AVK,
                     (HCI_ENABLE_SNIFF_MODE | HCI_ENABLE_MASTER_SLAVE_SWITCH),
                     p_scb->peer_addr);
  p_scb->sco_suspend = false;
}

/*******************************************************************************
 *
 * Function         bta_avk_str_closed
 *
 * Description      Stream closed.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_avdt_close_timer_timeout(void *data_in) {
  BTIF_TRACE_ERROR("%s",__func__);
  tBTA_AVK_SCB* p_scb = (tBTA_AVK_SCB*)data_in; 

  tBTA_AVK data;
  tBTA_AVK_EVT event;
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;

  BTIF_TRACE_DEBUG(
      "%s: peer_addr=%s open_status=%d chnl=%d hndl=%d co_started=%d", __func__,
      p_scb->peer_addr.ToString().c_str(), p_scb->open_status, p_scb->chnl,
      p_scb->hndl, p_scb->co_started);

  if ((bta_avk_cb.features & BTA_AVK_FEAT_MASTER) == 0 ||
      bta_avk_cb.audio_open_cnt == 1)
    policy |= HCI_ENABLE_MASTER_SLAVE_SWITCH;
  bta_sys_set_policy(BTA_ID_AVK, policy, p_scb->peer_addr);
  if (bta_avk_cb.audio_open_cnt <= 1) {
    /* last connection - restore the allow switch flag */
    L2CA_SetDesireRole(L2CAP_ROLE_ALLOW_SWITCH);
  }

  L2CA_SetMediaStreamChannel(p_scb->l2c_cid, false);

  if (p_scb->open_status != BTA_AVK_SUCCESS) {
    APPL_TRACE_WARNING("%s Fail !!!", __func__);
    /* must be failure when opening the stream */
    data.open.bd_addr = p_scb->peer_addr;
    data.open.status = p_scb->open_status;
    data.open.chnl = p_scb->chnl;
    data.open.hndl = p_scb->hndl;

    if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SRC)
      data.open.sep = AVDT_TSEP_SNK;
    else if (p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SNK)
      data.open.sep = AVDT_TSEP_SRC;

    event = BTA_AVK_OPEN_EVT;
    p_scb->open_status = BTA_AVK_SUCCESS;

    bta_sys_conn_close(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
    bta_sys_conn_close(BTA_ID_AVK, bta_avk_cb.audio_open_cnt, p_scb->peer_addr);
    bta_avk_cleanup(p_scb, NULL);
    (*bta_avk_cb.p_cback)(event, &data);
  } else {
    APPL_TRACE_WARNING("%s success !!!", __func__);
    /* do stop if we were started */
    if (p_scb->co_started) {
      bta_avk_str_stopped(p_scb, NULL);
    }

    {
      p_scb->p_cos->close(p_scb->hndl);
      data.close.chnl = p_scb->chnl;
      data.close.hndl = p_scb->hndl;
      event = BTA_AVK_CLOSE_EVT;

      bta_sys_conn_close(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
      bta_sys_conn_close(BTA_ID_AVK, bta_avk_cb.audio_open_cnt, p_scb->peer_addr);
      bta_avk_cleanup(p_scb, NULL);
      (*bta_avk_cb.p_cback)(event, &data);
    }
  }
}

void bta_avk_str_closed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  BTIF_TRACE_DEBUG(
      "%s: peer_addr=%s open_status=%d chnl=%d hndl=%d co_started=%d", __func__,
      p_scb->peer_addr.ToString().c_str(), p_scb->open_status, p_scb->chnl,
      p_scb->hndl, p_scb->co_started);
  /* in codec switch case, remote phone would do avdt_close first, and then a  */
  /* avdt setconfig, buffer the close event for 200ms for this case. this can  */
  /* avoid the cost in bta, btif, and l2cap layer, also avoid disconnect avrcp */
  /* connections.                                                              */
  if(bta_avk_cb.avdt_close_timer[p_scb->hdi]) {
    alarm_set_on_mloop(bta_avk_cb.avdt_close_timer[p_scb->hdi],
      BTA_TIMEOUT_AVDT_CLOSE_MS,
      bta_avk_avdt_close_timer_timeout, p_scb);
  } else {
    bta_avk_avdt_close_timer_timeout((void*) p_scb);
  }
}
/*******************************************************************************
 *
 * Function         bta_avk_clr_cong
 *
 * Description      Clear stream congestion flag.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_clr_cong(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  if (p_scb->co_started) p_scb->cong = false;
}

/*******************************************************************************
 *
 * Function         bta_avk_suspend_cfm
 *
 * Description      process the suspend response
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_suspend_cfm(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_SUSPEND suspend_rsp;
  uint8_t err_code = p_data->str_msg.msg.hdr.err_code;
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;
  p_scb->suspend_local_sent = FALSE;

  APPL_TRACE_DEBUG ("%s:audio_open_cnt = %d, err_code = %d, scb_started = %d",
                    __func__,bta_avk_cb.audio_open_cnt,err_code,p_scb->started);

  if (p_scb->started == false) {
    /* handle the condition where there is a collision of SUSPEND req from
     *either side
     ** Second SUSPEND req could be rejected. Do not treat this as a failure
     */
    APPL_TRACE_WARNING("%s: already suspended, ignore, err_code %d", __func__,
                       err_code);
    return;
  }

  suspend_rsp.status = BTA_AVK_SUCCESS;
  if (err_code && (err_code != AVDT_ERR_BAD_STATE)) {
    /* Disable suspend feature only with explicit rejection(not with timeout & connect error) */
    if ((err_code != AVDT_ERR_TIMEOUT) && (err_code != AVDT_ERR_CONNECT)) {
      p_scb->suspend_sup = false;
    }
    suspend_rsp.status = BTA_AVK_FAIL;

    APPL_TRACE_ERROR("%s: suspend failed, closing connection", __func__);

    /* SUSPEND failed. Close connection. */
    bta_avk_ssm_execute(p_scb, BTA_AVK_API_CLOSE_EVT, NULL);
  } else {
    /* only set started to false when suspend is successful */
    p_scb->started = false;
  }

  if (p_scb->role & BTA_AVK_ROLE_SUSPEND) {
    p_scb->role &= ~BTA_AVK_ROLE_SUSPEND;
    p_scb->cong = false;
  }

  bta_sys_idle(BTA_ID_AVK, p_scb->hdi, p_scb->peer_addr);
  if ((bta_avk_cb.features & BTA_AVK_FEAT_MASTER) == 0 ||
      bta_avk_cb.audio_open_cnt == 1)
    policy |= HCI_ENABLE_MASTER_SLAVE_SWITCH;
  bta_sys_set_policy(BTA_ID_AVK, policy, p_scb->peer_addr);

  if (BTM_IS_QTI_CONTROLLER() && p_scb->offload_supported) {
    bta_avk_vendor_offload_stop(p_scb);
    p_scb->offload_supported = false;
  }
  /* in case that we received suspend_ind, we may need to call co_stop here */
  if (p_scb->co_started) {
    /* TODO(eisenbach): RE-IMPLEMENT USING VSC OR HAL EXTENSION
    vendor_get_interface()->send_command(
        (vendor_opcode_t)BT_VND_OP_A2DP_OFFLOAD_STOP, (void*)&p_scb->l2c_cid);
    if (p_scb->offload_start_pending) {
      tBTA_AVK_STATUS status = BTA_AVK_FAIL_STREAM;
      tBTA_AVK bta_avk_data;
      bta_avk_data.status = status;
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, &bta_avk_data);
    }
    p_scb->offload_start_pending = false;
    */
    /*if (BTM_IS_QTI_CONTROLLER() && p_scb->offload_supported) {
      bta_avk_vendor_offload_stop(p_scb);
      p_scb->offload_supported = false;
    }*/
    bta_avk_stream_chg(p_scb, false);

    {
      p_scb->co_started = false;
      p_scb->p_cos->stop(p_scb->hndl);
    }
    L2CA_SetFlushTimeout(p_scb->peer_addr, L2CAP_DEFAULT_FLUSH_TO);
  }

  {
    suspend_rsp.chnl = p_scb->chnl;
    suspend_rsp.hndl = p_scb->hndl;
    suspend_rsp.initiator = p_data->str_msg.initiator;
    tBTA_AVK bta_avk_data;
    bta_avk_data.suspend = suspend_rsp;
    (*bta_avk_cb.p_cback)(BTA_AVK_SUSPEND_EVT, &bta_avk_data);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_rcfg_str_ok
 *
 * Description      report reconfigure successful
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_rcfg_str_ok(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t avdt_handle = 0;
  p_scb->l2c_cid = AVDT_GetL2CapChannel(p_scb->avdt_handle);
  APPL_TRACE_DEBUG("%s: l2c_cid: %d", __func__, p_scb->l2c_cid);

  if (p_data != NULL)
    avdt_handle = p_data->str_msg.handle;

  uint8_t peer_seid = AVDT_GetPeerSeid(avdt_handle);
  uint8_t rcfg_seid = p_scb->sep_info[p_scb->rcfg_idx].seid;
  APPL_TRACE_WARNING("%s: avdt_handle: %d, peer_seid: %d, rcfg_seid: %d, rcfg_idx: %d",
       __func__, avdt_handle, peer_seid, rcfg_seid, p_scb->rcfg_idx);
  if (peer_seid != 0 && rcfg_seid != 0 && peer_seid != rcfg_seid) {
    APPL_TRACE_WARNING("%s: rcfg_idx is changed, reconfig again", __func__);
    p_scb->state = BTA_AVK_RCFG_SST;
    AVDT_CloseReq(avdt_handle);
    return;
  }

  if (p_data != NULL) {
    // p_data could be NULL if the reconfig was triggered by the local device
    p_scb->stream_mtu =
        p_data->str_msg.msg.open_ind.peer_mtu - AVDT_MEDIA_HDR_SIZE;
    uint16_t mtu = bta_avk_chk_mtu(p_scb, p_scb->stream_mtu);
    APPL_TRACE_DEBUG("%s: l2c_cid: 0x%x stream_mtu: %d mtu: %d", __func__,
                     p_scb->l2c_cid, p_scb->stream_mtu, mtu);
    if (mtu == 0 || mtu > p_scb->stream_mtu) mtu = p_scb->stream_mtu;
    APPL_TRACE_DEBUG("%s: updated mtu: %d", __func__, mtu);
    p_scb->p_cos->update_mtu(p_scb->hndl, mtu);
  }

  /* rc listen */
  bta_avk_st_rc_timer(p_scb, NULL);
  osi_free_and_reset((void**)&p_scb->p_cap);

  /* No need to keep the role bits once reconfig is done. */
  p_scb->role &= ~BTA_AVK_ROLE_AD_ACP;
  p_scb->role &= ~BTA_AVK_ROLE_SUSPEND_OPT;
  p_scb->role &= ~BTA_AVK_ROLE_START_INT;

  {
    /* reconfigure success  */
    tBTA_AVK_RECONFIG reconfig;
    reconfig.status = BTA_AVK_SUCCESS;
    reconfig.chnl = p_scb->chnl;
    reconfig.hndl = p_scb->hndl;
    tBTA_AVK bta_avk_data;
    bta_avk_data.reconfig = reconfig;
    (*bta_avk_cb.p_cback)(BTA_AVK_RECONFIG_EVT, &bta_avk_data);
  }
  bta_avk_update_flow_spec(p_scb);
}

/*******************************************************************************
 *
 * Function         bta_avk_rcfg_failed
 *
 * Description      process reconfigure failed
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_rcfg_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  APPL_TRACE_ERROR("%s: num_recfg=%d conn_lcb=0x%x peer_addr=%s", __func__,
                   p_scb->num_recfg, bta_avk_cb.conn_lcb,
                   p_scb->peer_addr.ToString().c_str());

  if (p_scb->num_recfg > BTA_AVK_RECONFIG_RETRY) {
    bta_avk_cco_close(p_scb, p_data);
    /* report failure */
    tBTA_AVK_RECONFIG reconfig;
    reconfig.status = BTA_AVK_FAIL_STREAM;
    reconfig.chnl = p_scb->chnl;
    reconfig.hndl = p_scb->hndl;
    tBTA_AVK bta_avk_data;
    bta_avk_data.reconfig = reconfig;
    (*bta_avk_cb.p_cback)(BTA_AVK_RECONFIG_EVT, &bta_avk_data);
    /* go to closing state */
    bta_avk_ssm_execute(p_scb, BTA_AVK_API_CLOSE_EVT, NULL);
  } else {
    /* open failed. try again */
    p_scb->num_recfg++;
    if (bta_avk_cb.conn_lcb) {
      AVDT_DisconnectReq(p_scb->peer_addr, bta_avk_dt_cback[p_scb->hdi]);
    } else {
      bta_avk_connect_req(p_scb, NULL);
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_rcfg_connect
 *
 * Description      stream closed. reconnect the stream
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_rcfg_connect(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  p_scb->cong = false;
  p_scb->num_recfg++;
  APPL_TRACE_DEBUG("%s: num_recfg: %d", __func__, p_scb->num_recfg);
  if (p_scb->num_recfg > BTA_AVK_RECONFIG_RETRY) {
    /* let bta_avk_rcfg_failed report fail */
    bta_avk_rcfg_failed(p_scb, NULL);
  } else
    AVDT_ConnectReq(p_scb->peer_addr, p_scb->sec_mask,
                    bta_avk_dt_cback[p_scb->hdi]);
}

/*******************************************************************************
 *
 * Function         bta_avk_rcfg_discntd
 *
 * Description      AVDT disconnected. reconnect the stream
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_rcfg_discntd(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  APPL_TRACE_ERROR("%s: num_recfg=%d conn_lcb=0x%x peer_addr=%s", __func__,
                   p_scb->num_recfg, bta_avk_cb.conn_lcb,
                   p_scb->peer_addr.ToString().c_str());

  p_scb->num_recfg++;
  if (p_scb->num_recfg > BTA_AVK_RECONFIG_RETRY) {
    /* report failure */
    tBTA_AVK_RECONFIG reconfig;
    reconfig.status = BTA_AVK_FAIL_STREAM;
    reconfig.chnl = p_scb->chnl;
    reconfig.hndl = p_scb->hndl;
    tBTA_AVK bta_avk_data;
    bta_avk_data.reconfig = reconfig;
    (*bta_avk_cb.p_cback)(BTA_AVK_RECONFIG_EVT, &bta_avk_data);
    /* report close event & go to init state */
    bta_avk_ssm_execute(p_scb, BTA_AVK_STR_DISC_FAIL_EVT, NULL);
  } else
    AVDT_ConnectReq(p_scb->peer_addr, p_scb->sec_mask,
                    bta_avk_dt_cback[p_scb->hdi]);
}

/*******************************************************************************
 *
 * Function         bta_avk_suspend_cont
 *
 * Description      received the suspend response.
 *                  continue to reconfigure the stream
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_suspend_cont(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t err_code = p_data->str_msg.msg.hdr.err_code;

  p_scb->started = false;
  p_scb->cong = false;
  p_scb->suspend_local_sent = FALSE;
  if (err_code) {
    if (AVDT_ERR_CONNECT == err_code) {
      /* report failure */
      tBTA_AVK_RECONFIG reconfig;
      reconfig.status = BTA_AVK_FAIL;
      tBTA_AVK bta_avk_data;
      bta_avk_data.reconfig = reconfig;
      (*bta_avk_cb.p_cback)(BTA_AVK_RECONFIG_EVT, &bta_avk_data);
      APPL_TRACE_ERROR("%s: BTA_AVK_STR_DISC_FAIL_EVT: peer_addr=%s", __func__,
                       p_scb->peer_addr.ToString().c_str());
      bta_avk_ssm_execute(p_scb, BTA_AVK_STR_DISC_FAIL_EVT, NULL);
    } else {
      APPL_TRACE_ERROR("%s: suspend rejected, try close", __func__);
      /* Disable suspend feature only with explicit rejection(not with timeout)
       */
      if (err_code != AVDT_ERR_TIMEOUT) {
        p_scb->suspend_sup = false;
      }
      /* drop the buffers queued in L2CAP */
      L2CA_FlushChannel(p_scb->l2c_cid, L2CAP_FLUSH_CHANS_ALL);

      AVDT_CloseReq(p_scb->avdt_handle);
    }
  } else {
    APPL_TRACE_DEBUG("%s: calling AVDT_ReconfigReq", __func__);
    /* reconfig the stream */

    A2DP_DumpCodecInfo(p_scb->cfg.codec_info);
    AVDT_ReconfigReq(p_scb->avdt_handle, &p_scb->cfg);
    p_scb->cfg.psc_mask = p_scb->cur_psc_mask;
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_rcfg_cfm
 *
 * Description      if reconfigure is successful, report the event
 *                  otherwise, close the stream.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_rcfg_cfm(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  uint8_t err_code = p_data->str_msg.msg.hdr.err_code;

  APPL_TRACE_DEBUG("%s: err_code = %d", __func__, err_code);

  // Disable AVDTP RECONFIGURE for blacklisted devices
  bool disable_avdtp_reconfigure = false;
  {
    if (interop_match_addr_or_name(INTEROP_DISABLE_AVDTP_RECONFIGURE,
                           (const RawAddress*)&p_scb->peer_addr)) {
      VLOG(1) << __func__ << ": disable AVDTP RECONFIGURE: interop matched address "
                          << p_scb->peer_addr;
      disable_avdtp_reconfigure = true;
    }
  }

  if ((err_code != 0) || disable_avdtp_reconfigure) {
    APPL_TRACE_ERROR("%s: reconfig rejected, try close with error code = %d", __func__, err_code);
    /* Disable reconfiguration feature only with explicit rejection(not with
     * timeout) */
    if ((err_code != AVDT_ERR_TIMEOUT) || disable_avdtp_reconfigure) {
      p_scb->recfg_sup = false;
    }
    /* started flag is false when reconfigure command is sent */
    /* drop the buffers queued in L2CAP */
    L2CA_FlushChannel(p_scb->l2c_cid, L2CAP_FLUSH_CHANS_ALL);
    AVDT_CloseReq(p_scb->avdt_handle);
  } else {
    /* update the codec info after rcfg cfm */
    APPL_TRACE_DEBUG(
        "%s: updating from codec %s to codec %s", __func__,
        A2DP_CodecName(p_scb->cfg.codec_info),
        A2DP_CodecName(p_data->str_msg.msg.reconfig_cfm.p_cfg->codec_info));
    memcpy(p_scb->cfg.codec_info,
           p_data->str_msg.msg.reconfig_cfm.p_cfg->codec_info, AVDT_CODEC_SIZE);
    /* take the SSM back to OPEN state */
    bta_avk_ssm_execute(p_scb, BTA_AVK_STR_OPEN_OK_EVT, NULL);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_rcfg_open
 *
 * Description      AVDT is connected. open the stream with the new
 *                  configuration
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_rcfg_open(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  APPL_TRACE_DEBUG("%s: num_disc_snks = %d", __func__, p_scb->num_disc_snks);

  if (p_scb->num_disc_snks == 0) {
    /* Need to update call-out module so that it will be ready for discover */
    p_scb->p_cos->stop(p_scb->hndl);

    /* send avdtp discover request */
    AVDT_DiscoverReq(p_scb->peer_addr, p_scb->sep_info, BTA_AVK_NUM_SEPS,
                     bta_avk_dt_cback[p_scb->hdi]);
  } else {
    APPL_TRACE_DEBUG("%s: calling AVDT_OpenReq()", __func__);
    A2DP_DumpCodecInfo(p_scb->cfg.codec_info);

    /* we may choose to use a different SEP at reconfig.
     * adjust the sep_idx now */
    bta_avk_adjust_seps_idx(p_scb, bta_avk_get_scb_handle(p_scb, AVDT_TSEP_SRC));

    /* open the stream with the new config */
    p_scb->sep_info_idx = p_scb->rcfg_idx;
    AVDT_OpenReq(p_scb->avdt_handle, p_scb->peer_addr,
                 p_scb->sep_info[p_scb->sep_info_idx].seid, &p_scb->cfg);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_security_rej
 *
 * Description      Send an AVDTP security reject.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_security_rej(tBTA_AVK_SCB* p_scb, UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  AVDT_SecurityRsp(p_scb->avdt_handle, p_scb->avdt_label, AVDT_ERR_BAD_STATE,
                   NULL, 0);
}

/*******************************************************************************
 *
 * Function         bta_avk_chk_2nd_start
 *
 * Description      check if this is 2nd stream and if it needs to be started.
 *                  This function needs to be kept very similar to
 *                  bta_avk_chk_start
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_chk_2nd_start(tBTA_AVK_SCB* p_scb,
                          UNUSED_ATTR tBTA_AVK_DATA* p_data) {
  tBTA_AVK_SCB* p_scbi;
  int i;
  bool new_started = false;
#if (TWS_ENABLED == TRUE)
  RawAddress tws_pair_addr;
  bool is_tws_pair = false;
  if (p_scb->tws_device) {
    if (BTM_SecGetTwsPlusPeerDev(p_scb->peer_addr,
                             tws_pair_addr) == true) {
      APPL_TRACE_EVENT("%s: tws pair found", __func__);
      is_tws_pair = true;
    } else {
      APPL_TRACE_EVENT("%s: tws pair not found", __func__);
      is_tws_pair = true;
    }
  }
#endif
  if ((p_scb->chnl == BTA_AVK_CHNL_AUDIO) && (bta_avk_cb.audio_open_cnt >= 2) &&
      (bta_avk_is_multicast_enabled()
#if (TWS_ENABLED == TRUE)
     || p_scb->tws_device
#endif
     )) {
    /* more than one audio channel is connected */
    if (!(p_scb->role & BTA_AVK_ROLE_SUSPEND_OPT)) {
      /* this channel does not need to be reconfigured.
       * if there is other channel streaming, start the stream now */
      for (i = 0; i < BTA_AVK_NUM_STRS; i++) {
        p_scbi = bta_avk_cb.p_scb[i];
        if (p_scbi && p_scbi->chnl == BTA_AVK_CHNL_AUDIO && p_scbi->co_started &&
           (bta_avk_is_multicast_enabled()
#if (TWS_ENABLED == TRUE)
           || (p_scbi->tws_device &&
           is_tws_pair && p_scbi->peer_addr == tws_pair_addr)
#endif
           )) {
          if (!new_started) {
            /* start the new stream */
            new_started = true;
            bta_avk_ssm_execute(p_scb, BTA_AVK_AP_START_EVT, NULL);
          }
          /* may need to update the flush timeout of this already started stream
           */
          if (p_scbi->co_started != bta_avk_cb.audio_open_cnt) {
            p_scbi->co_started = bta_avk_cb.audio_open_cnt;
            L2CA_SetFlushTimeout(
                p_scbi->peer_addr,
                p_bta_avk_cfg->p_audio_flush_to[p_scbi->co_started - 1]);
          }
        }
      }
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_open_rc
 *
 * Description      Send a message to main SM to open RC channel.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_open_rc(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  APPL_TRACE_DEBUG("%s: use_rc: %d, wait: x%x role:x%x", __func__,
                   p_scb->use_rc, p_scb->wait, p_scb->role);
  if ((p_scb->wait & BTA_AVK_WAIT_ROLE_SW_BITS) &&
      (p_scb->q_tag == BTA_AVK_Q_TAG_START)) {
    /* waiting for role switch for some reason & the timer expires */
    if (!bta_avk_link_role_ok(p_scb, A2DP_SET_ONE_BIT)) {
      APPL_TRACE_ERROR(
          "%s: failed to start streaming for role management reasons!!",
          __func__);
      alarm_cancel(p_scb->avrc_ct_timer);

      tBTA_AVK_START start;
      start.chnl = p_scb->chnl;
      start.status = BTA_AVK_FAIL_ROLE;
      start.initiator = true;
      start.hndl = p_scb->hndl;
      p_scb->wait &= ~BTA_AVK_WAIT_ROLE_SW_BITS;
      bta_avk_cb.rs_idx = 0;
      tBTA_AVK bta_avk_data;
      bta_avk_data.start = start;
      (*bta_avk_cb.p_cback)(BTA_AVK_START_EVT, &bta_avk_data);
    } else {
      /* role switch is done. continue to start streaming */
      bta_avk_cb.rs_idx = 0;
      p_data->hdr.offset = BTA_AVK_RS_OK;
      bta_avk_start_ok(p_scb, p_data);
    }
    return;
  }

  if (p_scb->use_rc == true || (p_scb->role & BTA_AVK_ROLE_AD_ACP)) {
    if (bta_avk_cb.disc) {
      /* AVRC discover db is in use */
      if (p_scb->rc_handle == BTA_AVK_RC_HANDLE_NONE) {
        /* AVRC channel is not connected. delay a little bit */
        if ((p_scb->wait & BTA_AVK_WAIT_ROLE_SW_BITS) == 0) {
          bta_sys_start_timer(p_scb->avrc_ct_timer, BTA_AVK_RC_DISC_TIME_VAL,
                              BTA_AVK_AVRC_TIMER_EVT, p_scb->hndl);
        } else {
          p_scb->wait |= BTA_AVK_WAIT_CHECK_RC;
        }
      }
    } else {
      /* use main SM for AVRC SDP activities */
      bta_avk_rc_disc((uint8_t)(p_scb->hdi + 1));
    }
  } else {
    if (BTA_AVK_RC_HANDLE_NONE != p_scb->rc_handle) {
      /* the open API said that this handle does not want a RC connection.
       * disconnect it now */
      AVRC_Close(p_scb->rc_handle);
    }
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_open_at_inc
 *
 * Description      This function is called if API open is called by application
 *                  while state-machine is at incoming state.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_open_at_inc(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  if (!p_scb) {
    APPL_TRACE_WARNING("scb is NULL, bailing out!");
    return;
  }
  memcpy (&(p_scb->open_api), &(p_data->api_open), sizeof(tBTA_AVK_API_OPEN));
  if (p_scb->coll_mask & BTA_AVK_COLL_SETCONFIG_IND) {
    APPL_TRACE_WARNING(" SetConfig is already called, timer stopped");
    /* make mask 0, timer shld have already been closed in setconfig_ind */
    p_scb->coll_mask = 0;
    return;
  }

  if (p_scb->coll_mask & BTA_AVK_COLL_INC_TMR) {
    p_scb->coll_mask |= BTA_AVK_COLL_API_CALLED;

    /* API open will be handled at timeout if SNK did not start signalling. */
    /* API open will be ignored if SNK starts signalling.                   */
  } else {
    /* SNK did not start signalling, API was called N seconds timeout. */
    /* We need to switch to INIT state and start opening connection. */
    APPL_TRACE_ERROR("%s():  ReSetting collision mask", __func__);
    p_scb->coll_mask = 0;
    bta_avk_set_scb_sst_init(p_scb);

    tBTA_AVK_API_OPEN* p_buf =
        (tBTA_AVK_API_OPEN*)osi_malloc(sizeof(tBTA_AVK_API_OPEN));
    memcpy(p_buf, &(p_scb->open_api), sizeof(tBTA_AVK_API_OPEN));
    p_scb->skip_sdp = true;
    bta_sys_sendmsg(p_buf);
  }
}

static void offload_vendor_callback(tBTM_VSC_CMPL *param)
{
  unsigned char status = 0;
  unsigned char sub_opcode = 0;
  APPL_TRACE_DEBUG("offload_vendor_callback: param_len = %d subopcode = %d status = %d",
                     param->param_len, param->p_param_buf[1], param->p_param_buf[0]);
  if (param->param_len)
  {
    status = param->p_param_buf[0];
  }
  sub_opcode =  param->p_param_buf[1];
  if (status == 0)
  {
    //sub_opcode =  param->p_param_buf[1];
    switch(sub_opcode)
    {
      case VS_QHCI_SCRAMBLE_A2DP_MEDIA:
        {
          bta_avk_vendor_offload_select_codec(avk_offload_start.p_scb);
          break;
        }
      case VS_QHCI_A2DP_SELECTED_CODEC:
        {
          uint8_t param[10],index=0;
          APPL_TRACE_DEBUG("%s: VS_QHCI_A2DP_SELECTED_CODEC successful", __func__);
          param[index++] = VS_QHCI_A2DP_TRANSPORT_CONFIGURATION;
          param[index++] = 0;//slimbus
          param[index++] = avk_offload_start.codec_type;//Define offload struct and copy reusable parameters
          BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,index,
                                     param, offload_vendor_callback);
          break;
        }
      case VS_QHCI_A2DP_TRANSPORT_CONFIGURATION:
        {
          uint8_t param[20],index = 0;
          uint16_t streaming_hdl = avk_offload_start.l2c_rcid;
          uint16_t hci_hdl = avk_offload_start.acl_hdl;
          APPL_TRACE_DEBUG("%s: VS_QHCI_A2DP_TRANSPORT_CONFIGURATION successful", __func__);

          param[index++] = VS_QHCI_WRITE_A2DP_MEDIA_CHANNEL_CFG;
          param[index++] = 0;
          param[index++] = (uint8_t)(hci_hdl & 0x00FF);
          param[index++] = (uint8_t)(((hci_hdl & 0xFF00) >> 8) & 0x00FF);
          param[index++] = (uint8_t)(streaming_hdl & 0x00FF);
          param[index++] = (uint8_t)(((streaming_hdl & 0xFF00) >> 8) & 0x00FF);
          param[index++] = (uint8_t)(avk_offload_start.mtu & 0x00FF);
          param[index++] = (uint8_t)(((avk_offload_start.mtu & 0xFF00) >> 8) & 0x00FF);
          BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,index,
                                    param, offload_vendor_callback);
          break;
        }
      case VS_QHCI_WRITE_A2DP_MEDIA_CHANNEL_CFG:
        {
          uint8_t param[2];
          APPL_TRACE_DEBUG("%s: VS_QHCI_WRITE_A2DP_MEDIA_CHANNEL_CFG successful", __func__);
          APPL_TRACE_DEBUG("%s: Last cached VSC command: 0x0%x", __func__, last_sent_vsc_cmd);
          if (!btif_a2dp_src_vsc.vs_configs_exchanged &&
              btif_a2dp_src_vsc.tx_start_initiated)
            btif_a2dp_src_vsc.vs_configs_exchanged = TRUE;
          else {
            APPL_TRACE_ERROR("Dont send start, stream suspended update fail to Audio");
            status = 1;//FAIL
            (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
            break;
          }
#if (BTA_AVK_CO_CP_SCMS_T == TRUE)
         if (avk_offload_start.cp_active){
           param[0] = VS_QHCI_A2DP_WRITE_SCMS_T_CP;
           param[1] = avk_offload_start.cp_flag;
         }else{
            if (last_sent_vsc_cmd == VS_QHCI_START_A2DP_MEDIA) {
                   APPL_TRACE_DEBUG("%s: START VSC already exchanged.", __func__);
                   status = 0;
                   (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
                   return;
               }
               last_sent_vsc_cmd = VS_QHCI_START_A2DP_MEDIA;
               param[0] = VS_QHCI_START_A2DP_MEDIA;
               param[1] = 0;
             }

#else
          if (last_sent_vsc_cmd == VS_QHCI_START_A2DP_MEDIA) {
            APPL_TRACE_DEBUG("%s: START VSC already exchanged.", __func__);
            status = 0;
            (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
            return;
          }
          last_sent_vsc_cmd = VS_QHCI_START_A2DP_MEDIA;
          param[0] = VS_QHCI_START_A2DP_MEDIA;
          param[1] = 0;
#endif
          BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,2,
                                    param, offload_vendor_callback);
          break;
        }
      case VS_QHCI_A2DP_WRITE_SCMS_T_CP:
        {
          uint8_t param[2];
          APPL_TRACE_DEBUG("%s: VS_QHCI_A2DP_WRITE_SCMS_T_CP successful", __func__);
          APPL_TRACE_DEBUG("%s: Last cached VSC command: 0x0%x", __func__, last_sent_vsc_cmd);
          if (last_sent_vsc_cmd == VS_QHCI_START_A2DP_MEDIA) {
            APPL_TRACE_DEBUG("%s: START VSC already exchanged.", __func__);
            status = 0;
            (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
            return;
          }
          last_sent_vsc_cmd = VS_QHCI_START_A2DP_MEDIA;
          param[0] = VS_QHCI_START_A2DP_MEDIA;
          param[1] = 0;
          BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,2,
                                    param, offload_vendor_callback);
          break;
        }
      case VS_QHCI_START_A2DP_MEDIA:
          APPL_TRACE_DEBUG("%s: Multi VS_QHCI_START_A2DP_MEDIA successful", __func__);
          (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
          break;
      case VS_QHCI_STOP_A2DP_MEDIA:
          APPL_TRACE_DEBUG("%s: VS_QHCI_STOP_A2DP_MEDIA successful", __func__);
          (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_STOP_RSP_EVT, (tBTA_AVK*)&status);
          if (btif_a2dp_src_vsc.start_reset) {
            bta_avk_offload_req(avk_offload_start.p_scb, NULL);
            btif_a2dp_src_vsc.start_reset = false;
          }
          break;
      case VS_QHCI_A2DP_OFFLOAD_START:
          APPL_TRACE_DEBUG("%s: single VSC success: %d",__func__, param->p_param_buf[1]);
          avk_offload_start.p_scb->vendor_start = true;
          (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
          break;
      default:
      break;
    }
  } else if ((status == QHCI_INVALID_VSC && sub_opcode == VS_QHCI_A2DP_OFFLOAD_START)
      || (status == QHCI_INVALID_VSC && last_sent_vsc_cmd == VS_QHCI_A2DP_OFFLOAD_START)) {
    APPL_TRACE_DEBUG("%s: single VSC failed, sending multi VSC: %d",
                             __func__, param->p_param_buf[1]);
    btif_a2dp_src_vsc.multi_vsc_support = true;
    bta_avk_vendor_offload_start(avk_offload_start.p_scb);
    last_sent_vsc_cmd = 0;
  } else {
    APPL_TRACE_DEBUG("Offload failed for subopcode= %d",param->p_param_buf[1]);
    if (param->opcode != VS_QHCI_STOP_A2DP_MEDIA)
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
  }
}

/* static uint8_t bta_avk_vendor_offload_convert_sample_rate(uint16_t sample_rate) {
  uint8_t rate;
  switch (sample_rate) {
    case 44100:
      rate = 0x1 << 0;
      break;
    case 48000:
      rate = 0x1 << 1;
      break;
    default:
      APPL_TRACE_ERROR("Invalid sample rate, going with default 44.1");
      rate = 0x1 << 0;
      break;
  }
  return rate;
} */

static void bta_avk_vendor_offload_select_codec(tBTA_AVK_SCB* p_scb)
{
  const char *codec_name = A2DP_CodecName(p_scb->cfg.codec_info);
  uint8_t codec_type = 0, index = 0;
  uint8_t param[40];
  uint16_t sample_rate = 0;
  if (strcmp(codec_name,"SBC") == 0) codec_type = 0;
  else if (strcmp(codec_name,"AAC") == 0) codec_type = 2;
  else if (strcmp(codec_name,"aptX") == 0) codec_type = 8;
  else if (strcmp(codec_name,"aptX-HD") == 0) codec_type = 9;
  else if ((strcmp(codec_name,"LDAC")) == 0) codec_type = 4;
  param[index++] = VS_QHCI_A2DP_SELECTED_CODEC;
  param[index++] = codec_type;
  param[index++] = 0;//max latency
  param[index++] = 0;//delay reporting
#if (BTA_AVK_CO_CP_SCMS_T == TRUE)
  param[index++] = avk_offload_start.cp_active;
#else
  param[index++] = 0;
#endif
  param[index++] = avk_offload_start.cp_flag;
  sample_rate  = A2DP_GetTrackSampleRate(p_scb->cfg.codec_info);
  param[index++] = (uint8_t)(sample_rate & 0x00FF);
  param[index++] = (uint8_t)(((sample_rate & 0xFF00) >> 8) & 0x00FF);
  if (codec_type == A2DP_MEDIA_CT_SBC)
  {
    param[index++] = A2DP_GetNumberOfSubbandsSbc(p_scb->cfg.codec_info);
    param[index++] = A2DP_GetNumberOfBlocksSbc(p_scb->cfg.codec_info);
  }
  APPL_TRACE_DEBUG("bta_avk_vendor_offload_start: VS_QHCI_A2DP_SELECTED_CODEC");
  BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,index,
                               param, offload_vendor_callback);
}

void bta_avk_vendor_offload_start(tBTA_AVK_SCB* p_scb)
{
  uint16_t len = sizeof(tBT_AVK_VENDOR_A2DP_OFFLOAD);
  uint8_t param[len];// codec_type;//index = 0;
  const char *codec_name;
  codec_name = A2DP_CodecName(p_scb->cfg.codec_info);
  APPL_TRACE_DEBUG("bta_avk_vendor_offload_start param size %ld", sizeof(param));
  APPL_TRACE_DEBUG("%s: enc_update_in_progress = %d", __func__, enc_update_in_progress);
  APPL_TRACE_DEBUG("%s: Last cached VSC command: 0x0%x", __func__, last_sent_vsc_cmd);
  APPL_TRACE_IMP("bta_avk_vendor_offload_start: vsc flags:-"
    "vs_configs_exchanged:%u tx_started:%u tx_start_initiated:%u"
    "tx_enc_update_initiated:%u tx_stop_initiated: %u", btif_a2dp_src_vsc.vs_configs_exchanged,
    btif_a2dp_src_vsc.tx_started, btif_a2dp_src_vsc.tx_start_initiated, tx_enc_update_initiated,
    btif_a2dp_src_vsc.tx_stop_initiated);
  enc_update_in_progress = FALSE;
  if (btif_a2dp_src_vsc.multi_vsc_support) {
    unsigned char status = 0;
    uint16_t bitrate = 0;
    APPL_TRACE_IMP("bta_avk_vendor_offload_start: vsc flags:-"
      "vs_configs_exchanged:%u tx_started:%u tx_start_initiated:%u"
      "tx_enc_update_initiated:%u", btif_a2dp_src_vsc.vs_configs_exchanged, btif_a2dp_src_vsc.tx_started,
      btif_a2dp_src_vsc.tx_start_initiated, tx_enc_update_initiated);
    if (!bluetooth::headset::btif_hf_is_call_vr_idle()) {
      APPL_TRACE_IMP("ignore VS start request as Call is not idle");
      status = 2; //INCALL_FAILIRE
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
      return;
    } else if (!btif_a2dp_src_vsc.tx_started
        && (!btif_a2dp_src_vsc.tx_start_initiated || tx_enc_update_initiated)) {
      btif_a2dp_src_vsc.tx_start_initiated = TRUE;
      tx_enc_update_initiated = FALSE;
      if (last_sent_vsc_cmd == VS_QHCI_START_A2DP_MEDIA) {
        APPL_TRACE_DEBUG("%s: START VSC already exchanged.", __func__);
        status = 0;
        (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
        return;
      }
      if(btif_a2dp_src_vsc.vs_configs_exchanged) {
        param[0] = VS_QHCI_START_A2DP_MEDIA;
        param[1] = 0;
        BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,2, param,
            offload_vendor_callback);
        return;
      }
    } else {
      APPL_TRACE_IMP("ignore VS start request");
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
      return;
    }

    if(p_scb->do_scrambling) {
      uint8_t *p_param = param;
      *p_param++ = VS_QHCI_SCRAMBLE_A2DP_MEDIA;
      bitrate = A2DP_GetTrackBitRate(p_scb->cfg.codec_info);
      if (bitrate == 0) {
        UINT8_TO_STREAM(p_param, 1);
      } else {
        UINT8_TO_STREAM(p_param, 2);
      }
      UINT16_TO_STREAM(p_param,avk_offload_start.acl_hdl);
      BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE,4, param,
          offload_vendor_callback);
      avk_offload_start.p_scb = p_scb;
      return;
    } else {
      bta_avk_vendor_offload_select_codec(p_scb);
    }
  } else { //Single VSC
    unsigned char status = 0;
    if (last_sent_vsc_cmd == VS_QHCI_A2DP_OFFLOAD_START && !p_scb->tws_device) {
      APPL_TRACE_DEBUG("%s: START single VSC already exchanged", __func__);
      (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, (tBTA_AVK*)&status);
      return;
    }

    uint8_t *p_param = param;
    int param_len = 0;
    *p_param++ = VS_QHCI_A2DP_OFFLOAD_START;
    UINT8_TO_STREAM(p_param,avk_offload_start.codec_type);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.transport_type);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.stream_type);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.dev_index);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.max_latency);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.delay_reporting);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.cp_active);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.cp_flag);
    param_len++;
    UINT16_TO_STREAM(p_param,avk_offload_start.sample_rate);
    param_len += 2;
    UINT16_TO_STREAM(p_param,avk_offload_start.acl_hdl);
    param_len += 2;
    UINT16_TO_STREAM(p_param,avk_offload_start.l2c_rcid);
    param_len += 2;
    UINT16_TO_STREAM(p_param,avk_offload_start.mtu);
    param_len += 2;
    UINT8_TO_STREAM(p_param,avk_offload_start.stream_start);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.split_acl);
    param_len++;
    UINT8_TO_STREAM(p_param,avk_offload_start.ch_mode);
    param_len++;
    UINT16_TO_STREAM(p_param,avk_offload_start.ttp);
    param_len += 2;
    ARRAY_TO_STREAM(p_param,avk_offload_start.codec_info,
                                    AVDT_CODEC_SIZE);
    param_len += AVDT_CODEC_SIZE;
    BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE, param_len,
                                 param, offload_vendor_callback);
    last_sent_vsc_cmd = VS_QHCI_A2DP_OFFLOAD_START;
    avk_offload_start.p_scb = p_scb;
    if(strcmp(codec_name,"aptX-adaptive") == 0)
    {
        tBTA_AVK_DATA av_data;
        av_data.encoder_mode.enc_mode = (btif_avk_get_aptx_mode_info() & APTX_MODE_MASK);
        switch(av_data.encoder_mode.enc_mode) {
            case APTX_HQ:
            case APTX_LL:
              bta_avk_update_enc_mode(&av_data);
              break;
            case APTX_ULL:
              av_data.encoder_mode.enc_mode = APTX_LL;
              bta_avk_update_enc_mode(&av_data);
              BTA_AvkUpdateAptxData(APTX_ULL);
              break;
            default:
              av_data.encoder_mode.enc_mode = APTX_HQ;
              bta_avk_update_enc_mode(&av_data);
              APPL_TRACE_WARNING("Unknown aptX mode, send HQ as default");
        }
    }
  }
}
void bta_avk_vendor_offload_stop(tBTA_AVK_SCB* p_scb)
{
  uint8_t param[2];
  unsigned char status = 0;
  APPL_TRACE_DEBUG("bta_avk_vendor_offload_stop");

  if (p_scb == NULL) {
    APPL_TRACE_DEBUG("stop called from upper layer");
  }else if (p_scb->tws_device) {
    for (int xx = 0; xx < BTA_AVK_NUM_STRS; xx++) {
      if (bta_avk_cb.p_scb[xx] != NULL && bta_avk_cb.p_scb[xx] != p_scb &&
        bta_avk_cb.p_scb[xx]->started == true && bta_avk_cb.p_scb[xx]->tws_device) {
        APPL_TRACE_DEBUG("%s:Playing on other device ignore stop", __func__);
        return;
      }
    }
  }

  if (!btif_a2dp_src_vsc.multi_vsc_support) {
    APPL_TRACE_DEBUG("bta_avk_vendor_offload_stop: sending STOP");
    if (p_scb != NULL && !p_scb->vendor_start) {
      APPL_TRACE_WARNING("VSC Start is not sent for this device");
      return;
    }
    goto stop;
  } else {
    APPL_TRACE_DEBUG("bta_avk_vendor_offload_stop, btif_a2dp_src_vsc.tx_started: %u,"
        "btif_a2dp_src_vsc.tx_stop_initiated: %u",
        btif_a2dp_src_vsc.tx_started, btif_a2dp_src_vsc.tx_stop_initiated);
    if (btif_a2dp_src_vsc.tx_started && !btif_a2dp_src_vsc.tx_stop_initiated) {
      btif_a2dp_src_vsc.tx_stop_initiated = TRUE;
      goto stop;
/*      param[0] = VS_QHCI_STOP_A2DP_MEDIA;
      param[1] = 0;
      BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE, 2, param,
          offload_vendor_callback);
      p_scb->offload_supported = false;
*/
    } else if((btif_a2dp_src_vsc.tx_start_initiated || tx_enc_update_initiated)
        && !btif_a2dp_src_vsc.tx_started) {
      APPL_TRACE_IMP("Suspend Req when VSC exchange in progress,reset VSC");
      btif_media_send_reset_vendor_state();
      return;
    } else {
      APPL_TRACE_IMP("ignore VS stop request");
      return;
    }
  }
stop:
  btif_a2dp_src_vsc.tx_stop_initiated = TRUE;
  param[0] = VS_QHCI_STOP_A2DP_MEDIA;
  param[1] = 0;
  if (last_sent_vsc_cmd == VS_QHCI_STOP_A2DP_MEDIA) {
    APPL_TRACE_DEBUG("%s: STOP VSC already exchanged.", __func__);
    status = 0;
    (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_STOP_RSP_EVT, (tBTA_AVK*)&status);
    return;
  }
  last_sent_vsc_cmd = VS_QHCI_STOP_A2DP_MEDIA;
  BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE, 2, param,
      offload_vendor_callback);
  if (p_scb != NULL) {
    p_scb->offload_supported = false;
    p_scb->vendor_start = false;
  }
  /*if (p_scb->tws_device) {
    for (int xx = 0; xx < BTA_AVK_NUM_STRS; xx++) {
      if (bta_avk_cb.p_scb[xx] != NULL && bta_avk_cb.p_scb[xx] != p_scb &&
        bta_avk_cb.p_scb[xx]->started == true) {
        APPL_TRACE_DEBUG("%s:Playing on other device ignore stop", __func__);
        return;
      }
    }
  }
  param[0] = VS_QHCI_STOP_A2DP_MEDIA;
  param[1] = 0;
  BTM_VendorSpecificCommand(HCI_VSQC_CONTROLLER_A2DP_OPCODE, 2,
                               param, offload_vendor_callback);
  */
  //p_scb->offload_supported = false;
}
void bta_avk_vendor_offload_check_stop_start(tBTA_AVK_SCB* p_scb) {
  APPL_TRACE_DEBUG("%s",__func__);
  int i = 0;
  tBTA_AVK_SCB *p_scbi;
  for (i = 0; i < BTA_AVK_NUM_STRS; i++) {
    p_scbi = bta_avk_cb.p_scb[i];
    if (p_scbi != NULL && p_scbi != p_scb && p_scbi->tws_device &&
        (p_scbi->offload_supported == true) && (p_scbi->offload_started ==  false)) {
      bta_avk_vendor_offload_stop(p_scbi);
      btif_a2dp_src_vsc.start_reset = true;
      avk_offload_start.p_scb = p_scbi;
      break;
    }
  }
}
/*******************************************************************************
 *
 * Function         bta_avk_offload_req
 *
 * Description      This function is called if application requests offload of
 *                  a2dp audio.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_offload_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_STATUS status = BTA_AVK_FAIL_RESOURCES;
  bool do_scrambling = false;
  if (p_data != NULL)
    do_scrambling = p_data->api_offload_start.do_scrambling;
  else
    do_scrambling = p_scb->do_scrambling;

  APPL_TRACE_DEBUG("%s: stream %s, audio channels open %d", __func__,
                   p_scb->started ? "STARTED" : "STOPPED",
                   bta_avk_cb.audio_open_cnt);

  /* Check if stream has already been started. */
  /* Support offload if only one audio source stream is open. */
  if (p_scb->started != true) {
    status = BTA_AVK_FAIL_STREAM;
  }
/* QC Implementation */
  if (BTM_IS_QTI_CONTROLLER())
  {
    char *codec_name = (char *)A2DP_CodecName(p_scb->cfg.codec_info);
    uint8_t codec_type = 0;
    uint16_t mtu = bta_avk_chk_mtu(p_scb,p_scb->stream_mtu);
    p_scb->offload_supported = true;
    p_scb->do_scrambling = do_scrambling;

    if (mtu == 0 || mtu > p_scb->stream_mtu) mtu = p_scb->stream_mtu;

    if (btif_avk_is_peer_edr() && (btif_avk_peer_supports_3mbps() == FALSE)) {
      // This condition would be satisfied only if the remote device is
      // EDR and supports only 2 Mbps, but the effective AVDTP MTU size
      // exceeds the 2DH5 packet size.
      APPL_TRACE_DEBUG("%s The remote devce is EDR but does not support 3 Mbps", __func__);
      if (mtu > MAX_2MBPS_AVDTP_MTU) {
        APPL_TRACE_WARNING("%s Restricting AVDTP MTU size to %d", __func__, MAX_2MBPS_AVDTP_MTU);
        mtu = MAX_2MBPS_AVDTP_MTU;
      }
    }

    if ((strcmp(codec_name,"SBC")) == 0) codec_type = 0;
    else if ((strcmp(codec_name,"AAC")) == 0) codec_type = 2;
    else if ((strcmp(codec_name,"aptX")) == 0) codec_type = 8;
    else if ((strcmp(codec_name,"aptX-HD")) == 0) codec_type = 9;
    else if ((strcmp(codec_name,"aptX-adaptive")) == 0) {
      if (BTM_SecIsTwsPlusDev(p_scb->peer_addr)) {
        codec_type = 12;
      } else {
        codec_type = 10;
      }
    }
    else if ((strcmp(codec_name,"LDAC")) == 0) codec_type = 4;
    else if ((strcmp(codec_name,"aptX-TWS")) == 0) codec_type = 11;
    if ((codec_type == 8) || (codec_type == 9) || (codec_type == 4)) {
      if (mtu > MAX_2MBPS_AVDTP_MTU) {
        APPL_TRACE_IMP("Restricting AVDTP MTU size to 663 for APTx codecs");
        mtu = MAX_2MBPS_AVDTP_MTU;
      }
    }
    if (codec_type == 11) {
        APPL_TRACE_IMP("Restricting AVDTP MTU size to 360 for aptX-TWS codecs");
        mtu = AVDTP_2DH3_MTU;
    }
    if ((codec_type == 0) &&
        (A2DP_GetMaxBitpoolSbc(p_scb->cfg.codec_info) <= BTIF_A2DP_MAX_BITPOOL_MQ)) {
      APPL_TRACE_IMP("Restricting streaming MTU size for MQ Bitpool");
      mtu = MAX_2MBPS_AVDTP_MTU;
    }

    mtu = mtu + AVDT_MEDIA_HDR_SIZE;
    APPL_TRACE_DEBUG("%s Adding AVDTP media Header in stream_mtu : %d", __func__, mtu);

    if (mtu > BTA_AVK_MAX_A2DP_MTU)
        mtu = BTA_AVK_MAX_A2DP_MTU;

    avk_offload_start.codec_type = codec_type;
    avk_offload_start.transport_type = A2DP_TRANSPORT_TYPE_SLIMBUS;
    avk_offload_start.stream_type = codec_type;
    avk_offload_start.dev_index = 0;//for multicast use p_scb->hndl;
    avk_offload_start.delay_reporting = 0;
    avk_offload_start.max_latency = 0;
    avk_offload_start.mtu = mtu;
    avk_offload_start.acl_hdl = BTM_GetHCIConnHandle(p_scb->peer_addr,BT_TRANSPORT_BR_EDR);
#if (BTA_AVK_CO_CP_SCMS_T == TRUE)
    avk_offload_start.cp_active = p_scb->p_cos->cp_is_active();
#else
    avk_offload_start.cp_active = 0;
#endif
    avk_offload_start.cp_flag = p_scb->p_cos->cp_flag();

    switch (A2DP_GetTrackSampleRate(p_scb->cfg.codec_info)) {
      case SAMPLING_FREQ_44100:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_44100;
         break;
       case SAMPLING_FREQ_48000:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_48000;
         break;
       case SAMPLING_FREQ_88200:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_88200;
         break;
       case SAMPLING_FREQ_96000:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_96000;
         break;
       case SAMPLING_FREQ_176400:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_176400;
         break;
       case SAMPLING_FREQ_192000:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_192000;
         break;
       case SAMPLING_FREQ_16000:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_16000;
         break;
       case SAMPLING_FREQ_24000:
         avk_offload_start.sample_rate = A2DP_SAMPLING_FREQ_24000;
         break;
       default:
         APPL_TRACE_IMP("%s: Unknown sampling frequency ", __func__);
         break;
    }
    if (L2CA_GetIdentifiers(p_scb->l2c_cid,&avk_offload_start.l2c_rcid,NULL) == false)
    {
      APPL_TRACE_DEBUG("Failed to fetch l2c rcid");
      avk_offload_start.l2c_rcid = 0;
    }
    avk_offload_start.ch_mode = p_scb->channel_mode;
    avk_offload_start.stream_start = true;
    if (p_scb->tws_device)
      avk_offload_start.split_acl = true;
    else
      avk_offload_start.split_acl = false;
/*
    if (p_scb->do_scrambling) {
    //TODO 44.1k should be integrated to single VSC
      APPL_TRACE_DEBUG("%s:Scrambling enabled, enable multi VSC",__func__);
      avk_offload_start.split_acl = false;
      btif_a2dp_src_vsc.multi_vsc_support = true;
    }
*/
#if (TWS_ENABLED == TRUE)
    //We cannot rely on second earbud to set start flag to true.
    //If sencond earbud NACKs avdtp start then we end up in no audio on other device
    //which is in streaming state as well.
    //Skipping this logic to avoid dependency to start streaming.

    if (p_scb->tws_device &&
        btif_a2dp_src_vsc.start_reset == false) {
      APPL_TRACE_DEBUG("%s:TWS device setting split acl to true",__func__);
      avk_offload_start.split_acl = true;
      //Iterate through SCBs, check for tws device and set set stream_start accordingly.
      if(bta_avk_cb.audio_open_cnt > 1) {
        int i = 0;
        tBTA_AVK_SCB *p_scbi;
        for (i = 0; i < BTA_AVK_NUM_STRS; i++) {
          p_scbi = bta_avk_cb.p_scb[i];
          if (p_scbi != NULL && p_scbi != p_scb &&
            p_scbi->tws_device) {
            RawAddress tws_pair_addr;
            if (BTM_SecGetTwsPlusPeerDev(p_scbi->peer_addr,
                                         tws_pair_addr) == true) {
              APPL_TRACE_DEBUG("%s:TWS pair found",__func__);
              if (tws_pair_addr == p_scb->peer_addr &&
                !p_scbi->offload_supported && p_scbi->eb_state == TWSP_EB_STATE_IN_EAR) {
                APPL_TRACE_DEBUG("%s:VSC is not exchanged for second earbud",__func__);
                avk_offload_start.stream_start = false;
              }
            }
          }
        }
      }
    }
#endif
    if (avk_offload_start.stream_start) {
      p_scb->offload_started = true;
      if (p_scb->tws_device && bta_avk_cb.audio_open_cnt > 1) {
        tBTA_AVK_SCB *p_scbi;
        RawAddress tws_pair_addr;
        for (int i = 0; i < BTA_AVK_NUM_STRS; i++) {
          p_scbi = bta_avk_cb.p_scb[i];
          if (p_scbi != NULL && p_scbi != p_scb &&
            p_scbi->tws_device && p_scbi->offload_supported &&
            p_scbi->offload_started == false &&
            (BTM_SecGetTwsPlusPeerDev(p_scbi->peer_addr,
                                  tws_pair_addr) == true)) {
            if (tws_pair_addr == p_scb->peer_addr) {
              p_scbi->offload_started = true;//Set offload started for other TWS+ pair
            }
          }
        }
      }
    }
    else
      p_scb->offload_started = false;
    avk_offload_start.ttp = 150;
    memset(avk_offload_start.codec_info, 0 , AVDT_CODEC_SIZE);
    memcpy(avk_offload_start.codec_info, p_scb->cfg.codec_info,
              AVDT_CODEC_SIZE);
    bta_avk_vendor_offload_start(p_scb);
    return;
  }
  /* TODO(eisenbach): RE-IMPLEMENT USING VSC OR HAL EXTENSION
   uint16_t mtu = bta_avk_chk_mtu(p_scb, p_scb->stream_mtu);
   else if (bta_avk_cb.audio_open_cnt == 1 &&
              p_scb->seps[p_scb->sep_idx].tsep == AVDT_TSEP_SRC &&
              p_scb->chnl == BTA_AVK_CHNL_AUDIO) {
     bt_vendor_op_a2dp_offload_t a2dp_offload_start;

     if (L2CA_GetConnectionConfig(
             p_scb->l2c_cid, &a2dp_offload_start.acl_data_size,
             &a2dp_offload_start.remote_cid, &a2dp_offload_start.lm_handle)) {
       APPL_TRACE_DEBUG("%s: l2cmtu %d lcid 0x%02X rcid 0x%02X lm_handle
   0x%02X",
                        __func__, a2dp_offload_start.acl_data_size,
                        p_scb->l2c_cid, a2dp_offload_start.remote_cid,
                        a2dp_offload_start.lm_handle);

       a2dp_offload_start.bta_avk_handle = p_scb->hndl;
       a2dp_offload_start.xmit_quota = BTA_AVK_A2DP_OFFLOAD_XMIT_QUOTA;
       a2dp_offload_start.stream_mtu =
           (mtu < p_scb->stream_mtu) ? mtu : p_scb->stream_mtu;
       a2dp_offload_start.local_cid = p_scb->l2c_cid;
       a2dp_offload_start.is_flushable = true;
       a2dp_offload_start.stream_source =
           ((uint32_t)(p_scb->cfg.codec_info[1] | p_scb->cfg.codec_info[2]));

       memcpy(a2dp_offload_start.codec_info, p_scb->cfg.codec_info,
              sizeof(a2dp_offload_start.codec_info));

       if (!vendor_get_interface()->send_command(
               (vendor_opcode_t)BT_VND_OP_A2DP_OFFLOAD_START,
               &a2dp_offload_start)) {
         status = BTA_AVK_SUCCESS;
         p_scb->offload_start_pending = true;
       }
     }
   }
   */
  if (status != BTA_AVK_SUCCESS) {
    tBTA_AVK bta_avk_data;
    bta_avk_data.status = status;
    (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, &bta_avk_data);
  }
}

/*******************************************************************************
 *
 * Function         bta_avk_offload_rsp
 *
 * Description      This function is called when the vendor lib responds to
 *                  BT_VND_OP_A2DP_OFFLOAD_START.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_offload_rsp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  tBTA_AVK_STATUS status = p_data->api_status_rsp.status;

  APPL_TRACE_DEBUG("%s: stream %s status %s", __func__,
                   p_scb->started ? "STARTED" : "STOPPED",
                   status ? "FAIL" : "SUCCESS");

  /* Check if stream has already been started. */
  if (status == BTA_AVK_SUCCESS && p_scb->started != true) {
    status = BTA_AVK_FAIL_STREAM;
  }

  p_scb->offload_start_pending = false;
  tBTA_AVK bta_avk_data;
  bta_avk_data.status = status;
  (*bta_avk_cb.p_cback)(BTA_AVK_OFFLOAD_START_RSP_EVT, &bta_avk_data);
}

/*******************************************************************************
 *
 * Function         bta_avk_fake_suspend_rsp
 *
 * Description      This function is called when the 2sec timer expired, to fake
 *                  suspend response to btif
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_fake_suspend_rsp(const RawAddress &remote_bdaddr) {
  tBTA_AVK_SCB* p_scb = NULL;
  tBTA_AVK_SUSPEND suspend_rsp;
  p_scb = bta_avk_addr_to_scb(remote_bdaddr);
  if (p_scb == NULL) {
    APPL_TRACE_IMP("%s: p_scb is null, return", __func__);
    return;
  }
  APPL_TRACE_IMP("%s: add: %s hdi = %d", __func__,
                           remote_bdaddr.ToString().c_str(), p_scb->hdi);

  suspend_rsp.status = BTA_AVK_SUCCESS;
  suspend_rsp.chnl = p_scb->chnl;
  suspend_rsp.hndl = p_scb->hndl;
  suspend_rsp.initiator = true;
  tBTA_AVK bta_avk_data;
  bta_avk_data.suspend = suspend_rsp;
  (*bta_avk_cb.p_cback)(BTA_AVK_SUSPEND_EVT, &bta_avk_data);
}

/*******************************************************************************
 *
 * Function         bta_avk_disc_fail_as_acp
 *
 * Description      This function is called when for AVDTP_DISC, remote gives
 *                  nagative reply and do get_caps for the sep on which remote
 *                  does set_config.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_avk_disc_fail_as_acp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data) {
  if (p_scb->cache_setconfig) {
    APPL_TRACE_DEBUG("%s: Need to reuse the cached set_config, to do get_caps"
                      " for the same SEP", __func__);
    memcpy(p_data, p_scb->cache_setconfig, sizeof(tBTA_AVK_DATA));
    memset(p_scb->cache_setconfig, 0, sizeof(tBTA_AVK_DATA));
    osi_free(p_scb->cache_setconfig);
    p_scb->cache_setconfig = NULL;
  }

  uint8_t num = p_data->ci_setconfig.num_seid + 1;
  uint8_t avdt_handle = p_data->ci_setconfig.avdt_handle;
  uint8_t* p_seid = p_data->ci_setconfig.p_seid;
  uint8_t local_sep;
  APPL_TRACE_DEBUG("%s: num_seid: %d, p_seid: %d", __func__,
                                p_data->ci_setconfig.num_seid, *p_seid);

  /* we like this codec_type. find the sep_idx */
  local_sep = bta_avk_get_scb_sep_type(p_scb, avdt_handle);
  if (local_sep == AVDT_TSEP_SRC)
    p_scb->p_cos->disc_res(p_scb->hndl, num, num, 0, p_scb->peer_addr,
                           UUID_SERVCLASS_AUDIO_SOURCE);

  APPL_TRACE_DEBUG("%s: peer sep_id[%d]", __func__, *p_seid);
  /* initialize the sep_info[] to get capabilities */
  p_scb->sep_info[0].in_use = false;
  p_scb->sep_info[0].tsep = AVDT_TSEP_SNK;
  p_scb->sep_info[0].media_type = p_scb->media_type;
  p_scb->sep_info[0].seid = *p_seid;

  //We know here that we need to do get_caps for only
  //one SEP for which remote does set_config.
  //so, num_seps is '1' and sep_info_idx would be '0'
  p_scb->num_seps = 1;
  p_scb->sep_info_idx = 0;

  /* only in case of local sep as SRC we need to look for other SEPs, In case
   * of SINK we don't */
  if (local_sep == AVDT_TSEP_SRC) {
    /* Make sure UUID has been initialized... */
    if (p_scb->uuid_int == 0) p_scb->uuid_int = UUID_SERVCLASS_AUDIO_SOURCE;
      bta_avk_next_getcap(p_scb, p_data);
  }
}
