/******************************************************************************
 *  Copyright (C) 2017, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
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
 *  This is the private interface file for the BTA advanced audio/video.
 *
 ******************************************************************************/
#ifndef BTA_AVK_INT_H
#define BTA_AVK_INT_H

#include "avdt_api.h"
#include "bta_api.h"
#include "bta_avk_api.h"
#include "bta_avk_co.h"
#include "bta_sys.h"
#include "osi/include/list.h"
#include "stack/a2dp/a2dp_int.h"
#include "stack/include/a2dp_api.h"
/*****************************************************************************
 *  Constants
 ****************************************************************************/

enum {
  /* these events are handled by the AVK main state machine */
  BTA_AVK_API_DISABLE_EVT = BTA_SYS_EVT_START(BTA_ID_AVK),
  BTA_AVK_API_REMOTE_CMD_EVT,
  BTA_AVK_API_VENDOR_CMD_EVT,
  BTA_AVK_API_VENDOR_RSP_EVT,
  BTA_AVK_API_META_RSP_EVT,
  BTA_AVK_API_RC_CLOSE_EVT,
  BTA_AVK_AVRC_OPEN_EVT,
  BTA_AVK_AVRC_MSG_EVT,
  BTA_AVK_AVRC_NONE_EVT,

  /* these events are handled by the AVK stream state machine */
  BTA_AVK_API_OPEN_EVT,
  BTA_AVK_API_CLOSE_EVT,
  BTA_AVK_AP_START_EVT, /* the following 2 events must be in the same order as
                          the *API_*EVT */
  BTA_AVK_AP_STOP_EVT,
  BTA_AVK_API_RECONFIG_EVT,
  BTA_AVK_API_PROTECT_REQ_EVT,
  BTA_AVK_API_PROTECT_RSP_EVT,
  BTA_AVK_API_RC_OPEN_EVT,
  BTA_AVK_SRC_DATA_READY_EVT,
  BTA_AVK_CI_SETCONFIG_OK_EVT,
  BTA_AVK_CI_SETCONFIG_FAIL_EVT,
  BTA_AVK_SDP_DISC_OK_EVT,
  BTA_AVK_SDP_DISC_FAIL_EVT,
  BTA_AVK_STR_DISC_OK_EVT,
  BTA_AVK_STR_DISC_FAIL_EVT,
  BTA_AVK_STR_GETCAP_OK_EVT,
  BTA_AVK_STR_GETCAP_FAIL_EVT,
  BTA_AVK_STR_OPEN_OK_EVT,
  BTA_AVK_STR_OPEN_FAIL_EVT,
  BTA_AVK_STR_START_OK_EVT,
  BTA_AVK_STR_START_FAIL_EVT,
  BTA_AVK_STR_CLOSE_EVT,
  BTA_AVK_STR_CONFIG_IND_EVT,
  BTA_AVK_STR_SECURITY_IND_EVT,
  BTA_AVK_STR_SECURITY_CFM_EVT,
  BTA_AVK_STR_WRITE_CFM_EVT,
  BTA_AVK_STR_SUSPEND_CFM_EVT,
  BTA_AVK_STR_RECONFIG_CFM_EVT,
  BTA_AVK_AVRC_TIMER_EVT,
  BTA_AVK_AVDT_CONNECT_EVT,
  BTA_AVK_AVDT_DISCONNECT_EVT,
  BTA_AVK_ROLE_CHANGE_EVT,
  BTA_AVK_AVDT_DELAY_RPT_EVT,
  BTA_AVK_ACP_CONNECT_EVT,
  BTA_AVK_API_OFFLOAD_START_EVT,
  BTA_AVK_API_OFFLOAD_START_RSP_EVT,

  /* these events are handled outside of the state machine */
  BTA_AVK_API_ENABLE_EVT,
  BTA_AVK_API_REGISTER_EVT,
  BTA_AVK_API_DEREGISTER_EVT,
  BTA_AVK_API_DISCONNECT_EVT,
  BTA_AVK_CI_SRC_DATA_READY_EVT,
  BTA_AVK_SIG_CHG_EVT,
  BTA_AVK_SIGNALLING_TIMER_EVT,
  BTA_AVK_SDP_AVRC_DISC_EVT,
  BTA_AVK_AVRC_CLOSE_EVT,
  BTA_AVK_AVRC_BROWSE_OPEN_EVT,
  BTA_AVK_AVRC_BROWSE_CLOSE_EVT,
  BTA_AVK_AVRC_RETRY_DISC_EVT,
  BTA_AVK_CONN_CHG_EVT,
  BTA_AVK_DEREG_COMP_EVT,
  BTA_AVK_BROWSE_ACTIVE_EVT,
#if (AVDT_REPORTING == TRUE)
  BTA_AVK_AVDT_RPT_CONN_EVT,
#endif
  BTA_AVK_API_START_EVT, /* the following 2 events must be in the same order as
                           the *AP_*EVT */
  BTA_AVK_API_STOP_EVT,
  BTA_AVK_UPDATE_MAX_AV_CLIENTS_EVT,
  BTA_AVK_ENABLE_MULTICAST_EVT, /* Event for enable and disable multicast */
  BTA_AVK_RC_COLLISSION_DETECTED_EVT,
  BTA_AVK_UPDATE_ENCODER_MODE_EVT,
  BTA_AVK_UPDATE_APTX_DATA_EVT,
#if (TWS_ENABLED == TRUE)
#if (TWS_STATE_ENABLED == TRUE)
  BTA_AVK_SET_EARBUD_STATE_EVT, /* Set TWS earbud state */
#endif
  BTA_AVK_SET_EARBUD_ROLE_EVT, /* Set TWS earbud role */
  BTA_AVK_SET_TWS_DEVICE_EVT, /* Update TWS state */
#endif
};

/* MultiBrowse specific connection events */
enum {
  BTA_AVK_BROWSE_CONNECT,
  BTA_AVK_BROWSE_ACTIVE,
  BTA_AVK_BROWSE_HANDOFF,
};

/* events for AVK control block state machine */
#define BTA_AVK_FIRST_SM_EVT BTA_AVK_API_DISABLE_EVT
#define BTA_AVK_LAST_SM_EVT BTA_AVK_AVRC_NONE_EVT

/* events for AVK stream control block state machine */
#define BTA_AVK_FIRST_SSM_EVT BTA_AVK_API_OPEN_EVT

/* events that do not go through state machine */
#define BTA_AVK_FIRST_NSM_EVT BTA_AVK_API_ENABLE_EVT
#define BTA_AVK_LAST_NSM_EVT BTA_AVK_RC_COLLISSION_DETECTED_EVT

/* API events passed to both SSMs (by bta_avk_api_to_ssm) */
#define BTA_AVK_FIRST_A2S_API_EVT BTA_AVK_API_START_EVT
#define BTA_AVK_FIRST_A2S_SSM_EVT BTA_AVK_AP_START_EVT

#if (TWS_ENABLED == TRUE)
#define BTA_AVK_LAST_EVT BTA_AVK_SET_TWS_DEVICE_EVT
#else
#define BTA_AVK_LAST_EVT BTA_AVK_RC_COLLISSION_DETECTED_EVT
#endif

/* Info ID from updating aptX Adaptive Encoder mode */
#define BTA_AVK_ENCODER_MODE_CHANGE_ID 5

#define BTA_AVK_ENCODER_DATA_ID 0x0E

/* maximum number of SEPS in stream discovery results */
#define BTA_AVK_NUM_SEPS 32

/* initialization value for AVRC handle */
#define BTA_AVK_RC_HANDLE_NONE 0xFF

/* size of database for service discovery */
#define BTA_AVK_DISC_BUF_SIZE 2000

/* maximum length of AVDTP security data */
#define BTA_AVK_SECURITY_MAX_LEN 400

/* check number of buffers queued at L2CAP when this amount of buffers are
 * queued to L2CAP */
#define BTA_AVK_QUEUE_DATA_CHK_NUM L2CAP_HIGH_PRI_MIN_XMIT_QUOTA

/* the number of ACL links with AVDT */
#define BTA_AVK_NUM_LINKS AVDT_NUM_LINKS

#define BTA_AVK_BE_STREAM_TO_CO_ID(u32, p)                                 \
  {                                                                       \
    (u32) = (((uint32_t)(*((p) + 2))) + (((uint32_t)(*((p) + 1))) << 8) + \
             (((uint32_t)(*(p))) << 16));                                 \
    (p) += 3;                                                             \
  }

/* A2dp sampling frequencies */

#define SAMPLING_FREQ_44100 44100
#define SAMPLING_FREQ_48000 48000
#define SAMPLING_FREQ_88200 88200
#define SAMPLING_FREQ_96000 96000
#define SAMPLING_FREQ_176400 176400
#define SAMPLING_FREQ_192000 192000
#define SAMPLING_FREQ_16000 16000
#define SAMPLING_FREQ_24000 24000

/*Mapping of above sampling frequencies when sending VSC*/

#define A2DP_SAMPLING_FREQ_44100 0x01
#define A2DP_SAMPLING_FREQ_48000 0x02
#define A2DP_SAMPLING_FREQ_88200 0x04
#define A2DP_SAMPLING_FREQ_96000 0x08
#define A2DP_SAMPLING_FREQ_176400 0x10
#define A2DP_SAMPLING_FREQ_192000 0x20
#define A2DP_SAMPLING_FREQ_16000 0x40
#define A2DP_SAMPLING_FREQ_24000 0x80

/*****************************************************************************
 *  Data types
 ****************************************************************************/

/* function types for call-out functions */
typedef bool (*tBTA_AVK_CO_INIT)(btav_a2dp_codec_index_t codec_index,
                                tAVDT_CFG* p_cfg);
typedef void (*tBTA_AVK_CO_DISC_RES)(tBTA_AVK_HNDL hndl, uint8_t num_seps,
                                    uint8_t num_snk, uint8_t num_src,
                                    const RawAddress& addr,
                                    uint16_t uuid_local);
typedef tA2DP_STATUS (*tBTA_AVK_CO_GETCFG)(tBTA_AVK_HNDL hndl,
                                          uint8_t* p_codec_info,
                                          uint8_t* p_sep_info_idx, uint8_t seid,
                                          uint8_t* p_num_protect,
                                          uint8_t* p_protect_info);
typedef void (*tBTA_AVK_CO_SETCFG)(tBTA_AVK_HNDL hndl,
                                  const uint8_t* p_codec_info, uint8_t seid,
                                  const RawAddress& addr, uint8_t num_protect,
                                  const uint8_t* p_protect_info,
                                  uint8_t t_local_sep, uint8_t avdt_handle);
typedef void (*tBTA_AVK_CO_OPEN)(tBTA_AVK_HNDL hndl, uint16_t mtu);
typedef void (*tBTA_AVK_CO_CLOSE)(tBTA_AVK_HNDL hndl);
typedef void (*tBTA_AVK_CO_START)(tBTA_AVK_HNDL hndl, uint8_t* p_codec_info,
                                 bool* p_no_rtp_hdr);
typedef void (*tBTA_AVK_CO_STOP)(tBTA_AVK_HNDL hndl);
typedef void* (*tBTA_AVK_CO_DATAPATH)(const uint8_t* p_codec_info,
                                     uint32_t* p_timestamp);
typedef void (*tBTA_AVK_CO_DELAY)(tBTA_AVK_HNDL hndl, uint16_t delay);
typedef void (*tBTA_AVK_CO_UPDATE_MTU)(tBTA_AVK_HNDL hndl, uint16_t mtu);
/* SPLITA2DP */
typedef uint8_t (*tBTA_AVK_CO_CP_GET_FLAG)(void);
typedef bool (*tBTA_AVK_CO_CP_IS_ACTIVE)(void);
/* SPLITA2DP */
/* the call-out functions for one stream */
typedef struct {
  tBTA_AVK_CO_INIT init;
  tBTA_AVK_CO_DISC_RES disc_res;
  tBTA_AVK_CO_GETCFG getcfg;
  tBTA_AVK_CO_SETCFG setcfg;
  tBTA_AVK_CO_OPEN open;
  tBTA_AVK_CO_CLOSE close;
  tBTA_AVK_CO_START start;
  tBTA_AVK_CO_STOP stop;
  tBTA_AVK_CO_DATAPATH data;
  tBTA_AVK_CO_DELAY delay;
  tBTA_AVK_CO_UPDATE_MTU update_mtu;
/* SPLITA2DP */
  tBTA_AVK_CO_CP_GET_FLAG cp_flag;
  tBTA_AVK_CO_CP_IS_ACTIVE cp_is_active;
/* SPLITA2DP */
} tBTA_AVK_CO_FUNCTS;

/* data type for BTA_AVK_API_ENABLE_EVT */
typedef struct {
  BT_HDR hdr;
  tBTA_AVK_CBACK* p_cback;
  tBTA_AVK_FEAT features;
  tBTA_SEC sec_mask;
} tBTA_AVK_API_ENABLE;

/* data type for BTA_AVK_API_REGISTER_EVT */
typedef struct {
  BT_HDR hdr;
  char p_service_name[BTA_SERVICE_NAME_LEN + 1];
  uint8_t app_id;
  tBTA_AVK_SINK_DATA_CBACK* p_app_sink_data_cback;
  uint16_t service_uuid;
} tBTA_AVK_API_REG;

enum {
  BTA_AVK_RS_NONE, /* straight API call */
  BTA_AVK_RS_OK,   /* the role switch result - successful */
  BTA_AVK_RS_FAIL, /* the role switch result - failed */
  BTA_AVK_RS_DONE  /* the role switch is done - continue */
};
typedef uint8_t tBTA_AVK_RS_RES;
/* data type for BTA_AVK_API_OPEN_EVT */
typedef struct {
  BT_HDR hdr;
  RawAddress bd_addr;
  bool use_rc;
  tBTA_SEC sec_mask;
  tBTA_AVK_RS_RES switch_res;
  uint16_t uuid; /* uuid of initiator */
} tBTA_AVK_API_OPEN;

/* data type for BTA_AVK_API_STOP_EVT */
typedef struct {
  BT_HDR hdr;
  bool suspend;
  bool flush;
  bool reconfig_stop;  // True if the stream is stopped for reconfiguration
} tBTA_AVK_API_STOP;

/* data type for BTA_AVK_ENABLE_MULTICAST_EVT */
typedef struct
{
  BT_HDR hdr;
  bool is_multicast_enabled;
} tBTA_AVK_ENABLE_MULTICAST;

#if (TWS_ENABLED == TRUE)
#define TWSP_EB_STATE_UNKNOWN 0
#define TWSP_EB_STATE_IN_CASE 1
#define TWSP_EB_STATE_OUT_OF_EAR 2
#define TWSP_EB_STATE_IN_EAR 3
/* data type for BTA_AVK_TWS_SET_EARBUD_STATE_EVT */
#if (TWS_STATE_ENABLED == TRUE)
typedef struct
{
  BT_HDR hdr;
  uint8_t eb_state;
} tBTA_AVK_TWS_SET_EARBUD_STATE;
#endif
/* data type for BTA_AVK_TWS_SET_EARBUD_ROLE_EVT */
typedef struct
{
  BT_HDR hdr;
  uint8_t chn_mode;
} tBTA_AVK_TWS_SET_EARBUD_ROLE;
/* data type for BTA_AVK_ENABLE_TWS_DEVICE_EVT */
typedef struct
{
  BT_HDR hdr;
  bool is_tws_device;
} tBTA_AVK_SET_TWS_DEVICE;

#endif

/* data type for BTA_AVK_UPDATE_MAX_AV_CLIENTS_EVTT */
typedef struct
{
   BT_HDR hdr;
   uint8_t max_clients;
} tBTA_AVK_MAX_CLIENT;

/* data type for BTA_AVK_API_DISCONNECT_EVT */
typedef struct {
  BT_HDR hdr;
  RawAddress bd_addr;
} tBTA_AVK_API_DISCNT;

/* data type for BTA_AVK_API_PROTECT_REQ_EVT */
typedef struct {
  BT_HDR hdr;
  uint8_t* p_data;
  uint16_t len;
} tBTA_AVK_API_PROTECT_REQ;

/* data type for BTA_AVK_API_PROTECT_RSP_EVT */
typedef struct {
  BT_HDR hdr;
  uint8_t* p_data;
  uint16_t len;
  uint8_t error_code;
} tBTA_AVK_API_PROTECT_RSP;

/* data type for BTA_AVK_API_REMOTE_CMD_EVT */
typedef struct {
  BT_HDR hdr;
  tAVRC_MSG_PASS msg;
  uint8_t label;
} tBTA_AVK_API_REMOTE_CMD;

/* data type for BTA_AVK_API_VENDOR_CMD_EVT and RSP */
typedef struct {
  BT_HDR hdr;
  tAVRC_MSG_VENDOR msg;
  uint8_t label;
} tBTA_AVK_API_VENDOR;

/* data type for BTA_AVK_API_RC_OPEN_EVT */
typedef struct { BT_HDR hdr; } tBTA_AVK_API_OPEN_RC;

/* data type for BTA_AVK_API_RC_CLOSE_EVT */
typedef struct { BT_HDR hdr; } tBTA_AVK_API_CLOSE_RC;

/* data type for BTA_AVK_API_META_RSP_EVT */
typedef struct {
  BT_HDR hdr;
  bool is_rsp;
  uint8_t label;
  tBTA_AVK_CODE rsp_code;
  BT_HDR* p_pkt;
} tBTA_AVK_API_META_RSP;

/* data type for BTA_AVK_API_RECONFIG_EVT */
typedef struct {
  BT_HDR hdr;
  uint8_t codec_info[AVDT_CODEC_SIZE]; /* codec configuration */
  uint8_t* p_protect_info;
  uint8_t num_protect;
  bool suspend;
  uint8_t sep_info_idx;
} tBTA_AVK_API_RCFG;

/* data type for BTA_AVK_UPDATE_ENCODER_MODE_EVT */
typedef struct {
  BT_HDR hdr;
  uint16_t enc_mode;
} tBTA_AVK_ENC_MODE;

typedef struct {
  BT_HDR hdr;
  uint16_t type;
  uint16_t data;
} tBTA_AVK_APTX_DATA;

/* data type for BTA_AVK_CI_SETCONFIG_OK_EVT and BTA_AVK_CI_SETCONFIG_FAIL_EVT */
typedef struct {
  BT_HDR hdr;
  tBTA_AVK_HNDL hndl;
  uint8_t err_code;
  uint8_t category;
  uint8_t num_seid;
  uint8_t* p_seid;
  bool recfg_needed;
  uint8_t avdt_handle; /* local sep type for which this stream will be set up */
} tBTA_AVK_CI_SETCONFIG;

/* data type for all stream events from AVDTP */
typedef struct {
  BT_HDR hdr;
  tAVDT_CFG cfg;   /* configuration/capabilities parameters */
  tAVDT_CTRL msg;  /* AVDTP callback message parameters */
  RawAddress bd_addr; /* bd address */
  uint8_t handle;
  uint8_t avdt_event;
  bool initiator; /* true, if local device initiates the SUSPEND */
} tBTA_AVK_STR_MSG;

/* data type for BTA_AVK_AVRC_MSG_EVT */
typedef struct {
  BT_HDR hdr;
  tAVRC_MSG msg;
  uint8_t handle;
  uint8_t label;
  uint8_t opcode;
} tBTA_AVK_RC_MSG;

/* data type for BTA_AVK_AVRC_OPEN_EVT, BTA_AVK_AVRC_CLOSE_EVT */
typedef struct {
  BT_HDR hdr;
  RawAddress peer_addr;
  uint8_t handle;
} tBTA_AVK_RC_CONN_CHG;

/* data type for BTA_AVK_AVRC_COLL_DETECTED_EVT */
typedef struct {
  BT_HDR hdr;
  RawAddress peer_addr;
  uint8_t handle;
} tBTA_AVK_RC_COLLISSION_DETECTED;

/* data associated with BTA_AVK_BROWSE_ACTIVE_EVT */
typedef struct {
  BT_HDR hdr;
  uint8_t browse_device_evt;
  RawAddress peer_addr;
} tBTA_AVK_API_ACTIVE_BROWSE_RC;

/* data type for BTA_AVK_CONN_CHG_EVT */
typedef struct {
  BT_HDR hdr;
  RawAddress peer_addr;
  bool is_up;
} tBTA_AVK_CONN_CHG;

/* data type for BTA_AVK_ROLE_CHANGE_EVT */
typedef struct {
  BT_HDR hdr;
  uint8_t new_role;
  uint8_t hci_status;
} tBTA_AVK_ROLE_RES;

/* data type for BTA_AVK_SDP_DISC_OK_EVT */
typedef struct {
  BT_HDR hdr;
  uint16_t avdt_version; /* AVDTP protocol version */
} tBTA_AVK_SDP_RES;

/* data type for BTA_AVK_API_OFFLOAD_RSP_EVT */
typedef struct {
  BT_HDR hdr;
  tBTA_AVK_STATUS status;
} tBTA_AVK_API_STATUS_RSP;

/* data type for BTA_AVK_API_OFFLOAD_RSP_EVT */
typedef struct {
  BT_HDR hdr;
  bool do_scrambling;
} tBTA_AVK_API_OFFLOAD_START;

/* type for SEP control block */
typedef struct {
  uint8_t av_handle;                   /* AVDTP handle */
  uint8_t tsep;                        /* SEP type of local SEP */
  uint8_t codec_info[AVDT_CODEC_SIZE]; /* Codec info */
  tBTA_AVK_SINK_DATA_CBACK*
      p_app_sink_data_cback; /* Sink application callback for media packets */
} tBTA_AVK_SEP;

/* initiator/acceptor role for adaption */
#define BTA_AVK_ROLE_AD_INT 0x00 /* initiator */
#define BTA_AVK_ROLE_AD_ACP 0x01 /* acceptor */

/* initiator/acceptor signaling roles */
#define BTA_AVK_ROLE_START_ACP 0x00
#define BTA_AVK_ROLE_START_INT 0x10 /* do not change this value */

#define BTA_AVK_ROLE_SUSPEND 0x20     /* suspending on start */
#define BTA_AVK_ROLE_SUSPEND_OPT 0x40 /* Suspend on Start option is set */

/* union of all event datatypes */
union tBTA_AVK_DATA {
  BT_HDR hdr;
  tBTA_AVK_API_ENABLE api_enable;
  tBTA_AVK_API_REG api_reg;
  tBTA_AVK_API_OPEN api_open;
  tBTA_AVK_API_STOP api_stop;
  tBTA_AVK_API_DISCNT api_discnt;
  tBTA_AVK_API_PROTECT_REQ api_protect_req;
  tBTA_AVK_API_PROTECT_RSP api_protect_rsp;
  tBTA_AVK_API_REMOTE_CMD api_remote_cmd;
  tBTA_AVK_API_VENDOR api_vendor;
  tBTA_AVK_API_RCFG api_reconfig;
  tBTA_AVK_CI_SETCONFIG ci_setconfig;
  tBTA_AVK_STR_MSG str_msg;
  tBTA_AVK_RC_MSG rc_msg;
  tBTA_AVK_RC_CONN_CHG rc_conn_chg;
  tBTA_AVK_CONN_CHG conn_chg;
  tBTA_AVK_ROLE_RES role_res;
  tBTA_AVK_SDP_RES sdp_res;
  tBTA_AVK_API_META_RSP api_meta_rsp;
  tBTA_AVK_API_OFFLOAD_START api_offload_start;
  tBTA_AVK_API_STATUS_RSP api_status_rsp;
  tBTA_AVK_ENABLE_MULTICAST multicast_state;
  tBTA_AVK_MAX_CLIENT max_av_clients;
  tBTA_AVK_ENC_MODE encoder_mode;
  tBTA_AVK_APTX_DATA aptx_data;
#if (TWS_ENABLED == TRUE)
#if (TWS_STATE_ENABLED == TRUE)
  tBTA_AVK_TWS_SET_EARBUD_STATE tws_set_earbud_state;
#endif
  tBTA_AVK_TWS_SET_EARBUD_ROLE tws_set_earbud_role;
  tBTA_AVK_SET_TWS_DEVICE tws_set_device;
#endif
};

typedef union {
  tBTA_AVK_API_OPEN open; /* used only before open and role switch
                            is needed on another AVK channel */
} tBTA_AVK_Q_INFO;

#define BTA_AVK_Q_TAG_OPEN 0x01   /* after API_OPEN, before STR_OPENED */
#define BTA_AVK_Q_TAG_START 0x02  /* before start sending media packets */
#define BTA_AVK_Q_TAG_STREAM 0x03 /* during streaming */

#define BTA_AVK_WAIT_ACP_CAPS_ON 0x01 /* retriving the peer capabilities */
#define BTA_AVK_WAIT_ACP_CAPS_STARTED \
  0x02 /* started while retriving peer capabilities */
#define BTA_AVK_WAIT_ROLE_SW_RES_OPEN \
  0x04 /* waiting for role switch result after API_OPEN, before STR_OPENED */
#define BTA_AVK_WAIT_ROLE_SW_RES_START \
  0x08 /* waiting for role switch result before streaming */
#define BTA_AVK_WAIT_ROLE_SW_STARTED \
  0x10 /* started while waiting for role switch result */
#define BTA_AVK_WAIT_ROLE_SW_RETRY 0x20 /* set when retry on timeout */
#define BTA_AVK_WAIT_CHECK_RC \
  0x40 /* set when the timer is used by role switch */
#define BTA_AVK_WAIT_ROLE_SW_FAILED 0x80 /* role switch failed */

#define BTA_AVK_WAIT_ROLE_SW_BITS                                  \
  (BTA_AVK_WAIT_ROLE_SW_RES_OPEN | BTA_AVK_WAIT_ROLE_SW_RES_START | \
   BTA_AVK_WAIT_ROLE_SW_STARTED | BTA_AVK_WAIT_ROLE_SW_RETRY)

/* Bitmap for collision, coll_mask */
#define BTA_AVK_COLL_INC_TMR \
  0x01 /* Timer is running for incoming L2C connection */
#define BTA_AVK_COLL_API_CALLED \
  0x02 /* API open was called while incoming timer is running */
#define BTA_AVK_COLL_SETCONFIG_IND \
  0x04 /* SetConfig indication has been called by remote */

/* type for AVK stream control block */
struct tBTA_AVK_SCB {
  const tBTA_AVK_ACT* p_act_tbl; /* the action table for stream state machine */
  const tBTA_AVK_CO_FUNCTS* p_cos; /* the associated callout functions */
  bool sdp_discovery_started; /* variable to determine whether SDP is started */
  tBTA_AVK_SEP seps[BTAV_A2DP_CODEC_INDEX_MAX];
  tAVDT_CFG* p_cap;  /* buffer used for get capabilities */
  list_t* a2dp_list; /* used for audio channels only */
  tBTA_AVK_Q_INFO q_info;
  tAVDT_SEP_INFO sep_info[BTA_AVK_NUM_SEPS]; /* stream discovery results */
  tAVDT_CFG cfg;                            /* local SEP configuration */
  alarm_t* avrc_ct_timer;                   /* delay timer for AVRC CT */
  RawAddress peer_addr;                     /* peer BD address */
  uint16_t l2c_cid;                         /* L2CAP channel ID */
  uint16_t stream_mtu;                      /* MTU of stream */
  uint16_t avdt_version;      /* the avdt version of peer device */
  tBTA_SEC sec_mask;          /* security mask */
  uint8_t media_type;         /* Media type: AVDT_MEDIA_TYPE_* */
  bool cong;                  /* true if AVDTP congested */
  tBTA_AVK_STATUS open_status; /* open failure status */
  tBTA_AVK_CHNL chnl;          /* the channel: audio/video */
  tBTA_AVK_HNDL hndl;          /* the handle: ((hdi + 1)|chnl) */
  uint16_t cur_psc_mask;      /* Protocol service capabilities mask for current
                                 connection */
  uint8_t avdt_handle;        /* AVDTP handle */
  uint8_t hdi;                /* the index to SCB[] */
  uint8_t num_seps;           /* number of seps returned by stream discovery */
  uint8_t num_disc_snks;      /* number of discovered snks */
  uint8_t num_disc_srcs;      /* number of discovered srcs */
  uint8_t sep_info_idx;       /* current index into sep_info */
  uint8_t sep_idx;            /* current index into local seps[] */
  uint8_t rcfg_idx;           /* reconfig requested index into sep_info */
  uint8_t state;              /* state machine state */
  uint8_t avdt_label;         /* AVDTP label */
  uint8_t app_id;             /* application id */
  uint8_t num_recfg;          /* number of reconfigure sent */
  uint8_t role;
  uint8_t l2c_bufs;  /* the number of buffers queued to L2CAP */
  uint8_t rc_handle; /* connected AVRCP handle */
  bool use_rc;       /* true if AVRCP is allowed */
  bool started;      /* true if stream started */
  A2dpCodecConfig* current_codec; /* The current A2DP codec */
  uint8_t
      co_started;    /* non-zero, if stream started from call-out perspective */
  bool recfg_sup;    /* true if the first attempt to reconfigure the stream was
                        successfull, else False if command fails */
  bool suspend_sup;  /* true if Suspend stream is supported, else false if
                        suspend command fails */
  bool suspend_local_sent;    /* TRUE if outgoing Suspend is sent, else FALSE if
                        confirmation is received */
  bool deregistring; /* true if deregistering */
  bool sco_suspend;  /* true if SUSPEND is issued automatically for SCO */
  uint8_t coll_mask; /* Mask to check incoming and outgoing collision */
  tBTA_AVK_API_OPEN open_api; /* Saved OPEN api message */
  uint8_t wait;  /* set 0x1, when getting Caps as ACP, set 0x2, when started */
  uint8_t q_tag; /* identify the associated q_info union member */
  bool no_rtp_hdr;   /* true if add no RTP header*/
  uint16_t uuid_int; /*intended UUID of Initiator to connect to */
  bool offload_start_pending;
  bool skip_sdp; /* Decides if sdp to be done prior to profile connection */
  bool offload_supported;
  bool do_scrambling;
//#ifdef TWS_ENABLED
  bool tws_device; //true for earbud false otherwise
  uint8_t eb_state; //TWS+ EB state, IN-EAR=3, OUT-OF-EAR=2
  uint8_t channel_mode; //L:0 R:1 S:2 M:3
  uint8_t start_pending;
  bool offload_started;
//#endif
  bool vendor_start;
  tBTA_AVK_DATA *cache_setconfig;
  int rc_ccb_alloc_handle;
};

#define BTA_AVK_RC_ROLE_MASK 0x10
#define BTA_AVK_RC_ROLE_INT 0x00
#define BTA_AVK_RC_ROLE_ACP 0x10

#define BTA_AVK_RC_CONN_MASK 0x20

/* type for AVK RCP control block */
/* index to this control block is the rc handle */
typedef struct {
  uint8_t status;
  uint8_t handle;
  uint8_t shdl;               /* stream handle (hdi + 1) */
  uint8_t lidx;               /* (index+1) to LCB */
  tBTA_AVK_FEAT peer_features; /* peer features mask */
  uint16_t  cover_art_psm;  /* l2cap psm for cover art on remote */
  bool is_browse_active;    /* active for browse connetion */
  bool browse_open;
  bool rc_opened;
  RawAddress peer_addr;
} tBTA_AVK_RCB;
#define BTA_AVK_NUM_RCB (BTA_AVK_NUM_STRS + 2)

enum { BTA_AVK_LCB_FREE, BTA_AVK_LCB_FIND };

/* type for AVK ACL Link control block */
typedef struct {
  RawAddress addr;  /* peer BD address */
  uint8_t conn_msk; /* handle mask of connected stream handle */
  uint8_t lidx;     /* index + 1 */
} tBTA_AVK_LCB;

/* type for stream state machine action functions */
typedef void (*tBTA_AVK_SACT)(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);

/* type for AVK control block */
typedef struct {
  tBTA_AVK_SCB* p_scb[BTA_AVK_NUM_STRS];   /* stream control block */
  tSDP_DISCOVERY_DB* p_disc_db;          /* pointer to discovery database */
  tBTA_AVK_CBACK* p_cback;                /* application callback function */
  tBTA_AVK_RCB rcb[BTA_AVK_NUM_RCB];       /* RCB control block */
  tBTA_AVK_LCB lcb[BTA_AVK_NUM_LINKS + 1]; /* link control block */
  alarm_t* link_signalling_timer;
  alarm_t* browsing_channel_open_timer;  /* timer to initiate avrcp browsing open channel*/
  alarm_t*
      accept_signalling_timer[BTA_AVK_NUM_STRS];  /* timer to monitor signalling when accepting */
  uint32_t sdp_a2dp_handle;     /* SDP record handle for audio src */
  uint32_t sdp_a2dp_snk_handle; /* SDP record handle for audio snk */
  uint32_t sdp_vdp_handle;      /* SDP record handle for video src */
  tBTA_AVK_FEAT features;        /* features mask */
  tBTA_SEC sec_mask;            /* security mask */
  tBTA_AVK_HNDL handle;          /* the handle for SDP activity */
  bool disabling;               /* true if api disabled called */
  uint8_t
      disc; /* (hdi+1) or (rc_handle|BTA_AVK_CHNL_MSK) if p_disc_db is in use */
  uint8_t state;          /* state machine state */
  uint8_t conn_rc;        /* handle mask of connected RCP channels */
  uint8_t conn_audio;     /* handle mask of connected audio channels */
  uint8_t conn_video;     /* handle mask of connected video channels */
  uint8_t conn_lcb;       /* index mask of used LCBs */
  uint8_t audio_open_cnt; /* number of connected audio channels */
  uint8_t reg_audio;      /* handle mask of registered audio channels */
  uint8_t reg_video;      /* handle mask of registered video channels */
  uint8_t rc_acp_handle;
  uint8_t rc_acp_idx; /* (index + 1) to RCB */
  uint8_t rs_idx;    /* (index + 1) to SCB for the one waiting for RS on open */
  bool sco_occupied; /* true if SCO is being used or call is in progress */
  uint8_t audio_streams; /* handle mask of streaming audio channels */
  uint8_t video_streams; /* handle mask of streaming video channels */
} tBTA_AVK_CB;

/* SPLITA2DP */
typedef struct {
  uint8_t codec_type;
  uint8_t transport_type;
  uint8_t stream_type;
  uint8_t dev_index;
  uint8_t max_latency;
  uint8_t delay_reporting;
  uint8_t cp_active;
  uint8_t cp_flag;
  uint16_t sample_rate;
  uint16_t acl_hdl;
  uint16_t l2c_rcid;
  uint16_t mtu;
//#ifdef TWS_ENABLED
  uint8_t stream_start;// Start Stream:1
  uint8_t split_acl; // BR-EDR Device:0 TWS Device:1
  uint8_t ch_mode; //None:0 Left:1 Right:2
  uint16_t ttp; //time to play
//#endif
  uint8_t codec_info[AVDT_CODEC_SIZE];
  tBTA_AVK_SCB* p_scb;
}tBT_AVK_VENDOR_A2DP_OFFLOAD;

// A2DP offload VSC parameters
class tBT_AVK_A2DP_OFFLOAD {
 public:
  uint32_t codec_type;            /* codec types ex: SBC/AAC/LDAC/APTx */
  uint16_t max_latency;           /* maximum latency */
  uint16_t scms_t_enable;         /* content protection enable */
  uint32_t sample_rate;           /* Sample rates ex: 44.1/48/88.2/96 Khz */
  uint8_t bits_per_sample;        /* bits per sample ex: 16/24/32 */
  uint8_t ch_mode;                /* None:0 Left:1 Right:2 */
  uint32_t encoded_audio_bitrate; /* encoder audio bitrates */
  uint16_t acl_hdl;               /* connection handle */
  uint16_t l2c_rcid;              /* l2cap channel id */
  uint16_t mtu;                   /* MTU size */
  uint8_t codec_info[32];         /* Codec specific information */
};

extern tBT_AVK_VENDOR_A2DP_OFFLOAD avk_offload_start;

/* Vendor OFFLOAD VSC */
#define HCI_VSQC_CONTROLLER_A2DP_OPCODE 0x000A

#define VS_QHCI_READ_A2DP_CFG                 0x01
#define VS_QHCI_WRITE_SBC_CFG                 0x02
#define VS_QHCI_WRITE_A2DP_MEDIA_CHANNEL_CFG  0x03
#define VS_QHCI_START_A2DP_MEDIA              0x04
#define VS_QHCI_STOP_A2DP_MEDIA               0x05
#define VS_QHCI_A2DP_WRITE_SUGGESTED_BITRATE  0x06
#define VS_QHCI_A2DP_TRANSPORT_CONFIGURATION  0x07
#define VS_QHCI_A2DP_WRITE_SCMS_T_CP          0x08
#define VS_QHCI_A2DP_SELECTED_CODEC           0x09
#define VS_QHCI_A2DP_OFFLOAD_START            0x0A
#define VS_QHCI_GET_SCRAMBLING_FREQS          0x11
#define VS_QHCI_SCRAMBLE_A2DP_MEDIA           0x12
#define VS_QHCI_ENCODER_MODE_CHANGE           0X13
#define A2DP_TRANSPORT_TYPE_SLIMBUS     0
#define QHCI_INVALID_VSC 0x01
/* SPLITA2DP */
/*****************************************************************************
 *  Global data
 ****************************************************************************/

/* control block declaration */
extern tBTA_AVK_CB bta_avk_cb;

/* config struct */
extern tBTA_AVK_CFG* p_bta_avk_cfg;
extern const tBTA_AVK_CFG bta_avk_sink_cfg;

/* rc id config struct */
extern uint16_t* p_bta_avk_rc_id;
extern uint16_t* p_bta_avk_rc_id_ac;

extern const tBTA_AVK_SACT bta_avk_a2dp_action[];
extern const tBTA_AVK_CO_FUNCTS bta_avk_a2dp_cos;
extern tAVDT_CTRL_CBACK* const bta_avk_dt_cback[];
extern void bta_avk_sink_data_cback(uint8_t handle, BT_HDR* p_pkt,
                                   uint32_t time_stamp, uint8_t m_pt);

/*****************************************************************************
 *  Function prototypes
 ****************************************************************************/
/* utility functions */
extern tBTA_AVK_SCB* bta_avk_hndl_to_scb(uint16_t handle);
extern bool bta_avk_chk_start(tBTA_AVK_SCB* p_scb);
extern void bta_avk_restore_switch(void);
extern uint16_t bta_avk_chk_mtu(tBTA_AVK_SCB* p_scb, uint16_t mtu);
extern void bta_avk_conn_cback(uint8_t handle, const RawAddress* bd_addr,
                              uint8_t event, tAVDT_CTRL* p_data);
extern bool bta_avk_is_scb_available(void);
extern uint8_t bta_avk_rc_create(tBTA_AVK_CB* p_cb, uint8_t role, uint8_t shdl,
                                uint8_t lidx);
extern void bta_avk_stream_chg(tBTA_AVK_SCB* p_scb, bool started);
extern bool bta_avk_is_scb_opening(tBTA_AVK_SCB* p_scb);
extern bool bta_avk_is_scb_incoming(tBTA_AVK_SCB* p_scb);
extern void bta_avk_set_scb_sst_init(tBTA_AVK_SCB* p_scb);
extern bool bta_avk_is_scb_init(tBTA_AVK_SCB* p_scb);
extern void bta_avk_set_scb_sst_incoming(tBTA_AVK_SCB* p_scb);
extern tBTA_AVK_LCB* bta_avk_find_lcb(const RawAddress& addr, uint8_t op);
extern bool bta_avk_is_multicast_enabled();

/* main functions */
extern void bta_avk_api_deregister(tBTA_AVK_DATA* p_data);
extern void bta_avk_dup_audio_buf(tBTA_AVK_SCB* p_scb, BT_HDR* p_buf);
extern void bta_avk_sm_execute(tBTA_AVK_CB* p_cb, uint16_t event,
                              tBTA_AVK_DATA* p_data);
extern void bta_avk_ssm_execute(tBTA_AVK_SCB* p_scb, uint16_t event,
                               tBTA_AVK_DATA* p_data);
extern bool bta_avk_hdl_event(BT_HDR* p_msg);
extern const char* bta_avk_evt_code(uint16_t evt_code);
extern bool bta_avk_switch_if_needed(tBTA_AVK_SCB* p_scb);
extern bool bta_avk_link_role_ok(tBTA_AVK_SCB* p_scb, uint8_t bits);
extern bool bta_avk_is_rcfg_sst(tBTA_AVK_SCB* p_scb);

/* nsm action functions */
extern void bta_avk_api_disconnect(tBTA_AVK_DATA* p_data);
extern void bta_avk_sig_chg(tBTA_AVK_DATA* p_data);
extern void bta_avk_signalling_timer(tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_disc_done(tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_closed(tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_browse_opened(tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_browse_closed(tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_disc(uint8_t disc);
extern void bta_avk_rc_retry_disc(tBTA_AVK_DATA* p_data);
extern void bta_avk_conn_chg(tBTA_AVK_DATA* p_data);
extern void bta_avk_dereg_comp(tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_collission_detected(tBTA_AVK_DATA *p_data);
extern void bta_avk_update_enc_mode(tBTA_AVK_DATA* p_data);
extern void bta_avk_update_aptx_data(tBTA_AVK_DATA* p_data);
extern void bta_avk_active_browse(tBTA_AVK_DATA *p_data);

/* sm action functions */
extern void bta_avk_disable(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_opened(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_remote_cmd(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_vendor_cmd(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_vendor_rsp(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_msg(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_close(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_meta_rsp(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_free_rsp(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rc_free_browse_msg(tBTA_AVK_CB* p_cb, tBTA_AVK_DATA* p_data);

extern tBTA_AVK_RCB* bta_avk_get_rcb_by_shdl(uint8_t shdl);
extern void bta_avk_del_rc(tBTA_AVK_RCB* p_rcb);

/* ssm action functions */
extern void bta_avk_do_disc_a2dp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_cleanup(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_free_sdb(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_config_ind(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_disconnect_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_security_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_security_rsp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_setconfig_rsp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_str_opened(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_security_ind(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_security_cfm(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_do_close(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_connect_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_sdp_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_disc_results(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_disc_res_as_acp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_open_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_getcap_results(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_setconfig_rej(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_discover_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_conn_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_do_start(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_str_stopped(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_reconfig(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_data_path(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_start_ok(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_start_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_str_closed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_clr_cong(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_suspend_cfm(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rcfg_str_ok(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rcfg_failed(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rcfg_connect(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rcfg_discntd(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_suspend_cont(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rcfg_cfm(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rcfg_open(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_security_rej(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_open_rc(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_chk_2nd_start(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_save_caps(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rej_conn(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_rej_conn(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_set_use_rc(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_cco_close(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_switch_role(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_role_res(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_delay_rpt(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_open_at_inc(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_offload_req(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_offload_rsp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
extern void bta_avk_vendor_offload_stop(tBTA_AVK_SCB* p_scb);
extern void bta_avk_disc_fail_as_acp(tBTA_AVK_SCB* p_scb, tBTA_AVK_DATA* p_data);
#if (TWS_ENABLED == TRUE)
extern void bta_avk_set_tws_chn_mode(tBTA_AVK_SCB* p_scb, bool adjust);
#endif
#endif /* BTA_AVK_INT_H */
