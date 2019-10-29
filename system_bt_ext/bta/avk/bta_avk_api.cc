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
 *  Copyright (C) 2011-2012 Broadcom Corporation
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
 *  This is the implementation of the API for the advanced audio/video (AVK)
 *  subsystem of BTA, Broadcom's Bluetooth application layer for mobile
 *  phones.
 *
 ******************************************************************************/

#include <base/logging.h>

#include "bt_target.h"

#include <string.h>
#include "bt_common.h"
#include "bta_api.h"
#include "bta_avk_api.h"
#include "bta_avk_int.h"
#include "bta_sys.h"
#include "btif/include/btif_avk.h"

#include "osi/include/allocator.h"

/*****************************************************************************
 *  Constants
 ****************************************************************************/

static const tBTA_SYS_REG bta_avk_reg = {bta_avk_hdl_event, BTA_AvkDisable};

/*******************************************************************************
 *
 * Function         BTA_AvkEnable
 *
 * Description      Enable the advanced audio/video service. When the enable
 *                  operation is complete the callback function will be
 *                  called with a BTA_AVK_ENABLE_EVT. This function must
 *                  be called before other function in the AVK API are
 *                  called.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkEnable(tBTA_SEC sec_mask, tBTA_AVK_FEAT features,
                  tBTA_AVK_CBACK* p_cback) {
  APPL_TRACE_ERROR("%s:", __func__);
  tBTA_AVK_API_ENABLE* p_buf =
      (tBTA_AVK_API_ENABLE*)osi_malloc(sizeof(tBTA_AVK_API_ENABLE));

  /* register with BTA system manager */
  bta_sys_register(BTA_ID_AVK, &bta_avk_reg);

  p_buf->hdr.event = BTA_AVK_API_ENABLE_EVT;
  p_buf->p_cback = p_cback;
  p_buf->features = features;
  p_buf->sec_mask = sec_mask;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkDisable
 *
 * Description      Disable the advanced audio/video service.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkDisable(void) {
  if (!bta_sys_is_register(BTA_ID_AVK)) {
    APPL_TRACE_ERROR("BTA AVK is already disabled, ignoring ...");
    return;
  }

  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));

  bta_sys_deregister(BTA_ID_AVK);
  p_buf->event = BTA_AVK_API_DISABLE_EVT;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkRegister
 *
 * Description      Register the audio or video service to stack. When the
 *                  operation is complete the callback function will be
 *                  called with a BTA_AVK_REGISTER_EVT. This function must
 *                  be called before AVDT stream is open.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkRegister(tBTA_AVK_CHNL chnl, const char* p_service_name,
                    uint8_t app_id, tBTA_AVK_SINK_DATA_CBACK* p_sink_data_cback,
                    uint16_t service_uuid) {
  tBTA_AVK_API_REG* p_buf =
      (tBTA_AVK_API_REG*)osi_malloc(sizeof(tBTA_AVK_API_REG));

  p_buf->hdr.layer_specific = chnl;
  p_buf->hdr.event = BTA_AVK_API_REGISTER_EVT;
  if (p_service_name)
    strlcpy(p_buf->p_service_name, p_service_name, BTA_SERVICE_NAME_LEN + 1);
  else
    p_buf->p_service_name[0] = 0;
  p_buf->app_id = app_id;
  p_buf->p_app_sink_data_cback = p_sink_data_cback;
  p_buf->service_uuid = service_uuid;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkDeregister
 *
 * Description      Deregister the audio or video service
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkDeregister(tBTA_AVK_HNDL hndl) {
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));

  p_buf->layer_specific = hndl;
  p_buf->event = BTA_AVK_API_DEREGISTER_EVT;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkOpen
 *
 * Description      Opens an advanced audio/video connection to a peer device.
 *                  When connection is open callback function is called
 *                  with a BTA_AVK_OPEN_EVT.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkOpen(const RawAddress& bd_addr, tBTA_AVK_HNDL handle, bool use_rc,
                tBTA_SEC sec_mask, uint16_t uuid) {
  tBTA_AVK_API_OPEN* p_buf =
      (tBTA_AVK_API_OPEN*)osi_malloc(sizeof(tBTA_AVK_API_OPEN));

  p_buf->hdr.event = BTA_AVK_API_OPEN_EVT;
  p_buf->hdr.layer_specific = handle;
  p_buf->bd_addr = bd_addr;
  p_buf->use_rc = use_rc;
  p_buf->sec_mask = sec_mask;
  p_buf->switch_res = BTA_AVK_RS_NONE;
  p_buf->uuid = uuid;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkClose
 *
 * Description      Close the current streams.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkClose(tBTA_AVK_HNDL handle) {
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));

  p_buf->event = BTA_AVK_API_CLOSE_EVT;
  p_buf->layer_specific = handle;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkDisconnect
 *
 * Description      Close the connection to the address.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkDisconnect(const RawAddress& bd_addr) {
  tBTA_AVK_API_DISCNT* p_buf =
      (tBTA_AVK_API_DISCNT*)osi_malloc(sizeof(tBTA_AVK_API_DISCNT));

  p_buf->hdr.event = BTA_AVK_API_DISCONNECT_EVT;
  p_buf->bd_addr = bd_addr;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkStart
 *
 * Description      Start audio/video stream data transfer.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkStart(tBTA_AVK_HNDL handle) {
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));

  p_buf->event = BTA_AVK_API_START_EVT;
  p_buf->layer_specific = handle;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkOffloadStart
 *
 * Description      Start a2dp audio offloading.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkOffloadStart(tBTA_AVK_HNDL hndl, bool do_scrambling) {
  tBTA_AVK_API_OFFLOAD_START* p_buf =
     (tBTA_AVK_API_OFFLOAD_START*)osi_malloc(sizeof(tBTA_AVK_API_OFFLOAD_START));

  p_buf->hdr.event = BTA_AVK_API_OFFLOAD_START_EVT;
  p_buf->hdr.layer_specific = hndl;
  p_buf->do_scrambling = do_scrambling;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkOffloadStartRsp
 *
 * Description      Response from vendor lib for A2DP Offload Start request.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkOffloadStartRsp(tBTA_AVK_HNDL hndl, tBTA_AVK_STATUS status) {
  tBTA_AVK_API_STATUS_RSP* p_buf =
      (tBTA_AVK_API_STATUS_RSP*)osi_malloc(sizeof(tBTA_AVK_API_STATUS_RSP));

  p_buf->hdr.event = BTA_AVK_API_OFFLOAD_START_RSP_EVT;
  p_buf->hdr.layer_specific = hndl;
  p_buf->status = status;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkStop
 *
 * Description      Stop audio/video stream data transfer.
 *                  If suspend is true, this function sends AVDT suspend signal
 *                  to the connected peer(s).
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkStop(bool suspend, tBTA_AVK_HNDL handle) {
  tBTA_AVK_API_STOP* p_buf =
      (tBTA_AVK_API_STOP*)osi_malloc(sizeof(tBTA_AVK_API_STOP));

  p_buf->hdr.event = BTA_AVK_API_STOP_EVT;
  p_buf->flush = true;
  p_buf->suspend = suspend;
  p_buf->reconfig_stop = false;
  p_buf->hdr.layer_specific = handle;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkEnableMultiCast
 *
 * Description      Enable/Disable Avdtp MultiCast
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkEnableMultiCast(bool state, tBTA_AVK_HNDL handle)
{
  tBTA_AVK_ENABLE_MULTICAST  *p_buf;

  if ((p_buf = (tBTA_AVK_ENABLE_MULTICAST *)osi_malloc(sizeof(tBTA_AVK_ENABLE_MULTICAST))) != NULL) {
    p_buf->hdr.event = BTA_AVK_ENABLE_MULTICAST_EVT;
    p_buf->hdr.layer_specific   = handle;
    p_buf->is_multicast_enabled = state;
    bta_sys_sendmsg(p_buf);
  }
}

#if (TWS_ENABLED == TRUE)
#if (TWS_STATE_ENABLED == TRUE)
void BTA_AVKSetEarbudState(uint8_t state, tBTA_AVK_HNDL handle)
{
  APPL_TRACE_DEBUG("%s",__func__);
  tBTA_AVK_TWS_SET_EARBUD_STATE *p_buf;

  if ((p_buf = (tBTA_AVK_TWS_SET_EARBUD_STATE *)osi_malloc(sizeof(tBTA_AVK_TWS_SET_EARBUD_STATE))) != NULL) {
    p_buf->hdr.event = BTA_AVK_SET_EARBUD_STATE_EVT;
    p_buf->hdr.layer_specific = handle;
    p_buf->eb_state = state;
    bta_sys_sendmsg(p_buf);
  }
}
#endif
void BTA_AVKSetEarbudRole(uint8_t role, tBTA_AVK_HNDL handle)
{
  APPL_TRACE_DEBUG("%s",__func__);
  tBTA_AVK_TWS_SET_EARBUD_ROLE *p_buf;

  if ((p_buf = (tBTA_AVK_TWS_SET_EARBUD_ROLE *)osi_malloc(sizeof(tBTA_AVK_TWS_SET_EARBUD_ROLE))) != NULL) {
    p_buf->hdr.event = BTA_AVK_SET_EARBUD_ROLE_EVT;
    p_buf->hdr.layer_specific = handle;
    p_buf->chn_mode = role;
    bta_sys_sendmsg(p_buf);
  }
}
void BTA_AvkUpdateTWSDevice(bool state, tBTA_AVK_HNDL handle)
{
  tBTA_AVK_SET_TWS_DEVICE *p_buf;

  if ((p_buf = (tBTA_AVK_SET_TWS_DEVICE *)osi_malloc(sizeof(tBTA_AVK_SET_TWS_DEVICE))) != NULL) {
    p_buf->hdr.event = BTA_AVK_SET_TWS_DEVICE_EVT;
    p_buf->hdr.layer_specific = handle;
    p_buf->is_tws_device = state;
    bta_sys_sendmsg(p_buf);
  }
}
#endif
/*******************************************************************************
 *
 * Function         BTA_AvkUpdateMaxAVClient
 *
 * Description      Update max av connections supported simultaneously
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkUpdateMaxAVClient(uint8_t max_clients)
{
  tBTA_AVK_MAX_CLIENT *p_buf;

  if ((p_buf = (tBTA_AVK_MAX_CLIENT *) osi_malloc(sizeof(tBTA_AVK_MAX_CLIENT))) != NULL) {
    p_buf->hdr.event = BTA_AVK_UPDATE_MAX_AV_CLIENTS_EVT;
    p_buf->max_clients = max_clients;
    bta_sys_sendmsg(p_buf);
  }
}

/*******************************************************************************
 *
 * Function         BTA_AvkReconfig
 *
 * Description      Reconfigure the audio/video stream.
 *                  If suspend is true, this function tries the
 *                  suspend/reconfigure procedure first.
 *                  If suspend is false or when suspend/reconfigure fails,
 *                  this function closes and re-opens the AVDT connection.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkReconfig(tBTA_AVK_HNDL hndl, bool suspend, uint8_t sep_info_idx,
                    uint8_t* p_codec_info, uint8_t num_protect,
                    const uint8_t* p_protect_info) {
  tBTA_AVK_API_RCFG* p_buf =
      (tBTA_AVK_API_RCFG*)osi_malloc(sizeof(tBTA_AVK_API_RCFG) + num_protect);

  p_buf->hdr.layer_specific = hndl;
  p_buf->hdr.event = BTA_AVK_API_RECONFIG_EVT;
  p_buf->num_protect = num_protect;
  p_buf->suspend = suspend;
  p_buf->sep_info_idx = sep_info_idx;
  p_buf->p_protect_info = (uint8_t*)(p_buf + 1);
  memcpy(p_buf->codec_info, p_codec_info, AVDT_CODEC_SIZE);
  memcpy(p_buf->p_protect_info, p_protect_info, num_protect);

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkUpdateEncoderMode
 *
 * Description      Update current encoder mode to SoC by sending
 *                  Vendor Specific Command. It is called based on
 *                  Encoder feedback of Low Latency and High Quality
 *                  modes
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkUpdateEncoderMode(uint16_t enc_mode) {
  tBTA_AVK_ENC_MODE* p_buf =
      (tBTA_AVK_ENC_MODE*)osi_malloc(sizeof(tBTA_AVK_ENC_MODE));

  p_buf->hdr.event = BTA_AVK_UPDATE_ENCODER_MODE_EVT;
  p_buf->enc_mode = enc_mode;

  bta_sys_sendmsg(p_buf);
}

void BTA_AvkUpdateAptxData(uint32_t data) {
  bool battery_info = (data & APTX_BATTERY_INFO);
  uint16_t aptx_mode = (uint16_t)(data & APTX_MODE_MASK);
  if(battery_info) {
    tBTA_AVK_APTX_DATA* p_buf_battery =
        (tBTA_AVK_APTX_DATA*)osi_malloc(sizeof(tBTA_AVK_APTX_DATA));
    p_buf_battery->type = 4;
    p_buf_battery->data = (uint16_t)data;
    p_buf_battery->hdr.event = BTA_AVK_UPDATE_APTX_DATA_EVT;
    bta_sys_sendmsg(p_buf_battery);
  }
  if(aptx_mode == APTX_ULL || aptx_mode == APTX_ULL_S) {
    tBTA_AVK_APTX_DATA* p_buf_ull =
        (tBTA_AVK_APTX_DATA*)osi_malloc(sizeof(tBTA_AVK_APTX_DATA));
    p_buf_ull->type = 3;
    p_buf_ull->data = 1;
    p_buf_ull->hdr.event = BTA_AVK_UPDATE_APTX_DATA_EVT;
    bta_sys_sendmsg(p_buf_ull);
  }
  if(aptx_mode == APTX_HQ || aptx_mode == APTX_LL) {
    tBTA_AVK_APTX_DATA* p_buf_ull =
        (tBTA_AVK_APTX_DATA*)osi_malloc(sizeof(tBTA_AVK_APTX_DATA));
    p_buf_ull->type = 3;
    p_buf_ull->data = 0;
    p_buf_ull->hdr.event = BTA_AVK_UPDATE_APTX_DATA_EVT;
    bta_sys_sendmsg(p_buf_ull);
  }
}

/*******************************************************************************
 *
 * Function         BTA_AvkProtectReq
 *
 * Description      Send a content protection request.  This function can only
 *                  be used if AVK is enabled with feature BTA_AVK_FEAT_PROTECT.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkProtectReq(tBTA_AVK_HNDL hndl, uint8_t* p_data, uint16_t len) {
  tBTA_AVK_API_PROTECT_REQ* p_buf = (tBTA_AVK_API_PROTECT_REQ*)osi_malloc(
      sizeof(tBTA_AVK_API_PROTECT_REQ) + len);

  p_buf->hdr.layer_specific = hndl;
  p_buf->hdr.event = BTA_AVK_API_PROTECT_REQ_EVT;
  p_buf->len = len;
  if (p_data == NULL) {
    p_buf->p_data = NULL;
  } else {
    p_buf->p_data = (uint8_t*)(p_buf + 1);
    memcpy(p_buf->p_data, p_data, len);
  }

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkProtectRsp
 *
 * Description      Send a content protection response.  This function must
 *                  be called if a BTA_AVK_PROTECT_REQ_EVT is received.
 *                  This function can only be used if AVK is enabled with
 *                  feature BTA_AVK_FEAT_PROTECT.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkProtectRsp(tBTA_AVK_HNDL hndl, uint8_t error_code, uint8_t* p_data,
                      uint16_t len) {
  tBTA_AVK_API_PROTECT_RSP* p_buf = (tBTA_AVK_API_PROTECT_RSP*)osi_malloc(
      sizeof(tBTA_AVK_API_PROTECT_RSP) + len);

  p_buf->hdr.layer_specific = hndl;
  p_buf->hdr.event = BTA_AVK_API_PROTECT_RSP_EVT;
  p_buf->len = len;
  p_buf->error_code = error_code;
  if (p_data == NULL) {
    p_buf->p_data = NULL;
  } else {
    p_buf->p_data = (uint8_t*)(p_buf + 1);
    memcpy(p_buf->p_data, p_data, len);
  }

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkRemoteCmd
 *
 * Description      Send a remote control command.  This function can only
 *                  be used if AVK is enabled with feature BTA_AVK_FEAT_RCCT.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkRemoteCmd(uint8_t rc_handle, uint8_t label, tBTA_AVK_RC rc_id,
                     tBTA_AVK_STATE key_state) {
  tBTA_AVK_API_REMOTE_CMD* p_buf =
      (tBTA_AVK_API_REMOTE_CMD*)osi_malloc(sizeof(tBTA_AVK_API_REMOTE_CMD));

  p_buf->hdr.event = BTA_AVK_API_REMOTE_CMD_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->msg.op_id = rc_id;
  p_buf->msg.state = key_state;
  p_buf->msg.p_pass_data = NULL;
  p_buf->msg.pass_len = 0;
  p_buf->label = label;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkRemoteVendorUniqueCmd
 *
 * Description      Send a remote control command with Vendor Unique rc_id.
 *                  This function can only be used if AVK is enabled with
 *                  feature BTA_AVK_FEAT_RCCT.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkRemoteVendorUniqueCmd(uint8_t rc_handle, uint8_t label,
                                 tBTA_AVK_STATE key_state, uint8_t* p_msg,
                                 uint8_t buf_len) {
  tBTA_AVK_API_REMOTE_CMD* p_buf = (tBTA_AVK_API_REMOTE_CMD*)osi_malloc(
      sizeof(tBTA_AVK_API_REMOTE_CMD) + buf_len);

  p_buf->label = label;
  p_buf->hdr.event = BTA_AVK_API_REMOTE_CMD_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->msg.op_id = AVRC_ID_VENDOR;
  p_buf->msg.state = key_state;
  p_buf->msg.pass_len = buf_len;
  if (p_msg == NULL) {
    p_buf->msg.p_pass_data = NULL;
  } else {
    p_buf->msg.p_pass_data = (uint8_t*)(p_buf + 1);
    memcpy(p_buf->msg.p_pass_data, p_msg, buf_len);
  }
  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkVendorCmd
 *
 * Description      Send a vendor dependent remote control command.  This
 *                  function can only be used if AVK is enabled with feature
 *                  BTA_AVK_FEAT_VENDOR.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkVendorCmd(uint8_t rc_handle, uint8_t label, tBTA_AVK_CODE cmd_code,
                     uint8_t* p_data, uint16_t len) {
  tBTA_AVK_API_VENDOR* p_buf =
      (tBTA_AVK_API_VENDOR*)osi_malloc(sizeof(tBTA_AVK_API_VENDOR) + len);

  p_buf->hdr.event = BTA_AVK_API_VENDOR_CMD_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->msg.hdr.ctype = cmd_code;
  p_buf->msg.hdr.subunit_type = AVRC_SUB_PANEL;
  p_buf->msg.hdr.subunit_id = 0;
  p_buf->msg.company_id = p_bta_avk_cfg->company_id;
  p_buf->label = label;
  p_buf->msg.vendor_len = len;
  if (p_data == NULL) {
    p_buf->msg.p_vendor_data = NULL;
  } else {
    p_buf->msg.p_vendor_data = (uint8_t*)(p_buf + 1);
    memcpy(p_buf->msg.p_vendor_data, p_data, len);
  }

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkVendorRsp
 *
 * Description      Send a vendor dependent remote control response.
 *                  This function must be called if a BTA_AVK_VENDOR_CMD_EVT
 *                  is received. This function can only be used if AVK is
 *                  enabled with feature BTA_AVK_FEAT_VENDOR.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkVendorRsp(uint8_t rc_handle, uint8_t label, tBTA_AVK_CODE rsp_code,
                     uint8_t* p_data, uint16_t len, uint32_t company_id) {
  tBTA_AVK_API_VENDOR* p_buf =
      (tBTA_AVK_API_VENDOR*)osi_malloc(sizeof(tBTA_AVK_API_VENDOR) + len);

  p_buf->hdr.event = BTA_AVK_API_VENDOR_RSP_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->msg.hdr.ctype = rsp_code;
  p_buf->msg.hdr.subunit_type = AVRC_SUB_PANEL;
  p_buf->msg.hdr.subunit_id = 0;
  if (company_id)
    p_buf->msg.company_id = company_id;
  else
    p_buf->msg.company_id = p_bta_avk_cfg->company_id;
  p_buf->label = label;
  p_buf->msg.vendor_len = len;
  if (p_data == NULL) {
    p_buf->msg.p_vendor_data = NULL;
  } else {
    p_buf->msg.p_vendor_data = (uint8_t*)(p_buf + 1);
    memcpy(p_buf->msg.p_vendor_data, p_data, len);
  }

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkOpenRc
 *
 * Description      Open an AVRCP connection toward the device with the
 *                  specified handle
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkOpenRc(tBTA_AVK_HNDL handle) {
  tBTA_AVK_API_OPEN_RC* p_buf =
      (tBTA_AVK_API_OPEN_RC*)osi_malloc(sizeof(tBTA_AVK_API_OPEN_RC));

  p_buf->hdr.event = BTA_AVK_API_RC_OPEN_EVT;
  p_buf->hdr.layer_specific = handle;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkCloseRc
 *
 * Description      Close an AVRCP connection
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkCloseRc(uint8_t rc_handle) {
  tBTA_AVK_API_CLOSE_RC* p_buf =
      (tBTA_AVK_API_CLOSE_RC*)osi_malloc(sizeof(tBTA_AVK_API_CLOSE_RC));

  p_buf->hdr.event = BTA_AVK_API_RC_CLOSE_EVT;
  p_buf->hdr.layer_specific = rc_handle;

  bta_sys_sendmsg(p_buf);
}

 /*******************************************************************************
  *
  * Function         BTA_AvkBrowseActive
  *
  * Description      Set Active Browse AVRCP
  *
  * Returns          void
  ******************************************************************************/
void BTA_AvkBrowseActive(uint8_t rc_handle, const RawAddress& bd_addr,
                        uint8_t browse_device_evt) {
  APPL_TRACE_DEBUG("%s: Send Browse Active msg",__func__);
  tBTA_AVK_API_ACTIVE_BROWSE_RC* p_buf =
      (tBTA_AVK_API_ACTIVE_BROWSE_RC*)osi_malloc(sizeof(tBTA_AVK_API_ACTIVE_BROWSE_RC));

  p_buf->hdr.event = BTA_AVK_BROWSE_ACTIVE_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->peer_addr = bd_addr;
  p_buf->browse_device_evt = browse_device_evt;

  bta_sys_sendmsg(p_buf);
}
/*******************************************************************************
 *
 * Function         BTA_AvkMetaRsp
 *
 * Description      Send a Metadata/Advanced Control response. The message
 *                  contained in p_pkt can be composed with AVRC utility
 *                  functions.
 *                  This function can only be used if AVK is enabled with feature
 *                  BTA_AVK_FEAT_METADATA.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkMetaRsp(uint8_t rc_handle, uint8_t label, tBTA_AVK_CODE rsp_code,
                   BT_HDR* p_pkt) {
  tBTA_AVK_API_META_RSP* p_buf =
      (tBTA_AVK_API_META_RSP*)osi_malloc(sizeof(tBTA_AVK_API_META_RSP));

  p_buf->hdr.event = BTA_AVK_API_META_RSP_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->rsp_code = rsp_code;
  p_buf->p_pkt = p_pkt;
  p_buf->is_rsp = true;
  p_buf->label = label;

  bta_sys_sendmsg(p_buf);
}

/*******************************************************************************
 *
 * Function         BTA_AvkMetaCmd
 *
 * Description      Send a Metadata/Advanced Control command. The message
*contained
 *                  in p_pkt can be composed with AVRC utility functions.
 *                  This function can only be used if AVK is enabled with feature
 *                  BTA_AVK_FEAT_METADATA.
 *                  This message is sent only when the peer supports the TG
*role.
*8                  The only command makes sense right now is the absolute
*volume command.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkMetaCmd(uint8_t rc_handle, uint8_t label, tBTA_AVK_CMD cmd_code,
                   BT_HDR* p_pkt) {
  tBTA_AVK_API_META_RSP* p_buf =
      (tBTA_AVK_API_META_RSP*)osi_malloc(sizeof(tBTA_AVK_API_META_RSP));

  p_buf->hdr.event = BTA_AVK_API_META_RSP_EVT;
  p_buf->hdr.layer_specific = rc_handle;
  p_buf->p_pkt = p_pkt;
  p_buf->rsp_code = cmd_code;
  p_buf->is_rsp = false;
  p_buf->label = label;

  bta_sys_sendmsg(p_buf);
}

void bta_avk_sniff_enable(bool policy_enable, const RawAddress& peer_addr) {
  APPL_TRACE_DEBUG("%s: sniff policy: %d, peer_addr: %s: ",
                       __func__, policy_enable, peer_addr.ToString().c_str());
  uint8_t policy = HCI_ENABLE_SNIFF_MODE;

  if (policy_enable)
    bta_sys_set_policy(BTA_ID_AVK, policy, peer_addr);
  else
    bta_sys_clear_policy(BTA_ID_AVK, policy, peer_addr);
}
