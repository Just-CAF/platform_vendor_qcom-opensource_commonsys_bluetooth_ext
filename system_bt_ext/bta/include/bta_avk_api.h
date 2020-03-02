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
 *  This is the public interface file for the advanced audio/video streaming
 *  (AVK) subsystem of BTA, Broadcom's Bluetooth application layer for mobile
 *  phones.
 *
 ******************************************************************************/
#ifndef BTA_AVK_API_H
#define BTA_AVK_API_H

#include "a2dp_codec_api.h"
#include "avdt_api.h"
#include "avrc_api.h"
#include "bta_api.h"

/*****************************************************************************
 *  Constants and data types
 ****************************************************************************/
/* Set to TRUE if seperate authorization prompt desired for AVCTP besides A2DP
 * authorization */
/* Typically FALSE when AVRCP is used in conjunction with A2DP */
#ifndef BTA_AVK_WITH_AVCTP_AUTHORIZATION
#define BTA_AVK_WITH_AVCTP_AUTHORIZATION FALSE
#endif

/* AVK status values */
#define BTA_AVK_SUCCESS 0        /* successful operation */
#define BTA_AVK_FAIL 1           /* generic failure */
#define BTA_AVK_FAIL_SDP 2       /* service not found */
#define BTA_AVK_FAIL_STREAM 3    /* stream connection failed */
#define BTA_AVK_FAIL_RESOURCES 4 /* no resources */
#define BTA_AVK_FAIL_ROLE 5 /* failed due to role management related issues */
#define BTA_AVK_FAIL_GET_CAP \
  6 /* get capability failed due to no SEP availale on the peer  */
#define BTA_AVK_FAIL_UNSUPPORTED 7 /* Offload Start Rsp handling in open state */
#define BTA_AVK_SUCCESS_BR_HANDOFF 8 /* successful browse handoff operation */

typedef uint8_t tBTA_AVK_STATUS;

/* AVK features masks */
#define BTA_AVK_FEAT_RCTG 0x0001    /* remote control target */
#define BTA_AVK_FEAT_RCCT 0x0002    /* remote control controller */
#define BTA_AVK_FEAT_PROTECT 0x0004 /* streaming media contect protection */
#define BTA_AVK_FEAT_VENDOR                                                    \
  0x0008                          /* remote control vendor dependent commands \
                                     */
#define BTA_AVK_FEAT_REPORT 0x0020 /* use reporting service for VDP */
#define BTA_AVK_FEAT_METADATA \
  0x0040 /* remote control Metadata Transfer command/response */
#define BTA_AVK_FEAT_MULTI_AV \
  0x0080                          /* use multi-av, if controller supports it */
#define BTA_AVK_FEAT_BROWSE 0x0010 /* use browsing channel */
#define BTA_AVK_FEAT_MASTER 0x0100 /* stream only as master role */
#define BTA_AVK_FEAT_ADV_CTRL \
  0x0200 /* remote control Advanced Control command/response */
#define BTA_AVK_FEAT_DELAY_RPT 0x0400 /* allow delay reporting */
#define BTA_AVK_FEAT_ACP_START \
  0x0800 /* start stream when 2nd SNK was accepted   */
#define BTA_AVK_FEAT_CA          0x1000  /* use cover art */
#define BTA_AVK_FEAT_APP_SETTING 0x2000 /* Player app setting support */

/* Internal features */
#define BTA_AVK_FEAT_AVRC_UI_UPDATE 0x4000 /* UI notification for browsing capable remote*/
#define BTA_AVK_FEAT_NO_SCO_SSPD \
  0x8000 /* Do not suspend av streaming as to AG events(SCO or Call) */

typedef uint16_t tBTA_AVK_FEAT;

/* AVK channel values */
#define BTA_AVK_CHNL_MSK 0xC0
#define BTA_AVK_CHNL_AUDIO 0x40 /* audio channel */
#define BTA_AVK_CHNL_VIDEO 0x80 /* video channel */
typedef uint8_t tBTA_AVK_CHNL;

#define BTA_AVK_HNDL_MSK 0x3F
typedef uint8_t tBTA_AVK_HNDL;
/* handle index to mask */
#define BTA_AVK_HNDL_TO_MSK(h) ((uint8_t)(1 << (h)))

/* maximum number of streams created: 1 for audio, 1 for video */
#ifndef BTA_AVK_NUM_STRS
#define BTA_AVK_NUM_STRS AVDT_NUM_LINKS
#endif

#ifndef BTA_AVK_MAX_A2DP_MTU
/*#define BTA_AVK_MAX_A2DP_MTU     668 //224 (DM5) * 3 - 4(L2CAP header) */
#define BTA_AVK_MAX_A2DP_MTU 1008
#endif

#ifndef BTA_AVK_MAX_VDP_MTU
#define BTA_AVK_MAX_VDP_MTU 1008
#endif

//Whenever we update peer MTU value, we need to deduct A2DP header, and
//if SCMS-T enabled it would be 2 bytes else it would be 1 byte.
#if (BTA_AVK_CO_CP_SCMS_T == TRUE)
#define A2DP_HEADER_SIZE  2
#else
#define A2DP_HEADER_SIZE  1
#endif

/* operation id list for BTA_AvkRemoteCmd */
#define BTA_AVK_RC_SELECT AVRC_ID_SELECT         /* select */
#define BTA_AVK_RC_UP AVRC_ID_UP                 /* up */
#define BTA_AVK_RC_DOWN AVRC_ID_DOWN             /* down */
#define BTA_AVK_RC_LEFT AVRC_ID_LEFT             /* left */
#define BTA_AVK_RC_RIGHT AVRC_ID_RIGHT           /* right */
#define BTA_AVK_RC_RIGHT_UP AVRC_ID_RIGHT_UP     /* right-up */
#define BTA_AVK_RC_RIGHT_DOWN AVRC_ID_RIGHT_DOWN /* right-down */
#define BTA_AVK_RC_LEFT_UP AVRC_ID_LEFT_UP       /* left-up */
#define BTA_AVK_RC_LEFT_DOWN AVRC_ID_LEFT_DOWN   /* left-down */
#define BTA_AVK_RC_ROOT_MENU AVRC_ID_ROOT_MENU   /* root menu */
#define BTA_AVK_RC_SETUP_MENU AVRC_ID_SETUP_MENU /* setup menu */
#define BTA_AVK_RC_CONT_MENU AVRC_ID_CONT_MENU   /* contents menu */
#define BTA_AVK_RC_FAV_MENU AVRC_ID_FAV_MENU     /* favorite menu */
#define BTA_AVK_RC_EXIT AVRC_ID_EXIT             /* exit */
#define BTA_AVK_RC_0 AVRC_ID_0                   /* 0 */
#define BTA_AVK_RC_1 AVRC_ID_1                   /* 1 */
#define BTA_AVK_RC_2 AVRC_ID_2                   /* 2 */
#define BTA_AVK_RC_3 AVRC_ID_3                   /* 3 */
#define BTA_AVK_RC_4 AVRC_ID_4                   /* 4 */
#define BTA_AVK_RC_5 AVRC_ID_5                   /* 5 */
#define BTA_AVK_RC_6 AVRC_ID_6                   /* 6 */
#define BTA_AVK_RC_7 AVRC_ID_7                   /* 7 */
#define BTA_AVK_RC_8 AVRC_ID_8                   /* 8 */
#define BTA_AVK_RC_9 AVRC_ID_9                   /* 9 */
#define BTA_AVK_RC_DOT AVRC_ID_DOT               /* dot */
#define BTA_AVK_RC_ENTER AVRC_ID_ENTER           /* enter */
#define BTA_AVK_RC_CLEAR AVRC_ID_CLEAR           /* clear */
#define BTA_AVK_RC_CHAN_UP AVRC_ID_CHAN_UP       /* channel up */
#define BTA_AVK_RC_CHAN_DOWN AVRC_ID_CHAN_DOWN   /* channel down */
#define BTA_AVK_RC_PREV_CHAN AVRC_ID_PREV_CHAN   /* previous channel */
#define BTA_AVK_RC_SOUND_SEL AVRC_ID_SOUND_SEL   /* sound select */
#define BTA_AVK_RC_INPUT_SEL AVRC_ID_INPUT_SEL   /* input select */
#define BTA_AVK_RC_DISP_INFO AVRC_ID_DISP_INFO   /* display information */
#define BTA_AVK_RC_HELP AVRC_ID_HELP             /* help */
#define BTA_AVK_RC_PAGE_UP AVRC_ID_PAGE_UP       /* page up */
#define BTA_AVK_RC_PAGE_DOWN AVRC_ID_PAGE_DOWN   /* page down */
#define BTA_AVK_RC_POWER AVRC_ID_POWER           /* power */
#define BTA_AVK_RC_VOL_UP AVRC_ID_VOL_UP         /* volume up */
#define BTA_AVK_RC_VOL_DOWN AVRC_ID_VOL_DOWN     /* volume down */
#define BTA_AVK_RC_MUTE AVRC_ID_MUTE             /* mute */
#define BTA_AVK_RC_PLAY AVRC_ID_PLAY             /* play */
#define BTA_AVK_RC_STOP AVRC_ID_STOP             /* stop */
#define BTA_AVK_RC_PAUSE AVRC_ID_PAUSE           /* pause */
#define BTA_AVK_RC_RECORD AVRC_ID_RECORD         /* record */
#define BTA_AVK_RC_REWIND AVRC_ID_REWIND         /* rewind */
#define BTA_AVK_RC_FAST_FOR AVRC_ID_FAST_FOR     /* fast forward */
#define BTA_AVK_RC_EJECT AVRC_ID_EJECT           /* eject */
#define BTA_AVK_RC_FORWARD AVRC_ID_FORWARD       /* forward */
#define BTA_AVK_RC_BACKWARD AVRC_ID_BACKWARD     /* backward */
#define BTA_AVK_RC_ANGLE AVRC_ID_ANGLE           /* angle */
#define BTA_AVK_RC_SUBPICT AVRC_ID_SUBPICT       /* subpicture */
#define BTA_AVK_RC_F1 AVRC_ID_F1                 /* F1 */
#define BTA_AVK_RC_F2 AVRC_ID_F2                 /* F2 */
#define BTA_AVK_RC_F3 AVRC_ID_F3                 /* F3 */
#define BTA_AVK_RC_F4 AVRC_ID_F4                 /* F4 */
#define BTA_AVK_RC_F5 AVRC_ID_F5                 /* F5 */
#define BTA_AVK_VENDOR AVRC_ID_VENDOR            /* vendor unique */

typedef uint8_t tBTA_AVK_RC;

/* state flag for pass through command */
#define BTA_AVK_STATE_PRESS AVRC_STATE_PRESS     /* key pressed */
#define BTA_AVK_STATE_RELEASE AVRC_STATE_RELEASE /* key released */

typedef uint8_t tBTA_AVK_STATE;

/* command codes for BTA_AvkVendorCmd */
#define BTA_AVK_CMD_CTRL AVRC_CMD_CTRL
#define BTA_AVK_CMD_STATUS AVRC_CMD_STATUS
#define BTA_AVK_CMD_SPEC_INQ AVRC_CMD_SPEC_INQ
#define BTA_AVK_CMD_NOTIF AVRC_CMD_NOTIF
#define BTA_AVK_CMD_GEN_INQ AVRC_CMD_GEN_INQ

typedef uint8_t tBTA_AVK_CMD;

/* response codes for BTA_AvkVendorRsp */
#define BTA_AVK_RSP_NOT_IMPL AVRC_RSP_NOT_IMPL
#define BTA_AVK_RSP_ACCEPT AVRC_RSP_ACCEPT
#define BTA_AVK_RSP_REJ AVRC_RSP_REJ
#define BTA_AVK_RSP_IN_TRANS AVRC_RSP_IN_TRANS
#define BTA_AVK_RSP_IMPL_STBL AVRC_RSP_IMPL_STBL
#define BTA_AVK_RSP_CHANGED AVRC_RSP_CHANGED
#define BTA_AVK_RSP_INTERIM AVRC_RSP_INTERIM

typedef uint8_t tBTA_AVK_CODE;

/* error codes for BTA_AvkProtectRsp */
#define BTA_AVK_ERR_NONE A2DP_SUCCESS /* Success, no error */
#define BTA_AVK_ERR_BAD_STATE \
  AVDT_ERR_BAD_STATE /* Message cannot be processed in this state */
#define BTA_AVK_ERR_RESOURCE AVDT_ERR_RESOURCE /* Insufficient resources */
#define BTA_AVK_ERR_BAD_CP_TYPE                                               \
  A2DP_BAD_CP_TYPE /* The requested Content Protection Type is not supported \
                      */
#define BTA_AVK_ERR_BAD_CP_FORMAT                                             \
  A2DP_BAD_CP_FORMAT /* The format of Content Protection Data is not correct \
                        */

typedef uint8_t tBTA_AVK_ERR;

/* AVK callback events */
#define BTA_AVK_ENABLE_EVT 0      /* AVK enabled */
#define BTA_AVK_REGISTER_EVT 1    /* registered to AVDT */
#define BTA_AVK_OPEN_EVT 2        /* connection opened */
#define BTA_AVK_CLOSE_EVT 3       /* connection closed */
#define BTA_AVK_START_EVT 4       /* stream data transfer started */
#define BTA_AVK_STOP_EVT 5        /* stream data transfer stopped */
#define BTA_AVK_PROTECT_REQ_EVT 6 /* content protection request */
#define BTA_AVK_PROTECT_RSP_EVT 7 /* content protection response */
#define BTA_AVK_RC_OPEN_EVT 8     /* remote control channel open */
#define BTA_AVK_RC_CLOSE_EVT 9    /* remote control channel closed */
#define BTA_AVK_REMOTE_CMD_EVT 10 /* remote control command */
#define BTA_AVK_REMOTE_RSP_EVT 11 /* remote control response */
#define BTA_AVK_VENDOR_CMD_EVT 12 /* vendor dependent remote control command */
#define BTA_AVK_VENDOR_RSP_EVT                                              \
  13                           /* vendor dependent remote control response \
                                  */
#define BTA_AVK_RECONFIG_EVT 14 /* reconfigure response */
#define BTA_AVK_SUSPEND_EVT 15  /* suspend response */
#define BTA_AVK_PENDING_EVT                                             \
  16                           /* incoming connection pending:         \
                                * signal channel is open and stream is \
                                * not open after                       \
                                * BTA_AVK_SIGNALLING_TIMEOUT_MS */
#define BTA_AVK_META_MSG_EVT 17 /* metadata messages */
#define BTA_AVK_REJECT_EVT 18   /* incoming connection rejected */
#define BTA_AVK_RC_FEAT_EVT \
  19 /* remote control channel peer supported features update */
#define BTA_AVK_SINK_MEDIA_CFG_EVT 20    /* command to configure codec */
#define BTA_AVK_SINK_MEDIA_DATA_EVT 21   /* sending data to Media Task */
#define BTA_AVK_OFFLOAD_START_RSP_EVT 22 /* a2dp offload start response */
#define BTA_AVK_RC_BROWSE_OPEN_EVT 23    /* remote control channel open */
#define BTA_AVK_RC_BROWSE_CLOSE_EVT 24   /* remote control channel closed */
#define BTA_AVK_ROLE_CHANGED_EVT 25
#define BTA_AVK_RC_COLL_DETECTED_EVT 26  /* RC channel collission detected */
#define BTA_AVK_DELAY_REPORT_EVT 27      /* update delay report */
#define BTA_AVK_OFFLOAD_STOP_RSP_EVT 28 /* a2dp offload start response */

/* Max BTA event */
#define BTA_AVK_MAX_EVT 29

typedef uint8_t tBTA_AVK_EVT;

/* Event associated with BTA_AVK_ENABLE_EVT */
typedef struct { tBTA_AVK_FEAT features; } tBTA_AVK_ENABLE;

/* Event associated with BTA_AVK_REGISTER_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl; /* audio/video */
  tBTA_AVK_HNDL hndl; /* Handle associated with the stream. */
  uint8_t app_id;    /* ID associated with call to BTA_AvkRegister() */
  tBTA_AVK_STATUS status;
} tBTA_AVK_REGISTER;

/* data associated with BTA_AVK_OPEN_EVT */
#define BTA_AVK_EDR_2MBPS 0x01
#define BTA_AVK_EDR_3MBPS 0x02
typedef uint8_t tBTA_AVK_EDR;

typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
  RawAddress bd_addr;
  tBTA_AVK_STATUS status;
  bool starting;
  tBTA_AVK_EDR edr; /* 0, if peer device does not support EDR */
  uint8_t sep;     /*  sep type of peer device */
  uint8_t role;    /* 0x00 master, 0x01 slave , 0xFF unkown */
  bool tws_device;
} tBTA_AVK_OPEN;

/* data associated with BTA_AVK_CLOSE_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
} tBTA_AVK_CLOSE;

/* data associated with BTA_AVK_START_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
  tBTA_AVK_STATUS status;
  bool initiator; /* true, if local device initiates the START */
  bool suspending;
  uint8_t role;   /* 0x00 master, 0x01 slave , 0xFF unkown */
} tBTA_AVK_START;

/* data associated with BTA_AVK_SUSPEND_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
  bool initiator; /* true, if local device initiates the SUSPEND */
  tBTA_AVK_STATUS status;
} tBTA_AVK_SUSPEND;

/* data associated with BTA_AVK_RECONFIG_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
  tBTA_AVK_STATUS status;
} tBTA_AVK_RECONFIG;

/* data associated with BTA_AVK_PROTECT_REQ_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
  uint8_t* p_data;
  uint16_t len;
} tBTA_AVK_PROTECT_REQ;

/* data associated with BTA_AVK_PROTECT_RSP_EVT */
typedef struct {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_HNDL hndl;
  uint8_t* p_data;
  uint16_t len;
  tBTA_AVK_ERR err_code;
} tBTA_AVK_PROTECT_RSP;

/* data associated with BTA_AVK_RC_OPEN_EVT */
typedef struct {
  uint8_t rc_handle;
  tBTA_AVK_FEAT peer_features;
  RawAddress peer_addr;
  tBTA_AVK_STATUS status;
  uint16_t cover_art_psm;  /* l2cap psm for cover art on remote */
} tBTA_AVK_RC_OPEN;

/* data associated with BTA_AVK_RC_CLOSE_EVT */
typedef struct {
  uint8_t rc_handle;
  RawAddress peer_addr;
} tBTA_AVK_RC_CLOSE;

/* data associated with BTA_AVK_RC_COLL_DETECTED */
typedef struct {
  uint8_t rc_handle;
  RawAddress peer_addr;
} tBTA_AVK_RC_COLL_DETECTED;

/* data associated with BTA_AVK_RC_BROWSE_OPEN_EVT */
typedef struct {
  uint8_t rc_handle;
  RawAddress peer_addr;
  tBTA_AVK_STATUS status;
} tBTA_AVK_RC_BROWSE_OPEN;

/* data associated with BTA_AVK_RC_BROWSE_CLOSE_EVT */
typedef struct {
  uint8_t rc_handle;
  RawAddress peer_addr;
} tBTA_AVK_RC_BROWSE_CLOSE;

/* data associated with BTA_AVK_RC_FEAT_EVT */
typedef struct {
  uint8_t rc_handle;
  tBTA_AVK_FEAT peer_features;
  RawAddress peer_addr;
  uint16_t cover_art_psm;  /* l2cap psm for cover art on remote */
} tBTA_AVK_RC_FEAT;

/* data associated with BTA_AVK_REMOTE_CMD_EVT */
typedef struct {
  uint8_t rc_handle;
  tBTA_AVK_RC rc_id;
  tBTA_AVK_STATE key_state;
  uint8_t len;
  uint8_t* p_data;
  tAVRC_HDR hdr; /* Message header. */
  uint8_t label;
} tBTA_AVK_REMOTE_CMD;

/* data associated with BTA_AVK_REMOTE_RSP_EVT */
typedef struct {
  uint8_t rc_handle;
  tBTA_AVK_RC rc_id;
  tBTA_AVK_STATE key_state;
  uint8_t len;
  uint8_t* p_data;
  tBTA_AVK_CODE rsp_code;
  uint8_t label;
} tBTA_AVK_REMOTE_RSP;

/* data associated with BTA_AVK_VENDOR_CMD_EVT, BTA_AVK_VENDOR_RSP_EVT */
typedef struct {
  uint8_t rc_handle;
  uint16_t len; /* Max vendor dependent message is 512 */
  uint8_t label;
  tBTA_AVK_CODE code;
  uint32_t company_id;
  uint8_t* p_data;
} tBTA_AVK_VENDOR;

/* data associated with BTA_AVK_META_MSG_EVT */
typedef struct {
  uint8_t rc_handle;
  uint16_t len;
  uint8_t label;
  tBTA_AVK_CODE code;
  uint32_t company_id;
  uint8_t* p_data;
  tAVRC_MSG* p_msg;
} tBTA_AVK_META_MSG;

/* data associated with BTA_AVK_PENDING_EVT */
typedef struct {
  RawAddress bd_addr;
  tBTA_AVK_HNDL hndl; /* Handle associated with the stream. */
} tBTA_AVK_PEND;

/* data associated with BTA_AVK_REJECT_EVT */
typedef struct {
  RawAddress bd_addr;
  tBTA_AVK_HNDL hndl; /* Handle associated with the stream that rejected the
                        connection. */
} tBTA_AVK_REJECT;

/* data associated with BTA_AVK_ROLE_CHANGED */
typedef struct
{
  RawAddress bd_addr;
  uint8_t new_role;  /* 0x00 master, 0x01 slave , 0xFF unkown */
  tBTA_AVK_HNDL hndl; /* Handle associated with role change event */
} tBTA_AVK_ROLE_CHANGED;

typedef uint16_t tBTA_AVK_LATENCY;

/* data associated with BTA_AVK_DELAY_REPORT_EVT */
typedef struct {
  RawAddress bd_addr;
  tBTA_AVK_HNDL hndl;
  tBTA_AVK_LATENCY sink_delay;
} tBTA_AVK_DELAY_RPT;

/* union of data associated with AVK callback */
typedef union {
  tBTA_AVK_CHNL chnl;
  tBTA_AVK_ENABLE enable;
  tBTA_AVK_REGISTER registr;
  tBTA_AVK_OPEN open;
  tBTA_AVK_CLOSE close;
  tBTA_AVK_START start;
  tBTA_AVK_PROTECT_REQ protect_req;
  tBTA_AVK_PROTECT_RSP protect_rsp;
  tBTA_AVK_RC_OPEN rc_open;
  tBTA_AVK_RC_CLOSE rc_close;
  tBTA_AVK_RC_COLL_DETECTED rc_col_detected;
  tBTA_AVK_RC_BROWSE_OPEN rc_browse_open;
  tBTA_AVK_RC_BROWSE_CLOSE rc_browse_close;
  tBTA_AVK_REMOTE_CMD remote_cmd;
  tBTA_AVK_REMOTE_RSP remote_rsp;
  tBTA_AVK_VENDOR vendor_cmd;
  tBTA_AVK_VENDOR vendor_rsp;
  tBTA_AVK_RECONFIG reconfig;
  tBTA_AVK_SUSPEND suspend;
  tBTA_AVK_PEND pend;
  tBTA_AVK_META_MSG meta_msg;
  tBTA_AVK_REJECT reject;
  tBTA_AVK_RC_FEAT rc_feat;
  tBTA_AVK_STATUS status;
  tBTA_AVK_ROLE_CHANGED role_changed;
  tBTA_AVK_DELAY_RPT delay_rpt;
} tBTA_AVK;

typedef struct {
  uint8_t* codec_info;
  RawAddress bd_addr;
} tBTA_AVK_SINK_CONFIG;

/* union of data associated with AVK Media callback */
typedef union {
  BT_HDR* p_data;
  tBTA_AVK_SINK_CONFIG avk_config;
} tBTA_AVK_SINK_MEDIA;

#define BTA_GROUP_NAVI_MSG_OP_DATA_LEN 5

/* AVK callback */
typedef void(tBTA_AVK_CBACK)(tBTA_AVK_EVT event, tBTA_AVK* p_data);
typedef void(tBTA_AVK_SINK_DATA_CBACK)(tBTA_AVK_EVT event, tBTA_AVK_SINK_MEDIA* p_data, RawAddress bd_addr);

/* type for stream state machine action functions */
struct tBTA_AVK_SCB;
union tBTA_AVK_DATA;
typedef void (*tBTA_AVK_ACT)(tBTA_AVK_SCB* p_cb, tBTA_AVK_DATA* p_data);

/* type for registering VDP */
typedef void(tBTA_AVK_REG)(tAVDT_CS* p_cs, char* p_service_name, void* p_data);

/* AVK configuration structure */
typedef struct {
  uint32_t company_id;  /* AVRCP Company ID */
  uint16_t avrc_mtu;    /* AVRCP MTU at L2CAP for control channel */
  uint16_t avrc_br_mtu; /* AVRCP MTU at L2CAP for browsing channel */
  uint16_t avrc_ct_cat; /* AVRCP controller categories */
  uint16_t avrc_tg_cat; /* AVRCP target categories */
  uint16_t sig_mtu;     /* AVDTP signaling channel MTU at L2CAP */
  uint16_t audio_mtu;   /* AVDTP audio transport channel MTU at L2CAP */
  const uint16_t*
      p_audio_flush_to;    /* AVDTP audio transport channel flush timeout */
  uint16_t audio_mqs;      /* AVDTP audio channel max data queue size */
  uint16_t video_mtu;      /* AVDTP video transport channel MTU at L2CAP */
  uint16_t video_flush_to; /* AVDTP video transport channel flush timeout */
  bool avrc_group;     /* true, to accept AVRC 1.3 group nevigation command */
  uint8_t num_co_ids;  /* company id count in p_meta_co_ids */
  uint8_t num_evt_ids; /* event id count in p_meta_evt_ids */
  tBTA_AVK_CODE
      rc_pass_rsp; /* the default response code for pass through commands */
  const uint32_t*
      p_meta_co_ids; /* the metadata Get Capabilities response for company id */
  const uint8_t* p_meta_evt_ids; /* the the metadata Get Capabilities response
                                    for event id */
  const tBTA_AVK_ACT* p_act_tbl;  /* the action function table for VDP stream */
  tBTA_AVK_REG* p_reg;            /* action function to register VDP */
  char avrc_controller_name[BTA_SERVICE_NAME_LEN]; /* Default AVRCP controller
                                                      name */
  char avrc_target_name[BTA_SERVICE_NAME_LEN]; /* Default AVRCP target name*/
} tBTA_AVK_CFG;

/*****************************************************************************
 *  External Function Declarations
 ****************************************************************************/

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
                  tBTA_AVK_CBACK* p_cback);

/*******************************************************************************
 *
 * Function         BTA_AvkDisable
 *
 * Description      Disable the advanced audio/video service.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkDisable(void);

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
                    uint16_t service_uuid);

/*******************************************************************************
 *
 * Function         BTA_AvkDeregister
 *
 * Description      Deregister the audio or video service
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkDeregister(tBTA_AVK_HNDL hndl);

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
                tBTA_SEC sec_mask, uint16_t uuid);

/*******************************************************************************
 *
 * Function         BTA_AvkClose
 *
 * Description      Close the current streams.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkClose(tBTA_AVK_HNDL handle);

/*******************************************************************************
 *
 * Function         BTA_AvkDisconnect
 *
 * Description      Close the connection to the address.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkDisconnect(const RawAddress& bd_addr);

/*******************************************************************************
 *
 * Function         BTA_AvkStart
 *
 * Description      Start audio/video stream data transfer.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkStart(tBTA_AVK_HNDL handle);

/*******************************************************************************
 *
 * Function         BTA_AvkStop
 *
 * Description      Stop audio/video stream data transfer on the AVK handle.
 *                  If suspend is true, this function sends AVDT suspend signal
 *                  to the connected peer(s).
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkStop(bool suspend, tBTA_AVK_HNDL handle);

/*******************************************************************************
 *
 * Function         BTA_AvkEnableMultiCast
 *
 * Description      Enable/disable Avdtp MultiCast
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkEnableMultiCast(bool state, tBTA_AVK_HNDL handle);

/*******************************************************************************
 *
 * Function         BTA_AvkUpdateMaxAVClient
 *
 * Description      Update max simultaneous AVK connections supported
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkUpdateMaxAVClient(uint8_t max_clients);

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
                    const uint8_t* p_protect_info);

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
void BTA_AvkUpdateEncoderMode(uint16_t enc_mode);

/*******************************************************************************
 *
 * Function          BTA_AvkUpdateAptxData
 *
 * Description      Update extended Aptx Data to Soc via
 *                          Vendor Specific Command.
 *                          Sends Battery level, scan mode and
 *                          ULL mode info to SoC
 *
 * Returns            void
 *
 ******************************************************************************/
void BTA_AvkUpdateAptxData(uint32_t data);


/*******************************************************************************
 * Function         BTA_AvkProtectReq
 *
 * Description      Send a content protection request.  This function can only
 *                  be used if AVK is enabled with feature BTA_AVK_FEAT_PROTECT.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkProtectReq(tBTA_AVK_HNDL hndl, uint8_t* p_data, uint16_t len);

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
                      uint16_t len);

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
                     tBTA_AVK_STATE key_state);

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
                                 uint8_t buf_len);

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
                     uint8_t* p_data, uint16_t len);

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
                     uint8_t* p_data, uint16_t len, uint32_t company_id);

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
void BTA_AvkOpenRc(tBTA_AVK_HNDL handle);

/*******************************************************************************
 *
 * Function         BTA_AvkCloseRc
 *
 * Description      Close an AVRCP connection
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkCloseRc(uint8_t rc_handle);

 /*******************************************************************************
  *
  * Function         BTA_AvkBrowseActive
  *
  * Description      Set Active Browse AVRCP
  *
  * Returns          void
  ******************************************************************************/
void BTA_AvkBrowseActive(uint8_t rc_handle, const RawAddress& bd_addr,
                        uint8_t browse_device_evt);

/*******************************************************************************
 *
 * Function         BTA_AvkMetaRsp
 *
 * Description      Send a Metadata command/response. The message contained
 *                  in p_pkt can be composed with AVRC utility functions.
 *                  This function can only be used if AVK is enabled with feature
 *                  BTA_AVK_FEAT_METADATA.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkMetaRsp(uint8_t rc_handle, uint8_t label, tBTA_AVK_CODE rsp_code,
                   BT_HDR* p_pkt);

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
                   BT_HDR* p_pkt);

/*******************************************************************************
 *
 * Function         BTA_AvkOffloadStart
 *
 * Description      Request Starting of A2DP Offload.
 *                  This function is used to start A2DP offload if vendor lib
 *                  has the feature enabled.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkOffloadStart(tBTA_AVK_HNDL hndl, bool do_scrambling);

/*******************************************************************************
 *
 * Function         BTA_AvkOffloadStartRsp
 *
 * Description      Response from vendor library indicating response for
 *                  OffloadStart.
 *
 * Returns          void
 *
 ******************************************************************************/
void BTA_AvkOffloadStartRsp(tBTA_AVK_HNDL hndl, tBTA_AVK_STATUS status);
void BTA_AvkUpdateTWSDevice(bool isTwsDevice, tBTA_AVK_HNDL hndl);
void BTA_AVKSetEarbudState(uint8_t state, tBTA_AVK_HNDL hndl);
void BTA_AVKSetEarbudRole(uint8_t role, tBTA_AVK_HNDL hndl);
void bta_avk_sniff_enable(bool policy_enable, const RawAddress& peer_addr);
bool bta_avk_get_is_peer_state_incoming(const RawAddress& bd_addr);
void bta_avk_refresh_accept_signalling_timer(const RawAddress &remote_bdaddr);
tBTA_AVK_SCB* bta_avk_addr_to_scb(const RawAddress& bd_addr);
void bta_avk_fake_suspend_rsp(const RawAddress &remote_bdaddr);
#endif /* BTA_AVK_API_H */
