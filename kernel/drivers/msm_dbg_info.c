/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */
#define DEBUG  1

#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
//#include <arch/arm/mach-msm/smd_rpcrouter.h>

//#include "comdef.h"

#define HS_REMPROG					0x30000091
#define HS_REMVERS					0x00030005 /* 0x00020001 */

#define ONCRPC_HS_RF_CARD_NV_BAND_ID_PROC 6
#define ONCRPC_HS_RF_MODE_PROC 7
#define ONCRPC_HS_RF_CHANNEL_PROC 8
#define ONCRPC_HS_RF_PILOT_PN_PROC 9
#define ONCRPC_HS_RF_RX_AGC_DBM_PROC 10
#define ONCRPC_HS_RF_LNA_STATES_PROC 11
#define ONCRPC_HS_RF_INTELLICEIVER_STATE_PROC 12
#define ONCRPC_HS_RF_TX_AGC_DBM_PROC 13
#define ONCRPC_HS_RF_PA_STATE_PROC 14
#define ONCRPC_HS_RF_HDET_PROC 15
#define ONCRPC_HS_RF_THERM_PROC 16
#define ONCRPC_HS_RF_VBATT_PROC 17
#define ONCRPC_HS_RF_SID_PROC 18
#define ONCRPC_HS_RF_NID_PROC 19
#define ONCRPC_HS_RF_HDR_DEBUG_PROC 20
#define ONCRPC_HS_SW_VER_PROC 21
#define ONCRPC_HS_HW_VER_PROC 22
#define ONCRPC_HS_ESN_COMPARE_PROC 23

#define AUTHPROG					0x30000017
#define AUTHVERS					0x00010001 /* 0x00010001 */

#define ONCRPC_AUTH_VALIDATE_A_KEY_PROC 2
#define ONCRPC_AUTH_SEND_UPDATE_A_KEY_CMD_PROC 3


#define DBG_INFO_RPC_TIMEOUT    3000	/* 3 sec */


#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))


#define NVPROG					0x3000000e
#define NVVERS					0x00090002 /* 0x00060001 */

#define ONCRPC_NV_CMD_REMOTE_PROC 9

/*  Command codes when command is issued to the NV task.                   */
  typedef enum {
    NV_READ_F,          /* Read item */
    NV_WRITE_F,         /* Write item */
    NV_PEEK_F,          /* Peek at a location */
    NV_POKE_F,          /* Poke into a location */
    NV_FREE_F,          /* Free an nv item's memory allocation */
    NV_CHKPNT_DIS_F,    /* Disable cache checkpointing for glitch recovery */
    NV_CHKPNT_ENA_F,    /* Enable cache checkpointing for glitch recovery */
    NV_OTASP_COMMIT_F,  /* Commit (write) OTASP parameters to nv */
    NV_REPLACE_F,       /* Replace (overwrite) a dynamic pool item */
    NV_INCREMENT_F,     /* Increment the rental timer item */
    NV_RTRE_OP_CONFIG_F,/* Set the operational configuration of RTRE */
    NV_FUNC_ENUM_PAD = 0x7FFF,     /* Pad to 16 bits on ARM */
    NV_FUNC_ENUM_MAX = 0x7fffffff /* Pad to 32 bits */

  } nv_func_enum_type;

/*  Returned status codes for requested operation.                         */
  typedef enum {
    NV_DONE_S,          /* Request completed okay */
    NV_BUSY_S,          /* Request is queued */
    NV_BADCMD_S,        /* Unrecognizable command field */
    NV_FULL_S,          /* The NVM is full */
    NV_FAIL_S,          /* Command failed, reason other than NVM was full */
    NV_NOTACTIVE_S,     /* Variable was not active */
    NV_BADPARM_S,       /* Bad parameter in command block */
    NV_READONLY_S,      /* Parameter is write-protected and thus read only */
    NV_BADTG_S,         /* Item not valid for Target */
    NV_NOMEM_S,         /* free memory exhausted */
    NV_NOTALLOC_S,      /* address is not a valid allocation */
    NV_STAT_ENUM_PAD = 0x7FFF,     /* Pad to 16 bits on ARM */
    NV_STAT_ENUM_MAX = 0x7FFFFFFF     /* Pad to 16 bits on ARM */
  } nv_stat_enum_type;

typedef enum {
    //NV_ESN_I                                       = 0,
    //NV_ESN_CHKSUM_I                                = 1,
    NV_SLOT_CYCLE_INDEX_I                          = 5,
    NV_PCDMACH_I                                   = 20,
    NV_SCDMACH_I                                   = 21,
    NV_MIN1_I                                      = 32,
    NV_MIN2_I                                      = 33,
    NV_MOB_TERM_HOME_I                             = 34,
    NV_MOB_TERM_FOR_SID_I                          = 35,
    NV_MOB_TERM_FOR_NID_I                          = 36,
    NV_ACCOLC_I                                    = 37,
    NV_SEC_CODE_I                                  = 85,
    NV_IMSI_MCC_I                                  = 176,
    NV_IMSI_11_12_I                                = 177,
    NV_DATA_QNC_ENABLED_I                          = 240,
    NV_SID_NID_LOCK_I                              = 255,
    NV_PRL_ENABLED_I                               = 256,
    NV_HOME_SID_NID_I                              = 259,
    NV_DATA_PKT_ORIG_STR_I                         = 298,
    NV_BD_ADDR_I                                                  = 447,
    NV_RF_CAL_VER_I                                = 569,
    NV_RF_CAL_DATE_I                               = 571,
    NV_HDR_RX_DIVERSITY_CTRL_I                     = 818,
    NV_CDMA_RX_DIVERSITY_CTRL_I                    = 1018,
    NV_MEID_I                                                          = 1943,
    NV_WLAN_MAC_ADDRESS_I                          = 4678,
    NV_CDROM_ENABLE_I                              = 7234,
    /*~ SENTINEL nv_items_enum_type.NV_MAX_I */ 
    NV_ITEMS_ENUM_MAX                             = 0x7fffffff
} nv_items_enum_type;
#define CM_NAM_1 0
#define HDRLOG_NUM_STREAMS 4
#define MODEM_SW_VERSION_LEN  32
#define NV_SIZE_OF_VERSION         8
#define NV_MAX_HOME_SID_NID  20 /* Up to 20 home SID/NID pairs */
#define NV_DATA_DIAL_STR_LEN  16
//#define NV_SECCODE_LEN  NV_SEC_CODE_SIZE
#define AKEY_LEN 26
enum
{
//#ifdef T_MSM6500
  CFGI_DEBUG_BAND_BC0 = 0,
  CFGI_DEBUG_BAND_BC1,
  CFGI_DEBUG_BAND_BC2,
  CFGI_DEBUG_BAND_BC3,
  CFGI_DEBUG_BAND_BC4,
  CFGI_DEBUG_BAND_BC5,
  CFGI_DEBUG_BAND_BC6,
  CFGI_DEBUG_BAND_BC7,
  CFGI_DEBUG_BAND_BC8,
  CFGI_DEBUG_BAND_BC9,
  CFGI_DEBUG_BAND_BC10,
  CFGI_DEBUG_BAND_BC11,
  CFGI_DEBUG_BAND_BC12,
  CFGI_DEBUG_BAND_BC13,
  CFGI_DEBUG_BAND_BC14,
//#endif //T_MSM6500
  CFGI_DEBUG_BAND_AMPS,
  CFGI_DEBUG_BAND_CDMA,
  CFGI_DEBUG_BAND_PCS,
  CFGI_DEBUG_BAND_GPS,
  CFGI_DEBUG_BAND_SLEEP,
  CFGI_DEBUG_BAND_LAST,
  CFGI_DEBUG_BAND_GSM,
  CFGI_DEBUG_BAND_WCDMA
};
#define FEATURE_HDR
#ifdef FEATURE_HDR
typedef enum
{
  HDRDEBUG_HDRSRCH_INACTIVE_STATE        = 0x00,
    /* Searcher is idle ... */

  HDRDEBUG_HDRSRCH_ACQ_STATE             = 0x01,
    /* Attempting to acquire a system */

  HDRDEBUG_HDRSRCH_SYNC_STATE            = 0x02,
    /* Acquired ... waiting for SYNC message */

  HDRDEBUG_HDRSRCH_IDLE_STATE            = 0x03,
    /* Idle state, included monitoring control channel, access attempts */

  HDRDEBUG_HDRSRCH_SUSPENDED_IDLE_STATE  = 0x3a,
    /* Idle state, included monitoring control channel, access attempts */

  HDRDEBUG_HDRSRCH_OFS_IDLE_STATE        = 0x3c,
    /* Off frequency neighbour searching */

  HDRDEBUG_HDRSRCH_SLEEP_STATE           = 0x3e,
    /* Sleep and Reacquisition after sleep*/

  HDRDEBUG_HDRSRCH_REACQ_STATE           = 0x3f,
    /* Reacquisition after sleep*/

  HDRDEBUG_HDRSRCH_CONNECTED_STATE       = 0x04,
    /* Traffic */

  HDRDEBUG_HDRSRCH_SUSPENDED_TC_STATE    = 0x4a,
    /* Traffic Suspended */

  HDRDEBUG_HDRSRCH_OFS_TC_STATE          = 0x4c,
    /* Traffic Suspended */

  HDRDEBUG_HDRSRCH_NO_STATE              = 0xFF
    /* Start / Terminate HDR operation */
}hdrdebug_srch_state;

typedef enum
{
  HDRDEBUG_SLEEP_MODE_SCC = 0x00,
  HDRDEBUG_SLEEP_MODE_CCC,
  HDRDEBUG_SLEEP_MODE_LONG,
  HDRDEBUG_SLEEP_MODE_URH
}hdrdebug_sleep_mode;

typedef enum
{
  HDRDEBUG_RX_DIV_RX0_ONLY = 0x00,
  HDRDEBUG_RX_DIV_BOTH
}hdr_debug_rx_div;

typedef enum
{
  HDRDEBUG_SESSION_STATE_CLOSE     = 0, /* Inactive State */
  HDRDEBUG_SESSION_STATE_AMP_SETUP = 1, /* AMP Setup State */
  HDRDEBUG_SESSION_STATE_AT_INIT   = 2, /* AT initiated State */
  HDRDEBUG_SESSION_STATE_AN_INIT   = 3, /* AN initiated State */
  HDRDEBUG_SESSION_STATE_OPEN      = 4  /* Open state */
}hdrdebug_session_state;

typedef enum
{
  HDRDEBUG_PROT_STATE_INACTIVE  = 0, /* Inactive State */
  HDRDEBUG_PROT_STATE_ACQ       = 1, /* Acquistion */
  HDRDEBUG_PROT_STATE_SYNC      = 2, /* Sync */
  HDRDEBUG_PROT_STATE_IDLE      = 3, /* Idle */
  HDRDEBUG_PROT_STATE_ACCESS    = 4, /* Access */
  HDRDEBUG_PROT_STATE_CONNECTED = 5  /* Connected */

}hdrdebug_prot_state;
#endif

#if DEBUG
#define DBG_LIMIT(x...) do {if (printk_ratelimit()) pr_debug(x); } while (0)
#else
#define DBG_LIMIT(x...) do {} while (0)
#endif

#ifndef ABS
   #define ABS(VAL) (((VAL)>0)?(VAL):(-(VAL)))
#endif


struct hdrlog_debug_info
{

  struct
  {
    u32 chan_num;                /* Channel Number                     */
    u32  band_class;              /* Band Class                         */

  } hdr_freq;

  u32 rx_agc0;                     /* Receive signal strength chain0     */
  u32 rx_agc1;                     /* Receive signal strength chain1     */
  u32 tx_agc;                      /* Connected state Tx signal strength */


  u32 serving_pn;                /* PN Offset of serving ASET pilot    */
  u32 aset_pn[6];                /* ASET pilot PN offsets              */
  u32  sleep_mode;                /* HDR Sleep mode
                                       Possible values- 0 : SCC, 1 : CCC,
                                       2 : Long, 3 : Until Reacq. handoff,
                                       4 : Rel A Idle                     */

  struct
  {
    u32 rel0_sci;                 /* Release 0 Slot Cycle Index
                                       Possible values:
                                       0 : 5.12s,  1 : 10.24s,
                                       2 : 20.48s, 3 : 40.96s,
                                       4 : Rel. A SCI                     */
    u32 relA_sci;                 /* Release A Slot Cycle Index         */

  } sci;

  u32 srch_state;                 /* HDR Searcher state                 */
  u32 rx_div;                      /* HDR Receive Diversity    
                                         0 : Rx0 only, 1 : Rx1 only       */

  u32 prot_state;                 /* HDR Protocol state                 
                                        0 : Inactive, 1 : Acquistion        
                                        2 : Sync,     3 : Idle,
                                        4 : Access,   5 : Connected       */

  u32 hdr_session_state;          /* HDR session state                  
                                       0 : Inactive, 1 :AMP Setup State
                                       2 : AT initiated, 3 : AN initiated
                                       4 : Open state                     */

  struct
  {
    u32 uati24;                  /* UATI                               */
    u32  color_code;              /* Color code for subnet              */
  } uati_info;

  u32  stream_config[HDRLOG_NUM_STREAMS]; 
                                    /* Stream configuration Release 0     */ 

};
#if 0
/* Type to hold SID+NID pairs. */
/* 
 * The SID is 15 bits, per CAI 2.3.8, and the NID is 16 bits, 
 * per CAI section 2.3.10.3.
 */
struct nv_sid_nid_pair_type { 
  /* 15 bits, per CAI 2.3.8 */
  u16                                             sid;
  /* 16 bits, per CAI section 2.3.10.3 */
  u16                                             nid;
} ;

/* Type to hold SID+NID pairs to be locked out from CDMA acquisition. */
struct nv_sid_nid_lock_type { 
  /* NAM id 0-N */
  u8  nam;
  /* SID+NID Pair */
  struct nv_sid_nid_pair_type      pair[NV_MAX_SID_NID_LOCK];
} ;

/* Type to hold CDMA channel and associated NAM. */
/* 
 * Value is 11 bits for Primary and Secondary channels, 
 * per CAI section 6.1.1.
 */
struct nv_cdmach_type { 
  /* NAM id 0-N */
  u8                                             nam;
  /* A carrier channel number */
  u16                                             channel_a;
  /* B carrier channel number */
  u16                                             channel_b;
} ;
#else
#ifndef	word
#define	word   u16
#endif
#ifndef uint16
#define  uint16 u16
#endif
#ifndef	dword
#define	dword  u32
#endif
#ifndef uint32
#define uint32 u32
#endif
#ifndef	qword
#define	qword  u64
#endif
#ifndef NV_MAX_MINS
#define NV_MAX_MINS 2
#endif
#define  NV_SEC_CODE_SIZE                                        6
/* Up to 10 SID/NIDs to lock out */
#define  NV_MAX_SID_NID_LOCK                                    10

/* Up to 20 home SID/NID pairs */
#define  NV_MAX_HOME_SID_NID                                    20
/* Maximum 16 digit number */
#define  NV_MAX_PKT_ORIG_DIGITS                                 16

/* 6 bytes per addr */
#define  NV_BD_ADDR_SIZE                                         6

#ifndef	boolean
#define boolean uint8_t
#endif
#ifndef PACKED
//#define PACKED
//#define PACKED		__attribute__((packed))
#endif

//#if defined(__ARMCC_VERSION)
//#undef PACKED_POST
//#define PACKED_POST
//#define PACKED_POST    __attribute__((__packed__))
//#endif
/* Type to hold CDMA channel and associated NAM. */
/* 
 * Value is 11 bits for Primary and Secondary channels, 
 * per CAI section 6.1.1.
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* A carrier channel number */
  u32                                             channel_a;
  /* B carrier channel number */
  u32                                             channel_b;
} nv_cdmach_type;

/* Type to hold MIN1p for 2 MINs along with the associated NAM id. */
/* 
 * The number is 24 bits, per CAI section 2.3.1.
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* MIN1 */
  u32                                            min1[NV_MAX_MINS];
} nv_min1_type;

/* Type to hold MIN2p for 2 MINs along with the associated NAM id. */
/* 
 * The number is 10 bits, per CAI section 2.3.1.
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* MIN2 */
  u32                                             min2[NV_MAX_MINS];
} nv_min2_type;

/* Type to hold CDMA mobile termination type along with associated NAM. */
/* 
 * To be used for the MOB_TERM_... variables for registration.
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* Registration enabled */
  u32                                          enabled[NV_MAX_MINS];
} nv_mob_term_type;

/* Type for ACCOLCp, along with associated NAM id. */
/* 
 * Value is 4 bit, per CAI section 2.3.5
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* ACCLOCp class */
  u8                                             ACCOLCpClass[NV_MAX_MINS];
} nv_accolc_type;

/* Type to hold security code. */
/* 
 * The security code is fixed length and is stored as ASCII string.
 */
typedef  struct { 
  /* Security code array */
  u8                                             digits[NV_SEC_CODE_SIZE];
} nv_sec_code_type;

/* Type to hold IMSI MCC , along with the associated NAM id. */
/* 
 * The number is 24 bits
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* imsi_mcc */
  u32                                             imsi_mcc;
} nv_imsi_mcc_type;

/* Type to hold IMSI_11_12 for 4 MINs along with the associated NAM id */
/* 
 * The number is 8 bits.
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* imsi_11_12 */
  u32                                             imsi_11_12;
} nv_imsi_11_12_type;

/* Type to hold SID+NID pairs. */
/* 
 * The SID is 15 bits, per CAI 2.3.8, and the NID is 16 bits, 
 * per CAI section 2.3.10.3.
 */
typedef  struct { 
  /* 15 bits, per CAI 2.3.8 */
  u32                                             sid;
  /* 16 bits, per CAI section 2.3.10.3 */
  u32                                             nid;
} nv_sid_nid_pair_type;

/* Type to hold SID+NID pairs to be locked out from CDMA acquisition. */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* SID+NID Pair */
  nv_sid_nid_pair_type                             pair[NV_MAX_SID_NID_LOCK];
} nv_sid_nid_lock_type;

/* Generic enabled/disabled per NAM type */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* Enabled flag */
  u32                                             enabled;
} nv_enabled_type;
/* Type to hold 'home' SID+NID pairs for CDMA acquisition */
/* 
 * The type also holds NAM id. Note that this item is NOT
 * 'per-MIN'
 */
typedef  struct { 
  /* NAM id 0-N */
  u32                                             nam;
  /* SID+NID pair */
  nv_sid_nid_pair_type                             pair[NV_MAX_HOME_SID_NID];
} nv_home_sid_nid_type;

/* Type to hold the dial string used for originating packet data calls */
typedef  struct { 
  /* Number of digits */
  u32                                             num_digits;
  /* The digit array */
  u8                                             digits[NV_MAX_PKT_ORIG_DIGITS];
} nv_data_pkt_orig_str_type;

typedef  struct { 
  
  u8                                             bd_addr[NV_BD_ADDR_SIZE];
} nv_bd_addr_type;
#endif

typedef  union {
    /* The mobile Slot Cycle index is 8 bits, per CAI section 1.1.2.2 */
  u32                                             slot_cycle_index;
/* Primary CDMA channel for the given NAM */
  nv_cdmach_type                                   pcdmach;
  /* Secondary CDMA channel for the given NAM */
  nv_cdmach_type                                   scdmach;
  /* MIN 1 (phone number) for the given NAM, in quotes, or encoded MIN1 */
  nv_min1_type                                     min1;
  /* MIN 2 (area code) for the given NAM, in quotes, or encoded MIN2 */
  nv_min2_type                                     min2;
  /* CDMA Mobile terminated home SID for the given NAM (true, false) */
  nv_mob_term_type                                 mob_term_home;
  /* CDMA Mobile terminated foreign SID for the given NAM (true, false) */
  nv_mob_term_type                                 mob_term_for_sid;
  /* CDMA Mobile terminated foreign NID for the given NAM (true, false) */
  nv_mob_term_type                                 mob_term_for_nid;
  /* Access overload class for the given nam */
  nv_accolc_type                                   accolc;
   /* 6 digit security code, represented by a string of digits (0-9) */
  nv_sec_code_type                                 sec_code;
   
  nv_imsi_mcc_type                                 imsi_mcc;
  /* True IMSI Mobile Network Code for given NAM */
  nv_imsi_11_12_type                               imsi_11_12;

  u32                                          data_qnc_enabled;

  nv_sid_nid_lock_type                             sid_nid_lock;

  /* Roaming list enabled/disabled */
  nv_enabled_type                                  prl_enabled;
  /* Home SID+NID pair item */
  nv_home_sid_nid_type                             home_sid_nid;
  /* Packet data call origination string. Maximum 16 digits. */
  nv_data_pkt_orig_str_type                        data_pkt_orig_str;
  
  u8                                             rf_cal_ver[NV_SIZE_OF_VERSION];
  u32                                            rf_cal_date;
  u32                                            hdr_rx_diversity_ctrl;
  u32                                             cdma_rx_diversity_ctrl;
  u32                                            meid[2]; 
  nv_bd_addr_type                        bd_addr;
  u8                                              wlan_mac_address[6];
  u32                                             cdrom_enable;
} nv_item_type_dbg;

struct msm_debug_info {
	struct msm_rpc_endpoint *hs_ept;
	struct msm_rpc_client *nv_client;
      struct msm_rpc_client *auth_client;
	u32 hs_api_version;
	u32 nv_api_version;
	u32 auth_api_version;
   
      u32 rf_card_nv_id;
      u32 rf_mode;
      u32 pilot_chann;
      u32 pilot_PN;
      s32 rx_AGC_dBm;
      u32 LNA_state;
      u32 IntelliCeiver_State;
      u32 Tx_AGC_dBm;
      u32 PA_State;
      u32 rf_HEDT;
      u32 vbatt_raw;
      u8 fld_version;
      s32 rf_PA_THERM;
      u32 sid;
      u32 nid;
      
      struct hdrlog_debug_info hdr_dbg_info;

      char sw_version[MODEM_SW_VERSION_LEN + 1]; 
      char rf_cal_version[NV_SIZE_OF_VERSION + 1];  // NV_RF_CAL_VER_I
      u32 hw_version;
      u32 local_rf_cal_date;  // NV_RF_CAL_DATE_I

      /*struct*/ nv_home_sid_nid_type home_sid_nid; // NV_HOME_SID_NID_I
      /*struct*/ nv_sid_nid_lock_type lock_sid_nid;   // NV_SID_NID_LOCK_I
      u16 mcc; // NV_IMSI_MCC_I
      u16 mnc; // NV_IMSI_11_12_I
      //u32 min1; // NV_MIN1_I
      //u32 min2; // NV_MIN1_2
      char         imsi_s[11];
      /*struct*/ nv_cdmach_type pcdmach; // NV_PCDMACH_I
      /*struct*/ nv_cdmach_type scdmach; // NV_SCDMACH_I
      u8 prl_enable; // NV_PRL_ENABLED_I

      u8 home_sid_reg; // NV_MOB_TERM_HOME_I
      u8 foreign_sid_reg; // NV_MOB_TERM_FOR_SID_I
      u8 foreign_nid_reg;  // NV_MOB_TERM_FOR_NID_I
      u8 overload_class; // NV_ACCOLC_I

      u8 QNC_enable; // NV_DATA_QNC_ENABLED_I
      u8 slot_cycle_index; // NV_SLOT_CYCLE_INDEX_I
      u8 pkt_dial_str[NV_DATA_DIAL_STR_LEN + 1]; // NV_DATA_PKT_ORIG_STR_I
      u8 sec_code_str[NV_SEC_CODE_SIZE + 1];//NV_SEC_CODE_I 
      
      u8 AKEY[AKEY_LEN]; // reference to qcril function auth_validate_a_key & auth_send_update_a_key_cmd

      u8 CDMA_Rx_Diversity; // NV_CDMA_RX_DIVERSITY_CTRL_I
      u8 HDR_Rx_Diversity; // NV_HDR_RX_DIVERSITY_CTRL_I
      u32   meid[2]; 
      u32 esn_compare;
      nv_bd_addr_type         bd_addr; //NV_BD_ADDR_I
      u8                          wlan_mac_address[6];//NV_WLAN_MAC_ADDRESS_I
      u8                    cdrom_enable;
};

static struct msm_debug_info msm_dbg_info = {
	.fld_version = 0x01,
};

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))
#define	be16_to_cpu_self(v)	(v = be16_to_cpu(v))

typedef struct  _nv_cmd
{
    nv_func_enum_type cmd;
    nv_items_enum_type item;
    uint32                       more_data;
    nv_item_type_dbg *data_ptr;
}nv_cmd;

typedef struct  _nv_reply
{
    nv_stat_enum_type result;
    u32 _more_data_;
    nv_items_enum_type item;
    nv_item_type_dbg *data_ptr;
}nv_reply;
typedef struct  _nv_replyex
{
    nv_stat_enum_type result;
    u32 _more_data_;
    nv_items_enum_type item;
    nv_item_type_dbg data_ptr;
}nv_replyex;
static int msm_dbg_atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

#if 0 // #9
static u16 msm_cpu_to_be8(u8 ** req, u8 num)
{
    **req = num;
    (*req) ++;
    return sizeof(u8);
}


static u16 msm_cpu_to_be16(u8 ** require, u16 num)
{
   u16 ** req = (u16**)require;
    **req = cpu_to_be16(num);
    (*req) ++;
    return sizeof(u16);
}
#endif

//static u16 msm_cpu_to_be32(u32 ** req, u32 num)
//{
//   //u32 ** req = (u32**)require;
//    **req = cpu_to_be32(num);
//    (*req) ++;
//    return sizeof(u32);
//}
#define msm_cpu_to_be32(req,num,size) \
    *req = cpu_to_be32(num);\
    req ++ ;\
    size = size + sizeof(u32);

static int msm_nv_cmd_arg_func(struct msm_rpc_client *nv_client, void *buf, void *data)
{
    nv_cmd *nv_req =( nv_cmd *)data;
    u32 *req = (u32 *)buf;
    int size = 0;
    int i;

    msm_cpu_to_be32(req,nv_req->cmd,size);
    msm_cpu_to_be32(req,nv_req->item,size);
    msm_cpu_to_be32(req,1,size);
    msm_cpu_to_be32(req,nv_req->item,size);

    switch(nv_req->item)
    {
        case NV_SLOT_CYCLE_INDEX_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->slot_cycle_index,size);
            break;

        case NV_PCDMACH_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->pcdmach.nam,size);

            msm_cpu_to_be32(req,nv_req->data_ptr->pcdmach.channel_a,size);

            msm_cpu_to_be32(req,nv_req->data_ptr->pcdmach.channel_b,size);
            break;

        case NV_SCDMACH_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->scdmach.nam,size);

            msm_cpu_to_be32(req,nv_req->data_ptr->scdmach.channel_a,size);

            msm_cpu_to_be32(req,nv_req->data_ptr->scdmach.channel_b,size);
            break;

        case NV_MIN1_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->min1.nam,size);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->min1.min1[i],size);
            }
            /*lint -restore */
            break;

        case NV_MIN2_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->min2.nam,size);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->min2.min2[i],size);
            }
            /*lint -restore */
            break;

        case NV_MOB_TERM_HOME_I://mob_term_home
            msm_cpu_to_be32(req,nv_req->data_ptr->mob_term_home.nam,size);
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->mob_term_home.enabled[i],size);
            }
            break;

        case NV_MOB_TERM_FOR_SID_I://mob_term_home
            msm_cpu_to_be32(req,nv_req->data_ptr->mob_term_for_sid.nam,size);
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->mob_term_for_sid.enabled[i],size);
            }
            break;

        case NV_MOB_TERM_FOR_NID_I://mob_term_home
            msm_cpu_to_be32(req,nv_req->data_ptr->mob_term_for_nid.nam,size);
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->mob_term_for_nid.enabled[i],size);
            }
            break;

        case NV_ACCOLC_I://accolc
            msm_cpu_to_be32(req,nv_req->data_ptr->accolc.nam,size);
            memset((void*)req,0,4);
            memcpy((void*)req,(void*)nv_req->data_ptr->accolc.ACCOLCpClass,NV_MAX_MINS);
            size = size +4;
            //for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            //{
                /*lint -save -e545*/
            //    msm_cpu_to_be32(req,nv_req->data_ptr->accolc.ACCOLCpClass[i],size);
            //}
            break;

        case NV_SEC_CODE_I://sec_code 
            //for ( i = 0; i < (NV_SEC_CODE_SIZE); i++ ) 
            //{
                /*lint -save -e545*/
            //    size += msm_cpu_to_be8(&req,nv_req->data_ptr->sec_code.digits[i]);
            //}
            memset((void*)req,0,8);
            memcpy((void*)req,(void*)nv_req->data_ptr->sec_code.digits,6);
            size = size +8;
            break;

        case NV_IMSI_MCC_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->imsi_mcc.nam,size);
            msm_cpu_to_be32(req,nv_req->data_ptr->imsi_mcc.imsi_mcc,size);
            break;

        case NV_IMSI_11_12_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->imsi_11_12.nam,size);
            msm_cpu_to_be32(req,nv_req->data_ptr->imsi_11_12.imsi_11_12,size);
            break;

        case NV_DATA_QNC_ENABLED_I://data_qnc_enabled
            msm_cpu_to_be32(req,nv_req->data_ptr->data_qnc_enabled,size);
            break;

        case NV_SID_NID_LOCK_I://sid_nid_lock
            msm_cpu_to_be32(req,nv_req->data_ptr->sid_nid_lock.nam,size);
            for ( i = 0; i < (NV_MAX_SID_NID_LOCK); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->sid_nid_lock.pair[i].sid,size);
                msm_cpu_to_be32(req,nv_req->data_ptr->sid_nid_lock.pair[i].nid,size);
            }
            break;

        case NV_PRL_ENABLED_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->prl_enabled.nam,size);
            msm_cpu_to_be32(req,nv_req->data_ptr->prl_enabled.enabled,size);
            break;

        case NV_HOME_SID_NID_I://home_sid_nid
            msm_cpu_to_be32(req,nv_req->data_ptr->home_sid_nid.nam,size);
            for ( i = 0; i < (NV_MAX_HOME_SID_NID); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->home_sid_nid.pair[i].sid,size);
                msm_cpu_to_be32(req,nv_req->data_ptr->home_sid_nid.pair[i].nid,size);
            }
            break;

        case NV_DATA_PKT_ORIG_STR_I://data_pkt_orig_str
            msm_cpu_to_be32(req,nv_req->data_ptr->data_pkt_orig_str.num_digits,size);
            memcpy((void*)req,(void*)nv_req->data_ptr->data_pkt_orig_str.digits,NV_MAX_PKT_ORIG_DIGITS);
            size = size +NV_MAX_PKT_ORIG_DIGITS;
            //for ( i = 0; i < (NV_MAX_PKT_ORIG_DIGITS); i++ ) 
            //{
                /*lint -save -e545*/
            //    msm_cpu_to_be8(req,nv_req->data_ptr->data_pkt_orig_str.digits[i],size);
            //}
            break;

        case NV_RF_CAL_VER_I://rf_cal_ver
            memcpy((void*)req,(void*)nv_req->data_ptr->rf_cal_ver,NV_SIZE_OF_VERSION);
            size = size +NV_SIZE_OF_VERSION;
            //for(i = 0; i< (NV_SIZE_OF_VERSION); i++ ) 
            //{
            //   msm_cpu_to_be8(req,nv_req->data_ptr->rf_cal_ver[i]);
            //}
            break;

        case NV_RF_CAL_DATE_I://rf_cal_date
            msm_cpu_to_be32(req,nv_req->data_ptr->rf_cal_date,size);
            break;

        case NV_HDR_RX_DIVERSITY_CTRL_I://hdr_rx_diversity_ctrl
            msm_cpu_to_be32(req,nv_req->data_ptr->hdr_rx_diversity_ctrl,size);
            break;

        case NV_CDMA_RX_DIVERSITY_CTRL_I:
            msm_cpu_to_be32(req,nv_req->data_ptr->cdma_rx_diversity_ctrl,size);
            break;

        case NV_MEID_I:
            for ( i = 0; i < (2); i++ ) 
            {
                /*lint -save -e545*/
                msm_cpu_to_be32(req,nv_req->data_ptr->meid[i],size);
            }
            break;

        case NV_BD_ADDR_I://sec_code 
            memset((void*)req,0,8);
            memcpy((void*)req,(void*)nv_req->data_ptr->bd_addr.bd_addr,6);
            size = size +8;
            break;

        case NV_WLAN_MAC_ADDRESS_I://sec_code 
            memset((void*)req,0,8);
            memcpy((void*)req,(void*)nv_req->data_ptr->wlan_mac_address,6);
            size = size +8;
            break;

        case NV_CDROM_ENABLE_I://data_qnc_enabled
            msm_cpu_to_be32(req,nv_req->data_ptr->cdrom_enable,size);
            break;

        default:
            size = 0;
            break;

    }
    return size;
}

static int msm_nv_cmd_ret_func(struct msm_rpc_client *nv_client, void *buf, void *data)
{
    nv_reply *data_ptr;
    nv_replyex *buf_ptr;
    int i;
    data_ptr = (nv_reply *)data;
    buf_ptr = (nv_replyex *)buf;
   
    data_ptr->result = be32_to_cpu(buf_ptr->result);
    data_ptr->_more_data_ = be32_to_cpu(buf_ptr->_more_data_);
    data_ptr->item = be32_to_cpu(buf_ptr->item);
    //pr_debug("%s: nv:%d %d\n", __func__,data_ptr->item,data_ptr->result);
    if(data_ptr->result !=  NV_DONE_S)
    {
        return data_ptr->result;
    }
    switch(data_ptr->item)
    {
        case NV_SLOT_CYCLE_INDEX_I:
            data_ptr->data_ptr->slot_cycle_index = be32_to_cpu(buf_ptr->data_ptr.slot_cycle_index);
            //pr_debug("NV_SLOT_CYCLE_INDEX_I %d\n", data_ptr->data_ptr->slot_cycle_index);
            break;

        case NV_PCDMACH_I:
            data_ptr->data_ptr->pcdmach.nam = be32_to_cpu(buf_ptr->data_ptr.pcdmach.nam);
            data_ptr->data_ptr->pcdmach.channel_a = be32_to_cpu(buf_ptr->data_ptr.pcdmach.channel_a);
            data_ptr->data_ptr->pcdmach.channel_b = be32_to_cpu(buf_ptr->data_ptr.pcdmach.channel_b);
            //pr_debug("NV_PCDMACH_I %d %d\n", data_ptr->data_ptr->pcdmach.channel_a,data_ptr->data_ptr->pcdmach.channel_b);
            break;

        case NV_SCDMACH_I:
            data_ptr->data_ptr->scdmach.nam = be32_to_cpu(buf_ptr->data_ptr.scdmach.nam);
            data_ptr->data_ptr->scdmach.channel_a = be32_to_cpu(buf_ptr->data_ptr.scdmach.channel_a);
            data_ptr->data_ptr->scdmach.channel_b = be32_to_cpu(buf_ptr->data_ptr.scdmach.channel_b);
            //pr_debug("NV_SCDMACH_I %d %d\n", data_ptr->data_ptr->scdmach.channel_a,data_ptr->data_ptr->scdmach.channel_b);
            break;

        case NV_MIN1_I:
            data_ptr->data_ptr->min1.nam = be32_to_cpu(buf_ptr->data_ptr.min1.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->min1.min1[i] =be32_to_cpu (buf_ptr->data_ptr.min1.min1[i]);
            }
            /*lint -restore */
            break;

        case NV_MIN2_I:
            data_ptr->data_ptr->min2.nam =be32_to_cpu( buf_ptr->data_ptr.min2.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->min2.min2[i] =be32_to_cpu (buf_ptr->data_ptr.min2.min2[i]);
            }
            /*lint -restore */
            break;

        case NV_MOB_TERM_HOME_I://mob_term_home
            data_ptr->data_ptr->mob_term_home.nam = be32_to_cpu(buf_ptr->data_ptr.mob_term_home.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->mob_term_home.enabled[i] =be32_to_cpu(buf_ptr->data_ptr.mob_term_home.enabled[i]);
            }
            /*lint -restore */
            break;

        case NV_MOB_TERM_FOR_SID_I://mob_term_for_sid
            data_ptr->data_ptr->mob_term_for_sid.nam = be32_to_cpu(buf_ptr->data_ptr.mob_term_for_sid.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->mob_term_for_sid.enabled[i] =be32_to_cpu(buf_ptr->data_ptr.mob_term_for_sid.enabled[i]);
            }
            /*lint -restore */
            break;

        case NV_MOB_TERM_FOR_NID_I://mob_term_for_nid
            data_ptr->data_ptr->mob_term_for_nid.nam = be32_to_cpu(buf_ptr->data_ptr.mob_term_for_nid.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->mob_term_for_nid.enabled[i] =be32_to_cpu(buf_ptr->data_ptr.mob_term_for_nid.enabled[i]);
            }
            /*lint -restore */
            break;

        case NV_ACCOLC_I://accolc
            data_ptr->data_ptr->accolc.nam = be32_to_cpu(buf_ptr->data_ptr.accolc.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_MINS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->accolc.ACCOLCpClass[i] =buf_ptr->data_ptr.accolc.ACCOLCpClass[i];
            }
            /*lint -restore */
            break;

        case NV_SEC_CODE_I://sec_code 
            for ( i = 0; i < (6); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->sec_code.digits[i] =buf_ptr->data_ptr.sec_code.digits[i];
            }
            break;

        case NV_IMSI_MCC_I:
            data_ptr->data_ptr->imsi_mcc.nam = be32_to_cpu(buf_ptr->data_ptr.imsi_mcc.nam);
            data_ptr->data_ptr->imsi_mcc.imsi_mcc = be32_to_cpu(buf_ptr->data_ptr.imsi_mcc.imsi_mcc);
            //pr_debug("NV_IMSI_MCC_I %d\n", data_ptr->data_ptr->imsi_mcc.imsi_mcc);
            break;

        case NV_IMSI_11_12_I:
            data_ptr->data_ptr->imsi_11_12.nam = be32_to_cpu(buf_ptr->data_ptr.imsi_11_12.nam);
            data_ptr->data_ptr->imsi_11_12.imsi_11_12 = be32_to_cpu(buf_ptr->data_ptr.imsi_11_12.imsi_11_12);
            //pr_debug("NV_IMSI_11_12_I %d\n", data_ptr->data_ptr->imsi_11_12.imsi_11_12);
            break;

        case NV_DATA_QNC_ENABLED_I://data_qnc_enabled
            data_ptr->data_ptr->data_qnc_enabled = be32_to_cpu(buf_ptr->data_ptr.data_qnc_enabled);
            break;

        case NV_SID_NID_LOCK_I://sid_nid_lock
            data_ptr->data_ptr->sid_nid_lock.nam = be32_to_cpu(buf_ptr->data_ptr.sid_nid_lock.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_SID_NID_LOCK); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->sid_nid_lock.pair[i].sid = be32_to_cpu(buf_ptr->data_ptr.sid_nid_lock.pair[i].sid);
                data_ptr->data_ptr->sid_nid_lock.pair[i].nid = be32_to_cpu(buf_ptr->data_ptr.sid_nid_lock.pair[i].nid);
            }
            /*lint -restore */
            //pr_debug("NV_SID_NID_LOCK_I %d %d\n", data_ptr->data_ptr->sid_nid_lock.pair[0].sid,data_ptr->data_ptr->sid_nid_lock.pair[0].nid);
            break;

        case NV_PRL_ENABLED_I:
            data_ptr->data_ptr->prl_enabled.nam = be32_to_cpu(buf_ptr->data_ptr.prl_enabled.nam);
            data_ptr->data_ptr->prl_enabled.enabled = be32_to_cpu(buf_ptr->data_ptr.prl_enabled.enabled);
            break;

        case NV_HOME_SID_NID_I://home_sid_nid
            data_ptr->data_ptr->home_sid_nid.nam = be32_to_cpu(buf_ptr->data_ptr.home_sid_nid.nam);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_HOME_SID_NID); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->home_sid_nid.pair[i].sid = be32_to_cpu(buf_ptr->data_ptr.home_sid_nid.pair[i].sid);
                data_ptr->data_ptr->home_sid_nid.pair[i].nid = be32_to_cpu(buf_ptr->data_ptr.home_sid_nid.pair[i].nid);
            }
            //pr_debug("NV_HOME_SID_NID_I %d %d\n", data_ptr->data_ptr->home_sid_nid.pair[0].sid,data_ptr->data_ptr->home_sid_nid.pair[0].nid);
            break;

        case NV_DATA_PKT_ORIG_STR_I://data_pkt_orig_str
            data_ptr->data_ptr->data_pkt_orig_str.num_digits = be32_to_cpu(buf_ptr->data_ptr.data_pkt_orig_str.num_digits);

            /* Calling array of XDR routines */
            for ( i = 0; i < (NV_MAX_PKT_ORIG_DIGITS); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->data_pkt_orig_str.digits[i] =buf_ptr->data_ptr.data_pkt_orig_str.digits[i];
            }
            //pr_debug("NV_DATA_PKT_ORIG_STR_I %s\n", data_ptr->data_ptr->data_pkt_orig_str.digits);
            break;

        case NV_RF_CAL_VER_I://rf_cal_ver
            for(i = 0; i< (NV_SIZE_OF_VERSION); i++ ) 
            {
               data_ptr->data_ptr->rf_cal_ver[i] = buf_ptr->data_ptr.rf_cal_ver[i];
            }
            break;

        case NV_RF_CAL_DATE_I://rf_cal_date
            data_ptr->data_ptr->rf_cal_date =be32_to_cpu (buf_ptr->data_ptr.rf_cal_date);
            //pr_debug("NV_RF_CAL_DATE_I %x\n", data_ptr->data_ptr->rf_cal_date);
            break;

        case NV_HDR_RX_DIVERSITY_CTRL_I://hdr_rx_diversity_ctrl
            data_ptr->data_ptr->hdr_rx_diversity_ctrl = be32_to_cpu (buf_ptr->data_ptr.hdr_rx_diversity_ctrl);
            break;

        case NV_CDMA_RX_DIVERSITY_CTRL_I:
            data_ptr->data_ptr->cdma_rx_diversity_ctrl =be32_to_cpu (buf_ptr->data_ptr.cdma_rx_diversity_ctrl);
            break;

        case NV_MEID_I:
            for ( i = 0; i < (2); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->meid[i] = be32_to_cpu(buf_ptr->data_ptr.meid[i]);
            }
            break;
            
        case NV_BD_ADDR_I://bd_addr 
            for ( i = 0; i < (6); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->bd_addr.bd_addr[i] =buf_ptr->data_ptr.bd_addr.bd_addr[i];
            }
            break;

        case NV_WLAN_MAC_ADDRESS_I://wlan_mac_address
            for ( i = 0; i < (6); i++ ) 
            {
                /*lint -save -e545*/
                data_ptr->data_ptr->wlan_mac_address[i] =buf_ptr->data_ptr.wlan_mac_address[i];
            }
            break;

        case NV_CDROM_ENABLE_I://data_qnc_enabled
            data_ptr->data_ptr->cdrom_enable= be32_to_cpu(buf_ptr->data_ptr.cdrom_enable);
            break;
            
        default:
            break;

    }
    return 0;
}

static nv_stat_enum_type msm_dbg_nv_cmd_remote(nv_func_enum_type cmd,  nv_items_enum_type item,  nv_item_type_dbg *data_ptr)
{
    int rc;
    nv_cmd  nv_req;
    nv_reply  nv_rep;

    nv_req.cmd = cmd;
    nv_req.item = item;
    nv_req.data_ptr = data_ptr;

    nv_rep.data_ptr = data_ptr;


    rc = msm_rpc_client_req(msm_dbg_info.nv_client,
                ONCRPC_NV_CMD_REMOTE_PROC,
                msm_nv_cmd_arg_func, &nv_req,
                msm_nv_cmd_ret_func, &nv_rep,
                msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));

    if (rc < 0) 
    {
        pr_err("%s: FAIL: nv read/write fial. rc=%d\n", __func__, rc);
        return rc;
    }

    if (nv_rep.result != NV_DONE_S)
    {
        pr_err("%s: FAIL: nv read/write fial: result=%d\n",__func__, nv_rep.result);
        return -EIO;
    }
    else
    {
        rc = NV_DONE_S;
    }

    pr_debug("%s: nv: OK\n", __func__);
    return rc;
} /* nv_cmd_remote */

typedef struct  _auth_check_cmd
{
    u8 *a_key;
}auth_check_cmd;

typedef struct  _auth_updata_cmd
{
    u32  length_int;
    u8 *a_key;
    u32 num_a_key_digits;
    u32 nam;
}auth_updata_cmd;

typedef struct  _auth_reply
{
    u32 result;
}auth_reply;

static int msm_auth_check_cmd_arg_func(struct msm_rpc_client *auth_client, void *buf, void *data)
{
    auth_check_cmd *auth_req =( auth_check_cmd *)data;
    int size = 0;
    memset((void*)buf,0,28);
    memcpy(buf ,(void*)auth_req->a_key,AKEY_LEN);
    size += AKEY_LEN * sizeof(u8) +2;
    return size;
}
static int msm_auth_updata_cmd_arg_func(struct msm_rpc_client *auth_client, void *buf, void *data)
{
    auth_updata_cmd *auth_req =( auth_updata_cmd *)data;
    u32 *req = (u32 *)buf;
    u8 *temp = NULL;
    int size = 0;
    msm_cpu_to_be32(req,auth_req->length_int,size);

    temp = (u8 *)req;
    memset((void*)temp,0,28);
    memcpy((void*)temp ,(void*)auth_req->a_key,AKEY_LEN);
    temp = temp + AKEY_LEN;
    //memcpy((void*)temp ,0,2);
    temp = temp + 2;
    
    req = (u32 *)temp;
    size += AKEY_LEN * sizeof(u8) +2;
    
    msm_cpu_to_be32(req,auth_req->num_a_key_digits,size);
    msm_cpu_to_be32(req,auth_req->nam,size);
    return size;
}

static int msm_auth_cmd_ret_func(struct msm_rpc_client *auth_client, void *buf, void *data)
{
    auth_reply *data_ptr, *buf_ptr;

    data_ptr = (auth_reply *)data;
    buf_ptr = (auth_reply *)buf;

    data_ptr->result = be32_to_cpu(buf_ptr->result);
    return 0;
}

static int msm_dbg_auth_cmd_remote(u8 *a_key,  u32 num_a_key_digits,  u32 nam)
{
    int rc;
    auth_check_cmd  check_req;
    auth_updata_cmd  updatta_req;
    auth_reply  auth_rep;

    check_req.a_key= a_key;
    
    updatta_req.length_int= AKEY_LEN;
    updatta_req.a_key= a_key;
    updatta_req.num_a_key_digits = AKEY_LEN;
    updatta_req.nam = nam;


    rc = msm_rpc_client_req(msm_dbg_info.auth_client,
                ONCRPC_AUTH_VALIDATE_A_KEY_PROC,
                msm_auth_check_cmd_arg_func, &check_req,
                msm_auth_cmd_ret_func, &auth_rep,
                msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));

    if (rc < 0) 
    {
        pr_err("%s: FAIL: auth check fial. rc=%d\n", __func__, rc);
        return rc;
    }

    if (!auth_rep.result)
    {
        pr_err("%s: FAIL: auth ckeck fial: result=%d\n",__func__, auth_rep.result);
        return -EIO;
    }
    else
    {
        rc = msm_rpc_client_req(msm_dbg_info.auth_client,
                ONCRPC_AUTH_SEND_UPDATE_A_KEY_CMD_PROC,
                msm_auth_updata_cmd_arg_func, &updatta_req,
                msm_auth_cmd_ret_func, &auth_rep,
                msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));

        if (rc < 0) 
        {
            pr_err("%s: FAIL: auth updata fial. rc=%d\n", __func__, rc);
            return rc;
        }

        if (!auth_rep.result)
        {
            pr_err("%s: FAIL: auth updata fial: result=%d\n",__func__, auth_rep.result);
            return -EIO;
        }
        else
        {
            rc = auth_rep.result;
        }
    }

    pr_debug("%s: nv: OK\n", __func__);
    return rc;
} /* nv_cmd_remote */

#if 0 // #9
static int msm_dbg_info_get_u8(uint32_t proc, u8 *u8_data_ptr)
{
	int rc;

	struct rpc_req_u8 {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_u8;

	struct rpc_reply_u8 {
		struct rpc_reply_hdr hdr;
		u32 more_data;
            u8 data;
	} reply_u8;

	req_u8.more_data = cpu_to_be32(1);
      rc = msm_rpc_call_reply(msm_dbg_info.hs_ept,
      		proc,
      		&req_u8, sizeof(req_u8),
      		&reply_u8, sizeof(reply_u8),
      		msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, proc, rc);
		return rc;
	} else if (be32_to_cpu(reply_u8.more_data)) {
            *u8_data_ptr = reply_u8.data;
	} else {
		pr_err("%s: No debug information data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;
}

static int msm_dbg_info_get_u16(uint32_t proc, u16 *u16_data_ptr)
{
	int rc;

	struct rpc_req_u16 {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_u16;

	struct rpc_reply_u16 {
		struct rpc_reply_hdr hdr;
		u32 more_data;
            u16 data;
	} reply_u16;

	req_u16.more_data = cpu_to_be32(1);
      rc = msm_rpc_call_reply(msm_dbg_info.hs_ept,
      		proc,
      		&req_u16, sizeof(req_u16),
      		&reply_u16, sizeof(reply_u16),
      		msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, proc, rc);
		return rc;
	} else if (be32_to_cpu(reply_u16.more_data)) {
            *u16_data_ptr = be16_to_cpu(reply_u16.data);	
	} else {
		pr_err("%s: No debug information data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;
}
#endif

static int msm_dbg_info_get_u32(uint32_t proc, u32 *u32_data_ptr)
{
	int rc;

	struct rpc_req_u32 {
		struct rpc_request_hdr hdr;
		//u32 more_data;
	} req_u32;

	struct rpc_reply_u32 {
		struct rpc_reply_hdr hdr;
		//u32 more_data;
            u32 data;
	} reply_u32;

	//req_u32.more_data = cpu_to_be32(1);
      rc = msm_rpc_call_reply(msm_dbg_info.hs_ept,
      		proc,
      		&req_u32, sizeof(req_u32),
      		&reply_u32, sizeof(reply_u32),
      		msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, proc, rc);
		return rc;
	} else {
            *u32_data_ptr = be32_to_cpu(reply_u32.data);	
	} 

	return 0;
}

static inline int msm_dbg_info_get_rf_card_nv_id(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_CARD_NV_BAND_ID_PROC,
         &msm_dbg_info.rf_card_nv_id);
}

static ssize_t
show_rf_card(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

      if (ddi_ptr->rf_card_nv_id == 0) {
         msm_dbg_info_get_rf_card_nv_id();
      }

	return snprintf(buf, PAGE_SIZE, "RF Card: %d\n", (((u16)ddi_ptr->rf_card_nv_id & 0xFF00) >> 8));
}

static ssize_t
show_nv_band_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

      if (ddi_ptr->rf_card_nv_id == 0) {
         msm_dbg_info_get_rf_card_nv_id();
      }
      
	return snprintf(buf, PAGE_SIZE, "NV band id:%d\n", ((u16)ddi_ptr->rf_card_nv_id & 0x00FF));
}


static inline int msm_dbg_info_get_rf_mode(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_MODE_PROC,
         &msm_dbg_info.rf_mode);
}

static ssize_t
show_rf_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    u16 rf_mode  =0;
    char band_name[6];
    if (ddi_ptr->rf_mode == 0) 
    {
        msm_dbg_info_get_rf_mode();
    }

    rf_mode = (u16)ddi_ptr->rf_mode;
    if(rf_mode <= CFGI_DEBUG_BAND_BC14 )
    {
        return snprintf(buf, PAGE_SIZE, "RF Mode: BC%d\n", rf_mode & 0x00FF);
    }
    else
    {
        switch (rf_mode)
        {
            case CFGI_DEBUG_BAND_AMPS:
              (void)strlcpy(band_name, "AMPS", sizeof(band_name));
              break;
            case CFGI_DEBUG_BAND_CDMA:
              (void)strlcpy(band_name, "CDMA", sizeof(band_name));
              break;
            case CFGI_DEBUG_BAND_PCS:
              (void)strlcpy(band_name, "PCS", sizeof(band_name));
              break;
            case CFGI_DEBUG_BAND_GPS:
              (void)strlcpy(band_name, "GPS", sizeof(band_name));
              break;
            case CFGI_DEBUG_BAND_SLEEP:
              (void)strlcpy(band_name, "SLEEP", sizeof(band_name));
              break;
            case CFGI_DEBUG_BAND_GSM:
              (void)strlcpy(band_name, "GSM", sizeof(band_name));
              break;
            case CFGI_DEBUG_BAND_WCDMA:
              (void)strlcpy(band_name, "WCDMA", sizeof(band_name));
              break;
            default:
              (void)strlcpy(band_name, "UKNW", sizeof(band_name));
              break;
        }
        return snprintf(buf, PAGE_SIZE, "RF Mode: %s\n", band_name);
    }
    //return snprintf(buf, PAGE_SIZE, "RF Mode: BC%d\n", ddi_ptr->rf_mode);
}


static inline int msm_dbg_info_get_pilot_chann(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_CHANNEL_PROC,
         &msm_dbg_info.pilot_chann);
}

static ssize_t
show_pilot_chann(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_pilot_chann();
    return snprintf(buf, PAGE_SIZE, "Channel: %d\n", (u16)ddi_ptr->pilot_chann & 0xFFFF);
}


static inline int msm_dbg_info_get_pilot_PN(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_PILOT_PN_PROC,
         &msm_dbg_info.pilot_PN);
}

static ssize_t
show_pilot_PN(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_pilot_PN();
    return snprintf(buf, PAGE_SIZE, "Pilot PN: %d\n", (u16)ddi_ptr->pilot_PN& 0xFFFF);
}


static inline int msm_dbg_info_get_rx_AGC_dBm(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_RX_AGC_DBM_PROC,
         (u32 *)&msm_dbg_info.rx_AGC_dBm);
}

static ssize_t
show_rx_AGC_dBm(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    s16 rx_agc0, rx_agc1;
    msm_dbg_info_get_rx_AGC_dBm();
    rx_agc0 = (s32)ddi_ptr->rx_AGC_dBm >> 16;
    rx_agc1 = (s32)ddi_ptr->rx_AGC_dBm & 0xFFFF;
    
    return snprintf(buf, PAGE_SIZE, "Rx AGC dBm: %d.%d : %d.%d\n", (rx_agc0/10), ABS(rx_agc0%10),
                                                     (rx_agc1/10), ABS(rx_agc1%10));
    //return snprintf(buf, PAGE_SIZE, "%d\n", ddi_ptr->rx_AGC_dBm);
}


static inline int msm_dbg_info_get_LNA_state(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_LNA_STATES_PROC,
        &msm_dbg_info.LNA_state);
}

static ssize_t
show_LNA_state(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_LNA_state();
    return snprintf(buf, PAGE_SIZE, "LNA States:  %d : %d \n", (u16)ddi_ptr->LNA_state>>4,(u16)ddi_ptr->LNA_state & 0xF);
}

static inline int msm_dbg_info_get_IntelliCeiver_State(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_INTELLICEIVER_STATE_PROC,
        &msm_dbg_info.IntelliCeiver_State);
}

static ssize_t
show_IntelliCeiver_State(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    u8 IntelliCeiver_State = 0;
    msm_dbg_info_get_LNA_state();
    IntelliCeiver_State = (u8)ddi_ptr->IntelliCeiver_State;
    switch ( IntelliCeiver_State )
    {
        case 0:
            return snprintf(buf, PAGE_SIZE, "%s\n", "IntelliCeiver State:  High");

        case 1:
            return snprintf(buf, PAGE_SIZE, "%s\n", "IntelliCeiver State:  Mid");

        case 2:
            return snprintf(buf, PAGE_SIZE, "%s\n", "IntelliCeiver State:  Low");

        default:
            return snprintf(buf, PAGE_SIZE, "%s\n", "IntelliCeiver State:  Invalid");
    }
}

static inline int msm_dbg_info_get_Tx_AGC_dBm(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_TX_AGC_DBM_PROC,
        &msm_dbg_info.Tx_AGC_dBm);
}

static ssize_t
show_Tx_AGC_dBm(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_LNA_state();
    return snprintf(buf, PAGE_SIZE, "Tx AGC dBm: %d.%d\n", ((u16)ddi_ptr->Tx_AGC_dBm/10), ABS((u16)ddi_ptr->Tx_AGC_dBm%10));
}

static inline int msm_dbg_info_get_PA_State(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_PA_STATE_PROC,
        &msm_dbg_info.PA_State);
}

static ssize_t
show_PA_State(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    char pa_line[7];
    char pa_on_off[4];
    u8 pa_data = 0;
    msm_dbg_info_get_LNA_state();
    pa_data = (u8)ddi_ptr->PA_State;
    if (pa_data & 0x10)
    {
        (void)strlcpy(pa_on_off,"ON ",sizeof(pa_on_off));

        if(pa_data & 0x02)
        {
            (void)strlcpy(pa_line,"HI PWR",sizeof(pa_line));
        }
        else
        {
            (void)strlcpy(pa_line,"LO PWR",sizeof(pa_line));
        }
    }
    else
    {
        (void)strlcpy(pa_on_off,"OFF",sizeof(pa_on_off));
        (void)strlcpy(pa_line,"",sizeof(pa_line));
    }
    return snprintf(buf, PAGE_SIZE, "PA State: %s %s\n", pa_on_off, pa_line);
}

static inline int msm_dbg_info_get_rf_HEDT(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_HDET_PROC,
        &msm_dbg_info.rf_HEDT);
}

static ssize_t
show_rf_HEDT(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;

    msm_dbg_info_get_rf_HEDT();
    if (((u8)ddi_ptr->PA_State & 0x10) == 0)
    {
        ddi_ptr->rf_HEDT = 0;
    }

    return snprintf(buf, PAGE_SIZE, "HDET: %d\n", (u16)ddi_ptr->rf_HEDT);
}


static inline int msm_dbg_info_get_vbatt_raw(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_VBATT_PROC,
        &msm_dbg_info.vbatt_raw);
}

static ssize_t
show_vbatt_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_rf_HEDT();
    return snprintf(buf, PAGE_SIZE, "VBATT: %d\n", (u16)ddi_ptr->vbatt_raw);
}


static ssize_t
show_fld_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    return snprintf(buf, PAGE_SIZE, "V0.%d\n", ddi_ptr->fld_version);
}


static inline int msm_dbg_info_get_rf_PA_THERM(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_THERM_PROC,
            (u32 *)&msm_dbg_info.rf_PA_THERM);
}

static ssize_t
show_rf_PA_THERM(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    u8 rf_PA_THERM = 0;
    msm_dbg_info_get_rf_PA_THERM();
    rf_PA_THERM = (u16)ddi_ptr->rf_PA_THERM >> 8;
    return snprintf(buf, PAGE_SIZE, "THERM: %d (raw: %d)\n", rf_PA_THERM&0x0ff, (u16)ddi_ptr->rf_PA_THERM&0x0ff);
}


static inline int msm_dbg_info_get_sid(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_SID_PROC,
            &msm_dbg_info.sid);
}

static ssize_t
show_sid(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_sid();
    return snprintf(buf, PAGE_SIZE, "SID: %d\n",(u16) ddi_ptr->sid);
}

static inline int msm_dbg_info_get_nid(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_RF_NID_PROC,
            &msm_dbg_info.nid);
}

static ssize_t
show_nid(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    msm_dbg_info_get_nid();
    return snprintf(buf, PAGE_SIZE, "NID: %d\n", (u16)ddi_ptr->nid);
}


static ssize_t
show_chan_num(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Channel Number: %d\n", ddi_ptr->hdr_dbg_info.hdr_freq.chan_num);
}

static ssize_t
show_band_class(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Band Class: %d\n", ddi_ptr->hdr_dbg_info.hdr_freq.band_class);
}

static ssize_t
show_rx_agc0(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Rx Chain 0 AGC dBm: %d\n", ddi_ptr->hdr_dbg_info.rx_agc0);
}

static ssize_t
show_rx_agc1(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Rx Chain 1 AGC dBm: %d\n", ddi_ptr->hdr_dbg_info.rx_agc1);
}

static ssize_t
show_tx_agc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Tx Chain GC dBm: %d\n", ddi_ptr->hdr_dbg_info.tx_agc);
}

static ssize_t
show_serving_pn(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Serving PN: %d\n", ddi_ptr->hdr_dbg_info.serving_pn);
}

static ssize_t
show_sleep_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    char sleep_mode[10];
    switch((u8)ddi_ptr->hdr_dbg_info.sleep_mode)
    {
     case HDRDEBUG_SLEEP_MODE_SCC:
        (void) strlcpy(sleep_mode,"SCC",sizeof(sleep_mode) );
        break;

     case HDRDEBUG_SLEEP_MODE_CCC:
        (void) strlcpy(sleep_mode,"CCC",sizeof(sleep_mode) );
        break;

     case HDRDEBUG_SLEEP_MODE_LONG:
        (void) strlcpy(sleep_mode,"LONG",sizeof(sleep_mode) );
        break;

     case HDRDEBUG_SLEEP_MODE_URH:
        (void) strlcpy(sleep_mode,"URH",sizeof(sleep_mode) );
        break;

     default:
        (void) strlcpy(sleep_mode,"None", sizeof(sleep_mode));
        break;
    }
    return snprintf(buf, PAGE_SIZE, "Sleep Mode: %s(%d)\n", sleep_mode,(u16)ddi_ptr->hdr_dbg_info.sleep_mode);
}

static ssize_t
show_rel0_sci(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Rel0 SCI: %d\n", ddi_ptr->hdr_dbg_info.sci.rel0_sci);
}

static ssize_t
show_relA_sci(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "RelA SCI: %d\n", ddi_ptr->hdr_dbg_info.sci.relA_sci);
}

static ssize_t
show_srch_state(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    char srch_state[12];
    switch((u8)ddi_ptr->hdr_dbg_info.srch_state)
    {
         case HDRDEBUG_HDRSRCH_INACTIVE_STATE:
            (void) strlcpy(srch_state,"Inactive" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_ACQ_STATE:
            (void) strlcpy(srch_state,"Acq" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_SYNC_STATE:
            (void) strlcpy(srch_state,"Sync" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_IDLE_STATE:
            (void) strlcpy(srch_state,"Idle",sizeof(srch_state) );
            break;

         case HDRDEBUG_HDRSRCH_SUSPENDED_IDLE_STATE:
            (void) strlcpy(srch_state,"SuspIdle" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_OFS_IDLE_STATE:
            (void) strlcpy(srch_state,"OfreqIdle",sizeof(srch_state) );
            break;

         case HDRDEBUG_HDRSRCH_SLEEP_STATE:
            (void) strlcpy(srch_state,"Sleep",sizeof(srch_state) );
            break;

         case HDRDEBUG_HDRSRCH_REACQ_STATE:
            (void) strlcpy(srch_state,"Reacq" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_CONNECTED_STATE:
            (void) strlcpy(srch_state,"Connect" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_SUSPENDED_TC_STATE:
            (void) strlcpy(srch_state,"SuspConn" ,sizeof(srch_state));
            break;

         case HDRDEBUG_HDRSRCH_OFS_TC_STATE:
            (void) strlcpy(srch_state,"OfreqConn",sizeof(srch_state) );
            break;

         default:
            (void) strlcpy(srch_state,"None" ,sizeof(srch_state));
            break;
    }
    return snprintf(buf, PAGE_SIZE, "Srch State: %s(0x%x)\n",srch_state, ddi_ptr->hdr_dbg_info.srch_state);
}

static ssize_t
show_rx_div(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    char rx_div[10];
    switch((u8)ddi_ptr->hdr_dbg_info.rx_div)
    {
         case HDRDEBUG_RX_DIV_RX0_ONLY:
            (void) strlcpy(rx_div,"Rx0 Only" ,sizeof(rx_div));
            break;

         case HDRDEBUG_RX_DIV_BOTH:
            (void) strlcpy(rx_div,"RxDiv" ,sizeof(rx_div));
            break;

         default:
            (void) strlcpy(rx_div,"None" ,sizeof(rx_div));
            break;
    }
    return snprintf(buf, PAGE_SIZE, "Rx Div: %s(%d)\n", rx_div, ddi_ptr->hdr_dbg_info.rx_div);
}

static ssize_t
show_prot_state(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    char prot_state[10];
    switch((u8)ddi_ptr->hdr_dbg_info.prot_state)
    {
         case HDRDEBUG_PROT_STATE_INACTIVE:
            (void) strlcpy(prot_state,"Inactive" ,sizeof(prot_state));
            break;

         case HDRDEBUG_PROT_STATE_ACQ:
            (void) strlcpy(prot_state,"Acq" ,sizeof(prot_state));
            break;

         case HDRDEBUG_PROT_STATE_SYNC:
            (void) strlcpy(prot_state,"Sync",sizeof(prot_state) );
            break;

         case HDRDEBUG_PROT_STATE_IDLE:
            (void) strlcpy(prot_state,"Idle" ,sizeof(prot_state));
            break;

         case HDRDEBUG_PROT_STATE_ACCESS:
            (void) strlcpy(prot_state,"Access" ,sizeof(prot_state));
            break;

         case HDRDEBUG_PROT_STATE_CONNECTED:
            (void) strlcpy(prot_state,"Connect",sizeof(prot_state) );
            break;

         default:
            (void) strlcpy(prot_state,"None" ,sizeof(prot_state));
            break;
    }
    return snprintf(buf, PAGE_SIZE, "Prot State: %s(%d)\n", prot_state, ddi_ptr->hdr_dbg_info.prot_state);
}

static ssize_t
show_hdr_session_state(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    char hdr_session_state[10];
    switch((u8)ddi_ptr->hdr_dbg_info.hdr_session_state)
    {
         case HDRDEBUG_SESSION_STATE_CLOSE:
            (void) strlcpy(hdr_session_state,"Inactive",sizeof(hdr_session_state) );
            break;

         case HDRDEBUG_SESSION_STATE_AMP_SETUP:
            (void) strlcpy(hdr_session_state,"AMPSetup",sizeof(hdr_session_state) );
            break;

         case HDRDEBUG_SESSION_STATE_AT_INIT:
            (void) strlcpy(hdr_session_state,"AT Init",sizeof(hdr_session_state) );
            break;

         case HDRDEBUG_SESSION_STATE_AN_INIT:
            (void) strlcpy(hdr_session_state,"AN Init",sizeof(hdr_session_state) );
            break;

         case HDRDEBUG_SESSION_STATE_OPEN:
            (void) strlcpy(hdr_session_state,"Open" ,sizeof(hdr_session_state));
            break;

         default:
            (void) strlcpy(hdr_session_state,"None" ,sizeof(hdr_session_state));
            break;
    }
    return snprintf(buf, PAGE_SIZE, "HDR Session: %s(%d)\n", hdr_session_state, ddi_ptr->hdr_dbg_info.hdr_session_state);
}

static ssize_t
show_UATI(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "UATI: 0x%x\n", ddi_ptr->hdr_dbg_info.uati_info.uati24);
}

static ssize_t
show_color_code(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "Color Code: %d\n", ddi_ptr->hdr_dbg_info.uati_info.color_code);
}

static int msm_dbg_info_get_hdrlog(void)
{
	int rc;
       u8 i;

	struct rpc_req_hdrlog {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_hdrlog;

	struct rpc_reply_hdrlog {
		struct rpc_reply_hdr hdr;
		u32 more_data;
            struct hdrlog_debug_info hdrlog;
	} reply_hdrlog;

	req_hdrlog.more_data = cpu_to_be32(1);
      rc = msm_rpc_call_reply(msm_dbg_info.hs_ept,
      		ONCRPC_HS_RF_HDR_DEBUG_PROC,
      		&req_hdrlog, sizeof(req_hdrlog),
      		&reply_hdrlog, sizeof(reply_hdrlog),
      		msecs_to_jiffies(DBG_INFO_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_HS_RF_HDR_DEBUG_PROC, rc);
		return rc;
	} else if (be32_to_cpu(req_hdrlog.more_data)) {
            msm_dbg_info.hdr_dbg_info.hdr_freq.chan_num = be32_to_cpu(reply_hdrlog.hdrlog.hdr_freq.chan_num);
            msm_dbg_info.hdr_dbg_info.hdr_freq.band_class = be32_to_cpu(reply_hdrlog.hdrlog.hdr_freq.band_class);

            msm_dbg_info.hdr_dbg_info.rx_agc0 = be32_to_cpu(reply_hdrlog.hdrlog.rx_agc0);
            msm_dbg_info.hdr_dbg_info.rx_agc1 = be32_to_cpu(reply_hdrlog.hdrlog.rx_agc1);
            msm_dbg_info.hdr_dbg_info.tx_agc = be32_to_cpu(reply_hdrlog.hdrlog.tx_agc);
            msm_dbg_info.hdr_dbg_info.serving_pn = be32_to_cpu(reply_hdrlog.hdrlog.serving_pn);

            for ( i = 0; i < (6); i++ ) 
            {
                msm_dbg_info.hdr_dbg_info.aset_pn[i] = be32_to_cpu(reply_hdrlog.hdrlog.aset_pn[i]);
            }

            msm_dbg_info.hdr_dbg_info.sleep_mode = be32_to_cpu(reply_hdrlog.hdrlog.sleep_mode);

            msm_dbg_info.hdr_dbg_info.sci.rel0_sci = be32_to_cpu(reply_hdrlog.hdrlog.sci.rel0_sci);
            msm_dbg_info.hdr_dbg_info.sci.relA_sci = be32_to_cpu(reply_hdrlog.hdrlog.sci.relA_sci);

            msm_dbg_info.hdr_dbg_info.srch_state = be32_to_cpu(reply_hdrlog.hdrlog.srch_state);
            msm_dbg_info.hdr_dbg_info.rx_div = be32_to_cpu(reply_hdrlog.hdrlog.rx_div);
            msm_dbg_info.hdr_dbg_info.prot_state = be32_to_cpu(reply_hdrlog.hdrlog.prot_state);
            msm_dbg_info.hdr_dbg_info.hdr_session_state = be32_to_cpu(reply_hdrlog.hdrlog.hdr_session_state);
            
            msm_dbg_info.hdr_dbg_info.uati_info.uati24 = be32_to_cpu(reply_hdrlog.hdrlog.uati_info.uati24);
            msm_dbg_info.hdr_dbg_info.uati_info.color_code = be32_to_cpu(reply_hdrlog.hdrlog.uati_info.color_code);
            
            for ( i = 0; i < (HDRLOG_NUM_STREAMS); i++ ) 
            {
                msm_dbg_info.hdr_dbg_info.stream_config[i] = be32_to_cpu(reply_hdrlog.hdrlog.stream_config[i]);
            }
            
	} else {
		pr_err("%s: No debug information data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;   
}

static ssize_t
show_hdrlog(struct device *dev, struct device_attribute *attr, char *buf)
{

#if 1
     char *p = buf;

      msm_dbg_info_get_hdrlog();
      // Ensure all the information can be placed in PAGE_SIZE
      p += show_chan_num(dev, attr, p);
      p += show_band_class(dev, attr, p);
      p += show_rx_agc0(dev, attr, p);
      p += show_rx_agc1(dev, attr, p);
      p += show_tx_agc(dev, attr, p);
      p += show_serving_pn(dev, attr, p);
      p += show_sleep_mode(dev, attr, p);
      p += show_rel0_sci(dev, attr, p);
      p += show_relA_sci(dev, attr, p);
      p += show_srch_state(dev, attr, p);
      p += show_rx_div(dev, attr, p);
      p += show_prot_state(dev, attr, p);
      p += show_hdr_session_state(dev, attr, p);
      p += show_UATI(dev, attr, p);
      p += show_color_code(dev, attr, p);

	pr_err("%s: total size=%d\n", __func__, (p - buf));
      return (ssize_t)(p - buf);
#else      
      msm_dbg_info_get_hdrlog();

      // size of struct hdrlog_debug_info is less than PAGE_SIZE
	memcpy((void *)buf, (void *)msm_dbg_info.hdr_dbg_info, sizeof(msm_dbg_info.hdr_dbg_info));
      return sizeof(msm_dbg_info.hdr_dbg_info);
#endif      
}


static int msm_dbg_info_get_sw_version(void)
{
	int rc, offset;
      struct msm_rpc_endpoint *ept = msm_dbg_info.hs_ept;

	struct rpc_req_sw_version {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_sw_version;

	struct rpc_reply_sw_version {
		struct rpc_reply_hdr hdr;
		//u32 more_data;
            char sw_version[MODEM_SW_VERSION_LEN]; 
	};

	struct rpc_reply_hdr *reply;
   
	struct rpc_request_hdr *req = &req_sw_version.hdr;

      msm_rpc_setup_req(req,
         HS_REMPROG,
         HS_REMVERS,
         ONCRPC_HS_SW_VER_PROC);
	req_sw_version.more_data = cpu_to_be32(1);
   
	rc = msm_rpc_write(ept, &req_sw_version, sizeof(req_sw_version));
	if (rc < 0)
		return rc;
   
	for (;;) {
		rc = msm_rpc_read(ept, (void*) &reply, -1, DBG_INFO_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < (3 * sizeof(uint32_t))) {
			rc = -EIO;
			break;
		}
		/* we should not get CALL packets -- ignore them */
		if (reply->type == 0) {
			kfree(reply);
			continue;
		}
		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req->xid) {
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != 0) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat != 0) {
			rc = -EINVAL;
			break;
		}

            offset = offsetof(struct rpc_reply_sw_version, sw_version);
            rc -= offset;
		if (rc <= 0) {
			rc = -EINVAL;
		} else {
			memcpy((void *)&msm_dbg_info.sw_version,
                           (void *)((u8 *)reply + offset),
                           (rc < MODEM_SW_VERSION_LEN) ? rc : MODEM_SW_VERSION_LEN);
		}
		break;
	}   

	kfree(reply);
	return rc;
}

static ssize_t
show_sw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

      if (msm_dbg_info_get_sw_version() < 0) {
            pr_err("%s: No debug information data in RPC reply\n", __func__);
      }
      
	return snprintf(buf, PAGE_SIZE, "SW Ver: %s\n", ddi_ptr->sw_version);
}

static inline int msm_dbg_info_get_hw_version(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_HW_VER_PROC,
         &msm_dbg_info.hw_version);
}


static ssize_t
show_hw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct msm_debug_info *ddi_ptr = &msm_dbg_info;
       //msm_dbg_info_get_hw_version();
       if (msm_dbg_info_get_hw_version() < 0) {
            pr_err("%s: No debug information data in RPC reply\n", __func__);
       }
	return snprintf(buf, PAGE_SIZE, "%02d\n", msm_dbg_info.hw_version);
}

#if 0 // #3
static inline int msm_dbg_info_get_esn_compare(void)
{
      return msm_dbg_info_get_u32(ONCRPC_HS_ESN_COMPARE_PROC,
         &msm_dbg_info.esn_compare);
}


static ssize_t
show_esn_compare(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct msm_debug_info *ddi_ptr = &msm_dbg_info;
       //msm_dbg_info_get_hw_version();
       if (msm_dbg_info_get_esn_compare() < 0) {
            pr_err("%s: No debug information data in RPC reply\n", __func__);
       }
	return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.esn_compare);
}
#endif

static ssize_t
show_rf_cal_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nv_item;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_RF_CAL_VER_I,&nv_item)) 
    {
        u8 i;
        memcpy((void*)msm_dbg_info.rf_cal_version,(void*)nv_item.rf_cal_ver,NV_SIZE_OF_VERSION + 1);
        for(i = 0;i < NV_SIZE_OF_VERSION;i++)
        {
            msm_dbg_info.rf_cal_version[i] = msm_dbg_info.rf_cal_version[i]+ '0';
        }
        msm_dbg_info.rf_cal_version[NV_SIZE_OF_VERSION] = 0;
    }

    return snprintf(buf, PAGE_SIZE, "RF Cal Version:%s\n", msm_dbg_info.rf_cal_version);
}

static ssize_t
show_local_rf_cal_date(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nv_item;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_RF_CAL_DATE_I,&nv_item)) 
    {
        msm_dbg_info.local_rf_cal_date = nv_item.rf_cal_date;
    }
    return snprintf(buf, PAGE_SIZE, "RF Cal Data:%08x\n", (s32)msm_dbg_info.local_rf_cal_date);
}

static ssize_t
show_home_sid_nid(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nv_item;
    nv_item.home_sid_nid.nam = CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_HOME_SID_NID_I ,&nv_item)) 
    {
        memcpy((void*)&msm_dbg_info.home_sid_nid,(void*)&nv_item.home_sid_nid,sizeof(nv_home_sid_nid_type));
    }

    return snprintf(buf, PAGE_SIZE, "home sid: %d, home nid: %d\n", (u16)msm_dbg_info.home_sid_nid.pair[0].sid & 0xFFFF, (u16)msm_dbg_info.home_sid_nid.pair[0].nid & 0xFFFF);
}

static ssize_t
show_lock_sid_nid(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nv_item;
    nv_item.home_sid_nid.nam = CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_SID_NID_LOCK_I ,&nv_item)) //sid_nid_lock.pair
    {
        memcpy((void*)&msm_dbg_info.lock_sid_nid,(void*)&nv_item.sid_nid_lock,sizeof(nv_sid_nid_lock_type));
    }
    return snprintf(buf, PAGE_SIZE, "lock sid: %d, lock nid: %d\n", msm_dbg_info.lock_sid_nid.pair[0].sid, msm_dbg_info.lock_sid_nid.pair[0].nid);
}

static u16 OEMPriv_MCC_TO_DEC(u16 mcc)
{
   static const char mintab[] = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'
   };
   char   txt[4];

   txt[0] = mintab[ mcc/100 ];
   mcc %= 100;
   txt[1] = mintab[ mcc/10 ];
   txt[2] = mintab[ mcc%10 ];
   txt[3] = 0;

   return (uint16) msm_dbg_atoi(txt);
}
static u16 OEMPriv_DEC_TO_MCC(u16 pBuff)
{
   u16       enc_mcc,
                user_mcc,
                digit;


   user_mcc = (u16 ) pBuff;

   /* Encode MCC as per IS-95A 6.3.1.3 */
   digit = (user_mcc / 100) % 10;
   if (0 == digit) {
      digit = 10;
   }
   enc_mcc = digit;

   digit = (user_mcc / 10) % 10;
   if (0 == digit) {
      digit = 10;
   }
   enc_mcc = (uint16) (enc_mcc * 10) + digit;

   digit = user_mcc % 10;
   if (0 == digit) {
      digit = 10;
   }
   enc_mcc = (uint16) (enc_mcc * 10) + digit;

   enc_mcc -= 111;

   //nvi.imsi_mcc.nam = (byte) CM_NAM_1;
   //nvi.imsi_mcc.imsi_mcc = enc_mcc;
   //if (NV_DONE_S != OEMNV_Put(NV_IMSI_MCC_I, &nvi)) {
   //   return EFAILED;
   //}
   return enc_mcc;
}
static ssize_t
show_mcc(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nv_item;
    nv_item.imsi_mcc.nam = CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_IMSI_MCC_I ,&nv_item)) 
    {
        msm_dbg_info.mcc = OEMPriv_MCC_TO_DEC((u16)nv_item.imsi_mcc.imsi_mcc);
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.mcc);
}

static ssize_t
set_mcc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    int value;
    nv_item_type_dbg nv_item;
    nv_item.imsi_mcc.nam = CM_NAM_1;
    sscanf(buf, "%d", &value);
    ddi_ptr->mcc = value;
    nv_item.imsi_mcc.imsi_mcc = OEMPriv_DEC_TO_MCC(value);
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_IMSI_MCC_I ,&nv_item)) 
    {
        //msm_dbg_info.mcc = nv_item.imsi_mcc.imsi_mcc;
    }
    return count;
}

static ssize_t
show_mnc(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nv_item;
    nv_item.imsi_11_12.nam = CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_IMSI_11_12_I ,&nv_item)) 
    {
        //uint16            mnc;
        u16            digit;
        //msm_dbg_info.mnc = nv_item.imsi_11_12.imsi_11_12;

        nv_item.imsi_11_12.imsi_11_12 %= 100;

        digit = (nv_item.imsi_11_12.imsi_11_12 / 10) + 1;
        msm_dbg_info.mnc   = 10 * (digit % 10);

        digit = (nv_item.imsi_11_12.imsi_11_12 % 10) + 1;
        msm_dbg_info.mnc  += (digit % 10);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.mnc);
}
static int OEMPriv_DEC_TO_IMSI_11_12(u16 pBuff)
{
   uint16       user,
                   enc,
                  digit;

   /* Encode IMSI_11_12 as per IS-95A 6.3.1.2 */
   user = (u16 ) pBuff;

   digit = (user / 10) % 10;
   if (0 == digit) {
      digit = 10;
   }
   enc = digit;

   digit = user % 10;
   if (0 == digit) {
      digit = 10;
   }
   enc = (uint16) (enc * 10) + digit;

   enc -= 11;
   enc &= 0x00FF;

   //nvi.imsi_11_12.nam = (byte) CM_NAM_1;
   //nvi.imsi_11_12.imsi_11_12 = (uint8) enc;

   //if (NV_DONE_S != OEMNV_Put(NV_IMSI_11_12_I, &nvi)) {
   //   return EFAILED;
   //}

   return enc;
}
static ssize_t
set_mnc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    int value;
    nv_item_type_dbg nv_item;
    nv_item.imsi_11_12.nam = CM_NAM_1;
    sscanf(buf, "%d", &value);

    ddi_ptr->mnc = value;
    nv_item.imsi_11_12.imsi_11_12 = OEMPriv_DEC_TO_IMSI_11_12(value);
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_IMSI_11_12_I ,&nv_item)) 
    {
        //msm_dbg_info.mcc = nv_item.imsi_mcc.imsi_mcc;
    }
    return count;
}
#define CDMAMIN 1

static uint16 OEMPriv_MIN2_TO_DEC(uint16 min2)
{
   static const char mintab[] = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'
   };
   char   txt[4];

   txt[0] = mintab[ min2/100 ];
   min2 %= 100;
   txt[1] = mintab[ min2/10 ];
   txt[2] = mintab[ min2%10 ];
   txt[3] = 0;

   return (uint16) msm_dbg_atoi(txt);
}
static void OEMPriv_MIN1_TO_STR(uint32  min1,    char   *txt)
{
   static const char mintab[] = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'
   };
   word temp;

   if (min1 == 0) {
      memset(txt, '0', 7);
   } else {
      temp = (word) (min1>>14);
      *txt++ = mintab[temp/100];
      temp %= 100;
      *txt++ = mintab[temp/10];
      *txt++ = mintab[temp%10];

      min1 &= 0x3FFFL;                  /* get bottom 14 bits */
      /* next digit is top 4 bits */
      temp = (word) (( min1 >> 10 ) & 0xF );
      *txt++ = (char) ( ( ( temp == 10 ) ? 0 : temp ) + '0' );
      temp = (word) ( min1 & 0x3FF );   /* get bottom 10 bits */
      *txt++ = mintab[ temp/100 ];
      temp %= 100;
      *txt++ = mintab[ temp/10 ];
      *txt++ = mintab[ temp%10 ];
      *txt = 0;
   }
 }

static ssize_t
show_min(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;      
    nv_item_type_dbg nv_item;
    uint16       min2;
    nv_item.min2.nam = CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_MIN2_I ,&nv_item)) 
    {
        min2 = OEMPriv_MIN2_TO_DEC(nv_item.min2.min2[CDMAMIN]);
        msm_dbg_info.imsi_s[0] = '0' + (char) (min2 / 100);
        msm_dbg_info.imsi_s[1] = '0' + (char) ( (min2 / 10) % 10);
        msm_dbg_info.imsi_s[2] = '0' + (char) (min2 % 10);
    }

    nv_item.min1.nam =   CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_MIN1_I ,&nv_item)) 
    {
        OEMPriv_MIN1_TO_STR(nv_item.min1.min1[CDMAMIN], &msm_dbg_info.imsi_s[3]);
    }
    
    return snprintf(buf, PAGE_SIZE, "%s\n", msm_dbg_info.imsi_s);
}

static ssize_t
set_min(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   nv_item_type_dbg nvi;
   char   local_buf[11];
   char      *imsi;
   uint16       min2;
   uint32       min1;
   uint16       digit;
   int          i = 0;

   memset(local_buf,0,sizeof(local_buf));
   strlcpy(local_buf,buf,sizeof(local_buf));

    imsi = local_buf;
    //pr_err("%s: set_min\n", imsi);
    while (*imsi) 
    {
        if ( (*imsi <  '0') || (*imsi >  '9') ) 
        {
            i ++;
            pr_err("%d %d: imsi\n", *imsi ,i);
            return 0;
        }
        imsi++;
    }

    //
    // Encode the first three digits (IS-95A 6.3.1.1)
    //
    imsi =  local_buf;
    min2 = 0;
    for (i = 0; i < 3; i++) 
    {
        digit = *imsi -  '0';
        if (0 == digit) 
        {
            digit = 10;
        }
        imsi++;

        min2 = (uint16) (min2 * 10) + digit;
    }
    min2 -= 111;

   //
   // Encode the last seven digits (IS-95A 6.3.1.1)
   //

   // Encode the second three digits into the ten most
   // significant bits (of the 24-bit number)...
    min1 = 0;
    for (i = 0; i < 3; i++) 
    {
        digit = *imsi -  '0';
        if (0 == digit) 
        {
            digit = 10;
        }
        imsi++;

        min1 = (uint32) (min1 * 10) + digit;
    }

   min1 -= 111;

   min1 <<= 14;
   min1 &= 0x00FFC000;

   // The fourth last digit is mapping as BCD into four bits
    digit = *imsi -  '0';
    if (0 == digit) 
    {
        digit = 10;
    }
    imsi++;

    min1 = min1 | (0x00003C00 & (digit << 10));

   // Encode the last three digits into the ten least significant bits
   {
        uint32 tmp = 0;

        for (i = 0; i < 3; i++) 
        {
            digit = *imsi -  '0';
            if (0 == digit) 
            {
                digit = 10;
            }
            imsi++;

            tmp = (uint32) (tmp * 10) + digit;
        }

        tmp -= 111;
        tmp &= 0x000003FF;

        min1 |= tmp;
   }

    //
    // Write the encoded values out to NV...
    //
    //pr_err("%d %d: min1 min2\n", min1,min2);

    nvi.min2.nam = (u8) CM_NAM_1;
    nvi.min2.min2[CDMAMIN] = min2;
    if (NV_DONE_S != msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_MIN2_I, &nvi)) 
    {
        return 0;
    }

    nvi.min1.nam = (u8) CM_NAM_1;
    nvi.min1.min1[CDMAMIN] = min1;

    if (NV_DONE_S != msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_MIN1_I, &nvi)) 
    {
        return 0;
    }

    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    //sscanf(buf, "%d%d", &ddi_ptr->min1, &ddi_ptr->min2);
    memcpy((void *)msm_dbg_info.imsi_s,(void *)buf,11);

    return count;
}

static ssize_t
show_pri_chann(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;
    nvi.pcdmach.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_PCDMACH_I ,&nvi)) 
    {
        msm_dbg_info.pcdmach.channel_a = nvi.pcdmach.channel_a;
        msm_dbg_info.pcdmach.channel_b = nvi.pcdmach.channel_b;
    }
    return snprintf(buf, PAGE_SIZE, "%d %d\n", msm_dbg_info.pcdmach.channel_a,msm_dbg_info.pcdmach.channel_b);
}

static ssize_t
set_pri_chann(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;
    nvi.pcdmach.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d%d", &msm_dbg_info.pcdmach.channel_a,&msm_dbg_info.pcdmach.channel_b);
    
    nvi.pcdmach.channel_a = msm_dbg_info.pcdmach.channel_a;
    nvi.pcdmach.channel_b = msm_dbg_info.pcdmach.channel_b;
    
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_PCDMACH_I ,&nvi)) 
    {
    }
    //ddi_ptr->pri_chann.channel_a = value;

    return count;
}

static ssize_t
show_sec_chann(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;

    //return snprintf(buf, PAGE_SIZE, "%d\n", ddi_ptr->pri_chann.channel_b);
    nv_item_type_dbg nvi;
    nvi.scdmach.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_SCDMACH_I ,&nvi)) 
    {
        msm_dbg_info.scdmach.channel_a = nvi.scdmach.channel_a;
        msm_dbg_info.scdmach.channel_b = nvi.scdmach.channel_b;
    }
    return snprintf(buf, PAGE_SIZE, "%d %d\n", msm_dbg_info.scdmach.channel_a,msm_dbg_info.scdmach.channel_b);
}
static ssize_t
set_sec_chann(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;
    nvi.scdmach.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d%d", &msm_dbg_info.scdmach.channel_a,&msm_dbg_info.scdmach.channel_b);

    nvi.scdmach.channel_a = msm_dbg_info.scdmach.channel_a;
    nvi.scdmach.channel_b = msm_dbg_info.scdmach.channel_b;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_SCDMACH_I ,&nvi)) 
    {
    }

    return count;
}
#if 0
static ssize_t
show_sec_chann_A(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "%d\n", ddi_ptr->sec_chann.channel_a);
}
static ssize_t
set_sec_chann_A(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;
	int value;

	sscanf(buf, "%d", &value);

	ddi_ptr->sec_chann.channel_a = value;

      return count;
}

static ssize_t
show_sec_chann_B(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;

	return snprintf(buf, PAGE_SIZE, "%d\n", ddi_ptr->sec_chann.channel_b);
}
static ssize_t
set_sec_chann_B(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct msm_debug_info *ddi_ptr = &msm_dbg_info;
	int value;

	sscanf(buf, "%d", &value);

	ddi_ptr->sec_chann.channel_b = value;

      return count;
}
#endif
static ssize_t
show_prl_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;
    nvi.prl_enabled.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_PRL_ENABLED_I,&nvi)) 
    {
        msm_dbg_info.prl_enable = nvi.prl_enabled.enabled;
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.prl_enable);
}
static ssize_t
set_prl_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    nvi.prl_enabled.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", ((int *) &msm_dbg_info.prl_enable));
    nvi.prl_enabled.enabled = msm_dbg_info.prl_enable;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_PRL_ENABLED_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_home_sid_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;

    nvi.mob_term_home.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_MOB_TERM_HOME_I,&nvi)) 
    {
        msm_dbg_info.home_sid_reg = nvi.mob_term_home.enabled[CDMAMIN];
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.home_sid_reg);
}

static ssize_t
set_home_sid_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    nvi.mob_term_home.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", ((int *)&msm_dbg_info.home_sid_reg));
    nvi.mob_term_home.enabled[CDMAMIN]= msm_dbg_info.home_sid_reg;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_MOB_TERM_HOME_I,&nvi)) 
    {
    }

    return count;
}

static ssize_t
show_foreign_sid_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;

    nvi.mob_term_for_sid.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_MOB_TERM_FOR_SID_I,&nvi)) 
    {
        msm_dbg_info.foreign_sid_reg = nvi.mob_term_for_sid.enabled[CDMAMIN];
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.foreign_sid_reg);
}
static ssize_t
set_foreign_sid_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    nvi.mob_term_home.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *) &msm_dbg_info.foreign_sid_reg);
    nvi.mob_term_for_sid.enabled[CDMAMIN]= msm_dbg_info.foreign_sid_reg;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_MOB_TERM_FOR_SID_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_foreign_nid_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    nv_item_type_dbg nvi;

    nvi.mob_term_for_nid.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_MOB_TERM_FOR_NID_I,&nvi)) 
    {
        msm_dbg_info.foreign_nid_reg = nvi.mob_term_for_nid.enabled[CDMAMIN];
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.foreign_nid_reg);
}
static ssize_t
set_foreign_nid_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    nvi.mob_term_for_nid.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *) &msm_dbg_info.foreign_nid_reg);
    nvi.mob_term_for_nid.enabled[CDMAMIN]= msm_dbg_info.foreign_nid_reg;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_MOB_TERM_FOR_NID_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_overload_class(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    nvi.accolc.nam = (u8) CM_NAM_1;
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_ACCOLC_I,&nvi)) 
    {
        msm_dbg_info.overload_class = nvi.accolc.ACCOLCpClass[CDMAMIN];
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.overload_class);
}
static ssize_t
set_overload_class(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    nvi.accolc.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *) &msm_dbg_info.overload_class);
    nvi.accolc.ACCOLCpClass[CDMAMIN]= msm_dbg_info.overload_class;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_ACCOLC_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_QNC_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_DATA_QNC_ENABLED_I,&nvi)) 
    {
        msm_dbg_info.QNC_enable = nvi.data_qnc_enabled;
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.QNC_enable);
}
static ssize_t
set_QNC_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    //nvi.accolc.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *) &msm_dbg_info.QNC_enable);
    nvi.data_qnc_enabled= msm_dbg_info.QNC_enable;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_DATA_QNC_ENABLED_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_slot_cycle_index(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_SLOT_CYCLE_INDEX_I,&nvi)) 
    {
        msm_dbg_info.slot_cycle_index = nvi.slot_cycle_index;
    }

    return snprintf(buf, PAGE_SIZE, "%d\n",  msm_dbg_info.slot_cycle_index);
}
static ssize_t
set_slot_cycle_index(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    //nvi.accolc.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *)&msm_dbg_info.slot_cycle_index);
    nvi.slot_cycle_index = msm_dbg_info.slot_cycle_index;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_SLOT_CYCLE_INDEX_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_pkt_dial_str(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_DATA_PKT_ORIG_STR_I,&nvi)) 
    {
        memcpy((void *)msm_dbg_info.pkt_dial_str,  (void *) nvi.data_pkt_orig_str.digits,  nvi.data_pkt_orig_str.num_digits);
    }
    return snprintf(buf, PAGE_SIZE, "%s\n", (char *)msm_dbg_info.pkt_dial_str);
}

static ssize_t
set_pkt_dial_str(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    if (count > NV_DATA_DIAL_STR_LEN)  
    {
        memcpy(msm_dbg_info.pkt_dial_str, buf, NV_DATA_DIAL_STR_LEN);    
    }
    else 
    {
        strcpy((char *)msm_dbg_info.pkt_dial_str, buf);
    }
    nvi.data_pkt_orig_str.num_digits = strlen((char *)msm_dbg_info.pkt_dial_str);
    memcpy((void*)nvi.data_pkt_orig_str.digits, (void*)msm_dbg_info.pkt_dial_str, NV_DATA_DIAL_STR_LEN); 
    
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_DATA_PKT_ORIG_STR_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_sec_code_str(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_SEC_CODE_I,&nvi)) 
    {
        memcpy((void *)msm_dbg_info.sec_code_str,  (void *) nvi.sec_code.digits,  sizeof(msm_dbg_info.sec_code_str));
    }
    return snprintf(buf, PAGE_SIZE, "%s\n", (char *)msm_dbg_info.sec_code_str);
}
static ssize_t
set_sec_code_str(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
    nv_item_type_dbg nvi;

    if (count > NV_SEC_CODE_SIZE)  
    {
        memcpy(msm_dbg_info.sec_code_str, buf, NV_SEC_CODE_SIZE);    
    }
    else 
    {
        strcpy((char *)msm_dbg_info.sec_code_str, buf);
    }

    memcpy((void*) nvi.sec_code.digits, (void*)msm_dbg_info.sec_code_str, sizeof(nvi.sec_code.digits)); 

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_SEC_CODE_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_AKEY(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    //not support show a_key
    return 0;
}

static ssize_t
set_AKEY(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct msm_debug_info *ddi_ptr = &msm_dbg_info;
    //int value;

    if (count > AKEY_LEN)  
    {
        memcpy(ddi_ptr->sec_code_str, buf, AKEY_LEN);    
    }
    else 
    {
        strcpy((char *)ddi_ptr->sec_code_str, buf);
    }

    return msm_dbg_auth_cmd_remote(ddi_ptr->sec_code_str,strlen((char *)ddi_ptr->sec_code_str),CM_NAM_1);
    //return count;
}

static ssize_t
show_CDMA_Rx_Diversity(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_CDMA_RX_DIVERSITY_CTRL_I,&nvi)) 
    {
        msm_dbg_info.CDMA_Rx_Diversity = nvi.cdma_rx_diversity_ctrl;
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.CDMA_Rx_Diversity);
}
static ssize_t
set_CDMA_Rx_Diversity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    //nvi.accolc.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *) &msm_dbg_info.CDMA_Rx_Diversity);
    nvi.cdma_rx_diversity_ctrl = msm_dbg_info.CDMA_Rx_Diversity;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_CDMA_RX_DIVERSITY_CTRL_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_HDR_Rx_Diversity(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_HDR_RX_DIVERSITY_CTRL_I,&nvi)) 
    {
        msm_dbg_info.HDR_Rx_Diversity = nvi.hdr_rx_diversity_ctrl;
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", msm_dbg_info.HDR_Rx_Diversity);
}
static ssize_t
set_HDR_Rx_Diversity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    //nvi.accolc.nam = (u8) CM_NAM_1;
    sscanf(buf, "%d", (int *) &msm_dbg_info.HDR_Rx_Diversity);
    nvi.hdr_rx_diversity_ctrl = msm_dbg_info.HDR_Rx_Diversity;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_HDR_RX_DIVERSITY_CTRL_I,&nvi)) 
    {
    }
    return count;
}
static ssize_t
show_meid(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

	nvi.meid[1] = 0;
	nvi.meid[0] = 0;
	
    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_MEID_I,&nvi)) 
    {
        //msm_dbg_info.meid[0]= nvi.meid[0];
        //msm_dbg_info.meid[1]= nvi.meid[1];
    }

    return snprintf(buf, PAGE_SIZE, "%06x%08x\n", nvi.meid[1]  & 0x0FFFFFFFF,nvi.meid[0] & 0x0FFFFFFFF);
}

static ssize_t
show_bd_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_BD_ADDR_I,&nvi)) 
    {
        memcpy((void *)msm_dbg_info.bd_addr.bd_addr,  (void *) nvi.bd_addr.bd_addr,  sizeof(msm_dbg_info.bd_addr.bd_addr));
    }
    return snprintf(buf, PAGE_SIZE, "%x:%x:%x:%x:%x:%x\n", msm_dbg_info.bd_addr.bd_addr[0],msm_dbg_info.bd_addr.bd_addr[1],msm_dbg_info.bd_addr.bd_addr[2],
        msm_dbg_info.bd_addr.bd_addr[3],msm_dbg_info.bd_addr.bd_addr[4],msm_dbg_info.bd_addr.bd_addr[5]);
}

static ssize_t
show_wlan_mac_address(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_WLAN_MAC_ADDRESS_I,&nvi)) 
    {
        memcpy((void *)msm_dbg_info.wlan_mac_address,  (void *) nvi.wlan_mac_address,  sizeof(msm_dbg_info.wlan_mac_address));
    }
    return snprintf(buf, PAGE_SIZE, "%x:%x:%x:%x:%x:%x\n", msm_dbg_info.wlan_mac_address[0],msm_dbg_info.wlan_mac_address[1],msm_dbg_info.wlan_mac_address[2],
        msm_dbg_info.wlan_mac_address[3],msm_dbg_info.wlan_mac_address[4],msm_dbg_info.wlan_mac_address[5]);
}

static ssize_t
set_CDROM_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    nv_item_type_dbg nvi;
    sscanf(buf, "%d", (int *)&msm_dbg_info.cdrom_enable);
    nvi.cdrom_enable= msm_dbg_info.cdrom_enable;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_WRITE_F,NV_CDROM_ENABLE_I,&nvi)) 
    {
    }
    return count;
}

static ssize_t
show_CDROM_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    nv_item_type_dbg nvi;

    if(NV_DONE_S == msm_dbg_nv_cmd_remote(NV_READ_F,NV_CDROM_ENABLE_I,&nvi)) 
    {
        msm_dbg_info.cdrom_enable= nvi.cdrom_enable;
    }

    return snprintf(buf, PAGE_SIZE, "%d\n",  msm_dbg_info.cdrom_enable);
}
static DEVICE_ATTR(rf_card, S_IRUGO|S_IWUSR, show_rf_card, NULL);
static DEVICE_ATTR(nv_band_id, S_IRUGO|S_IWUSR, show_nv_band_id, NULL);
static DEVICE_ATTR(rf_mode, S_IRUGO|S_IWUSR, show_rf_mode, NULL);
static DEVICE_ATTR(pilot_chann, S_IRUGO|S_IWUSR, show_pilot_chann, NULL);
static DEVICE_ATTR(pilot_PN, S_IRUGO|S_IWUSR, show_pilot_PN, NULL);
static DEVICE_ATTR(rx_AGC_dBm, S_IRUGO|S_IWUSR, show_rx_AGC_dBm, NULL);
static DEVICE_ATTR(LNA_state, S_IRUGO|S_IWUSR, show_LNA_state, NULL);
static DEVICE_ATTR(IntelliCeiver_State, S_IRUGO|S_IWUSR, show_IntelliCeiver_State, NULL);
static DEVICE_ATTR(Tx_AGC_dBm, S_IRUGO|S_IWUSR, show_Tx_AGC_dBm, NULL);
static DEVICE_ATTR(PA_State, S_IRUGO|S_IWUSR, show_PA_State, NULL);
static DEVICE_ATTR(rf_HEDT, S_IRUGO|S_IWUSR, show_rf_HEDT, NULL);
static DEVICE_ATTR(vbatt_raw, S_IRUGO|S_IWUSR, show_vbatt_raw, NULL);
static DEVICE_ATTR(fld_version, S_IRUGO|S_IWUSR, show_fld_version, NULL);
static DEVICE_ATTR(rf_PA_THERM, S_IRUGO|S_IWUSR, show_rf_PA_THERM, NULL);
static DEVICE_ATTR(sid, S_IRUGO|S_IWUSR, show_sid, NULL);
static DEVICE_ATTR(nid, S_IRUGO|S_IWUSR, show_nid, NULL);
static DEVICE_ATTR(hdrlog, S_IRUGO|S_IWUSR, show_hdrlog, NULL);
#if 0
static DEVICE_ATTR(chan_num, S_IRUGO|S_IWUSR, show_chan_num, NULL);
static DEVICE_ATTR(band_class, S_IRUGO|S_IWUSR, show_band_class, NULL);
static DEVICE_ATTR(rx_agc0, S_IRUGO|S_IWUSR, show_rx_agc0, NULL);
static DEVICE_ATTR(rx_agc1, S_IRUGO|S_IWUSR, show_rx_agc1, NULL);
static DEVICE_ATTR(tx_agc, S_IRUGO|S_IWUSR, show_tx_agc, NULL);
static DEVICE_ATTR(serving_pn, S_IRUGO|S_IWUSR, show_serving_pn, NULL);
static DEVICE_ATTR(sleep_mode, S_IRUGO|S_IWUSR, show_sleep_mode, NULL);
static DEVICE_ATTR(rel0_sci, S_IRUGO|S_IWUSR, show_rel0_sci, NULL);
static DEVICE_ATTR(relA_sci, S_IRUGO|S_IWUSR, show_relA_sci, NULL);
static DEVICE_ATTR(srch_state, S_IRUGO|S_IWUSR, show_srch_state, NULL);
static DEVICE_ATTR(rx_div, S_IRUGO|S_IWUSR, show_rx_div, NULL);
static DEVICE_ATTR(prot_state, S_IRUGO|S_IWUSR, show_prot_state, NULL);
static DEVICE_ATTR(hdr_session_state, S_IRUGO|S_IWUSR, show_hdr_session_state, NULL);
static DEVICE_ATTR(UATI, S_IRUGO|S_IWUSR, show_UATI, NULL);
static DEVICE_ATTR(color_code, S_IRUGO|S_IWUSR, show_color_code, NULL);
#endif
static DEVICE_ATTR(sw_version, S_IRUGO|S_IWUSR, show_sw_version, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO|S_IWUSR, show_hw_version, NULL);
static DEVICE_ATTR(rf_cal_version, S_IRUGO|S_IWUSR, show_rf_cal_version, NULL);
static DEVICE_ATTR(local_rf_cal_date, S_IRUGO|S_IWUSR, show_local_rf_cal_date, NULL);
static DEVICE_ATTR(home_sid_nid, S_IRUGO|S_IWUSR, show_home_sid_nid, NULL);
static DEVICE_ATTR(lock_sid_nid, S_IRUGO|S_IWUSR, show_lock_sid_nid, NULL);
static DEVICE_ATTR(mcc, S_IRUGO|S_IWUSR, show_mcc, set_mcc);
static DEVICE_ATTR(mnc, S_IRUGO|S_IWUSR, show_mnc, set_mnc);
static DEVICE_ATTR(min, S_IRUGO|S_IWUSR, show_min, set_min);
static DEVICE_ATTR(pri_chann, S_IRUGO|S_IWUSR, show_pri_chann, set_pri_chann);
static DEVICE_ATTR(sec_chann, S_IRUGO|S_IWUSR, show_sec_chann, set_sec_chann);
//static DEVICE_ATTR(sec_chann_A, S_IRUGO|S_IWUSR, show_sec_chann_A, set_sec_chann_A);
//static DEVICE_ATTR(sec_chann_B, S_IRUGO|S_IWUSR, show_sec_chann_B, set_sec_chann_B);
static DEVICE_ATTR(prl_enable, S_IRUGO|S_IWUSR, show_prl_enable, set_prl_enable);
static DEVICE_ATTR(home_sid_reg, S_IRUGO|S_IWUSR, show_home_sid_reg, set_home_sid_reg);
static DEVICE_ATTR(foreign_sid_reg, S_IRUGO|S_IWUSR, show_foreign_sid_reg, set_foreign_sid_reg);
static DEVICE_ATTR(foreign_nid_reg, S_IRUGO|S_IWUSR, show_foreign_nid_reg, set_foreign_nid_reg);
static DEVICE_ATTR(overload_class, S_IRUGO|S_IWUSR, show_overload_class, set_overload_class);
static DEVICE_ATTR(QNC_enable, S_IRUGO|S_IWUSR, show_QNC_enable, set_QNC_enable);
static DEVICE_ATTR(slot_cycle_index, S_IRUGO|S_IWUSR, show_slot_cycle_index, set_slot_cycle_index);
static DEVICE_ATTR(pkt_dial_str, S_IRUGO|S_IWUSR, show_pkt_dial_str, set_pkt_dial_str);
static DEVICE_ATTR(sec_code_str, S_IRUGO|S_IWUSR, show_sec_code_str, set_sec_code_str);
static DEVICE_ATTR(AKEY, S_IRUGO|S_IWUSR, show_AKEY, set_AKEY);
static DEVICE_ATTR(CDMA_Rx_Diversity, S_IRUGO|S_IWUSR, show_CDMA_Rx_Diversity, set_CDMA_Rx_Diversity);
static DEVICE_ATTR(HDR_Rx_Diversity, S_IRUGO|S_IWUSR, show_HDR_Rx_Diversity, set_HDR_Rx_Diversity);
static DEVICE_ATTR(meid, S_IRUGO|S_IWUSR, show_meid, NULL);
// static DEVICE_ATTR(esn_compare, S_IRUGO|S_IWUSR, show_esn_compare, NULL);
static DEVICE_ATTR(bd_addr, S_IRUGO|S_IWUSR, show_bd_addr, NULL);
static DEVICE_ATTR(wlan_mac_address, S_IRUGO|S_IWUSR, show_wlan_mac_address, NULL);
static DEVICE_ATTR(CDROM_enable, S_IRUGO|S_IWUSR, show_CDROM_enable, set_CDROM_enable);
static struct attribute *dev_attrs[] = {
	&dev_attr_rf_card.attr,
	&dev_attr_nv_band_id.attr,
	&dev_attr_rf_mode.attr,
	&dev_attr_pilot_chann.attr,
	&dev_attr_pilot_PN.attr,
	&dev_attr_rx_AGC_dBm.attr,
	&dev_attr_LNA_state.attr,
	&dev_attr_IntelliCeiver_State.attr,
	&dev_attr_Tx_AGC_dBm.attr,
	&dev_attr_PA_State.attr,
	&dev_attr_rf_HEDT.attr,	
	&dev_attr_vbatt_raw.attr,
	&dev_attr_fld_version.attr,
	&dev_attr_rf_PA_THERM.attr,
	&dev_attr_sid.attr,
	&dev_attr_nid.attr,
	&dev_attr_hdrlog.attr,
#if 0
	&dev_attr_chan_num.attr,
	&dev_attr_band_class.attr,
	&dev_attr_rx_agc0.attr,
	&dev_attr_rx_agc1.attr,
	&dev_attr_tx_agc.attr,
	&dev_attr_serving_pn.attr,
	&dev_attr_sleep_mode.attr,
	&dev_attr_rel0_sci.attr,
	&dev_attr_relA_sci.attr,
	&dev_attr_srch_state.attr,
	&dev_attr_rx_div.attr,
	&dev_attr_prot_state.attr,
	&dev_attr_hdr_session_state.attr,
	&dev_attr_UATI.attr,
	&dev_attr_color_code.attr,
#endif
	&dev_attr_sw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_rf_cal_version.attr,
	&dev_attr_local_rf_cal_date.attr,
	&dev_attr_home_sid_nid.attr,
	&dev_attr_lock_sid_nid.attr,
	&dev_attr_mcc.attr,
	&dev_attr_mnc.attr,
	&dev_attr_min.attr,
	&dev_attr_pri_chann.attr,
	&dev_attr_sec_chann.attr,
	//&dev_attr_sec_chann_A.attr,
	//&dev_attr_sec_chann_B.attr,
	&dev_attr_prl_enable.attr,
	&dev_attr_home_sid_reg.attr,
	&dev_attr_foreign_sid_reg.attr,
	&dev_attr_foreign_nid_reg.attr,
	&dev_attr_overload_class.attr,
	&dev_attr_QNC_enable.attr,
	&dev_attr_slot_cycle_index.attr,
	&dev_attr_pkt_dial_str.attr,
	&dev_attr_sec_code_str.attr,
	&dev_attr_AKEY.attr,
	&dev_attr_CDMA_Rx_Diversity.attr,
	&dev_attr_HDR_Rx_Diversity.attr,
	&dev_attr_meid.attr,
#if 0	
	&dev_attr_esn_compare.attr,
#endif	
	&dev_attr_bd_addr.attr,
	&dev_attr_wlan_mac_address.attr,
	&dev_attr_CDROM_enable.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};


static int msm_dbg_info_cleanup(void)
{
	int rc = 0;

	if (msm_dbg_info.nv_client)
		msm_rpc_unregister_client(msm_dbg_info.nv_client);

	if (msm_dbg_info.auth_client)
		msm_rpc_unregister_client(msm_dbg_info.auth_client);

	if (msm_dbg_info.hs_ept) {
		rc = msm_rpc_close(msm_dbg_info.hs_ept);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(hs_ept). rc=%d\n",
			       __func__, rc);
		}
	}

	return rc;
}

static int __devinit msm_dbg_info_probe(struct platform_device *pdev)
{
	int rc;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" debug info device ", __func__);
		return -EINVAL;
	}

      rc = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
      if (rc) {
	      msm_dbg_info_cleanup();
            return rc;
      }

	return 0;
}

static int __devexit msm_dbg_info_remove(struct platform_device *pdev)
{
	int rc;

      sysfs_remove_group(&pdev->dev.kobj, &dev_attr_grp);
   
	rc = msm_dbg_info_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_dbg_info_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static struct platform_driver msm_dbg_info_driver = {
	.probe = msm_dbg_info_probe,
	.remove = __devexit_p(msm_dbg_info_remove),
	.driver = {
		   .name = "msm-dbg-info",
		   .owner = THIS_MODULE,
		   },
};

static int __devinit msm_dbg_info_init_rpc(void)
{
	int rc;

	msm_dbg_info.hs_ept =
		msm_rpc_connect_compatible(HS_REMPROG, HS_REMVERS, 0);
	if (msm_dbg_info.hs_ept == NULL) {
		pr_err("%s: rpc connect HS_REMPROG = NULL\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(msm_dbg_info.hs_ept)) {
		rc = PTR_ERR(msm_dbg_info.hs_ept);
		pr_err("%s: FAIL: rpc connect for HS_REMPROG. rc=%d\n",
		       __func__, rc);
		msm_dbg_info.hs_ept = NULL;
		return rc;
	}

	msm_dbg_info.nv_client =
		msm_rpc_register_client("nv_client", NVPROG,
					NVVERS,
					0, NULL);

	if (IS_ERR(msm_dbg_info.nv_client)) {
		rc = PTR_ERR(msm_dbg_info.nv_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_dbg_info.nv_client = NULL;
		return rc;
	}

	msm_dbg_info.auth_client =
		msm_rpc_register_client("auth_client", AUTHPROG,
					AUTHVERS,
					0, NULL);

	if (IS_ERR(msm_dbg_info.auth_client)) {
		rc = PTR_ERR(msm_dbg_info.auth_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_dbg_info.auth_client = NULL;
		return rc;
	}
   
	rc = platform_driver_register(&msm_dbg_info_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);

	msm_dbg_info.hs_api_version =  HS_REMVERS;
	msm_dbg_info.nv_api_version =  NVVERS;
	msm_dbg_info.auth_api_version =  AUTHVERS;

	return rc;
}

static int __init msm_dbg_info_init(void)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	rc = msm_dbg_info_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_dbg_info_init_rpc.  rc=%d\n", __func__, rc);
		msm_dbg_info_cleanup();
		return rc;
	}

	pr_info("%s: Debug information = 0x%08x/0x%08x/0x%08x (RPC version)\n",
		__func__, 
		msm_dbg_info.hs_api_version,
		msm_dbg_info.nv_api_version,
		msm_dbg_info.auth_api_version);

	return 0;
}

static void __exit msm_dbg_info_exit(void)
{
	platform_driver_unregister(&msm_dbg_info_driver);
}

// PATH: /sys/devices/platform/msm-dbg-info


module_init(msm_dbg_info_init);
module_exit(msm_dbg_info_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Debug information driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_dbg_info");
