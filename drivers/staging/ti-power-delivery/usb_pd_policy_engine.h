/*
 * Texas Instruments TUSB422 Power Delivery
 *
 * Author: 
 * Copyright: (C) 2016 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __USB_PD_POLICY_ENGINE_H__
#define __USB_PD_POLICY_ENGINE_H__

#include "tusb422_common.h"
#include "tcpci.h"

#define MAX_EXT_MSG_LEN 260 /* 260-bytes */

#define MAX_SOP_NUM 5     /* SOP, SOP', SOP", SOP_DBG', SOP_DBG" */

#define PD_STATE_HISTORY_LEN  8  
#define PD_STATE_INDEX_MASK   0x7  /* bitmask based on history length */

typedef enum
{
    PDO_IDX_5V = 0,
    PDO_IDX_HI_VOLT
} pdo_idx_t;


typedef enum
{
    /* Source states */
    PE_SRC_STARTUP = 0,
    PE_SRC_DISCOVERY,
    PE_SRC_SEND_CAPS,
    PE_SRC_NEGOTIATE_CAPABILITY,
    PE_SRC_TRANSITION_SUPPLY,
    PE_SRC_TRANSITION_PS,
    PE_SRC_TRANSITION_PS_EXIT,
    PE_SRC_READY,
    PE_SRC_DISABLED,
    PE_SRC_CAPABILITY_RESPONSE,
    PE_SRC_HARD_RESET,
    PE_SRC_HARD_RESET_RECEIVED,
    PE_SRC_TRANSITION_TO_DEFAULT,
    PE_SRC_TRANSITION_TO_DEFAULT_EXIT,
    PE_SRC_GET_SINK_CAP,
    PE_SRC_WAIT_NEW_CAPS,
    PE_SRC_SEND_SOFT_RESET,
    PE_SRC_SOFT_RESET,
    PE_SRC_SEND_NOT_SUPPORTED,
    PE_SRC_NOT_SUPPORTED_RECEIVED,
    PE_SRC_PING,
    PE_SRC_SEND_SOURCE_ALERT,
    PE_SRC_SINK_ALERT_RECEIVED,
    PE_SRC_GIVE_SOURCE_CAP_EXT,
    PE_SRC_GIVE_SOURCE_STATUS,
    PE_SRC_GET_SINK_STATUS,

    /* Sink states */
    PE_SNK_STARTUP,
    PE_SNK_DISCOVERY,
    PE_SNK_WAIT_FOR_CAPS,
    PE_SNK_EVALUATE_CAPABILITY,
    PE_SNK_SELECT_CAPABILITY,
    PE_SNK_TRANSITION_SINK,
    PE_SNK_READY,
    PE_SNK_HARD_RESET,
    PE_SNK_TRANSITION_TO_DEFAULT,
    PE_SNK_GIVE_SINK_CAP,
    PE_SNK_GET_SOURCE_CAP,
    PE_SNK_SEND_SOFT_RESET,
    PE_SNK_SOFT_RESET,
    PE_SNK_SEND_NOT_SUPPORTED,
    PE_SNK_NOT_SUPPORTED_RECEIVED,
    PE_SNK_SOURCE_ALERT_RECEIVED,
    PE_SNK_SEND_SINK_ALERT,
    PE_SNK_GET_SOURCE_CAP_EXT,
    PE_SNK_GET_SOURCE_STATUS,
    PE_SNK_GIVE_SINK_STATUS,

    PE_BIST_CARRIER_MODE,
    PE_BIST_TEST_MODE,

    PE_DRS_DFP_UFP_REJECT_SWAP,
    PE_DRS_UFP_DFP_REJECT_SWAP,

    PE_PRS_SRC_SNK_REJECT_SWAP,
    PE_PRS_SNK_SRC_REJECT_SWAP,

    PE_NUM_STATES
} usb_pd_pe_state_t;


//typedef enum
//{
//    // BQ - maybe these can be bool in struct below.
//    PD_FLAGS_DUAL_ROLE_POWER = (0 << 0), 
//    PD_FLAGS_DUAL_ROLE_DATA  = (0 << 1), 
//} pd_flags_t;

typedef struct
{
    usb_pd_pe_state_t   state[PD_STATE_HISTORY_LEN];
    unsigned int        state_idx;
    usb_pd_pe_state_t   *current_state;
    bool                state_change;
                        
//    uint8_t             flags;
    unsigned int        port;
    struct tusb422_timer_t             timer;
    struct tusb422_timer_t             timer2;

    uint8_t             power_role;      
    uint8_t             data_role;
                        
    tcpc_transmit_t     tx_sop_type;           /* For incrementing correct msg ID when Tx is successful */
    uint8_t             msg_id[MAX_SOP_NUM];   /* For Tx.  Masked off to 3-bits when building msg header */
                        
    uint8_t             stored_msg_id[MAX_SOP_NUM];   /* For Rx */
    uint8_t             rx_msg_buf[MAX_EXT_MSG_LEN];  
    uint8_t             rx_msg_data_len;
    uint8_t             rx_msg_type;

    uint8_t             hard_reset_cnt;
    uint8_t             caps_cnt;

    uint8_t             discover_identity_cnt;   // When sending Discover Identity to cable plug.  Max 20.  Zero upon data role swap.
    uint8_t             vdm_busy_cnt;  // Max of 1 responder busy.  Reset upon non-busy response.

    bool                non_interruptable_ams;
    bool                power_role_swap;
    bool                pd_connected_since_attach;      /* PD connected at any point since attachment */
    bool                explicit_contract;
    bool                no_response_timed_out;

    uint8_t             object_position;  /* Range: 1 - 7 */
    uint16_t            min_voltage;  /* 25mV LSB */
    bool                request_goto_min;
    bool                wait_received;

    uint32_t            src_pdo[PD_MAX_PDO_NUM];
    uint32_t            snk_pdo[PD_MAX_PDO_NUM];

    uint16_t            src_settling_time;

    bool                high_pwr_cable;  /* If using cable with 5A support */
} usb_pd_port_t;

typedef enum
{
    STATUS_OK = 0,
    STATUS_REQUEST_NOT_SUPPORTED_IN_CURRENT_STATE
} usb_pd_pe_status_t;

typedef enum
{
    PD_POLICY_MNGR_REQ_GET_SINK_CAPS,
    PD_POLICY_MNGR_REQ_SRC_CAPS_CHANGE,
    PD_POLICY_MNGR_REQ_GOTO_MIN,
    PD_POLICY_MNGR_REQ_UPDATE_REMOTE_CAPS,
} pd_policy_manager_request_t;

//typedef enum
//{
//    PD_FLAG_USB_COMM_CAPABLE = (1 << 0),
//    
//} pd_flags_t;



int usb_pd_policy_manager_request(unsigned int port, pd_policy_manager_request_t req);
usb_pd_port_t * usb_pd_pe_get_device(unsigned int port);

void usb_pd_pe_init(unsigned int port, usb_pd_port_config_t *config);

#endif
