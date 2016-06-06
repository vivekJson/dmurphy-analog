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

#ifndef __TCPM_H__
#define __TCPM_H__

#include "tusb422_common.h"
#include "tcpci.h"

/*
* Macros and definitions
*/

 #define NUM_TCPC_DEVICES   1


/* Sink with Accessory Support is NOT supported by the state machine */

typedef enum
{
    TCPC_STATE_UNATTACHED_SRC = 0,
    TCPC_STATE_UNATTACHED_SNK,
    TCPC_STATE_ATTACH_WAIT_SRC,
    TCPC_STATE_ATTACH_WAIT_SNK,
    TCPC_STATE_ATTACH_WAIT_SNK_WAIT4VBUS,
    TCPC_STATE_WAITING_FOR_VBUS_SNK,
    TCPC_STATE_TRY_SNK,
    TCPC_STATE_TRY_SNK_LOOK4SRC,
    TCPC_STATE_TRY_SRC,
    TCPC_STATE_TRY_WAIT_SRC,
    TCPC_STATE_TRY_WAIT_SNK,
    TCPC_STATE_ATTACHED_SRC,
    TCPC_STATE_ATTACHED_SNK,
    TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC,
    TCPC_STATE_ORIENTED_DEBUG_ACC_SRC,
    TCPC_STATE_DEBUG_ACC_SNK,
    TCPC_STATE_AUDIO_ACC,
    TCPC_STATE_ERROR_RECOVERY,    
    TCPC_STATE_DISABLED  /* no CC terminations */
} tcpc_state_t;


typedef enum
{
    TC_FLAGS_TRY_SRC        = (1 << 0),   /* Either Try.SRC or Try.SNK support but not both */
    TC_FLAGS_TRY_SNK        = (1 << 1),   /* Either Try.SRC or Try.SNK support but not both */
} tc_flags_t;


typedef enum
{
    PLUG_UNFLIPPED = 0,  /* USB-PD comm on CC1 */
    PLUG_FLIPPED,        /* USB-PD comm on CC2 */
} plug_polarity_t;


//typedef enum 
//{
//	CC_STATE_OPEN        = 0,
//	CC_STATE_SRC_RA      = 1,
//	CC_STATE_SRC_RD      = 2,
//	CC_STATE_SNK_DEFAULT = (1 | CC_STATUS_CONNECT_RESULT), /* bit 2 is set if TCPC is presenting Rd */
//	CC_STATE_SNK_1P5A    = (2 | CC_STATUS_CONNECT_RESULT), /* bit 2 is set if TCPC is presenting Rd */
//	CC_STATE_SNK_3P0A    = (3 | CC_STATUS_CONNECT_RESULT), /* bit 2 is set if TCPC is presenting Rd */
//} cc_state_t;


typedef struct 
{
//    smbus_interface_t  intf;
//    uint8_t            slave_addr;
    uint8_t            port;
    tcpc_state_t       state;
    tcpc_state_t       last_state;    /* for debug */
    bool               state_change;

    struct tusb422_timer_t timer;

    bool               vbus_present;
//    uint8_t            connect_result;  /* 0 = SRC, SNK otherwise */

    tc_role_t          role;
    tcpc_role_rp_val_t rp_val;

    uint16_t           flags;
//    bool               try_src;
//    bool               try_snk;

    uint8_t            cc_status;

    plug_polarity_t    plug_polarity;
//    uint8_t            cc1_state;
//    uint8_t            cc2_state;
    uint8_t            silicon_revision;
} tcpc_device_t;

typedef struct
{
    uint16_t      IE;
} tUSBPD_Intf_Context;

typedef enum
{
    TCPM_STATUS_OK = 0,
    TCPM_STATUS_SMBUS_ERROR,
    TCPM_STATUS_TIMEOUT,
    TCPM_STATUS_PARAM_ERROR

} tcpm_status_t;


typedef enum
{
    TX_STATUS_SUCCESS = 0,
    TX_STATUS_DISCARDED,
    TX_STATUS_FAILED
} tx_status_t;


typedef struct
{
    void (*conn_state_change_cbk)(unsigned int port, tcpc_state_t state);
    void (*current_change_cbk)(unsigned int port, tcpc_cc_snk_state_t cc_state);
    void (*volt_alarm_cbk)(unsigned int port, bool hi_volt);
    void (*pd_hard_reset_cbk)(unsigned int port);
    void (*pd_transmit_cbk)(unsigned int port, tx_status_t tx_status);
    void (*pd_receive_cbk)(unsigned int port);

} tcpm_callbacks_t;

//*****************************************************************************
//
//! \brief   Initializes the SMBus interface(s) used to talk to TCPCs.  
//!          
//!
//! \param   none
//!
//! \return  USB_PD_RET_OK
//
// *****************************************************************************
void tcpm_register_callbacks(const tcpm_callbacks_t *callbacks);

void tcpm_get_msg_header_type(unsigned int port, uint8_t *frame_type, uint16_t *header);
void tcpm_read_message(unsigned int port, uint8_t *buf, uint8_t len);
void tcpm_transmit(unsigned int port, uint8_t *buf, tcpc_transmit_t sop_type);

void tcpm_enable_pd_receive(unsigned int port);
void tcpm_disable_pd_receive(unsigned int port);

void tcpm_set_voltage_alarm_lo(unsigned int port, uint16_t threshold_25mv);
void tcpm_set_voltage_alarm_hi(unsigned int port, uint16_t threshold_25mv);

bool tcpm_is_vbus_present(unsigned int port);
uint16_t tcpm_get_vbus_voltage(unsigned int port);

void tcpm_set_autodischarge_disconnect(unsigned int port, bool enable);
void tcpm_set_sink_disconnect_threshold(unsigned int port, uint16_t threshold_25mv);
void tcpm_force_discharge(unsigned int port, uint16_t threshold_25mv);

void tcpm_error_recovery(unsigned int port);

void tcpm_src_vbus_disable(unsigned int port);
void tcpm_src_vbus_5v_enable(unsigned int port);
void tcpm_src_vbus_hi_volt_enable(unsigned int port);

void tcpm_snk_vbus_enable(unsigned int port);
void tcpm_snk_vbus_disable(unsigned int port);

void tcpm_vconn_control(unsigned int port, bool enable);
void tcpm_remove_rp_from_vconn_pin(unsigned int port);

void tcpm_set_bist_test_mode(unsigned int port);

tcpc_device_t* tcpm_get_device(unsigned int port);

void tcpm_cc_pin_control(unsigned int port, tc_role_t role);

#endif //__TCPM_H__

