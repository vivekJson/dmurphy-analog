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

#ifndef __USB_PD_PROTOCOL_H__
#define __USB_PD_PROTOCOL_H__

#include "tusb422_common.h"
#include "tcpci.h"
#include "usb_pd.h"

typedef enum
{
    PRL_ALERT_MSG_RECEIVED = 0,
    PRL_ALERT_MSG_TX_SUCCESS,
    PRL_ALERT_MSG_TX_DISCARDED,
    PRL_ALERT_MSG_TX_FAILED,
    PRL_ALERT_NO_RESPONSE_TIMEOUT,  /* no response after hard reset */
    PRL_ALERT_SENDER_RESPONSE_TIMEOUT,
    PRL_ALERT_VOLTAGE_ALARM_HI,
    PRL_ALERT_VOLTAGE_ALARM_LO,
    PRL_ALERT_HARD_RESET_RECEIVED
} usb_pd_prl_alert_t;


//#define PRL_RX_DATA_MSG_OFFSET  0x80
//
//typedef enum
//{
//    PRL_RX_MSG_GOTO_MIN        =  CTRL_MSG_TYPE_GOTO_MIN,        /* SOP only */
//    PRL_RX_MSG_ACCEPT          =  CTRL_MSG_TYPE_ACCEPT,          
//    PRL_RX_MSG_REJECT          =  CTRL_MSG_TYPE_REJECT,          /* SOP only */
//    PRL_RX_MSG_PING            =  CTRL_MSG_TYPE_PING,            /* SOP only */
//    PRL_RX_MSG_PS_RDY          =  CTRL_MSG_TYPE_PS_RDY,          /* SOP only */
//    PRL_RX_MSG_GET_SRC_CAP     =  CTRL_MSG_TYPE_GET_SRC_CAP,     /* SOP only */
//    PRL_RX_MSG_GET_SNK_CAP     =  CTRL_MSG_TYPE_GET_SNK_CAP,     /* SOP only */
//    PRL_RX_MSG_DR_SWAP         =  CTRL_MSG_TYPE_DR_SWAP,         /* SOP only */
//    PRL_RX_MSG_PR_SWAP         =  CTRL_MSG_TYPE_PR_SWAP,         /* SOP only */
//    PRL_RX_MSG_VCONN_SWAP      =  CTRL_MSG_TYPE_VCONN_SWAP,      /* SOP only */
//    PRL_RX_MSG_WAIT            =  CTRL_MSG_TYPE_WAIT,            /* SOP only */
//    PRL_RX_MSG_SOFT_RESET      =  CTRL_MSG_TYPE_SOFT_RESET,      
//    PRL_RX_MSG_NOT_SUPPORTED   =  CTRL_MSG_TYPE_NOT_SUPPORTED,   
//    PRL_RX_MSG_GET_SRC_CAP_EXT =  CTRL_MSG_TYPE_GET_SRC_CAP_EXT, /* SOP only */
//    PRL_RX_MSG_GET_STATUS      =  CTRL_MSG_TYPE_GET_STATUS,      /* SOP only */
//    PRL_RX_MSG_FR_SWAP         =  CTRL_MSG_TYPE_FR_SWAP,          /* SOP only */
//
//    /* Data Msgs */
//    PRL_RX_MSG_SRC_CAPS        =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_SRC_CAPS),   
//    PRL_RX_MSG_REQUEST         =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_REQUEST),    
//    PRL_RX_MSG_BIST            =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_BIST),       
//    PRL_RX_MSG_SNK_CAPS        =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_SNK_CAPS),   
//    PRL_RX_MSG_BATT_STATUS     =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_BATT_STATUS),
//    PRL_RX_MSG_ALERT           =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_ALERT),      
//    PRL_RX_MSG_VENDOR          =  (PRL_RX_DATA_MSG_OFFSET | DATA_MSG_TYPE_VENDOR),     
//        
//} usb_pd_prl_rx_msg_t; 


void usb_pd_prl_tx_ctrl_msg(unsigned int port, uint8_t *buf, msg_hdr_ctrl_msg_type_t msg_type, tcpc_transmit_t sop_type);

void usb_pd_prl_tx_data_msg(unsigned int port, uint8_t *buf, msg_hdr_data_msg_type_t msg_type, tcpc_transmit_t sop_type, unsigned int ndo);

void usb_pd_prl_reset(unsigned int port);

void usb_pd_prl_init(void);

#endif
