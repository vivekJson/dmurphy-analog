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

#include "usb_pd_policy_engine.h"
#include "tcpm.h"
#include "tusb422.h"
#include "usb_pd.h"
#include "usb_pd_protocol.h"

#define TUSB422_TIMER

/* PD Counter */
#define N_CAPS_COUNT        50
#define N_HARD_RESET_COUNT  2
#define N_DISCOVER_IDENTITY_COUNT 20

/* PD Time Values */
#define T_NO_RESPONSE_MS            5000   /* 4.5 - 5.5 s */
#define T_SENDER_RESPONSE_MS          25   /*  24 - 30 ms */
#define T_SWAP_SOURCE_START_MS        30   /*  20 - ? ms*/
#define T_TYPEC_SEND_SOURCE_CAP_MS   150   /* 100 - 200 ms */
#define T_TYPEC_SINK_WAIT_CAP_MS     500   /* 310 - 620 ms */
#define T_PS_HARD_RESET_MS            25   /* 25 - 35 ms */
#define T_PS_TRANSITION_MS           500   /* 450 - 550 ms */
#define T_SRC_RECOVER_MS             800   /* 660 - 1000 ms */
#define T_SRC_TRANSITION_MS           25   /* 25 - 35 ms */
#define T_SINK_REQUEST_MS            200   /* 100 - ? ms */
#define T_BIST_CONT_MODE_MS           50   /* 30 - 60 ms */
#define T_SRC_TURN_ON_MS             275   /* tSrcTurnOn: 0 - 275ms */
#define T_SRC_TURN_OFF_MS            275   /* tSrcSettle: 0 - 275ms */


usb_pd_port_t pd[NUM_TCPC_DEVICES];
uint8_t buf[32];

static bool vbus_timed_out;

extern uint32_t rdo;

extern void usb_pd_pm_evaluate_src_caps(unsigned int port);
extern usb_pd_port_config_t* usb_pd_pm_get_config(unsigned int port);
extern void build_rdo(unsigned int port);
extern uint32_t get_data_object(uint8_t *obj_data);


#if DEBUG_LEVEL >= 1
static const char *state2string[PE_NUM_STATES]=
{
    /* Source states */
    "SRC_STARTUP",
    "SRC_DISCOVERY",
    "SRC_SEND_CAPS",
    "SRC_NEGOTIATE_CAPABILITY",
    "SRC_TRANSITION_SUPPLY",
    "SRC_TRANSITION_PS",
    "SRC_TRANSITION_PS_EXIT",
    "SRC_READY",
    "SRC_DISABLED",
    "SRC_CAPABILITY_RESPONSE",
    "SRC_HARD_RESET",
    "SRC_HARD_RESET_RECEIVED",
    "SRC_TRANSITION_TO_DEFAULT",
    "SRC_TRANSITION_TO_DEFAULT_EXIT",
    "SRC_GET_SINK_CAP",
    "SRC_WAIT_NEW_CAPS",
    "SRC_SEND_SOFT_RESET",
    "SRC_SOFT_RESET",
    "SRC_SEND_NOT_SUPPORTED",
    "SRC_NOT_SUPPORTED_RECEIVED",
    "SRC_PING",
    "SRC_SEND_SOURCE_ALERT",
    "SRC_SINK_ALERT_RECEIVED",
    "SRC_GIVE_SOURCE_CAP_EXT",
    "SRC_GIVE_SOURCE_STATUS",
    "SRC_GET_SINK_STATUS",

    /* Sink states */
    "SNK_STARTUP",
    "SNK_DISCOVERY",
    "SNK_WAIT_FOR_CAPS",
    "SNK_EVALUATE_CAPABILITY",
    "SNK_SELECT_CAPABILITY",
    "SNK_TRANSITION_SINK",
    "SNK_READY",
    "SNK_HARD_RESET",
    "SNK_TRANSITION_TO_DEFAULT",
    "SNK_GIVE_SINK_CAP",
    "SNK_GET_SOURCE_CAP",
    "SNK_SEND_SOFT_RESET",
    "SNK_SOFT_RESET",
    "SNK_SEND_NOT_SUPPORTED",
    "SNK_NOT_SUPPORTED_RECEIVED",
    "SNK_SOURCE_ALERT_RECEIVED",
    "SNK_SEND_SINK_ALERT",
    "SNK_GET_SOURCE_CAP_EXT",
    "SNK_GET_SOURCE_STATUS",
    "SNK_GIVE_SINK_STATUS",

    "BIST_CARRIER_MODE",
    "BIST_TEST_MODE",

    "DRS_DFP_UFP_REJECT_SWAP",
    "DRS_UFP_DFP_REJECT_SWAP",

    "PRS_SRC_SNK_REJECT_SWAP",
    "PRS_SNK_SRC_REJECT_SWAP",

};

void pe_debug_state_history(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];
    uint8_t i;

    CRIT("\nPolicy Engine State History:\n");
    for (i = 0; i < PD_STATE_HISTORY_LEN; i++)
    {
        if (dev->state[i] < PE_NUM_STATES)
        {
            CRIT("%s[%u] %s\n", (&dev->state[i] == dev->current_state) ? "->" : "  ", i, state2string[dev->state[i]]);
        }
    }

    return;
}
#endif


static void pe_set_state(usb_pd_port_t *dev, usb_pd_pe_state_t new_state)
{
    CRIT("PE_%s\n", state2string[new_state]);

    dev->state[dev->state_idx] = new_state;
    dev->current_state = &dev->state[dev->state_idx];
    dev->state_idx++;
    dev->state_idx &= PD_STATE_INDEX_MASK;
    dev->state_change = true;

    return;
}

static void timeout_no_response(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    DEBUG("timeout_no_response\n");

    dev->no_response_timed_out = true;

    if (dev->hard_reset_cnt > N_HARD_RESET_COUNT)
    {
        if (!dev->pd_connected_since_attach)
        {
            if ((*dev->current_state == PE_SRC_SEND_CAPS) ||
                ((*dev->current_state == PE_SRC_DISCOVERY) && (dev->caps_cnt > N_CAPS_COUNT)))
            {
                pe_set_state(dev, PE_SRC_DISABLED);
            }
        }
        else /* previously PD connected */
        {
            tcpm_error_recovery(port);
        }
    }

    return;
}

static void timer_start_no_response(usb_pd_port_t *dev)
{
    INFO("timer_start_no_response\n");

    dev->no_response_timed_out = false;

#ifdef TUSB422_TIMER
    tusb422_lfo_timer_start(&dev->timer2, T_NO_RESPONSE_MS, timeout_no_response);
#else
    timer_start(&dev->timer2, T_NO_RESPONSE_MS, timeout_no_response);
#endif

    return;
}

static void timer_cancel_no_response(usb_pd_port_t *dev)
{
//    dev->no_response_timed_out = false;

    INFO("timer_cancel_no_response\n");
#ifdef TUSB422_TIMER
    tusb422_lfo_timer_cancel(&dev->timer2);
#else
    timer_cancel(&dev->timer2);
#endif

    return;
}


usb_pd_port_t * usb_pd_pe_get_device(unsigned int port)
{
    return &pd[port];
}


void usb_pd_pe_init(unsigned int port, usb_pd_port_config_t *config)
{
    usb_pd_port_t *dev = &pd[port];
    unsigned int i;

    pd[port].state_idx = 0;

    for (i = 0; i < PD_STATE_HISTORY_LEN; i++)
    {
        pd[port].state[i] = (usb_pd_pe_state_t)0xEE;
    }

    dev->current_state = &dev->state[0];

    dev->port = port;
    dev->state_change = false;
    dev->timer.data = port;
    dev->timer2.data = port;

    dev->power_role_swap = false;
    dev->pd_connected_since_attach = false;
    dev->explicit_contract = false;
    dev->no_response_timed_out = false;

    dev->request_goto_min = false;
    dev->hard_reset_cnt = 0;

    dev->src_settling_time = config->src_settling_time_ms;
    printk("dev->src_settling_time = 0x%x\n", dev->src_settling_time);
    dev->high_pwr_cable = false;

    return;
}


void usb_pd_pe_connection_state_change_handler(unsigned int port, tcpc_state_t state)
{
    usb_pd_port_t *dev = &pd[port];

    switch (state)
    {
        case TCPC_STATE_UNATTACHED_SRC:
        case TCPC_STATE_UNATTACHED_SNK:
            dev->power_role_swap = false;
            dev->pd_connected_since_attach = false;
            dev->explicit_contract = false;
            dev->no_response_timed_out = false;

            dev->request_goto_min = false;
            dev->wait_received = false;
            dev->hard_reset_cnt = 0;

            dev->high_pwr_cable = false;
            dev->non_interruptable_ams = false;

            timer_cancel_no_response(dev);
            timer_cancel(&dev->timer);
            break;

        case TCPC_STATE_ATTACHED_SRC:
            dev->data_role = PD_DATA_ROLE_DFP;
            pe_set_state(dev, PE_SRC_STARTUP);
//            tcpm_enable_pd_receive(port);
            break;

        case TCPC_STATE_ATTACHED_SNK:
            dev->data_role = PD_DATA_ROLE_UFP;
            pe_set_state(dev, PE_SNK_STARTUP);
//            tcpm_enable_pd_receive(port);
            break;

        case TCPC_STATE_ORIENTED_DEBUG_ACC_SRC:
        case TCPC_STATE_DEBUG_ACC_SNK:
            break;

        case TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC:
        case TCPC_STATE_AUDIO_ACC:
            // Do nothing.
            break;

        default:
            break;
    }

    return;
}

static void usb_pd_pe_tx_data_msg(unsigned int port, msg_hdr_data_msg_type_t msg_type, tcpc_transmit_t sop_type)
{
    usb_pd_port_t *dev = &pd[port];
    usb_pd_port_config_t *config = usb_pd_pm_get_config(port);
    uint8_t *payload_ptr = &buf[3];
    uint8_t ndo = 0;
    uint32_t *pdo;
    uint8_t pdo_idx;

    if ((msg_type == DATA_MSG_TYPE_SRC_CAPS) ||
        (msg_type == DATA_MSG_TYPE_SNK_CAPS))
    {
        if (msg_type == DATA_MSG_TYPE_SRC_CAPS)
        {
            pdo = dev->src_pdo;
            ndo = config->num_src_pdos;
        }
        else
        {
            pdo = dev->snk_pdo;
            ndo = config->num_snk_pdos;
        }

        for (pdo_idx = 0; pdo_idx < ndo; pdo_idx++)
        {
            *payload_ptr++ = (uint8_t)(pdo[pdo_idx] & 0xFF);
            *payload_ptr++ = (uint8_t)((pdo[pdo_idx] & 0xFF00) >> 8);
            *payload_ptr++ = (uint8_t)((pdo[pdo_idx] & 0xFF0000) >> 16);
            *payload_ptr++ = (uint8_t)((pdo[pdo_idx] & 0xFF000000) >> 24);
        }
    }
    else if (msg_type == DATA_MSG_TYPE_REQUEST)
    {
        ndo = 1;

        payload_ptr[0] = (uint8_t)(rdo & 0xFF);
        payload_ptr[1] = (uint8_t)((rdo & 0xFF00) >> 8);
        payload_ptr[2] = (uint8_t)((rdo & 0xFF0000) >> 16);
        payload_ptr[3] = (uint8_t)((rdo & 0xFF000000) >> 24);
    }
    else
    {
        CRIT("usb_pd_pe_tx_data_msg: msg_type %u not supported.\n", msg_type);
    }

    if (ndo > 0)
    {
        usb_pd_prl_tx_data_msg(port, buf, msg_type, sop_type, ndo);
    }

    return;
}


static void usb_pd_pe_unhandled_rx_msg(usb_pd_port_t *dev)
{
    if (dev->power_role == PD_PWR_ROLE_SNK)
    {
        if (*dev->current_state == PE_SNK_TRANSITION_SINK)
        {
            pe_set_state(dev, PE_SNK_HARD_RESET);
        }
        else if (*dev->current_state == PE_SNK_READY)
        {
            pe_set_state(dev, PE_SNK_SEND_NOT_SUPPORTED);
        }
        else if (dev->non_interruptable_ams)
        {
            pe_set_state(dev, PE_SNK_SEND_SOFT_RESET);
        }
    }
    else /* SRC */
    {
        if ((*dev->current_state == PE_SRC_TRANSITION_PS) ||
            (*dev->current_state == PE_SRC_TRANSITION_PS_EXIT) ||
            (*dev->current_state == PE_SRC_NEGOTIATE_CAPABILITY) ||
            (*dev->current_state == PE_SRC_TRANSITION_SUPPLY))
        {
            pe_set_state(dev, PE_SRC_HARD_RESET);
        }
        else if (*dev->current_state == PE_SRC_READY)
        {
            pe_set_state(dev, PE_SRC_SEND_NOT_SUPPORTED);
        }
        else if (dev->non_interruptable_ams)
        {
            pe_set_state(dev, PE_SRC_SEND_SOFT_RESET);
        }
    }

    return;
}

static void usb_pd_pe_ctrl_msg_rx_handler(usb_pd_port_t *dev)
{
    bool pinged = false;

    switch (dev->rx_msg_type)
    {
        case CTRL_MSG_TYPE_GOTO_MIN:
            if (*dev->current_state == PE_SNK_READY)
            {
                pe_set_state(dev, PE_SNK_TRANSITION_SINK);
            }
            break;

        case CTRL_MSG_TYPE_ACCEPT:
            if (*dev->current_state == PE_SRC_SEND_SOFT_RESET)
            {
                // Stop sender response timer.
                timer_cancel(&dev->timer);
                pe_set_state(dev, PE_SRC_SEND_CAPS);
            }
            else if (*dev->current_state == PE_SNK_SEND_SOFT_RESET)
            {
                // Stop sender response timer.
                timer_cancel(&dev->timer);
                pe_set_state(dev, PE_SNK_WAIT_FOR_CAPS);
            }
            else if (*dev->current_state == PE_SNK_SELECT_CAPABILITY)
            {
                // Stop sender response timer.
                timer_cancel(&dev->timer);
                pe_set_state(dev, PE_SNK_TRANSITION_SINK);
            }
            break;

        case CTRL_MSG_TYPE_REJECT:
            if (*dev->current_state == PE_SNK_SELECT_CAPABILITY)
            {
                if (!dev->explicit_contract)
                {
                    pe_set_state(dev, PE_SNK_WAIT_FOR_CAPS);
                }
                else
                {
                    pe_set_state(dev, PE_SNK_READY);
                }
            }
            break;

        case CTRL_MSG_TYPE_PING:
            // Sink may receive ping msgs but should ignore them.
            pinged = true;
            break;

        case CTRL_MSG_TYPE_PS_RDY:
            if (*dev->current_state == PE_SNK_TRANSITION_SINK)
            {
                // Cancel PSTransition timer.
                timer_cancel(&dev->timer);
                pe_set_state(dev, PE_SNK_READY);
            }
            break;

        case CTRL_MSG_TYPE_GET_SRC_CAP:
            if (*dev->current_state == PE_SRC_READY)
            {
                pe_set_state(dev, PE_SRC_SEND_CAPS);
            }
            break;

        case CTRL_MSG_TYPE_GET_SNK_CAP:
            if (*dev->current_state == PE_SNK_READY)
            {
                pe_set_state(dev, PE_SNK_GIVE_SINK_CAP);
            }
            break;

        case CTRL_MSG_TYPE_DR_SWAP:
            if (dev->data_role == PD_DATA_ROLE_UFP)
            {
                pe_set_state(dev, PE_DRS_UFP_DFP_REJECT_SWAP);
            }
            else
            {
                pe_set_state(dev, PE_DRS_DFP_UFP_REJECT_SWAP);
            }
            break;

        case CTRL_MSG_TYPE_PR_SWAP:
            if (dev->power_role == PD_PWR_ROLE_SNK)
            {
                pe_set_state(dev, PE_PRS_SNK_SRC_REJECT_SWAP);
            }
            else
            {
                pe_set_state(dev, PE_PRS_SRC_SNK_REJECT_SWAP);
            }
            break;

        case CTRL_MSG_TYPE_VCONN_SWAP:
            break;

        case CTRL_MSG_TYPE_WAIT:
            if (*dev->current_state == PE_SNK_SELECT_CAPABILITY)
            {
                if (!dev->explicit_contract)
                {
                    pe_set_state(dev, PE_SNK_WAIT_FOR_CAPS);
                }
                else
                {
                    dev->wait_received = true;
                    pe_set_state(dev, PE_SNK_READY);
                }
            }
            break;

        case CTRL_MSG_TYPE_SOFT_RESET:
            if (dev->power_role == PD_PWR_ROLE_SNK)
            {
                pe_set_state(dev, PE_SNK_SOFT_RESET);
            }
            else
            {
                pe_set_state(dev, PE_SRC_SOFT_RESET);
            }
            break;

        case CTRL_MSG_TYPE_NOT_SUPPORTED:
            if (*dev->current_state == PE_SRC_READY)
            {
                pe_set_state(dev, PE_SRC_NOT_SUPPORTED_RECEIVED);
            }
            else if (*dev->current_state == PE_SNK_READY)
            {
                pe_set_state(dev, PE_SNK_NOT_SUPPORTED_RECEIVED);
            }
            break;

        case CTRL_MSG_TYPE_GET_SRC_CAP_EXT:
            break;

        case CTRL_MSG_TYPE_GET_STATUS:
            break;

        case CTRL_MSG_TYPE_FR_SWAP:
            break;

        default:
            CRIT("Invalid ctrl rx_msg_type 0x%x\n!", dev->rx_msg_type);
            pe_set_state(dev, PE_SNK_SEND_SOFT_RESET);
            break;
    }

    if (!dev->state_change && !pinged)
    {
        usb_pd_pe_unhandled_rx_msg(dev);
    }

    return;
}

#define BIST_CARRIER_MODE_REQUEST  5
#define BIST_TEST_DATA             8

static void usb_pd_pe_data_msg_rx_handler(usb_pd_port_t *dev)
{
    switch (dev->rx_msg_type)
    {
        case DATA_MSG_TYPE_SRC_CAPS:
            if (*dev->current_state == PE_SNK_WAIT_FOR_CAPS)
            {
                // Cancel SinkWaitCap timer.
                timer_cancel(&dev->timer);
                pe_set_state(dev, PE_SNK_EVALUATE_CAPABILITY);
            }
            else if (*dev->current_state == PE_SNK_READY)
            {
                pe_set_state(dev, PE_SNK_EVALUATE_CAPABILITY);
            }
            else
            {
                // Hard reset for this particular protocol error.
                pe_set_state(dev, PE_SNK_HARD_RESET);
            }
            break;

        case DATA_MSG_TYPE_REQUEST:
            if ((*dev->current_state == PE_SRC_SEND_CAPS) ||
                (*dev->current_state == PE_SRC_READY))
            {
                // Cancel sender response timer.
                timer_cancel(&dev->timer);
                pe_set_state(dev, PE_SRC_NEGOTIATE_CAPABILITY);
            }
            break;

        case DATA_MSG_TYPE_BIST:
            if ((*dev->current_state == PE_SNK_READY) ||
                (*dev->current_state == PE_SRC_READY))
            {
                // Check we are operating at vSafe5V and power role is not swapped.
                if ((dev->object_position == 1) &&
                    (dev->power_role == dev->data_role))
                {
                    // Check BIST param, data_obj[31:28].
                    if ((dev->rx_msg_buf[3] >> 4) == BIST_CARRIER_MODE_REQUEST)
                    {
                        pe_set_state(dev, PE_BIST_CARRIER_MODE);
                    }
                    else if ((dev->rx_msg_buf[3] >> 4) == BIST_TEST_DATA)
                    {
                        // Set BIST mode so we don't keep getting Rx interrupts. (not functional in PG1.0)
                        tcpm_set_bist_test_mode(dev->port);

                        pe_set_state(dev, PE_BIST_TEST_MODE);

                        // This test shall be ended by a Hard Reset.
                    }
                }
            }
            else
            {
                CRIT("Ignoring BIST while in %s!\n", state2string[*dev->current_state]);
            }
            break;

        case DATA_MSG_TYPE_SNK_CAPS:
            if (*dev->current_state == PE_SRC_GET_SINK_CAP)
            {
                // Cancel sender response timer.
                timer_cancel(&dev->timer);
                // Pass sink caps to policy manager. - BQ

                pe_set_state(dev, PE_SRC_READY);
            }
            break;

        case DATA_MSG_TYPE_BATT_STATUS:
            break;

        case DATA_MSG_TYPE_ALERT:
            break;

        case DATA_MSG_TYPE_VENDOR:
            break;

        default:
            CRIT("Invalid data rx_msg_type 0x%x\n!", dev->rx_msg_type);
            pe_set_state(dev, PE_SNK_SEND_SOFT_RESET);
            break;
    }

    if (!dev->state_change)
    {
        usb_pd_pe_unhandled_rx_msg(dev);
    }

    return;
}

static void timeout_src_recover(unsigned int port)
{
    pe_set_state(&pd[port], PE_SRC_TRANSITION_TO_DEFAULT_EXIT);
//    non_reentrant_context_save();
//
//    pe_src_transition_to_default_exit(&pd[port]);
//
//    non_reentrant_context_restore();

    return;
}


static void timeout_swap_source_start(unsigned int port)
{
    pe_set_state(&pd[port], PE_SRC_SEND_CAPS);
    return;
}

static void timeout_typec_send_source_cap(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    dev->caps_cnt++;

    INFO("CapsCnt = %u\n", dev->caps_cnt);

    if ((dev->caps_cnt > N_CAPS_COUNT) &&
        !dev->pd_connected_since_attach)
    {
        pe_set_state(dev, PE_SRC_DISABLED);
    }
    else
    {
        pe_set_state(&pd[port], PE_SRC_SEND_CAPS);
    }

    return;
}


static void timeout_sender_response(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    CRIT("timeout_sender_response!\n");

    if ((*dev->current_state == PE_SRC_SEND_CAPS) ||
        (*dev->current_state == PE_SRC_SEND_SOFT_RESET))
    {
        pe_set_state(dev, PE_SRC_HARD_RESET);
    }
    else if (*dev->current_state == PE_SRC_GET_SINK_CAP)
    {
        pe_set_state(dev, PE_SRC_READY);
    }
    else if (*dev->current_state == PE_SNK_SELECT_CAPABILITY)
    {
        pe_set_state(dev, PE_SNK_HARD_RESET);
    }
    else
    {
        CRIT("Error: timeout_sender_response - state %u unhandled!\n", *dev->current_state);
    }

    return;
}

static void timeout_ps_hard_reset(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    if ((*dev->current_state == PE_SRC_HARD_RESET) ||
        (*dev->current_state == PE_SRC_HARD_RESET_RECEIVED))
    {
        pe_set_state(dev, PE_SRC_TRANSITION_TO_DEFAULT);
    }

    return;
}

static void pe_src_startup_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_reset(dev->port);

    // Reset source caps count.
    dev->caps_cnt = 0;

    dev->power_role = PD_PWR_ROLE_SRC;
    dev->non_interruptable_ams = false;

    if (dev->power_role_swap)
    {
        timer_start(&dev->timer, T_SWAP_SOURCE_START_MS, timeout_swap_source_start);
        dev->power_role_swap = false;
    }
    else
    {
        pe_set_state(dev, PE_SRC_SEND_CAPS);
    }

    return;
}

static void pe_src_discovery_entry(usb_pd_port_t *dev)
{
    // Start SourceCapabilityTimer.
    timer_start(&dev->timer, T_TYPEC_SEND_SOURCE_CAP_MS, timeout_typec_send_source_cap);

    return;
}


static void timeout_vbus(unsigned int port)
{
    vbus_timed_out = true;
}


#define MV_TO_25MV(x) ((x)/25)

#define VSAFE5V_MIN   MV_TO_25MV(4450)  /* 4.45V min in 25mV units */
#define VSAFE0V_MAX   MV_TO_25MV(800)   /* 0.8V max in 25mV units */

static void pe_src_transition_to_default_entry(usb_pd_port_t *dev)
{
    // Request policy manager to set the Port Data Role to DFP,

    // Disable VCONN.
    tcpm_vconn_control(dev->port, false);

    // Remove Rp from VCONN pin.
    tcpm_remove_rp_from_vconn_pin(dev->port);

    // Disable VBUS.
    tcpm_src_vbus_disable(dev->port);

    // Force VBUS discharge.
    tcpm_force_discharge(dev->port, 0);

    dev->explicit_contract = false;

    vbus_timed_out = false;
    timer_start(&dev->timer, T_SRC_TURN_OFF_MS, timeout_vbus);

    // Wait for vSafe0V before starting source recover timer.
    while ((tcpm_get_vbus_voltage(dev->port) > VSAFE0V_MAX) &&
           !vbus_timed_out);

    timer_start(&dev->timer, T_SRC_RECOVER_MS, timeout_src_recover);

    return;
}

static void pe_src_transition_to_default_exit(usb_pd_port_t *dev)
{
    // Restore vSafe5V.
    tcpm_src_vbus_5v_enable(dev->port);

    // Enable VCONN.
    tcpm_vconn_control(dev->port, true);

    timer_start_no_response(dev);

    vbus_timed_out = false;
    timer_start(&dev->timer, T_SRC_TURN_ON_MS, timeout_vbus);

    // Wait for VBUS to reach vSafe5V before going to startup state.
    while ((tcpm_get_vbus_voltage(dev->port) < VSAFE5V_MIN) &&
           !vbus_timed_out);

    pe_set_state(dev, PE_SRC_STARTUP);

    return;
}

static void pe_src_send_caps_entry(usb_pd_port_t *dev)
{
    usb_pd_pe_tx_data_msg(dev->port, DATA_MSG_TYPE_SRC_CAPS, TCPC_TX_SOP);
    return;
}


static void pe_src_negotiate_capability_entry(usb_pd_port_t *dev)
{
    uint16_t operating_current;
    uint16_t src_max_current;
    usb_pd_port_config_t *config = usb_pd_pm_get_config(dev->port);
    uint32_t rdo = get_data_object(dev->rx_msg_buf);

    // BQ - Battery RDO not supported.

    dev->object_position = (rdo >> 28) & 0x07;
    operating_current = (rdo >> 10) & 0x3FF;

    if ((dev->object_position > 0) &&
        (dev->object_position <= config->num_src_pdos) &&
        (dev->object_position <= PD_MAX_PDO_NUM))
    {
        src_max_current = dev->src_pdo[dev->object_position - 1] & 0x3FF;

        DEBUG("PE_SRC_NEG_CAP: ObjPos = %u, req = %u mA, avail = %u mA\n",
              dev->object_position, operating_current * 10, src_max_current * 10);

        if (operating_current <= src_max_current)
        {
            pe_set_state(dev, PE_SRC_TRANSITION_SUPPLY);
        }
        else
        {
            // Request cannot be met.
            pe_set_state(dev, PE_SRC_CAPABILITY_RESPONSE);
        }
    }
    else
    {
        DEBUG("PE_SRC_NEG_CAP: ObjPos = %u is invalid!\n", dev->object_position);

        // Request cannot be met.
        pe_set_state(dev, PE_SRC_CAPABILITY_RESPONSE);
    }

    return;
}


static void pe_src_transition_supply_entry(usb_pd_port_t *dev)
{
    if (dev->request_goto_min)
    {
        dev->request_goto_min = false;

        // Send GotoMin message.
        usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_GOTO_MIN, TCPC_TX_SOP);
    }
    else
    {
        // Send Accept message.
        usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_ACCEPT, TCPC_TX_SOP);
    }

    return;
}

static void pe_src_transition_supply_exit(usb_pd_port_t *dev)
{
//    non_reentrant_context_save();

    // Send PS_RDY.
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_PS_RDY, TCPC_TX_SOP);

    pe_set_state(dev, PE_SRC_READY);

//    non_reentrant_context_restore();

    return;
}

static void pd_power_supply_ready(usb_pd_port_t *dev)
{
//    pe_src_transition_supply_exit(dev);
    pe_set_state(dev, PE_SRC_TRANSITION_PS_EXIT);

    return;
}

#define V_SRC_VALID_MV  500  /* +/- 500 mV */

#if 1

static void pd_transition_power_supply(usb_pd_port_t *dev)
{
    uint16_t v_threshold;

//    non_reentrant_context_save();

    if (dev->object_position == 1)
    {
        // 5V.

        // Send PS_RDY.
//        pe_src_transition_supply_exit(dev);
        pe_set_state(dev, PE_SRC_TRANSITION_PS_EXIT);

    }
    else
    {
        // If supporting more than one higher voltage.  Check PDO to determine voltage.
        tcpm_src_vbus_hi_volt_enable(dev->port);

        // Use Hi voltage alarm to determine when power supply is ready.
        // Alteratively, policy manager could notify policy engine or may be possible to
        // fixed delay if system timing is known.

        // Program Hi voltage alarm based on PDO.
        v_threshold = (dev->src_pdo[dev->object_position - 1] >> 10) & 0x3FF;

        // If fixed PDO, multiply by 0.95 to get min.
        v_threshold *= 95;
        v_threshold /= 100;
        v_threshold -= (V_SRC_VALID_MV / 50);
        v_threshold = v_threshold << 1;  /* convert to 25mV units */

        tcpm_set_voltage_alarm_hi(dev->port, v_threshold);
    }

//    non_reentrant_context_restore();

    return;
}

#else

static void pd_transition_power_supply(usb_pd_port_t *dev)
{
    uint16_t v_threshold;
    uint16_t present_voltage;
    uint16_t requested_voltage;
    usb_pd_port_config_t *config = usb_pd_pm_get_config(dev->port);

    if (config->num_src_pdos == 1)
    {
        // Only 5V is supported so no voltage transition required.
        pe_set_state(dev, PE_SRC_TRANSITION_PS_EXIT);
        return;
    }

//    non_reentrant_context_save();

    // Get present VBUS voltage and convert to millivolts.
    present_voltage = tcpm_get_vbus_voltage(dev->port) * 25;

    // Get requested VBUS voltage in millivolts.
    requested_voltage = ((dev->src_pdo[dev->object_position - 1] >> 10) & 0x3FF) * 50;

    if (requested_voltage < present_voltage)
    {
        if ((present_voltage - requested_voltage) > 1000)
        {
            tcpm_src_vbus_disable(dev->port);

            v_threshold = (requested_voltage + V_SRC_VALID_MV) / 25;

            vbus_timed_out = false;
            timer_start(&dev->timer, T_SRC_TURN_OFF_MS, timeout_vbus);

            // Force VBUS discharge.
            tcpm_force_discharge(dev->port, v_threshold);

            // Wait for discharge to complete.
            while ((tcpm_get_vbus_voltage(dev->port) > v_threshold) &&
                   !vbus_timed_out);
        }
    }

    if (dev->object_position == 1)
    {
        // 5V.
        tcpm_src_vbus_5v_enable(dev->port);

        v_threshold = VSAFE5V_MIN;
    }
    else
    {
        // If supporting more than one higher voltage.  Check PDO to determine voltage.
        tcpm_src_vbus_hi_volt_enable(dev->port);

        // Program Hi voltage alarm based on PDO.
        v_threshold = (dev->src_pdo[dev->object_position - 1] >> 10) & 0x3FF;

        // If fixed PDO, multiply by 0.95 to get min.
        v_threshold *= 95;
        v_threshold /= 100;
        v_threshold -= (V_SRC_VALID_MV / 50);
        v_threshold = v_threshold << 1;  /* convert to 25mV units */
    }

    // Use Hi voltage alarm to determine when power supply is ready. 
    // Alteratively, policy manager could notify policy engine or may be possible to 
    // fixed delay if system timing is known.
    tcpm_set_voltage_alarm_hi(dev->port, v_threshold);

//    non_reentrant_context_restore();

    return;
}

#endif

static void timeout_src_transition(unsigned int port)
{
    pe_set_state(&pd[port], PE_SRC_TRANSITION_PS);
//    pd_transition_power_supply(&pd[port]);
    return;
}


static void pe_src_capability_response_entry(usb_pd_port_t *dev)
{
    // Send Reject.
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_REJECT, TCPC_TX_SOP);

    // BQ - Send Wait not supported.
}


static void pe_src_ready_entry(usb_pd_port_t *dev)
{
    // Notify PRL of end of AMS. - BQ
    dev->non_interruptable_ams = false;

    dev->pd_connected_since_attach = true;
    dev->explicit_contract = true;

    // If VCONN source, start DiscoveryIdentity timer and negotiate PD with cable plug. - BQ

    return;
}

static void pe_src_hard_reset_received_entry(usb_pd_port_t *dev)
{
    timer_start(&dev->timer, T_PS_HARD_RESET_MS, timeout_ps_hard_reset);
    return;
}

static void pe_src_hard_reset_entry(usb_pd_port_t *dev)
{
    dev->hard_reset_cnt++;
    tcpm_transmit(dev->port, NULL, TCPC_TX_HARD_RESET);
    timer_start(&dev->timer, T_PS_HARD_RESET_MS, timeout_ps_hard_reset);
    return;
}

static void pe_src_soft_reset_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_ACCEPT, TCPC_TX_SOP);
    return;
}

static void pe_src_get_sink_cap_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_GET_SNK_CAP, TCPC_TX_SOP);
    timer_start(&dev->timer, T_SENDER_RESPONSE_MS, timeout_sender_response);
    return;
}

static void pe_src_disabled_entry(usb_pd_port_t *dev)
{
    tcpm_disable_pd_receive(dev->port);
    return;
}

static void pe_src_send_soft_reset_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_SOFT_RESET, TCPC_TX_SOP);
    return;
}

static void pe_src_send_not_supported_entry(usb_pd_port_t *dev)
{
    // BQ - add option to send SOP'/SOP".
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_NOT_SUPPORTED, TCPC_TX_SOP);
    return;
}

static void pe_src_not_supported_received_entry(usb_pd_port_t *dev)
{
    // Inform policy manager.

    pe_set_state(dev, PE_SRC_READY);
    return;
}

/* can only be entered from PE_SRC_READY state */
static void pe_src_ping_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_PING, TCPC_TX_SOP);
    return;
}

static void pe_src_wait_new_caps_entry(usb_pd_port_t *dev)
{
    // Do nothing. Wait for policy manager to provide new caps.
}


static void timeout_sink_wait_cap(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    DEBUG("Wait Cap timeout, HR cnt = %u.\n", dev->hard_reset_cnt);

    if (dev->hard_reset_cnt <= N_HARD_RESET_COUNT)
    {
        pe_set_state(dev, PE_SNK_HARD_RESET);
    }

    return;
}

static void timeout_ps_transition(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    if (dev->hard_reset_cnt <= N_HARD_RESET_COUNT)
    {
        pe_set_state(dev, PE_SNK_HARD_RESET);
    }

    return;
}

static void timeout_sink_request(unsigned int port)
{
    pe_set_state(&pd[port], PE_SNK_SELECT_CAPABILITY);
    return;
}

static void pe_snk_startup_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_reset(dev->port);

    dev->power_role = PD_PWR_ROLE_SNK;
    dev->non_interruptable_ams = false;

    pe_set_state(dev, PE_SNK_DISCOVERY);

    return;
}

static void pe_snk_discovery_entry(usb_pd_port_t *dev)
{
    if (dev->no_response_timed_out &&
        dev->pd_connected_since_attach &&
        (dev->hard_reset_cnt > N_HARD_RESET_COUNT))
    {
        tcpm_error_recovery(dev->port);
    }
    else
    {
        vbus_timed_out = false;
        timer_start(&dev->timer, T_SRC_TURN_ON_MS, timeout_vbus);

        // Wait for VBUS present.
        while (!tcpm_is_vbus_present(dev->port) && !vbus_timed_out);

        // Re-enable AutoDischargeDisconnect. (May have been disabled due to hard reset)
        tcpm_set_autodischarge_disconnect(dev->port, true);

        pe_set_state(dev, PE_SNK_WAIT_FOR_CAPS);
    }

    return;
}

static void pe_snk_wait_for_caps_entry(usb_pd_port_t *dev)
{
    timer_start(&dev->timer, T_TYPEC_SINK_WAIT_CAP_MS, timeout_sink_wait_cap);
    return;
}

static void pe_snk_evaluate_capability_entry(usb_pd_port_t *dev)
{
    timer_cancel_no_response(dev);
    dev->hard_reset_cnt = 0;

    dev->non_interruptable_ams = true;

    // Ask policy manager to evaluate options based on supplied capabilities.
    usb_pd_pm_evaluate_src_caps(dev->port);

    pe_set_state(dev, PE_SNK_SELECT_CAPABILITY);

    return;
}

static void pe_snk_select_capability_entry(usb_pd_port_t *dev)
{
    // Build RDO based on policy manager response.
    build_rdo(dev->port);

    // Send RDO.
    usb_pd_pe_tx_data_msg(dev->port, DATA_MSG_TYPE_REQUEST, TCPC_TX_SOP);

    return;
}

static void pe_snk_transition_sink_entry(usb_pd_port_t *dev)
{
    timer_start(&dev->timer, T_PS_TRANSITION_MS, timeout_ps_transition);

    // Request policy manager to transition to new power level and wait for PS_RDY from source.

    return;
}

#define DEFAULT_5V  (5000 / 25)  /* 25mV LSB */

static void pe_snk_ready_entry(usb_pd_port_t *dev)
{
    uint16_t threshold;

    dev->pd_connected_since_attach = true;
    dev->explicit_contract = true;

    if (dev->wait_received)
    {
        dev->wait_received = false;
        // Start SinkRequest timer.
        timer_start(&dev->timer, T_SINK_REQUEST_MS, timeout_sink_request);
    }

    if (dev->min_voltage == DEFAULT_5V)
    {
        // Set Sink Disconnect Threshold to zero so default vSafe5V is used.
        threshold = 0;
    }
    else
    {
        // Set Sink Disconnect Threshold to 80% of min voltage.
        threshold = (dev->min_voltage * 8) / 10;
        DEBUG("SNK VBUS Disconn Thres = %u mV.\n", threshold * 25);
    }

    tcpm_set_sink_disconnect_threshold(dev->port, threshold);

    // On entry to the PE_SNK_Ready state if this is a DFP which needs to establish communication
    // with a Cable Plug, then the Policy Engine shall initialize and run the DiscoverIdentityTimer

    return;
}

static void pe_snk_hard_reset_entry(usb_pd_port_t *dev)
{
    // Disable AutoDischargeDisconnect.
    tcpm_set_autodischarge_disconnect(dev->port, false);

    // Disable sink VBUS.
    tcpm_snk_vbus_disable(dev->port);

    dev->hard_reset_cnt++;
    tcpm_transmit(dev->port, NULL, TCPC_TX_HARD_RESET);

    return;
}

static void pe_snk_soft_reset_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_ACCEPT, TCPC_TX_SOP);
    return;
}

static void pe_reject_swap_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_REJECT, TCPC_TX_SOP);
    return;
}

static void pe_snk_send_soft_reset_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_SOFT_RESET, TCPC_TX_SOP);
    return;
}

static void pe_snk_send_not_supported_entry(usb_pd_port_t *dev)
{
    // BQ - add option to send SOP'/SOP".
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_NOT_SUPPORTED, TCPC_TX_SOP);
    return;
}

static void pe_snk_not_supported_received_entry(usb_pd_port_t *dev)
{
    // Notify policy manager.

    pe_set_state(dev, PE_SNK_READY);
    return;
}

static void pe_snk_transition_to_default_entry(usb_pd_port_t *dev)
{
    // Notify policy manager sink shall transition to default.

    // Request policy manager to set data role to UFP.

    // Disable VCONN.
    tcpm_vconn_control(dev->port, false);
    dev->explicit_contract = false;

    timer_start_no_response(dev);

    // BQ - Assume sink has reached default level.
    pe_set_state(dev, PE_SNK_STARTUP);

    return;
}

static void pe_snk_give_sink_cap_entry(usb_pd_port_t *dev)
{
    // Request Sink Caps from policy manager.

    usb_pd_pe_tx_data_msg(dev->port, DATA_MSG_TYPE_SNK_CAPS, TCPC_TX_SOP);
    return;
}

static void pe_snk_get_source_cap_entry(usb_pd_port_t *dev)
{
    usb_pd_prl_tx_ctrl_msg(dev->port, buf, CTRL_MSG_TYPE_GET_SRC_CAP, TCPC_TX_SOP);
    return;
}

static void timeout_bist_cont_mode(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    if (dev->power_role == PD_PWR_ROLE_SNK)
    {
        pe_set_state(dev, PE_SNK_TRANSITION_TO_DEFAULT);
    }
    else
    {
        pe_set_state(dev, PE_SRC_TRANSITION_TO_DEFAULT);
    }

    return;
}


static void pe_bist_carrier_mode_entry(usb_pd_port_t *dev)
{
    // Start BIST transmit.  TCPC HW will automatically stop transmit after tBISTContMode.
    tcpm_transmit(dev->port, NULL, TCPC_TX_BIST_MODE2);

    timer_start(&dev->timer, T_BIST_CONT_MODE_MS, timeout_bist_cont_mode);

    return;
}

static void pe_bist_test_mode_entry(usb_pd_port_t *dev)
{
    return;
}

static void pe_dummy_state_entry(usb_pd_port_t *dev)
{
    INFO("Dummy State Entry\n");
    return;
}

typedef void (*state_entry_fptr)(usb_pd_port_t *dev);

static const state_entry_fptr pe_state_entry[PE_NUM_STATES] =
{
    pe_src_startup_entry,                 /* PE_SRC_STARTUP                */
    pe_src_discovery_entry,               /* PE_SRC_DISCOVERY              */
    pe_src_send_caps_entry,               /* PE_SRC_SEND_CAPS              */
    pe_src_negotiate_capability_entry,    /* PE_SRC_NEGOTIATE_CAPABILITY   */
    pe_src_transition_supply_entry,       /* PE_SRC_TRANSITION_SUPPLY      */
    pd_transition_power_supply,
    pe_src_transition_supply_exit,
    pe_src_ready_entry,                   /* PE_SRC_READY                  */
    pe_src_disabled_entry,                /* PE_SRC_DISABLED               */
    pe_src_capability_response_entry,     /* PE_SRC_CAPABILITY_RESPONSE    */
    pe_src_hard_reset_entry,              /* PE_SRC_HARD_RESET             */
    pe_src_hard_reset_received_entry,     /* PE_SRC_HARD_RESET_RECEIVED    */
    pe_src_transition_to_default_entry,   /* PE_SRC_TRANSITION_TO_DEFAULT  */
    pe_src_transition_to_default_exit,
    pe_src_get_sink_cap_entry,            /* PE_SRC_GET_SINK_CAP           */
    pe_src_wait_new_caps_entry,           /* PE_SRC_WAIT_NEW_CAPS          */
    pe_src_send_soft_reset_entry,         /* PE_SRC_SEND_SOFT_RESET        */
    pe_src_soft_reset_entry,              /* PE_SRC_SOFT_RESET             */
    pe_src_send_not_supported_entry,      /* PE_SRC_SEND_NOT_SUPPORTED     */
    pe_src_not_supported_received_entry,  /* PE_SRC_NOT_SUPPORTED_RECEIVED */
    pe_src_ping_entry,                    /* PE_SRC_PING                   */
    pe_dummy_state_entry,                 /* PE_SRC_SEND_SOURCE_ALERT      */
    pe_dummy_state_entry,                 /* PE_SRC_SINK_ALERT_RECEIVED    */
    pe_dummy_state_entry,                 /* PE_SRC_GIVE_SOURCE_CAP_EXT    */
    pe_dummy_state_entry,                 /* PE_SRC_GIVE_SOURCE_STATUS     */
    pe_dummy_state_entry,                 /* PE_SRC_GET_SINK_STATUS        */

    pe_snk_startup_entry,                 /* PE_SNK_STARTUP                */
    pe_snk_discovery_entry,               /* PE_SNK_DISCOVERY              */
    pe_snk_wait_for_caps_entry,           /* PE_SNK_WAIT_FOR_CAPS          */
    pe_snk_evaluate_capability_entry,     /* PE_SNK_EVALUATE_CAPABILITY    */
    pe_snk_select_capability_entry,       /* PE_SNK_SELECT_CAPABILITY      */
    pe_snk_transition_sink_entry,         /* PE_SNK_TRANSITION_SINK        */
    pe_snk_ready_entry,                   /* PE_SNK_READY                  */
    pe_snk_hard_reset_entry,              /* PE_SNK_HARD_RESET             */
    pe_snk_transition_to_default_entry,   /* PE_SNK_TRANSITION_TO_DEFAULT  */
    pe_snk_give_sink_cap_entry,           /* PE_SNK_GIVE_SINK_CAP          */
    pe_snk_get_source_cap_entry,          /* PE_SNK_GET_SOURCE_CAP         */
    pe_snk_send_soft_reset_entry,         /* PE_SNK_SEND_SOFT_RESET        */
    pe_snk_soft_reset_entry,              /* PE_SNK_SOFT_RESET             */
    pe_snk_send_not_supported_entry,      /* PE_SNK_SEND_NOT_SUPPORTED     */
    pe_snk_not_supported_received_entry,  /* PE_SNK_NOT_SUPPORTED_RECEIVED */
    pe_dummy_state_entry,                 /* PE_SNK_SOURCE_ALERT_RECEIVED  */
    pe_dummy_state_entry,                 /* PE_SNK_SEND_SINK_ALERT        */
    pe_dummy_state_entry,                 /* PE_SNK_GET_SOURCE_CAP_EXT     */
    pe_dummy_state_entry,                 /* PE_SNK_GET_SOURCE_STATUS      */
    pe_dummy_state_entry,                 /* PE_SNK_GIVE_SINK_STATUS       */

    pe_bist_carrier_mode_entry,           /* PE_BIST_CARRIER_MODE          */
    pe_bist_test_mode_entry,              /* PE_BIST_TEST_MODE             */

    pe_reject_swap_entry,                 /* PE_DRS_DFP_UFP_REJECT_SWAP    */
    pe_reject_swap_entry,                 /* PE_DRS_UFP_DFP_REJECT_SWAP    */

    pe_reject_swap_entry,                 /* PE_PRS_SRC_SNK_REJECT_SWAP    */
    pe_reject_swap_entry,                 /* PE_PRS_SNK_SRC_REJECT_SWAP    */

};


void usb_pd_pe_state_machine(unsigned int port)
{
    usb_pd_port_t *dev = &pd[port];

    if (!dev->state_change)
        return;

    while (dev->state_change)
    {
        dev->state_change = false;

        // Use branch table to execute "actions on entry" for the current state.
        if (*dev->current_state < PE_NUM_STATES)
        {
            pe_state_entry[*dev->current_state](dev);
        }
    }

    return;
}




// Call in main() loop.
void usb_pd_task(void)
{
#if NUM_TCPC_DEVICES > 1

    unsigned int port_num = NUM_TCPC_DEVICES - 1;

/* Dan M gotta fix this
    non_reentrant_context_save();
*/
    do
    {
        usb_pd_pe_state_machine(port_num);

    } while (port_num--);

/* Dan M gotta fix this
    non_reentrant_context_restore();
*/
#else
/* Dan M gotta fix this
    non_reentrant_context_save();
*/
    usb_pd_pe_state_machine(0);
/* Dan M gotta fix this
    non_reentrant_context_restore();
*/

#endif

    return;
}



void usb_pd_pe_notify(unsigned int port, usb_pd_prl_alert_t prl_alert)
{
    usb_pd_port_t *dev = &pd[port];

    switch (prl_alert)
    {
        case PRL_ALERT_HARD_RESET_RECEIVED:
            // PD message passing is enabled in protocol layer.

            if (dev->power_role == PD_PWR_ROLE_SNK)
            {
                // Disable AutoDischargeDisconnect.
                tcpm_set_autodischarge_disconnect(dev->port, false);

                // Disable sink VBUS.
                tcpm_snk_vbus_disable(dev->port);

                pe_set_state(dev, PE_SNK_TRANSITION_TO_DEFAULT);
            }
            else /* SRC */
            {
                pe_set_state(dev, PE_SRC_HARD_RESET_RECEIVED);
            }
            break;

        case PRL_ALERT_MSG_TX_SUCCESS:   /* GoodCRC Recieved */

            switch (*dev->current_state)
            {
                case PE_SRC_SEND_CAPS:
                    dev->caps_cnt = 0;
                    dev->hard_reset_cnt = 0;
                    timer_cancel_no_response(dev);
                    timer_start(&dev->timer, T_SENDER_RESPONSE_MS, timeout_sender_response);
                    dev->non_interruptable_ams = true;
                    break;

                case PE_SRC_HARD_RESET:
                    // Do nothing.  PD message passing is enabled in protocol layer.
                    break;

                case PE_SNK_HARD_RESET:
                    pe_set_state(dev, PE_SNK_TRANSITION_TO_DEFAULT);
                    break;

                case PE_SNK_SEND_SOFT_RESET:
                case PE_SRC_SEND_SOFT_RESET:
                    timer_start(&dev->timer, T_SENDER_RESPONSE_MS, timeout_sender_response);
                    break;

                case PE_SNK_SOFT_RESET:
                    pe_set_state(dev, PE_SNK_WAIT_FOR_CAPS);
                    break;

                case PE_SRC_SOFT_RESET:
                    pe_set_state(dev, PE_SRC_SEND_CAPS);
                    break;

                case PE_SNK_SELECT_CAPABILITY:
                    timer_start(&dev->timer, T_SENDER_RESPONSE_MS, timeout_sender_response);
                    break;

                case PE_SRC_TRANSITION_SUPPLY:
                    timer_start(&dev->timer, T_SRC_TRANSITION_MS, timeout_src_transition);
                    break;

                case PE_SRC_CAPABILITY_RESPONSE:
                    if (dev->explicit_contract /* || send_wait*/)
                    {
                        // Reject and Current contract is still valid or wait sent.
                        pe_set_state(dev, PE_SRC_READY);

                        // If explicit contract and current contract is invalid go to hard reset.
                    }
                    else /* No explicit contract */
                    {
                        pe_set_state(dev, PE_SRC_WAIT_NEW_CAPS);
                    }
                    break;

                case PE_SRC_PING:
                case PE_SRC_SEND_NOT_SUPPORTED:
                case PE_PRS_SRC_SNK_REJECT_SWAP:
                    pe_set_state(dev, PE_SRC_READY);
                    break;

                case PE_SNK_SEND_NOT_SUPPORTED:
                case PE_SNK_GET_SOURCE_CAP:
                case PE_SNK_GIVE_SINK_CAP:
                case PE_PRS_SNK_SRC_REJECT_SWAP:
                    pe_set_state(dev, PE_SNK_READY);
                    break;

                case PE_DRS_DFP_UFP_REJECT_SWAP:
                case PE_DRS_UFP_DFP_REJECT_SWAP:
                    if (dev->power_role == PD_PWR_ROLE_SNK)
                    {
                        pe_set_state(dev, PE_SNK_READY);
                    }
                    else
                    {
                        pe_set_state(dev, PE_SRC_READY);
                    }
                    break;

                default:
                    break;
            }
            break;

        case PRL_ALERT_MSG_TX_DISCARDED: /* Msg was received or hard reset issued before Tx */
            //timer_cancel_no_response(dev);
            break;

        case PRL_ALERT_MSG_TX_FAILED: /* No GoodCRC response */
            //timer_cancel_no_response(dev);

            if (dev->power_role == PD_PWR_ROLE_SNK)
            {
                if (*dev->current_state == PE_SNK_SEND_SOFT_RESET)
                {
                    pe_set_state(dev, PE_SNK_HARD_RESET);
                }
                else
                {
                    pe_set_state(dev, PE_SNK_SEND_SOFT_RESET);
                }
            }
            else /* SRC */
            {
                if (*dev->current_state == PE_SRC_SEND_SOFT_RESET)
                {
                    // Send Hard reset within tHardReset (5ms).
                    pe_set_state(dev, PE_SRC_HARD_RESET);
                }
                else if ((*dev->current_state == PE_SRC_SEND_CAPS) &&
                         (!dev->pd_connected_since_attach))
                {
                    pe_set_state(dev, PE_SRC_DISCOVERY);
                }
                else
                {
                    // PE_SRC_Send_Soft_Reset state shall be entered from any state
                    // when a Protocol Error is detected by the Protocol Layer during a Non-interruptible AMS
                    pe_set_state(dev, PE_SRC_SEND_SOFT_RESET);
                }
            }
            break;

        case PRL_ALERT_MSG_RECEIVED:
            timer_cancel(&dev->timer);
            if (dev->rx_msg_data_len)
            {
                usb_pd_pe_data_msg_rx_handler(dev);
            }
            else
            {
                usb_pd_pe_ctrl_msg_rx_handler(dev);
            }
            break;

        case PRL_ALERT_VOLTAGE_ALARM_HI:
            if (*dev->current_state == PE_SRC_TRANSITION_SUPPLY)
            {
                pe_set_state(dev, PE_SRC_TRANSITION_PS_EXIT);
//                pe_src_transition_supply_exit(dev);
                // Disable voltage alarm.
                tcpm_set_voltage_alarm_hi(port, 0);
            }
            break;

        case PRL_ALERT_VOLTAGE_ALARM_LO:
	case PRL_ALERT_NO_RESPONSE_TIMEOUT:
	case PRL_ALERT_SENDER_RESPONSE_TIMEOUT:
            break;
    }

    return;
}



int usb_pd_policy_manager_request(unsigned int port, pd_policy_manager_request_t req)
{
    usb_pd_port_t *dev = &pd[port];
    usb_pd_pe_status_t status = STATUS_OK;

    if (*dev->current_state == PE_SRC_READY)
    {
        if (req == PD_POLICY_MNGR_REQ_GET_SINK_CAPS)
        {
            pe_set_state(dev, PE_SRC_GET_SINK_CAP);
        }
        else if (req == PD_POLICY_MNGR_REQ_SRC_CAPS_CHANGE)
        {
            pe_set_state(dev, PE_SRC_SEND_CAPS);
        }
        else if (req == PD_POLICY_MNGR_REQ_GOTO_MIN)
        {
            dev->request_goto_min = true;
            pe_set_state(dev, PE_SRC_TRANSITION_SUPPLY);
        }
        else
        {
            status = STATUS_REQUEST_NOT_SUPPORTED_IN_CURRENT_STATE;
        }
    }
    else if (*dev->current_state == PE_SRC_WAIT_NEW_CAPS)
    {
        if (req == PD_POLICY_MNGR_REQ_SRC_CAPS_CHANGE)
        {
            pe_set_state(dev, PE_SRC_SEND_CAPS);
        }
        else
        {
            status = STATUS_REQUEST_NOT_SUPPORTED_IN_CURRENT_STATE;
        }
    }
    else if (*dev->current_state == PE_SNK_READY)
    {
        if (req == PD_POLICY_MNGR_REQ_UPDATE_REMOTE_CAPS)
        {
            pe_set_state(dev, PE_SNK_GET_SOURCE_CAP);
        }
        else
        {
            status = STATUS_REQUEST_NOT_SUPPORTED_IN_CURRENT_STATE;
        }
    }
    else
    {
        status = STATUS_REQUEST_NOT_SUPPORTED_IN_CURRENT_STATE;
    }

    return status;
}


static void timeout_src_settling(unsigned int port)
{
    pd_power_supply_ready(&pd[port]);
    return;
}


void usb_pd_pe_voltage_alarm_handler(unsigned int port, bool hi_voltage)
{
    usb_pd_port_t *dev = &pd[port];

    if ((*dev->current_state == PE_SRC_TRANSITION_PS /*PE_SRC_TRANSITION_SUPPLY*/) && hi_voltage)
    {
        // Start power supply settling timeout.
        timer_start(&dev->timer, dev->src_settling_time, timeout_src_settling);
    }

    return;
}
