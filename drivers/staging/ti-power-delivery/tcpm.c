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

#include "tusb422_common.h"
#include "tcpm.h"
#include "tcpci.h"
#include "tusb422.h"

//
//  Global variables
//

//#define LED_DEBUG

static tcpc_device_t tcpc_dev[NUM_TCPC_DEVICES];

static bool vbus_timed_out;


#ifdef LED_DEBUG
#define BLINK_DELAY_MS  250
static timer_t timer_led_blink;
#endif

#define T_CC_DEBOUNCE_MS     150    /* 100 - 200 ms */
#define T_PD_DEBOUNCE_MS     15     /*  10 - 20 ms */
#define T_DRP_TRY_MS         130    /*  75 - 150 ms */
#define T_ERROR_RECOVERY_MS  500    /*  25 - ?? ms */

#define T_VBUS_DISCHARGE_MS  600    /* based on TUSB422 HW timeout for discharge ~583ms */

static void (*conn_state_change_cbk)(unsigned int port, tcpc_state_t state) = NULL;
static void (*current_change_cbk)(unsigned int port, tcpc_cc_snk_state_t state) = NULL;
static void (*volt_alarm_cbk)(unsigned int port, bool hi_voltage) = NULL;
static void (*pd_hard_reset_cbk)(unsigned int port) = NULL;
static void (*pd_transmit_cbk)(unsigned int port, tx_status_t status) = NULL;
static void (*pd_receive_cbk)(unsigned int port) = NULL;

#if DEBUG_LEVEL >= 3
static const char *state2string[]=
{
    "UNATTACHED_SRC",
    "UNATTACHED_SNK",
    "ATTACH_WAIT_SRC",
    "ATTACH_WAIT_SNK",
    "ATTACH_WAIT_SNK_WAIT4VBUS",
    "WAITING_FOR_VBUS_SNK",
    "TRY_SNK",
    "TRY_SNK_LOOK4SRC",
    "TRY_SRC",
    "TRY_WAIT_SRC",
    "TRY_WAIT_SNK",
    "ATTACHED_SRC",
    "ATTACHED_SNK",
    "UNORIENTED_DEBUG_ACC_SRC",
    "ORIENTED_DEBUG_ACC_SRC",
    "DEBUG_ACC_SNK",
    "AUDIO_ACC",
    "ERROR_RECOVERY",    
    "DISABLED"  
};
#endif


tcpc_device_t* tcpm_get_device(unsigned int port)
{
    return &tcpc_dev[port];
}

void tcpm_dump_state(unsigned int port)
{
/* Dan M dev not used 
    tcpc_device_t *dev = tcpm_get_device(port);

    CRIT("\nType-C State: %s (last: %s)\n", state2string[dev->state], state2string[dev->last_state]);
*/
    return;
}

static void timeout_vbus(unsigned int port)
{
    vbus_timed_out = true;
}


#ifdef LED_DEBUG

void tcpm_led_blink(unsigned int port)
{
    tcpc_device_t *dev = tcpm_get_device(port);

    switch (dev->state)
    {
        case TCPC_STATE_ATTACHED_SRC:
        case TCPC_STATE_ATTACHED_SNK:
        case TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC:
        case TCPC_STATE_ORIENTED_DEBUG_ACC_SRC:
        case TCPC_STATE_DEBUG_ACC_SNK:
        case TCPC_STATE_AUDIO_ACC:
            // Do nothing.
            break;

        default:
            if (dev->role == ROLE_DRP)
            {
                tcpm_hal_led_toggle(LED_BOTH);
            }
            else if (dev->state == TCPC_STATE_UNATTACHED_SNK)
            {
                tcpm_hal_led_toggle(LED_RED);
            }
            else if (dev->state == TCPC_STATE_UNATTACHED_SRC)
            {
                tcpm_hal_led_toggle(LED_GREEN);
            }

            // Restart timer.
            timer_start(&timer_led_blink, BLINK_DELAY_MS, tcpm_led_blink);
            break;
    }

    return;
}
#endif

static void tcpm_notify_conn_state(unsigned int port, tcpc_state_t state)
{
    if (conn_state_change_cbk) conn_state_change_cbk(port, state);
    return;
}

static void tcpm_notify_current_change(unsigned int port, tcpc_cc_snk_state_t state)
{
    DEBUG("Current advertisement change detected: %s\n", 
          (state == CC_SNK_STATE_POWER30) ? "3.0A" : 
          (state == CC_SNK_STATE_POWER15) ? "1.5A" : 
          (state == CC_SNK_STATE_DEFAULT) ? "500/900mA" : "?"); 

    if (current_change_cbk) current_change_cbk(port, state);
    return;
}

static void tcpm_notify_hard_reset(unsigned int port)
{
    DEBUG("Hard Reset Rx'd.\n");

    if (pd_hard_reset_cbk) pd_hard_reset_cbk(port);
    return;
}

static void tcpm_notify_pd_transmit(unsigned int port, tx_status_t status)
{
    DEBUG("PD Tx %s.\n", (status == TX_STATUS_SUCCESS) ? "OK" :
          (status == TX_STATUS_DISCARDED) ? "discarded" :
          (status == TX_STATUS_FAILED) ? "failed" : "?");

    if (pd_transmit_cbk) pd_transmit_cbk(port, status);
    return;
}

static void tcpm_notify_pd_receive(unsigned int port)
{
    DEBUG("PD Rx\n");

    if (pd_receive_cbk) pd_receive_cbk(port);
    return;
}

static void tcpm_notify_voltage_alarm(unsigned int port, bool hi_voltage)
{
    DEBUG("%s-Voltage alarm\n", (hi_voltage) ? "Hi" : "Lo");

    if (volt_alarm_cbk) volt_alarm_cbk(port, hi_voltage);
    return;
}

void tcpm_src_vbus_disable(unsigned int port)
{
    DEBUG("SRC VBUS off.\n");
    tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_DISABLE_SRC_VBUS);
    tcpm_hal_vbus_disable(port, VBUS_SRC_HI_VOLT);
    tcpm_hal_vbus_disable(port, VBUS_SRC_5V);
    return;
}

void tcpm_src_vbus_5v_enable(unsigned int port)
{
    DEBUG("-> SRC VBUS 5V.\n");

    tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_SRC_VBUS_DEFAULT);
    tcpm_hal_vbus_enable(port, VBUS_SRC_5V);
    return;
}

void tcpm_src_vbus_hi_volt_enable(unsigned int port)
{
    DEBUG("-> SRC VBUS Hi.\n");

    tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_SRC_VBUS_HI_VOLTAGE);
    tcpm_hal_vbus_enable(port, VBUS_SRC_HI_VOLT);
    return;
}

void tcpm_snk_vbus_enable(unsigned int port)
{
    DEBUG("-> SNK VBUS.\n");

    tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_SNK_VBUS);
    tcpm_hal_vbus_enable(port, VBUS_SNK);
    return;
}

void tcpm_snk_vbus_disable(unsigned int port)
{
    DEBUG("SNK VBUS off.\n");

    tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_DISABLE_SNK_VBUS);
    tcpm_hal_vbus_disable(port, VBUS_SNK);
    return;
}

void tcpm_set_voltage_alarm_hi(unsigned int port, uint16_t threshold_25mv)
{
    tcpc_write16(port, TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG, threshold_25mv);
    return;
}

void tcpm_set_voltage_alarm_lo(unsigned int port, uint16_t threshold_25mv)
{
    tcpc_write16(port, TCPC_REG_VBUS_VOLTAGE_ALARM_LO_CFG, threshold_25mv);
    return;
}

/* for Sinks only */
bool tcpm_is_vbus_present(unsigned int port)
{
    uint8_t pwr_status;

    tcpc_read8(port, TCPC_REG_POWER_STATUS, &pwr_status);

    return(pwr_status & TCPC_PWR_STATUS_VBUS_PRESENT) ? true : false;
}


void tcpm_vconn_control(unsigned int port, bool enable)
{
    if (enable)
    {
        tcpc_modify8(port, TCPC_REG_POWER_CTRL, 0, TCPC_PWR_CTRL_ENABLE_VCONN);
    }
    else
    {
        tcpc_modify8(port, TCPC_REG_POWER_CTRL, TCPC_PWR_CTRL_ENABLE_VCONN, 0);
    }

    return;
}


void tcpm_remove_rp_from_vconn_pin(unsigned int port)
{
    tcpc_device_t *dev = &tcpc_dev[port];

    if (dev->plug_polarity == PLUG_UNFLIPPED)
    {
        // Remove Rp from CC2 pin.
        tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                    tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RP, CC_OPEN));
    }
    else
    {
        // Remove Rp from CC1 pin.
        tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                    tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_OPEN, CC_RP));
    }

    return;
}

void tcpm_cc_pin_control(unsigned int port, tc_role_t role)
{
    tcpc_device_t *dev = &tcpc_dev[port];

    if (dev->role == ROLE_SNK)
    {
        tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                    tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RD, CC_RD));
    }
    else if (dev->role == ROLE_SRC)
    {
        tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                    tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RP, CC_RP));
    }

    return;
}

void tcpm_set_bist_test_mode(unsigned int port)
{
    CRIT("Set BIST test mode.\n");
    tcpc_modify8(port, TCPC_REG_TCPC_CTRL, 0, TCPC_CTRL_BIST_TEST_MODE);
    return;
}

void tcpm_enable_voltage_monitoring(unsigned int port)
{
    tcpc_modify8(port, TCPC_REG_POWER_CTRL, TCPC_PWR_CTRL_VBUS_VOLTAGE_MONITOR, 0);

    return;
}

void tcpm_enable_vbus_detect(unsigned int port)
{
    // Enable VBUS detect. (not active until voltage monitoring is enabled)
    tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_ENABLE_VBUS_DETECT);

    tcpm_enable_voltage_monitoring(port);

    return;
}
#if 0
void tcpm_register_dump(unsigned int port)
{
    uint_fast8_t i;
	uint8_t i;
    uint8_t data;

    non_reentrant_context_save();   

    CRIT("\nTCPC-%u Dump:\n", port);

    for (i = 0x10; i < 0x16; i++)
    {
        tcpc_read8(port, i, &data);
        CRIT("[%02x] %02x\n", i, data); 
    }

    for (i = 0x18; i < 0x20; i++)
    {
        tcpc_read8(port, i, &data);
        CRIT("[%02x] %02x\n", i, data); 
    }

    for (i = 0x2E; i < 0x30; i++)
    {
        tcpc_read8(port, i, &data);
        CRIT("[%02x] %02x\n", i, data); 
    }

    for (i = 0x70; i < 0x7A; i++)
    {
        tcpc_read8(port, i, &data);
        CRIT("[%02x] %02x\n", i, data); 
    }

    tcpc_read8(port, 0x90, &data);
    CRIT("[90] %02x\n",data); 

    tcpc_read8(port, 0x94, &data);
    CRIT("[94] %02x\n",data); 
    
    // Restore interrupt enable.
    non_reentrant_context_restore();

    return;
}
#endif
//void tcpm_get_cc_state(uint8_t cc_status, cc_state_t *cc1, cc_state_t *cc2)
//{
//    *cc1 = cc1;
//    *cc2 = cc2;
//
//    if (*cc1 != CC_STATE_OPEN)
//    {
//        *cc1 |= (cc_status & CC_STATUS_CONNECT_RESULT);
//    }
//
//    if (*cc2 != CC_STATE_OPEN)
//    {
//        *cc2 |= (cc_status & CC_STATUS_CONNECT_RESULT);
//    }
//
//    return;
//}


void tcpm_register_callbacks(const tcpm_callbacks_t *callbacks)
{
    conn_state_change_cbk = callbacks->conn_state_change_cbk;
    current_change_cbk = callbacks->current_change_cbk;
    volt_alarm_cbk = callbacks->volt_alarm_cbk;

    pd_hard_reset_cbk = callbacks->pd_hard_reset_cbk;
    pd_transmit_cbk = callbacks->pd_transmit_cbk;
    pd_receive_cbk = callbacks->pd_receive_cbk;

    return;
}


static void tcpm_set_state(tcpc_device_t *dev, tcpc_state_t new_state)
{
    INFO("%s\n", state2string[new_state]);

    dev->last_state = dev->state;
    dev->state = new_state;
    dev->state_change = true;

#ifdef LED_DEBUG
    switch (dev->state)
    {
        // Unattached states.
        case TCPC_STATE_UNATTACHED_SNK:
        case TCPC_STATE_UNATTACHED_SRC:
            LED_OFF(LED_BOTH);
            if (dev->role == ROLE_DRP) LED_ON(LED_GREEN);
            timer_start(&timer_led_blink, BLINK_DELAY_MS, tcpm_led_blink);
            break;

        // Sink attached states.
        case TCPC_STATE_ATTACHED_SNK:
        case TCPC_STATE_DEBUG_ACC_SNK:
            timer_cancel(&timer_led_blink);
            LED_OFF(LED_BOTH);
            LED_ON(LED_RED);
            break;

            // Source attached states.
        case TCPC_STATE_ATTACHED_SRC:
        case TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC:
        case TCPC_STATE_ORIENTED_DEBUG_ACC_SRC:
        case TCPC_STATE_AUDIO_ACC:
            timer_cancel(&timer_led_blink);
            LED_OFF(LED_BOTH);
            LED_ON(LED_GREEN);
            break;
    }
#endif

    return;
}


static void timeout_cc_debounce(unsigned int port)
{
    //uint8_t cc_status;
    unsigned int cc1, cc2;
    tcpc_device_t *dev = &tcpc_dev[port];

    INFO("timeout_cc_debounce\n");

//    non_reentrant_context_save();
//
//    tcpc_read8(port, TCPC_REG_CC_STATUS, &cc_status);
//
//    // Verify CC status has not changed. (interrupt latency could delay update of CC status)
//    if (dev->cc_status == cc_status)
//    {
//        cc1 = TCPC_CC1_STATE(cc_status);
//        cc2 = TCPC_CC2_STATE(cc_status);

        cc1 = TCPC_CC1_STATE(dev->cc_status);
        cc2 = TCPC_CC2_STATE(dev->cc_status);

        if (dev->state == TCPC_STATE_ATTACH_WAIT_SRC)
        {
            if ((cc1 == CC_SRC_STATE_RA) && 
                (cc2 == CC_SRC_STATE_RA))
            {
                tcpm_set_state(dev, TCPC_STATE_AUDIO_ACC);
            }
            else if ((cc1 == CC_SRC_STATE_RD) && 
                     (cc2 == CC_SRC_STATE_RD))
            {
                tcpm_set_state(dev, TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC);
            }
            else if ((cc1 == CC_SRC_STATE_RD) || 
                     (cc2 == CC_SRC_STATE_RD))
            {
                if ((dev->flags & TC_FLAGS_TRY_SNK) && 
                    (dev->role == ROLE_DRP))
                {
                    tcpm_set_state(dev, TCPC_STATE_TRY_SNK);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_ATTACHED_SRC);
                }
            }
            else /* Invalid CC state */
            {
                tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
            }
        }
        else if (dev->state == TCPC_STATE_ATTACH_WAIT_SNK)
        {
//            if (tcpm_is_vbus_present(port))
            if (dev->vbus_present)
            {
                // Debug Accessory if SNK.Rp on both CC1 and CC2. 
                if ((cc1 != CC_SNK_STATE_OPEN) && 
                    (cc2 != CC_SNK_STATE_OPEN))
                {
                    tcpm_set_state(dev, TCPC_STATE_DEBUG_ACC_SNK);
                }
                else if ((dev->flags & TC_FLAGS_TRY_SRC) && 
                         (dev->role == ROLE_DRP))
                {
                    tcpm_set_state(dev, TCPC_STATE_TRY_SRC);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_ATTACHED_SNK);
                }                
            }
            else
            {
                tcpm_set_state(dev, TCPC_STATE_ATTACH_WAIT_SNK_WAIT4VBUS);
            }
        }
        else if (dev->state == TCPC_STATE_AUDIO_ACC)
        {
            if ((cc1 == CC_SNK_STATE_OPEN) && 
                (cc2 == CC_SNK_STATE_OPEN))
            {
                if (dev->role == ROLE_SNK)
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
                }
            }
        }
//    }
//    else /* CC pins changed since connection detection */
//    {
//        // Do nothing.
//    }

//    non_reentrant_context_restore();

    return;
}

static void timeout_drp_try(unsigned int port)
{
    unsigned int cc1, cc2;
//    uint8_t cc_status;
    tcpc_device_t *dev = &tcpc_dev[port];

    DEBUG("timeout_cc_debounce\n");
    
//    non_reentrant_context_save();

    if (dev->state == TCPC_STATE_TRY_SNK)
    {
        tcpm_set_state(dev, TCPC_STATE_TRY_SNK_LOOK4SRC);
    }
    else
    {
        // Read CC status in case there is latency in processing TCPC interrupts.
//        tcpc_read8(port, TCPC_REG_CC_STATUS, &cc_status);
//        CRIT("timeout_drp_try: cc_status = 0x%x\n", cc_status);
//        cc1 = TCPC_CC1_STATE(cc_status);
//        cc2 = TCPC_CC2_STATE(cc_status);

        cc1 = TCPC_CC1_STATE(dev->cc_status);
        cc2 = TCPC_CC2_STATE(dev->cc_status);

        // If neither CC pin is in Rd state.
        if ((cc1 != CC_SRC_STATE_RD) && 
            (cc2 != CC_SRC_STATE_RD))
        {
            if (dev->state == TCPC_STATE_TRY_SRC)
            {
                tcpm_set_state(dev, TCPC_STATE_TRY_WAIT_SNK);
            }
            else if (dev->state == TCPC_STATE_TRY_WAIT_SRC)
            {
                tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
            }
        }
    }

//    non_reentrant_context_restore();

    return;
}


static void timeout_pd_debounce(unsigned int port)
{
 //   uint8_t cc_status;
    unsigned int cc1, cc2;
    tcpc_device_t *dev = &tcpc_dev[port];

    INFO("timeout_pd_debounce\n");

//    non_reentrant_context_save();
//
//    tcpc_read8(port, TCPC_REG_CC_STATUS, &cc_status);

    // Check CC state has not changed.
//    if (dev->cc_status == cc_status)
//    {
//        cc1 = TCPC_CC1_STATE(cc_status);
//        cc2 = TCPC_CC2_STATE(cc_status);
        cc1 = TCPC_CC1_STATE(dev->cc_status);
        cc2 = TCPC_CC2_STATE(dev->cc_status);

        if ((dev->state == TCPC_STATE_TRY_SNK_LOOK4SRC) ||
            (dev->state == TCPC_STATE_TRY_WAIT_SNK))
        {
            // Check for Rp on exactly one pin.
            if (((cc1 == CC_SNK_STATE_OPEN) && (cc2 != CC_SNK_STATE_OPEN)) || 
                ((cc1 != CC_SNK_STATE_OPEN) && (cc2 == CC_SNK_STATE_OPEN)))
            {
                // Enable VBUS detection.
                tcpm_enable_vbus_detect(dev->port);

                if (tcpm_is_vbus_present(port))
                {
                    tcpm_set_state(dev, TCPC_STATE_ATTACHED_SNK);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_WAITING_FOR_VBUS_SNK);
                }
            }
            else /* No source detected */
            {
                tcpm_set_state(dev, TCPC_STATE_TRY_WAIT_SRC);
            }
        }
        else if ((dev->state == TCPC_STATE_TRY_SRC) || 
                 (dev->state == TCPC_STATE_TRY_WAIT_SRC))
        {
            // Check for Rd on exactly one pin.
            if (((cc1 == CC_SRC_STATE_RD) && (cc2 != CC_SRC_STATE_RD)) || 
                ((cc1 != CC_SRC_STATE_RD) && (cc2 == CC_SRC_STATE_RD)))
            {
                tcpm_set_state(dev, TCPC_STATE_ATTACHED_SRC);
            }
        }

        if ((cc1 == CC_STATE_OPEN) && (cc2 == CC_STATE_OPEN))
        {
            /* Disconnected */

            if ((dev->state == TCPC_STATE_ATTACHED_SRC) ||
//                (dev->state == TCPC_STATE_ATTACH_WAIT_SRC) || 
                (dev->state == TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC) ||
                (dev->state == TCPC_STATE_ORIENTED_DEBUG_ACC_SRC))
            {
                if (dev->role == ROLE_DRP)
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
                }
            }
            else if (dev->state == TCPC_STATE_ATTACH_WAIT_SNK)
            {
                if (dev->role == ROLE_DRP)
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
                }
            }
            else if (dev->state == TCPC_STATE_TRY_WAIT_SNK)
            {
                tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
            }
            else if (dev->state == TCPC_STATE_ATTACHED_SNK)
            {
                // Do nothing.  Wait for VBUS to be removed.
            }
            else if (dev->state == TCPC_STATE_DEBUG_ACC_SNK)
            {
                // Do nothing.  Wait for VBUS to be removed.
            }
            else
            {
                if (dev->role == ROLE_SNK)
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
                }
                else
                {
                    tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
                }
            }
        }
//    }

//    non_reentrant_context_restore();

    return;
}

static void timeout_error_recovery(unsigned int port)
{
    tcpc_device_t *dev = &tcpc_dev[port];

    if (dev->role == ROLE_SNK)
    {
        tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
    }
    else
    {
        tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
    }

    return;
}


#define RX_DET_MASK (TCPC_RX_EN_HARD_RESET | TCPC_RX_EN_SOP)

void tcpm_enable_pd_receive(unsigned int port)
{
    INFO("Enable PD Rx\n");
    // Ensure BIST test mode is disabled.
    tcpc_modify8(port, TCPC_REG_TCPC_CTRL, TCPC_CTRL_BIST_TEST_MODE, 0);

    // Enable PD recieve. (will be cleared by TCPC upon hard reset or disconnect)
    tcpc_write8(port, TCPC_REG_RX_DETECT, RX_DET_MASK);

    return;
}

void tcpm_disable_pd_receive(unsigned int port)
{
    // Disable PD recieve.
    tcpc_write8(port, TCPC_REG_RX_DETECT, 0);

    return;
}


/* Returns VBUS voltage in 25mV units.  Must be in ATTACHED state where VBUS monitoring is enabled. */
uint16_t tcpm_read_vbus_voltage(unsigned int port)
{
    uint16_t volt;

    tcpc_read16(port, TCPC_REG_VBUS_VOLTAGE, &volt);

    // Assume no scale factor.
    return(volt & 0x3FF);
}

void tcpm_force_discharge(unsigned int port, uint16_t threshold_25mv)
{
    tcpc_write16(port, TCPC_REG_VBUS_STOP_DISCHARGE_THRESH, threshold_25mv);

    tcpc_modify8(port, TCPC_REG_POWER_CTRL, 0, TCPC_PWR_CTRL_FORCE_DISCHARGE);

    return;
}

void tcpm_set_autodischarge_disconnect(unsigned int port, bool enable)
{
    if (enable)
    {
        tcpc_modify8(port, TCPC_REG_POWER_CTRL, 0, TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT);
    }
    else
    {
        tcpc_modify8(port, TCPC_REG_POWER_CTRL, TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT, 0);
    }

    return;
}

/* threshold has 25mV LSB, set to zero to use default (vSafe5V) */
void tcpm_set_sink_disconnect_threshold(unsigned int port, uint16_t threshold_25mv)
{
    tcpc_write16(port, TCPC_REG_VBUS_SINK_DISCONNECT_THRESH, threshold_25mv);
    return;
}

void tcpm_get_msg_header_type(unsigned int port, uint8_t *frame_type, uint16_t *header)
{
    // Read Rx header.
    tcpc_read16(port, TCPC_REG_RX_HDR, header);

    // Read Rx SOP type.
    tcpc_read8(port, TCPC_REG_RX_BUF_FRAME_TYPE, frame_type);

    return;
}

#if 0  // Use this funcion if using an I2C read transfer instead of SMBus block read.
void tcpm_read_message(unsigned int port, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t byte_cnt;

    // Read Rx Byte Cnt.
    tcpc_read8(port, TCPC_REG_RX_BYTE_CNT, &byte_cnt);

    if (tcpc_dev[port].silicon_revision == 0)
    {
        if (byte_cnt == 0)
        {
            CRIT("\n## ERROR: tcpm_read_msg: zero byte cnt!\n\n");
        }

        if (byte_cnt != (len + 3))
        {
            CRIT("RxByteCnt = %u invalid, using length %u from header!\n", byte_cnt, len);
            byte_cnt = len + 3;
        }
    }

    if (byte_cnt > 3)
    {
        // Subtract 3-bytes for frame type and header.
        byte_cnt -= 3;

        if (byte_cnt)
        {
            tcpc_read_block(port, TCPC_REG_RX_DATA, buf, byte_cnt);
        }
    }

    DEBUG("rx_buf: 0x");
    for (i = 0; i < byte_cnt; i++)
    {
        DEBUG("%02x ", buf[i]);
    }
    DEBUG("\n");

    return;
}
#else
void tcpm_read_message(unsigned int port, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t byte_cnt;
    uint8_t local_buf[32];

    // Read Rx Byte Cnt.
    tcpc_read8(port, TCPC_REG_RX_BYTE_CNT, &byte_cnt);

    if (tcpc_dev[port].silicon_revision == 0)
    {
        if (byte_cnt == 0)
        {
            CRIT("\n## ERROR: tcpm_read_msg: zero byte cnt!\n\n");
        }

        if (byte_cnt != (len + 3))
        {
            CRIT("RxByteCnt = %u invalid, using length %u from header!\n", byte_cnt, len);
            byte_cnt = len + 3;
        }
    }

    if (byte_cnt > 3)
    {
        tcpc_read_block(port, TCPC_REG_RX_BYTE_CNT, local_buf, byte_cnt + 1);
    
        // Copy message to buffer. (Subtract 3-bytes for frame type and header)
        memcpy(buf, &local_buf[4], (byte_cnt - 3));
    
        DEBUG("rx_buf: 0x");
        for (i = 0; i < (byte_cnt - 3); i++)
        {
            DEBUG("%02x ", buf[i]);
        }
        DEBUG("\n");
    }

    return;
}
#endif

// buf[0] Tx byte cnt including 2-byte header.
void tcpm_transmit(unsigned int port, uint8_t *buf, tcpc_transmit_t sop_type)
{
    if (buf)
    {
        // Write Tx byte cnt, header, and buffer all together. 
        // (Add 1 to length since we are also writing byte cnt)
//        tcpc_write_block(port, TCPC_REG_TX_BYTE_CNT, buf, buf[0] + 1);
        tcpc_write_block(port, TCPC_REG_TX_BYTE_CNT, &buf[1], buf[0]);
    }

    // Start transmit.
    tcpc_write8(port, TCPC_REG_TRANSMIT, TCPC_REG_TRANSMIT_SET(sop_type));

    return;
}


void tcpm_error_recovery(unsigned int port)
{
    tcpc_device_t *dev = &tcpc_dev[port];

    if (dev->state == TCPC_STATE_ERROR_RECOVERY)
    {
        return;
    }
/* Dan M gotta fix this
    non_reentrant_context_save();   
*/
    tcpm_set_state(dev, TCPC_STATE_ERROR_RECOVERY);

    // Disable VBUS.
    tcpm_src_vbus_disable(port);
    tcpm_snk_vbus_disable(port);

    // AutoDischargeDisconnect=1, Disable VCONN, Disable VBUS voltage monitor/alarms.
    tcpc_write8(port, TCPC_REG_POWER_CTRL, 
                (TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT | TCPC_PWR_CTRL_DEFAULTS));

    // Remove CC1 & CC2 terminations.
    tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_OPEN, CC_OPEN));

//    timer_setup(&dev->timer, timeout_error_recovery, port);
    timer_start(&dev->timer, T_ERROR_RECOVERY_MS, timeout_error_recovery);
/* Dan M gotta fix this
    non_reentrant_context_restore();
*/
    return;
}

#define POWER_ROLE_BIT  1

#define MV_TO_25MV(x) ((x)/25)
#define VSAFE0V_MAX   MV_TO_25MV(800)   /* 0.8V max in 25mV units */

void tcpm_connection_state_machine(unsigned int port)
{
    unsigned int cc1, cc2;
    tcpc_role_cc_t cc_pull;
    tcpc_device_t *dev = &tcpc_dev[port];
    uint8_t reg;

    if (!dev->state_change)
        return;

    cc1 = TCPC_CC1_STATE(dev->cc_status);
    cc2 = TCPC_CC2_STATE(dev->cc_status);

    switch (dev->state)
    {
        case TCPC_STATE_UNATTACHED_SNK:
        case TCPC_STATE_UNATTACHED_SRC:
            timer_cancel(&dev->timer);

            if (dev->silicon_revision == 0)
            {            
                /****** TUSB422 PG1.0 workaround for Tx Discarded issue (CDDS #38).  ******/
                tcpc_read8(port, TCPC_REG_MSG_HDR_INFO, &reg);
                
                if (reg & POWER_ROLE_BIT)
                {
                    // Clear message header info.
                    tcpc_write8(port, TCPC_REG_MSG_HDR_INFO, 0);     
                }
                else
                {
                    // Remove CC1 & CC2 terminations.
                    tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                                tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_OPEN, CC_OPEN));
                
                    // Enable Voltage monitoring.
                    tcpm_enable_voltage_monitoring(dev->port);

                    vbus_timed_out = false;
                    timer_start(&dev->timer, T_VBUS_DISCHARGE_MS, timeout_vbus);

                    // Wait for vSafe0V.
                    while ((tcpm_read_vbus_voltage(dev->port) > VSAFE0V_MAX) &&
                           !vbus_timed_out);
                
                    // Clear message header info.
                    tcpc_write8(port, TCPC_REG_MSG_HDR_INFO, 0);     
                }
            }

            // Disable VBUS.
            tcpm_src_vbus_disable(port);
            tcpm_snk_vbus_disable(port);

            // Configure role control.
            if (dev->role == ROLE_DRP)
            {
                // Set first CC pin state for autonomous DRP toggle.
                cc_pull = (dev->state == TCPC_STATE_UNATTACHED_SNK) ? CC_RD : CC_RP;

                tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                            tcpc_reg_role_ctrl_set(true, dev->rp_val, cc_pull, cc_pull));
            }
            else if (dev->role == ROLE_SNK)
            {
                tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                            tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RD, CC_RD));
            }
            else /* ROLE_SRC */
            {
                tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                            tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RP, CC_RP));
            }

            // AutoDischargeDisconnect=0, Disable VCONN, Disable VBUS voltage monitor/alarms.   
            tcpc_write8(port, TCPC_REG_POWER_CTRL, TCPC_PWR_CTRL_DEFAULTS);

            // Set sink disconnect threshold to default (vSafe5V).
            tcpm_set_sink_disconnect_threshold(port, 0);

            // Set TCPC control to default.
            tcpc_write8(port, TCPC_REG_TCPC_CTRL, 0);

            // Notify upper layer.
            tcpm_notify_conn_state(port, dev->state);

            // Look for connection.
            tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_SRC_LOOK4CONNECTION);
            break;

        case TCPC_STATE_TRY_WAIT_SRC:
            // Pull up both CC pins to Rp.
            tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                        tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RP, CC_RP));

            timer_start(&dev->timer, T_DRP_TRY_MS, timeout_drp_try);
            break;

        case TCPC_STATE_TRY_WAIT_SNK:
            // Terminate both CC pins to Rd.
            tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                        tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_RD, CC_RD));

            timer_start(&dev->timer, T_PD_DEBOUNCE_MS, timeout_pd_debounce);
            break;

        case TCPC_STATE_ATTACHED_SNK:
        case TCPC_STATE_DEBUG_ACC_SNK:
            timer_cancel(&dev->timer);

            if (cc1 > cc2)
            {
                // CC1 used for USB-PD.
                dev->plug_polarity = PLUG_UNFLIPPED;

                DEBUG("SRC current: %s\n", 
                      (cc1 == CC_SNK_STATE_POWER30) ? "3.0A" : 
                      (cc1 == CC_SNK_STATE_POWER15) ? "1.5A" : 
                      (cc1 == CC_SNK_STATE_DEFAULT) ? "500/900mA" : "?"); 
            }
            else /* CC1 voltage < CC2 voltage */
            {
                // CC2 used for USB-PD.
                dev->plug_polarity = PLUG_FLIPPED;

                DEBUG("SRC current: %s\n", 
                      (cc2 == CC_SNK_STATE_POWER30) ? "3.0A" : 
                      (cc2 == CC_SNK_STATE_POWER15) ? "1.5A" : 
                      (cc2 == CC_SNK_STATE_DEFAULT) ? "500/900mA" : "?"); 
            }

            // Set plug orientation for attached states.
            tcpc_write8(port, TCPC_REG_TCPC_CTRL, 
                        (dev->plug_polarity == PLUG_FLIPPED) ? TCPC_CTRL_PLUG_ORIENTATION : 0);

            if (dev->state == TCPC_STATE_ATTACHED_SNK)
            {
                // Enable VBUS sink.
                tcpm_snk_vbus_enable(port);
            }

            // Update msg header info for UFP sink.
            tcpc_write8(port, TCPC_REG_MSG_HDR_INFO, TCPC_REG_MSG_HDR_INFO_SET(0, PD_DATA_ROLE_UFP, PD_PWR_ROLE_SNK));

            // AutoDischargeDisconnect=1, Enable VBUS voltage monitor, Enable Bleed discharge, Disable voltage alarms.
            tcpc_write8(port, TCPC_REG_POWER_CTRL, 
                        (TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT | 
                         TCPC_PWR_CTRL_ENABLE_BLEED_DISCHARGE | 
                         TCPC_PWR_CTRL_DISABLE_VOLTAGE_ALARM));

            // Notify upper layers.
            tcpm_notify_conn_state(port, dev->state);
            break;

        case TCPC_STATE_AUDIO_ACC:
            timer_cancel(&dev->timer);

            // Notify upper layers.
            tcpm_notify_conn_state(port, dev->state);
            break;

        case TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC:
            timer_cancel(&dev->timer);

            // Update msg header info for DFP source.
            tcpc_write8(port, TCPC_REG_MSG_HDR_INFO, TCPC_REG_MSG_HDR_INFO_SET(0, PD_DATA_ROLE_DFP, PD_PWR_ROLE_SRC));               

            // AutoDischargeDisconnect=1, VCONN=off, Enable VBUS voltage monitor & voltage alarms.
            tcpc_write8(port, TCPC_REG_POWER_CTRL, TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT);

            // Enable VBUS source.
            tcpm_src_vbus_5v_enable(port);  

            // Notify upper layers.
            tcpm_notify_conn_state(port, dev->state);
            break;

        case TCPC_STATE_ATTACHED_SRC:
        case TCPC_STATE_ORIENTED_DEBUG_ACC_SRC:
            timer_cancel(&dev->timer);

            if (cc1 == CC_SRC_STATE_RD)
            {
                dev->plug_polarity = PLUG_UNFLIPPED;
            }
            else /* Rd on CC2 */
            {
                dev->plug_polarity = PLUG_FLIPPED;
            }

            // Set plug orientation for attached states.
            tcpc_write8(port, TCPC_REG_TCPC_CTRL, 
                        (dev->plug_polarity == PLUG_FLIPPED) ? TCPC_CTRL_PLUG_ORIENTATION : 0);

            // Update msg header info for DFP source.
            tcpc_write8(port, TCPC_REG_MSG_HDR_INFO, TCPC_REG_MSG_HDR_INFO_SET(0, PD_DATA_ROLE_DFP, PD_PWR_ROLE_SRC));               

            if (dev->state == TCPC_STATE_ATTACHED_SRC)
            {
                if ((cc1 == CC_SRC_STATE_RA) || 
                    (cc2 == CC_SRC_STATE_RA))
                {
                    // AutoDischargeDisconnect=1, Enable VCONN, Enable VBUS voltage monitor & alarms.
                    tcpc_write8(port, TCPC_REG_POWER_CTRL, 
                                (TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT | 
                                 TCPC_PWR_CTRL_ENABLE_VCONN));
                }
                else /* No Ra detected. Do not enable VCONN */
                {
                    // AutoDischargeDisconnect=1, VCONN=off, Enable VBUS voltage monitor/alarms.
                    tcpc_write8(port, TCPC_REG_POWER_CTRL, TCPC_PWR_CTRL_AUTO_DISCHARGE_DISCONNECT);
                }

                // Enable VBUS source.
                tcpm_src_vbus_5v_enable(port);     

                tcpm_enable_vbus_detect(port);
                while (!tcpm_is_vbus_present(port));
            }

            // Notify upper layers.
            tcpm_notify_conn_state(port, dev->state);
            break;

        case TCPC_STATE_TRY_SRC:
        case TCPC_STATE_TRY_SNK:
            if (dev->silicon_revision == 0)
            {
                /****** TUSB422 PG1.0 workaround for role change issue (CDDS #41).  ******/
                // Disable VBUS detect.
                tcpc_write8(port, TCPC_REG_COMMAND, TCPC_CMD_DISABLE_VBUS_DETECT);

                // Remove CC1 & CC2 terminations.
                tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                            tcpc_reg_role_ctrl_set(false, dev->rp_val, CC_OPEN, CC_OPEN));
            }

            // Try.SRC - Both CC1 and CC2 pulled to Rp.
            // Try.SNK - Both CC1 and CC2 terminated to Rd.
            cc_pull = (dev->state == TCPC_STATE_TRY_SNK) ? CC_RD : CC_RP;

            tcpc_write8(port, TCPC_REG_ROLE_CTRL, 
                        tcpc_reg_role_ctrl_set(false, dev->rp_val, cc_pull, cc_pull));

            
            timer_start(&dev->timer, T_DRP_TRY_MS, timeout_drp_try);
            break;

        case TCPC_STATE_TRY_SNK_LOOK4SRC:
            // Check for Rp on exactly one CC pin.
            if (((cc1 == CC_SNK_STATE_OPEN) && (cc2 != CC_SNK_STATE_OPEN)) || 
                ((cc1 != CC_SNK_STATE_OPEN) && (cc2 == CC_SNK_STATE_OPEN)))
            {
                timer_start(&dev->timer, T_PD_DEBOUNCE_MS, timeout_pd_debounce);
            }
            break;

        case TCPC_STATE_ERROR_RECOVERY:
            // Do nothing. Everything is handled in tcpm_error_recovery() which is called directly.
            break;

        default:
            // Do nothing.
            break;
    }

    // Clear flag.
    dev->state_change = false;

    return;
}



// Call in main() loop.
void tcpm_connection_task(void)
{
#if NUM_TCPC_DEVICES > 1

    unsigned int port_num = NUM_TCPC_DEVICES - 1;

    non_reentrant_context_save();

    do
    {
        tcpm_connection_state_machine(port_num);

    } while (port_num--);
/* Dan M gotta fix this
    non_reentrant_context_restore();
*/
#else
/* Dan M gotta fix this
    non_reentrant_context_save();
*/
    tcpm_connection_state_machine(0);
/* Dan M gotta fix this
    non_reentrant_context_restore();
*/
#endif
    return;
}


static void alert_cc_status_handler(tcpc_device_t *dev)
{
    uint8_t cc_status;
    unsigned int cc1, cc2;

    // Read CC status.
    tcpc_read8(dev->port, TCPC_REG_CC_STATUS, &cc_status);

    CRIT("CC status = 0x%x\n", cc_status);

    if (!(cc_status & CC_STATUS_LOOKING4CONNECTION))
    {
        cc1 = TCPC_CC1_STATE(cc_status);
        cc2 = TCPC_CC2_STATE(cc_status);

        switch (dev->state)
        {
            case TCPC_STATE_UNATTACHED_SRC:
            case TCPC_STATE_UNATTACHED_SNK:
                if (cc_status & CC_STATUS_CONNECT_RESULT)
                {
                    if ((cc1 != CC_SNK_STATE_OPEN) || 
                        (cc2 != CC_SNK_STATE_OPEN))
                    {
                        // Enable VBUS detection.
                        tcpm_enable_vbus_detect(dev->port);

                        tcpm_set_state(dev, TCPC_STATE_ATTACH_WAIT_SNK);

                        // Debounce CC. 
                        timer_start(&dev->timer, T_CC_DEBOUNCE_MS, timeout_cc_debounce);
                    }
                }
                else /* TCPC is presenting Rp */
                {
                    // If Rd on either CC pin or Ra on both CC pins.
                    if (((cc1 == CC_SRC_STATE_RD) || (cc2 == CC_SRC_STATE_RD)) ||
                        ((cc1 == CC_SRC_STATE_RA) && (cc2 == CC_SRC_STATE_RA)))
                    {
                        tcpm_set_state(dev, TCPC_STATE_ATTACH_WAIT_SRC);

                        // Debounce CC. 
                        timer_start(&dev->timer, T_CC_DEBOUNCE_MS, timeout_cc_debounce);
                    }                   
                }
                break;

            case TCPC_STATE_ATTACHED_SNK:
            case TCPC_STATE_DEBUG_ACC_SNK:
                // If open state on CC1 and CC2.
                if ((cc1 == CC_SNK_STATE_OPEN) && 
                    (cc2 == CC_SNK_STATE_OPEN))
                {
                    // Do nothing. Wait for VBUS removal. 
                }
                else
                {
                    // Report current change to upper layers.
                    if (dev->plug_polarity == PLUG_UNFLIPPED)
                    {
                        tcpm_notify_current_change(dev->port, (tcpc_cc_snk_state_t)cc1);
                    }
                    else
                    {
                        tcpm_notify_current_change(dev->port, (tcpc_cc_snk_state_t)cc2);
                    }
                }
                break;

            case TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC:
                if ((cc1 == CC_SRC_STATE_RA) || 
                    (cc2 == CC_SRC_STATE_RA))
                {
                    tcpm_set_state(dev, TCPC_STATE_ORIENTED_DEBUG_ACC_SRC);
                }
                break;

//            case TCPC_STATE_AUDIO_ACC:
//                if ((cc1 == CC_STATE_OPEN) && 
//                    (cc2 == CC_STATE_OPEN))
//                {
//                    // Debounce. 
//                    // (unlike other attached states, Audio Acc debounces for tCCDebounce)
//                    timer_start(&dev->timer, T_CC_DEBOUNCE_MS, timeout_cc_debounce);
//                }
//                break;

//            case TCPC_STATE_ATTACHED_SRC:
//            case TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC:
//            case TCPC_STATE_ORIENTED_DEBUG_ACC_SRC:
//            case TCPC_STATE_TRY_WAIT_SNK:
//            case TCPC_STATE_ATTACH_WAIT_SNK_WAIT4VBUS:
//                // If open state on CC1 and CC2.
//                if ((cc1 == CC_STATE_OPEN) && 
//                    (cc2 == CC_STATE_OPEN))
//                {
//                    // Debounce. 
//                    timer_start(&dev->timer, T_PD_DEBOUNCE_MS, timeout_pd_debounce);
//                }
//
//                if (dev->state == TCPC_STATE_UNORIENTED_DEBUG_ACC_SRC)
//                {
//                    if ((cc1 == CC_SRC_STATE_RA) || 
//                        (cc2 == CC_SRC_STATE_RA))
//                    {
//                        tcpm_set_state(dev, TCPC_STATE_ORIENTED_DEBUG_ACC_SRC);
//                    }
//                }
//                break;

            case TCPC_STATE_TRY_SNK:
                // Do nothing.  Do not monitor CC1 & CC2 until tDRPTry expires.
                break;

            case TCPC_STATE_TRY_SNK_LOOK4SRC:
                // Check for Rp on exactly one CC pin.
                if (((cc1 == CC_SNK_STATE_OPEN) && (cc2 != CC_SNK_STATE_OPEN)) || 
                    ((cc1 != CC_SNK_STATE_OPEN) && (cc2 == CC_SNK_STATE_OPEN)))
                {
                    // Debounce. 
                    timer_start(&dev->timer, T_PD_DEBOUNCE_MS, timeout_pd_debounce);
                }
                break;

            case TCPC_STATE_TRY_SRC:
            case TCPC_STATE_TRY_WAIT_SRC:
                // Check for Rd on exactly one CC pin.
                if (((cc1 == CC_SRC_STATE_RD) && (cc2 != CC_SRC_STATE_RD)) || 
                    ((cc1 != CC_SRC_STATE_RD) && (cc2 == CC_SRC_STATE_RD)))
                {
                    // Debounce. 
                    timer_start(&dev->timer, T_PD_DEBOUNCE_MS, timeout_pd_debounce);
                }
                break;

            case TCPC_STATE_ATTACH_WAIT_SRC:
                if (((cc1 == CC_STATE_OPEN) && (cc2 == CC_STATE_OPEN)) || 
                    ((cc1 == CC_STATE_OPEN) && (cc2 == CC_SRC_STATE_RA)) ||
                    ((cc1 == CC_SRC_STATE_RA) && (cc2 == CC_STATE_OPEN)))
                {
                    if (dev->role == ROLE_DRP)
                    {
                        tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
                    }
                    else
                    {
                        tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SRC);
                    }
                }
                break;

            default:
                break;
        }

        // ALL attached SRC states.  oriented, unoriented debug.src.
        // AttachWait SNK
        // TryWait.SNK
        
        // If open state on CC1 and CC2.
        if ((cc1 == CC_STATE_OPEN) && 
            (cc2 == CC_STATE_OPEN))
        {
            if ((dev->state != TCPC_STATE_UNATTACHED_SRC) ||
                (dev->state != TCPC_STATE_UNATTACHED_SNK))
            {
                if (dev->state == TCPC_STATE_AUDIO_ACC)
                {
                    // Debounce. 
                    // (unlike other attached states, Audio Acc debounces for tCCDebounce)
                    timer_start(&dev->timer, T_CC_DEBOUNCE_MS, timeout_cc_debounce);
                }
                else
                {
                    // Debounce. 
                    timer_start(&dev->timer, T_PD_DEBOUNCE_MS, timeout_pd_debounce);
                }
            }
        }
    }

    // Save CC status.
    dev->cc_status = cc_status;

    return;
}


static void alert_power_status_handler(tcpc_device_t *dev)
{
    unsigned int cc1, cc2;
    uint8_t pwr_status;

    cc1 = TCPC_CC1_STATE(dev->cc_status);
    cc2 = TCPC_CC2_STATE(dev->cc_status);

    // Read power status.
    tcpc_read8(dev->port, TCPC_REG_POWER_STATUS, &pwr_status);

    INFO("pwr_status = 0x%02x\n", pwr_status);

    if (pwr_status & TCPC_PWR_STATUS_VBUS_PRESENT)
    {
        DEBUG("VBUS present\n");
        dev->vbus_present = true;

        if ((dev->state == TCPC_STATE_ATTACH_WAIT_SNK_WAIT4VBUS) ||
            (dev->state == TCPC_STATE_WAITING_FOR_VBUS_SNK))
        {
            // Debug Accessory if SNK.Rp on both CC1 and CC2. 
            if ((cc1 != CC_SNK_STATE_OPEN) && 
                (cc2 != CC_SNK_STATE_OPEN))
            {
                tcpm_set_state(dev, TCPC_STATE_DEBUG_ACC_SNK);
            }
            else if ((dev->flags & TC_FLAGS_TRY_SRC) && 
                     (dev->role == ROLE_DRP) &&
                     (dev->state == TCPC_STATE_ATTACH_WAIT_SNK_WAIT4VBUS))
            {
                tcpm_set_state(dev, TCPC_STATE_TRY_SRC);
            }
            else
            {
                tcpm_set_state(dev, TCPC_STATE_ATTACHED_SNK);
            }
        }
    }
    else /* VBUS below threshold */
    {
        dev->vbus_present = false;

        if ((dev->state == TCPC_STATE_ATTACHED_SNK) ||
            (dev->state == TCPC_STATE_DEBUG_ACC_SNK))
        {
            // Unattached.SNK.
            tcpm_set_state(dev, TCPC_STATE_UNATTACHED_SNK);
        }
    }

    return;
}


static void alert_fault_status_handler(tcpc_device_t *dev)
{
    uint8_t status;

    tcpc_read8(dev->port, TCPC_REG_FAULT_STATUS, &status);

    CRIT("Fault_Status = 0x%02x\n", status);    

    if (status & TCPC_AUTO_DIS_FAIL_STATUS)
    {
        // Stop VBUS discharge.
        tcpc_write8(dev->port, TUSB422_REG_VBUS_VCONN_CTRL, INT_VBUSDIS_DISABLE);
    }

    // Clear status.
    tcpc_write8(dev->port, TCPC_REG_FAULT_STATUS, status);

    return;
}

static void tcpm_alert_handler(unsigned int port)
{
    uint16_t alert;
    uint16_t clear_bits;
    tcpc_device_t *dev = &tcpc_dev[port];

    // Read alerts.
    tcpc_read16(port, TCPC_REG_ALERT, &alert);

    INFO("->P%u IRQ: 0x%04x\n", port, alert);

    // Clear alerts except voltage alarms, fault, vendor IRQ, and Rx status which cannot be cleared until after servicing.
    clear_bits = alert & ~(TCPC_ALERT_RX_STATUS | TCPC_ALERT_FAULT | TCPC_ALERT_VOLT_ALARM_HI | TCPC_ALERT_VOLT_ALARM_LO | TUSB422_ALERT_IRQ_STATUS);

    if (clear_bits)
    {
        tcpc_write16(port, TCPC_REG_ALERT, clear_bits);
    }
                 
    if (alert & TUSB422_ALERT_IRQ_STATUS)
    {
        tusb422_isr(port);

        // Clear alert.
        tcpc_write16(port, TCPC_REG_ALERT, TUSB422_ALERT_IRQ_STATUS);
    }

    if (alert & TCPC_ALERT_VBUS_DISCONNECT)
    {
        // Not active unless VBUS SNK DISCONNECT THRESHOLD is set.
        // Do nothing.  If we are sink, wait for POWER_STATUS.VbusPresent = 0.
    }

    if (alert & TCPC_ALERT_RX_BUF_OVERFLOW)
    {
        CRIT("Alert: Rx Buff overflow!\n");
    }

    if (alert & TCPC_ALERT_FAULT)
    {
        alert_fault_status_handler(dev);
        // Clear alert.
        tcpc_write16(port, TCPC_REG_ALERT, TCPC_ALERT_FAULT);
    }

    if (alert & TCPC_ALERT_VOLT_ALARM_LO)
    {
        tcpm_notify_voltage_alarm(port, false);
        // Clear alarm threshold.
        tcpc_write16(port, TCPC_REG_VBUS_VOLTAGE_ALARM_LO_CFG, 0);
        // Clear alert.
        tcpc_write16(port, TCPC_REG_ALERT, TCPC_ALERT_VOLT_ALARM_LO);
    }

    if (alert & TCPC_ALERT_VOLT_ALARM_HI)
    {
        tcpm_notify_voltage_alarm(port, true);
        // Clear alarm threshold.
        tcpc_write16(port, TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG, 0x3FF);
        // Clear alert.
        tcpc_write16(port, TCPC_REG_ALERT, TCPC_ALERT_VOLT_ALARM_HI);
    }

    // Sending a Hard Reset will set both SUCCESS and FAILED status.
    if (alert & TCPC_ALERT_TX_SUCCESS)
    {
        tcpm_notify_pd_transmit(port, TX_STATUS_SUCCESS);
    }
    else if (alert & TCPC_ALERT_TX_DISCARDED)
    {
        tcpm_notify_pd_transmit(port, TX_STATUS_DISCARDED);
    }
    else if (alert & TCPC_ALERT_TX_FAILED)
    {
        tcpm_notify_pd_transmit(port, TX_STATUS_FAILED);
    }

    if (alert & TCPC_ALERT_RX_HARD_RESET)
    {
        tcpm_notify_hard_reset(port);
    }

    if (alert & TCPC_ALERT_RX_STATUS)
    {
        tcpm_notify_pd_receive(port);

        // Clear alert.
        tcpc_write16(port, TCPC_REG_ALERT, TCPC_ALERT_RX_STATUS);
    }

    if (alert & TCPC_ALERT_POWER_STATUS)
    {
        alert_power_status_handler(dev);
    }

    if (alert & TCPC_ALERT_CC_STATUS)
    {
        alert_cc_status_handler(dev);
    }

    return;
}


void tcpm_port_init(unsigned int port, const tcpc_config_t *config)
{
    tcpc_device_t *dev = &tcpc_dev[port];
    uint8_t pwr_status;
    uint16_t id;

    tcpc_config(port, config->intf, config->slave_addr);

    dev->role = config->role;
    dev->flags = config->flags;
    dev->port = port;
    dev->rp_val = config->rp_val;

    dev->timer.data = port;
    dev->state = TCPC_STATE_DISABLED;
    dev->vbus_present = false;
    
    if (dev->role == ROLE_SNK)
    {
        tcpm_set_state(dev,TCPC_STATE_UNATTACHED_SNK);
    }
    else /* ROLE_SRC or ROLE_DRP */
    {
        tcpm_set_state(dev,TCPC_STATE_UNATTACHED_SRC);
    }

    CRIT("Port[%u]: addr: 0x%02x, %s, Rp: %s, %s.\n", 
         port, config->slave_addr, 
         (dev->role == ROLE_SRC) ? "SRC" : 
         (dev->role == ROLE_SNK) ? "SNK" : "DRP", 
         (dev->rp_val == RP_DEFAULT_CURRENT) ? "def" : 
         (dev->rp_val == RP_MEDIUM_CURRENT) ? "mid" : "hi", 
         (dev->flags & TC_FLAGS_TRY_SRC) ? "Try_SRC" : (dev->flags & TC_FLAGS_TRY_SNK) ? "Try_SNK" : "");

    // Read VID/PID/DID.
    tcpc_read16(port, TCPC_REG_VENDOR_ID, &id);
    CRIT("VID: 0x%04x\n", id); 
    if (id != 0x0451)
    {
        // Non-TI device.
        while(1);
    }
    tcpc_read16(port, TCPC_REG_PRODUCT_ID, &id);
    CRIT("PID: 0x%04x\n", id); 
    tcpc_read16(port, TCPC_REG_DEVICE_ID, &id);
    CRIT("DID: 0x%04x\n", id); 

    // Wait for TCPC init status to clear.
    do
    {
        tcpc_read8(port, TCPC_REG_POWER_STATUS, &pwr_status);

    } while (pwr_status & TCPC_PWR_STATUS_TCPC_INIT_STATUS);

    // Init vendor-specific.
    tusb422_init(port);

    // Read the silicon revision.
    dev->silicon_revision = tusb422_get_revision(port);

    return;
}

void tcpm_init(const tcpc_config_t *config)
{
    unsigned int port;

/* Dan M this needs to be looked at I am sure we don't need this
* init function
    tcpc_init();
*/
#ifdef LED_DEBUG
    timer_led_blink.data = 0;
#endif 

    // Init all TCPC devices.
    for (port = 0; port < NUM_TCPC_DEVICES; port++)
    {
        tcpm_port_init(port, &config[port]);
    }

    return;
}



//
// Events
//
//*****************************************************************************
//
//! Event for alert
//!
//! Event called by HAL when alert line goes low to indicate that an interrupt
//! has occurred
//!
//! \return  true to wake-up MCU, false to stay in LPMx
//
// *****************************************************************************
bool tcpm_alert_event(unsigned int port) {
//#if NUM_TCPC_DEVICES == 1
//    unsigned int loop_cnt = 2;
/* Dan M gotta fix this
    non_reentrant_context_save();   
*/
//    do
//    {
    tcpm_alert_handler(port);
//    } while ((tcpm_hal_get_interrupts_asserted() & INT_ALERTN_P0) && --loop_cnt);

    // Disable interrupts to avoid re-entry once the alerts are re-enabled
    // the interrupts will be enabled automatically when the ISR exits
/*Dan M we need to disable this in the linux side
    __disable_interrupt();
*/
/* Dan M gotta fix this
    non_reentrant_context_restore();
*/
    return true;
}



bool tcpm_vbus_snk_fault_event(unsigned int port) {
    CRIT("VBUS SNK fault!\n");

    return false;
}


bool tcpm_vbus_src_fault_event(unsigned int port) {
    CRIT("VBUS SRC fault!\n");

    return false;
}
