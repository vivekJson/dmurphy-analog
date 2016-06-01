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
#include "usb_pd_policy_engine.h"
#include "usb_pd_protocol.h"
#include "version.h"

#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))


static usb_pd_port_config_t pd_port_config[NUM_TCPC_DEVICES];  

/* Power negotiation is an atomic msg sequence so these variables can be global */
uint32_t rdo;
static uint32_t selected_pdo;
static uint8_t  selected_snk_pdo_idx;

usb_pd_port_config_t* usb_pd_pm_get_config(unsigned int port)
{
    return &pd_port_config[port];
}

#define PDO_MAX_CURRENT(pdo) ((pdo) & 0x3FF)
#define PDO_MAX_POWER(pdo)   ((pdo) & 0x3FF)
#define PDO_MIN_VOLTAGE(pdo) (((pdo) >> 10) & 0x3FF)
#define PDO_MAX_VOLTAGE(pdo) (((pdo) >> 20) & 0x3FF)
#define PDO_SUPPLY_TYPE(pdo) ((pdo) >> 30)

uint32_t get_data_object(uint8_t *obj_data)
{
    return ((((uint32_t)obj_data[3]) << 24) | 
            (((uint32_t)obj_data[2]) << 16) | 
            (((uint32_t)obj_data[1]) << 8) | 
            (((uint32_t)obj_data[0]))); 
}

void usb_pd_pm_evaluate_src_caps(unsigned int port)
{
    usb_pd_port_config_t *config = usb_pd_pm_get_config(port);
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    uint8_t i, j;
    uint8_t num_offered_pdos = dev->rx_msg_data_len >> 2;
//    uint32_t *pdo = (uint32_t*)dev->rx_msg_buf;
    uint8_t *pdo_data = dev->rx_msg_buf;
    uint32_t pdo;

//    uint16_t max_supported_input_50mv = 0;   
//    uint16_fast_t output_250mw, output_50mv, output_10ma;
//    uint16_fast_t sel_250mw = 0;
//    uint16_fast_t max_current_10ma;
//
//    object_pos = 1;
//    max_current_10ma = (dev->high_pwr_cable) ? 500 : 300;    
//
//    // Search sink PDOs for highest voltage supported. 
//    for (i = 0; i < config->num_snk_pdos; i++)
//    {
//        if ((dev->snk_pdo[i] >> 30) == SUPPLY_TYPE_FIXED)
//        {
//            max_supported_input_50mv = MAX(max_supported_input_50mv, (dev->snk_pdo[i] >> 10) & 0x3FF);
//        }
//        else
//        {
//            max_supported_input_50mv = MAX(max_supported_input_50mv, (dev->snk_pdo[i] >> 20) & 0x3FF);
//        }
//    }
//
//    // Search source PDOs for highest power under the max supported input voltage.
//    for (i = 0; i < num_offered_pdos; i++)
//    {
//        // Get min output voltage for PDO.
//        output_50mv = ((pdo[i] >> 10) & 0x3FF);
//
//        // Skip this PDO if the voltage is not supported by sink.
//        if (output_50mv > max_supported_input_50mv)
//            continue;
//
//        if ((pdo[i] >> 30) == SUPPLY_TYPE_BATTERY)
//        {           
//            output_250mw = (pdo[i] & 0x3FF);
//        }
//        else
//        {
//            output_50mv = MIN((pdo[i] >> 20) & 0x3FF), max_supported_input_50mv);
//            output_10ma = (pdo[i] & 0x3FF);
//            output_10ma = MIN(max_current_10ma, output_10ma);
//            output_250mw = output_50mv * output_10ma * 2;
//        }
//
//        if (output_250mw > sel_250mw)
//        {
//            sel_250mw = output_250mw;
//            selected_pdo = pdo[i];
//            object_pos = i + 1;
//        }
//    }

    bool acceptable_pdo[PD_MAX_PDO_NUM] = {false};
    uint16_t idx_offset = 0;
    uint16_t min_src_voltage;
    uint16_t max_src_voltage;
    uint16_t max_src_current;
    uint16_t max_src_power;
    uint16_t min_snk_operating_power;
    uint16_t min_snk_operating_current;
    uint16_t max_snk_voltage;
    uint16_t min_snk_voltage[PD_MAX_PDO_NUM][PD_MAX_PDO_NUM] = {0};
    uint16_t max_snk_current[PD_MAX_PDO_NUM][PD_MAX_PDO_NUM] = {0};
    uint16_t max_snk_power[PD_MAX_PDO_NUM][PD_MAX_PDO_NUM] = {0};
    uint16_t selected_max = 0;
    enum supply_type_t supply_type;

    CRIT("num_offered_pdos = %u\n", num_offered_pdos);

    // Initialize globals for vSafe5V.
    dev->object_position = 1;
    selected_pdo = get_data_object(&pdo_data[0]);
    selected_snk_pdo_idx = 0;

    // Evaluate each PDO offered in source caps.
    for (i = 0; i < num_offered_pdos; i++)
    {
        // Doing this instead of casting to 32-bit pointer in case pdo_data pointer is not 4-byte aligned.
        pdo = get_data_object(&pdo_data[idx_offset]);

        CRIT("PDO: 0x%08lx\n", pdo);

        // Extract source 
        supply_type = (enum supply_type_t)PDO_SUPPLY_TYPE(pdo);

        min_src_voltage = PDO_MIN_VOLTAGE(pdo);

        if (supply_type == SUPPLY_TYPE_FIXED)
        {
            max_src_voltage = min_src_voltage;
        }
        else
        {
            max_src_voltage = PDO_MAX_VOLTAGE(pdo);
        }

        CRIT("PDO-%u ", i);
        CRIT("minV: %u mV, ", min_src_voltage * 50);
        CRIT("maxV: %u mV, ", max_src_voltage * 50);

        if (supply_type == SUPPLY_TYPE_BATTERY)
        {
            max_src_power = PDO_MAX_POWER(pdo);
            CRIT("maxP: %u mW\n", max_src_power * 250);
        }
        else
        {
            max_src_current = PDO_MAX_CURRENT(pdo);
            CRIT("maxI: %u mA\n", max_src_current * 10);
        }


        // Search sink caps to determine if source offered PDO is acceptable.  
        for (j = 0; j < config->num_snk_pdos; j++)
        {
            if (config->snk_caps[j].SupplyType == SUPPLY_TYPE_FIXED)
            {
                max_snk_voltage = config->snk_caps[j].MinV;
            }
            else
            {
                max_snk_voltage = config->snk_caps[j].MaxV;
            }

            if (supply_type == SUPPLY_TYPE_BATTERY)
            {

                if (config->snk_caps[j].SupplyType == SUPPLY_TYPE_BATTERY)
                {
                    min_snk_operating_power = config->snk_caps[j].MinOperatingPower;
                }
                else /* Fixed or variable */
                {
                    min_snk_operating_power = config->snk_caps[j].MinOperatingCurrent * config->snk_caps[j].MinV * 2; /* multiply by 2 to get 250mW units */
                }

                if ((min_src_voltage >= config->snk_caps[j].MinV) &&
                    (max_src_voltage <= max_snk_voltage) &&
                    (max_src_power >= min_snk_operating_power))
                {
                    acceptable_pdo[i] = true;
                    min_snk_voltage[i][j] = min_src_voltage;
                    if (config->snk_caps[j].SupplyType == SUPPLY_TYPE_BATTERY)
                    {
                        max_snk_power[i][j] = MIN(max_src_power, config->snk_caps[j].MaxOperatingPower);
                    }
                    else
                    {
                        max_snk_power[i][j] = MIN(max_src_power, (config->snk_caps[j].MaxOperatingCurrent * config->snk_caps[j].MaxV * 2));
                    }
                    max_snk_current[i][j] = max_snk_power[i][j] / min_src_voltage / 2;  /* divide by 2 to get 10mA units */
                    if (!dev->high_pwr_cable)
                    {
                        // Limit current to 3A.
                        max_snk_current[i][j] = MIN(300, max_snk_current[i][j]);
                        max_snk_power[i][j] = max_snk_current[i][j] * config->snk_caps[j].MaxV * 2;
                    }
                }
            }
            else /* Fixed or Variable supply */
            {
                if (config->snk_caps[j].SupplyType != SUPPLY_TYPE_BATTERY)
                {
                    min_snk_operating_current = config->snk_caps[j].MinOperatingCurrent;
                }
                else /* Battery */
                {
                    min_snk_operating_current = config->snk_caps[j].MinOperatingPower / config->snk_caps[j].MaxV / 2;  /* divide by 2 to get 10mA units */
                }

                if ((min_src_voltage >= config->snk_caps[j].MinV) &&
                    (max_src_voltage <= max_snk_voltage) &&
                    (max_src_current >= min_snk_operating_current))
                {
                    acceptable_pdo[i] = true;
                    min_snk_voltage[i][j] = min_src_voltage;                  
                    if (config->snk_caps[j].SupplyType != SUPPLY_TYPE_BATTERY)
                    {
                        max_snk_current[i][j] = MIN(max_src_current, config->snk_caps[j].MaxOperatingCurrent);
                    }
                    else
                    {
                        max_snk_current[i][j] = MIN(max_src_current, (config->snk_caps[j].MaxOperatingPower / config->snk_caps[j].MinV / 2));
                    }
                    max_snk_power[i][j] = min_src_voltage * max_snk_current[i][j] * 2;  /* multiply by 2 to get 250mW units */
                    if (!dev->high_pwr_cable)
                    {
                        // Limit current to 3A.
                        max_snk_current[i][j] = MIN(300, max_snk_current[i][j]);
                        max_snk_power[i][j] = max_snk_current[i][j] * config->snk_caps[j].MaxV * 2;
                    }
                }
            }
        }

        // Increment PDO data pointer offset to next PDO. 
        idx_offset += 4;
    }

    // Use specified priority to select from the acceptable offered PDOs.
    for (i = 0; i < num_offered_pdos; i++)
    {
        if (acceptable_pdo[i])
        {
            CRIT("PDO-%u is acceptable\n", i);
            for (j = 0; j < config->num_snk_pdos; j++)
            {
                if (config->priority == PRIORITY_VOLTAGE)
                {
                    // Find PDO with highest minimum voltage.
                    if (min_snk_voltage[i][j] > selected_max)
                    {
                        selected_max = min_snk_voltage[i][j];
                        selected_pdo = get_data_object(&pdo_data[i << 2]);
                        dev->object_position = i + 1;
                        selected_snk_pdo_idx = j;
                    }
                }
                else if (config->priority == PRIORITY_CURRENT)
                {
                    // Find PDO that will deliver the highest current.
                    if (max_snk_current[i][j] > selected_max)
                    {
                        selected_max = max_snk_current[i][j];
                        selected_pdo = get_data_object(&pdo_data[i << 2]);
                        dev->object_position = i + 1;
                        selected_snk_pdo_idx = j;
                    }
                }
                else /* PRIORITY_POWER */
                {
                    // Find PDO that will deliver the highest power.
                    if (max_snk_power[i][j] > selected_max)
                    {
                        selected_max = max_snk_power[i][j];
                        selected_pdo = get_data_object(&pdo_data[i << 2]);
                        dev->object_position = i + 1;
                        selected_snk_pdo_idx = j;
                    }
                }
            }
        }
    }

    // Save PDO min voltage (multiply by 2 to convert to 25mV LSB) so we can set discharge voltage threshold. 
    dev->min_voltage = PDO_MIN_VOLTAGE(selected_pdo) << 1;

    DEBUG("selected_pdo: %lx, selected_snk_pdo_idx: %u, min_voltage = %u mV\n", 
          selected_pdo, selected_snk_pdo_idx, dev->min_voltage * 25);

    return;
}

#define PDO_DUAL_ROLE_POWER_BIT           ((uint32_t)0x01 << 29)
#define SRC_PDO_USB_SUSPEND_BIT           ((uint32_t)0x01 << 28)
#define SNK_PDO_HIGHER_CAPABILITY_BIT     ((uint32_t)0x01 << 28)
#define PDO_EXTERNALLY_POWERED_BIT        ((uint32_t)0x01 << 27)
#define PDO_USB_COMM_CAPABLE_BIT          ((uint32_t)0x01 << 26)
#define PDO_DUAL_ROLE_DATA_BIT            ((uint32_t)0x01 << 25)
#define SRC_PDO_UNCHUNKED_EXT_MSG_SUP_BIT ((uint32_t)0x01 << 24)


static void build_src_caps(unsigned int port, usb_pd_port_config_t *config)
{
    unsigned int n;
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    tcpc_device_t *typec_dev = tcpm_get_device(port);

    // Clear PDOs
    for (n = 0; n < config->num_src_pdos; n++)
    {
        dev->src_pdo[n] = 0x00;
    }

    if (typec_dev->role == ROLE_DRP)
    {
        dev->src_pdo[0] |= PDO_DUAL_ROLE_POWER_BIT;
    }

    if (config->usb_suspend_supported)
    {
        dev->src_pdo[0] |= SRC_PDO_USB_SUSPEND_BIT;
    }

    if (config->externally_powered)
    {
        dev->src_pdo[0] |= PDO_EXTERNALLY_POWERED_BIT;
    }

    if (config->usb_comm_capable)
    {
        dev->src_pdo[0] |= PDO_USB_COMM_CAPABLE_BIT;
    }

    if (config->dual_role_data)
    {
        dev->src_pdo[0] |= PDO_DUAL_ROLE_DATA_BIT;
    }

    if (config->unchunked_msg_support)
    {
        dev->src_pdo[0] |= SRC_PDO_UNCHUNKED_EXT_MSG_SUP_BIT;
    }

    for (n = 0; n < config->num_src_pdos; n++)
    {
        // PDO(n)[31:30] = SourceCapabilities.PDO(n).SupplyType;
        dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].SupplyType) & 0x03) << 30;

        if (config->src_caps[n].SupplyType == SUPPLY_TYPE_FIXED)
        {
            // PDO(n)[21:20] = SourceCapabilities.PDO(n).PeakCurrent;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].PeakI) & 0x03) << 20;
            // PDO(n)[19:10] = SourceCapabilities.PDO(n).MinimumVoltage;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MinV) & 0x3FF) << 10;
            // PDO(n)[9:0] = SourceCapabilities.PDO(n).MaximumCurrent;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxI) & 0x3FF);
        }
        else if (config->src_caps[n].SupplyType == SUPPLY_TYPE_VARIABLE)
        {
            // PDO(n)[29:20] = SourceCapabilities.PDO(n).MaximumVoltage;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxV) & 0x3FF) << 20;
            // PDO(n)[19:10] = SourceCapabilities.PDO(n).MinimumVoltage;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MinV) & 0x3FF) << 10;
            // PDO(n)[9:0] = SourceCapabilities.PDO(n).MaximumCurrent;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxI) & 0x3FF);
        }
        else if (config->src_caps[n].SupplyType == SUPPLY_TYPE_BATTERY)
        {
            // PDO(n)[29:20] = SourceCapabilities.PDO(n).MaximumVoltage;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxV) & 0x3FF) << 20;
            // PDO(n)[19:10] = SourceCapabilities.PDO(n).MinimumVoltage;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MinV) & 0x3FF) << 10;
            // PDO(n)[9:0] = SourceCapabilities.PDO(n).MaximumPower;
            dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxPower) & 0x3FF);
        }
    }

    return;
}


static void build_snk_caps(unsigned int port, usb_pd_port_config_t *config)
{
    unsigned int n;
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    tcpc_device_t *typec_dev = tcpm_get_device(port);

    // Clear PDOs
    for (n = 0; n < config->num_snk_pdos; n++)
    {
        dev->snk_pdo[n] = 0x00;
    }

    if (typec_dev->role == ROLE_DRP)
    {
        dev->snk_pdo[0] |= PDO_DUAL_ROLE_POWER_BIT;
    }

    if (config->higher_capability)
    {
        dev->snk_pdo[0] |= SNK_PDO_HIGHER_CAPABILITY_BIT;
    }

    if (config->externally_powered)
    {
        dev->snk_pdo[0] |= PDO_EXTERNALLY_POWERED_BIT;
    }

    if (config->usb_comm_capable)
    {
        dev->snk_pdo[0] |= PDO_USB_COMM_CAPABLE_BIT;
    }

    if (config->dual_role_data)
    {
        dev->snk_pdo[0] |= PDO_DUAL_ROLE_DATA_BIT;
    }

    dev->snk_pdo[0] |= ((uint32_t)config->fast_role_support & 0x03) << 23;

    for (n = 0; n < config->num_snk_pdos; n++)
    {
        //SinkPDO(i)(31:30) = pSupplyType(i)
        dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].SupplyType) & 0x03) << 30;

        if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_FIXED)
        {
            //SinkPDO(i)(19:10) = pMinimumVoltage(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MinV) & 0x3FF) << 10;
            //SinkPDO(i)(9:0) = pOperationalCurrent(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].OperationalCurrent) & 0x3FF);

        }
        else if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_BATTERY)
        {
            //SinkPDO(i)(29:20) = pMaximumVoltage(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MaxV) & 0x3FF) << 20;
            //SinkPDO(i)(19:10) = pMinimumVoltage(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MinV) & 0x3FF) << 10;
            //SinkPDO(i)(9:0) = pOperationalPower(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].OperationalPower) & 0x3FF);

        }
        else if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_VARIABLE)
        {
            //SinkPDO(i)(29:20) = pMaximumVoltage(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MaxV) & 0x3FF) << 20;
            //SinkPDO(i)(19:10) = pMinimumVoltage(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MinV) & 0x3FF) << 10;
            //SinkPDO(i)(9:0) = pOperationalCurrent(i)
            dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].OperationalCurrent) & 0x3FF);
        }
    }

    return;
}

typedef enum
{
    RDO_GIVEBACK_FLAG         = ((uint32_t)1 << 27),
    RDO_CAPABILITY_MISMATCH   = ((uint32_t)1 << 26),
    RDO_USB_COMM_CAPABLE      = ((uint32_t)1 << 25),
    RDO_NO_USB_SUSPEND        = ((uint32_t)1 << 24),
    RDO_UNCHUNKED_MSG_SUPPORT = ((uint32_t)1 << 23)
} rdo_bits_t;


void build_rdo(unsigned int port)
{
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    usb_pd_port_config_t *config = usb_pd_pm_get_config(port);

    rdo = 0;

    rdo |= ((uint32_t)dev->object_position) << 28;

    if (config->giveback_flag)
    {
        rdo |= RDO_GIVEBACK_FLAG;
    }

    if ((dev->object_position == 1) && config->higher_capability)
    {
        // Sink requires voltage greater than 5V.
        rdo |= RDO_CAPABILITY_MISMATCH;
    }

    if (PDO_MAX_CURRENT(selected_pdo) < config->snk_caps[selected_snk_pdo_idx].MaxOperatingCurrent)
    {
        // Sink requires higher current for full operation.
        rdo |= RDO_CAPABILITY_MISMATCH;
    }

    if (config->usb_comm_capable)
    {
        rdo |= RDO_USB_COMM_CAPABLE;
    }

    if (config->no_usb_suspend)
    {
        rdo |= RDO_NO_USB_SUSPEND;
    }

    if (config->unchunked_msg_support)
    {
        rdo |= RDO_UNCHUNKED_MSG_SUPPORT;
    }

    if ((selected_pdo >> 30) == SUPPLY_TYPE_BATTERY)
    {
        rdo |= ((uint32_t)config->snk_caps[selected_snk_pdo_idx].OperationalPower & 0x3FF) << 10;

        if (config->giveback_flag)
        {
            rdo |= config->snk_caps[selected_snk_pdo_idx].MinOperatingPower & 0x3FF;
        }
        else
        {
            rdo |= config->snk_caps[selected_snk_pdo_idx].MaxOperatingPower & 0x3FF;
        }
    }
    else /* Fixed or Variable supply */
    {
        rdo |= ((uint32_t)config->snk_caps[selected_snk_pdo_idx].OperationalCurrent & 0x3FF) << 10;

        if (config->giveback_flag)
        {
            rdo |= config->snk_caps[selected_snk_pdo_idx].MinOperatingCurrent & 0x3FF;
        }
        else
        {
            rdo |= config->snk_caps[selected_snk_pdo_idx].MaxOperatingCurrent & 0x3FF;
        }
    }

    return;
}


void usb_pd_init(const usb_pd_port_config_t *port_config)
{
    unsigned int port;

    CRIT(" ________  _________  ____ ___  ___\n"); 
    CRIT("/_  __/ / / / __/ _ )/ / /|_  ||_  |\n");
    CRIT(" / / / /_/ /\\ \\/ _  /_  _/ __// __/\n"); 
    CRIT("/_/  \\____/___/____/ /_//____/____/\n"); 

    CRIT("\nPD Stack v%u.%02u\n", PD_LIB_VERSION_MAJOR, 
         PD_LIB_VERSION_MINOR);
    CRIT("=====================================\n");

    // Short sleep to allow UART print buffer to flush.
    msleep(50);

    usb_pd_prl_init();

    memcpy(pd_port_config, port_config, sizeof(pd_port_config));

    for (port = 0; port < NUM_TCPC_DEVICES; port++)
    {
        build_src_caps(port, &pd_port_config[port]);
        build_snk_caps(port, &pd_port_config[port]);
        usb_pd_pe_init(port, &pd_port_config[port]);
    }

    return;
}
