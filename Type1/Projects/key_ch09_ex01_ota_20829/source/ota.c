/******************************************************************************
* File Name:   ota.c
*
* Description: Definitions and data structures for the OTA example application
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#ifdef ENABLE_OTA


#include "ota.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "stdio.h"
#include "ota_serial_flash.h"
#include "cyabs_rtos.h"

ota_app_context_t ota_app;

/* OTA variables */
#ifndef  COMPONENT_OTA_BLUETOOTH_SECURE
/* UUID for non-secure Bluetooth */
const uint8_t NON_SECURE_UUID_SERVICE_OTA_FW_UPGRADE_SERVICE[] = { 0x1F, 0x38, 0xA1, 0x38, 0xAD, 0x82, 0x35, 0x86, 0xA0, 0x43, 0x13, 0x5C, 0x47, 0x1E, 0x5D, 0xAE };
#endif

/* UUID created by Bluetooth� Configurator, supplied in "GeneratedSource/cycfg_gatt_db.h" */
const uint8_t BLE_CONFIG_UUID_SERVICE_OTA_FW_UPGRADE_SERVICE[] = {__UUID_SERVICE_OTA_FW_UPGRADE_SERVICE };

cy_ota_agent_mem_interface_t storage_interfaces =
{
        .read = ota_mem_read,
        .write = ota_mem_write,
        .erase = ota_mem_erase,
        .get_erase_size = ota_mem_get_erase_size,
        .get_prog_size = ota_mem_get_prog_size
};

cy_ota_network_params_t     ota_test_network_params = { CY_OTA_CONNECTION_UNKNOWN };
cy_ota_agent_params_t     ota_test_agent_params = { 0 };

/******************************************************
 *               Function Definitions
 ******************************************************/

void ota_initialize_default_values(void)
{
    ota_app.tag               = OTA_APP_TAG_VALID;
    ota_app.connection_type   = CY_OTA_CONNECTION_BLE;
    ota_app.update_flow       = CY_OTA_JOB_FLOW;
    ota_app.reboot_at_end     = 1;

};

cy_rslt_t cy_ota_ble_check_build_vs_configurator( void )
{
    /* verify "make built" with proper Bluetooth� SECURE setting, compare to output on what Bluetooth� Configurator output for us */
#ifndef  COMPONENT_OTA_BLUETOOTH_SECURE
    /* Check UUID for non-secure Bluetooth� upgrade service */
    if (0 != memcmp(NON_SECURE_UUID_SERVICE_OTA_FW_UPGRADE_SERVICE, BLE_CONFIG_UUID_SERVICE_OTA_FW_UPGRADE_SERVICE, sizeof(NON_SECURE_UUID_SERVICE_OTA_FW_UPGRADE_SERVICE)))
    {
        printf("    SECURE <appname>.cybt File does not match NON-SECURE APP build!\n");
        printf( "      Change the <appname>.cybt File to use NON-SECURE OTA UUID.\n");
        printf("        (Set 'GATT->Server->OTA FW UPGRADE SERVICE' to 'ae5d1e47-5c13-43a0-8635-82ad38a1381f')\n");
        return CY_RSLT_OTA_ERROR_GENERAL;
    }
#endif

    return CY_RSLT_SUCCESS;
}
 cy_rslt_t init_ota(ota_app_context_t *ota)
{
    cy_rslt_t               result;

    if (ota == NULL || ota->tag != OTA_APP_TAG_VALID)
    {
        return CY_RSLT_OTA_ERROR_BADARG;
    }

#ifdef TEST_SWAP_SETUP
    ota_test_agent_params.validate_after_reboot  = 1;        /* Validate after reboot so that we can test revert */

#else
    ota_test_agent_params.validate_after_reboot  = 0;        /* Validate after download so we don't have to call
                                                                cy_ota_validated() on reboot */
#endif

    result = cy_ota_agent_start(&ota_test_network_params, &ota_test_agent_params,&storage_interfaces, &ota_app.ota_context);
    if (result != CY_RSLT_SUCCESS)
    {
        while (true)
        {
            cy_rtos_delay_milliseconds(10);
        }
    }

    return result;
}

#endif /* end of ENABLE_OTA */
