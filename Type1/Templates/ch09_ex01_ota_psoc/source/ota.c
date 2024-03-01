/******************************************************************************
* File Name:   ota.c
*
* Description: This file handles ota events and notifications.
*
*
* Related Document: See Readme.md
*
********************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifdef ENABLE_OTA

#include "ota.h"

#include "cyabs_rtos.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
/**
 * @brief OTA config parameters
 */
ota_config_t ota_config ={0};

/**
 * @brief Agent parameters for OTA
 */
cy_ota_agent_params_t ota_agent_params = {0};

/**
 * @brief network parameters for OTA
 */
cy_ota_network_params_t ota_network_params = {CY_OTA_CONNECTION_UNKNOWN};

/*******************************************************************************
*        Function Declarations
*******************************************************************************/
cy_rslt_t              app_bt_ota_init                        (ota_config_t *ota);

/*******************************************************************************
*        Function Definitions
*******************************************************************************/
/*
 * Function Name:
 * app_bt_ota_write_handler
 *
 * Function Description:
 * @brief  The function is invoked when GATTS_REQ_TYPE_WRITE is received from the
 *         client device and invokes GATT Server Event Callback function. This
 *         handles OTA related "Write Requests" received from Client device.
 *
 * @param p_write_req   Pointer to BLE GATT write request
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t app_bt_ota_write_handler(wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;;
    cy_rslt_t cy_result;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    CY_ASSERT(( NULL != p_data ) && (NULL != p_write_req));

    switch (p_write_req->handle)
    {
    case HDLD_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_CLIENT_CHAR_CONFIG:
        cy_log_msg(CYLF_OTA, CY_LOG_DEBUG, "%s() HDLD_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_CLIENT_CHAR_CONFIG\r\n",
                          __func__);
        /* Save Configuration descriptor in Application data structure (Notify & Indicate flags) */
        ota_config.bt_ota_config_descriptor = p_write_req->p_val[0];
        cy_log_msg(CYLF_OTA, CY_LOG_NOTICE, "ota_config.bt_ota_config_descriptor: %d %s\r\n",
                   ota_config.bt_ota_config_descriptor,
        (ota_config.bt_ota_config_descriptor == GATT_CLIENT_CONFIG_NOTIFICATION) ? "Notify" :
        (ota_config.bt_ota_config_descriptor == GATT_CLIENT_CONFIG_INDICATION) ? "Indicate": "Unknown");
        return WICED_BT_GATT_SUCCESS;

    case HDLC_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_VALUE:
        cy_log_msg(CYLF_OTA, CY_LOG_DEBUG, "%s() HDLC_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_VALUE \r\n", __func__);
        switch (p_write_req->p_val[0])
        {
        case CY_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD:
             /* Call application-level OTA initialization (calls cy_ota_agent_start() ) */
            cy_result = app_bt_ota_init(&ota_config);
            if (CY_RSLT_SUCCESS != cy_result)
            {
                cy_log_msg(CYLF_OTA, CY_LOG_ERR, "OTA initialization Failed - result: 0x%lx\r\n",
                           cy_result);
                return WICED_BT_GATT_ERROR;
            }
            cy_result = cy_ota_ble_download_prepare(ota_config.ota_context,
                                                    connection_id,
                                                    ota_config.bt_ota_config_descriptor);
            if (CY_RSLT_SUCCESS != cy_result)
            {
                cy_log_msg(CYLF_OTA, CY_LOG_ERR, "Download preparation Failed - result: 0x%lx\r\n", cy_result);
                return WICED_BT_GATT_ERROR;
            }
            return WICED_BT_GATT_SUCCESS;

        case CY_OTA_UPGRADE_COMMAND_DOWNLOAD:
            /* let OTA lib know what is going on */
            cy_log_msg(CYLF_OTA, CY_LOG_DEBUG, "%s() HDLC_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_VALUE : CY_OTA_UPGRADE_COMMAND_DOWNLOAD\r\n", __func__);
            cy_result = cy_ota_ble_download(ota_config.ota_context, p_data,
            								connection_id,
                                            ota_config.bt_ota_config_descriptor);
            if (CY_RSLT_SUCCESS != cy_result)
            {
                cy_log_msg(CYLF_OTA, CY_LOG_ERR, "Download Failed - result: 0x%lx\r\n", cy_result);
                return WICED_BT_GATT_ERROR;
            }
            return WICED_BT_GATT_SUCCESS;

        case CY_OTA_UPGRADE_COMMAND_VERIFY:
            cy_result = cy_ota_ble_download_verify(ota_config.ota_context, p_data,
            										connection_id);
            if (CY_RSLT_SUCCESS != cy_result)
            {
                cy_log_msg(CYLF_OTA, CY_LOG_ERR, "verification and Indication failed: 0x%d\r\n",
                           status);
                return WICED_BT_GATT_ERROR;
            }
            return status;

        case CY_OTA_UPGRADE_COMMAND_ABORT:
            cy_result = cy_ota_ble_download_abort(ota_config.ota_context);
            return WICED_BT_GATT_SUCCESS;
        }
        break;

    case HDLC_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_DATA_VALUE:
        cy_result = cy_ota_ble_download_write(ota_config.ota_context, p_data);
        return (cy_result == CY_RSLT_SUCCESS) ? WICED_BT_GATT_SUCCESS : WICED_BT_GATT_ERROR;

    default:
        cy_log_msg(CYLF_OTA,CY_LOG_DEBUG,"UNHANDLED OTA WRITE \r\n");
        break;
    }
    return WICED_BT_GATT_REQ_NOT_SUPPORTED;
}

/**
 * Function Name:
 * app_bt_ota_init
 *
 * Function Description :
 * @brief Initialize and start the OTA update
 *
 * @param app_context  pointer to Application context
 *
 * @return cy_rslt_t Result of initialization
 */
cy_rslt_t app_bt_ota_init(ota_config_t *config)
{
    cy_rslt_t cy_result;

    if (config == NULL || config->tag != OTA_APP_TAG_VALID)
    {
    	return CY_RSLT_OTA_ERROR_BADARG;
    }

    memset(&ota_network_params, 0, sizeof(ota_network_params));
    memset(&ota_agent_params, 0, sizeof(ota_agent_params));

    /* Common Network Parameters */
    ota_network_params.initial_connection = config->connection_type;

    /* OTA Agent parameters - used for ALL transport types*/
    /* Validate after reboot so that we can test revert */
    ota_agent_params.validate_after_reboot = 1;

    cy_result = cy_ota_agent_start(&ota_network_params, &ota_agent_params,
                                &(config->ota_context));
    if (CY_RSLT_SUCCESS != cy_result)
    {
        cy_log_msg(CYLF_OTA, CY_LOG_ERR, "cy_ota_agent_start() Failed - result: 0x%lx\r\n",
                   cy_result);
        while (true)
        {
            cy_rtos_delay_milliseconds(10);
        }
    }
    cy_log_msg(CYLF_OTA, CY_LOG_NOTICE, "OTA Agent Started \r\n");

    return cy_result;
}

/**
 * Function Name
 * app_bt_initialize_default_values
 *
 * Function Description:
 * @brief  Initialize default ota configuration values
 *
 * @param  None
 * @return void
 */
void app_bt_initialize_default_values(void)
{

    ota_config.tag = OTA_APP_TAG_VALID;
    ota_config.connection_type = CY_OTA_CONNECTION_BLE;
    ota_config.reboot_at_end = 1;
}

#endif /* end of ENABLE_OTA */
