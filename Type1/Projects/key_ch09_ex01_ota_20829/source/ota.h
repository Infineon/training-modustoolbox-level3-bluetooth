/******************************************************************************
* File Name:   ota.h
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
#ifndef OTA_CONTEXT_H_
#define OTA_CONTEXT_H_

#include "GeneratedSource/cycfg_gap.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#ifdef ENABLE_OTA
	#include "cy_ota_api.h"
#endif

/******************************************************
 *                     Macros
 ******************************************************/
#define OTA_APP_TAG_VALID               (0x51EDBA15)
#define OTA_APP_TAG_INVALID             (0xDEADBEEF)


typedef struct
{
    uint32_t tag;

    cy_ota_context_ptr      ota_context;
    cy_ota_connection_t     connection_type;
    cy_ota_update_flow_t    update_flow;
    uint8_t                 reboot_at_end;
    uint16_t                bt_ota_config_descriptor;   /* BT OTA configuration descriptor to determine if Device sends Notification/Indication */

} ota_app_context_t;

typedef struct gatt_write_req_buf
{
    uint8_t    value[CY_BT_MTU_SIZE];
    uint16_t   written;
    uint16_t   handle;
    bool       in_use;
}gatt_write_req_buf_t;

extern ota_app_context_t ota_app;

/******************************************************
 *               Function Declarations
 ******************************************************/

void ota_initialize_default_values(void);

/* Functions in ota_test_console.c used by main.c */
int ota_test_command_console_init(void);

cy_rslt_t cy_ota_ble_check_build_vs_configurator( void );
cy_rslt_t init_ota(ota_app_context_t *ota);

#endif /* #define OTA_CONTEXT_H_ */
