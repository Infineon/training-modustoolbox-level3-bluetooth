/******************************************************************************
* File Name:   main.c
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"

/* Library for malloc and free */
#include "stdlib.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* btstack */
#include "wiced_bt_stack.h"

/* Timer functions */
#include "wiced_timer.h"

/* App utilities */
#include "app_bt_utils.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

/* Eddystone values */
#define EDDY_UUID_LSB	0xAA
#define EDDY_UUID_MSB	0xFE

#define EDDY_UID		0x00
#define EDDY_URL		0x10
#define EDDY_TLM		0x20

#define EDDY_HTTPS_WWW	0x01
#define EDDY_DOTCOM		0x00
#define EDDY_TX_PWR		0xF0

#define EDDY_TLM_UNENCRYPTED 0x00

/* Allocate the multi-advertising instance numbers */
#define ADV_INSTANCE_URL		1
#define ADV_INSTANCE_UID		2
#define ADV_INSTANCE_TLM		3

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* Function to setup advertisement packet data */
void app_set_advertisement_data_url( void );
void app_set_advertisement_data_uid( void );
void app_set_advertisement_data_tlm( void );

/* Timer callback function */
void app_timer_callback(WICED_TIMER_PARAM_TYPE count );

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

/* Timer object and variable to keep track of beacon up time */
wiced_timer_t app_timer;
uint32_t uptime = 0;

/* Configure mulit-advertising parameters - these will be used for all 3 Eddystone packets (URL, UID, TLM) */
wiced_bt_ble_multi_adv_params_t adv_parameters =
{
    .adv_int_min =          CY_BT_HIGH_DUTY_ADV_MIN_INTERVAL,
    .adv_int_max =          CY_BT_HIGH_DUTY_ADV_MAX_INTERVAL,
    .adv_type =             MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map =          BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy =    BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
    .adv_tx_power =         MULTI_ADV_TX_POWER_MAX_INDEX,
    .peer_bd_addr =         {0},
    .peer_addr_type =       BLE_ADDR_PUBLIC,
    .own_bd_addr =          {0},
    .own_addr_type =        BLE_ADDR_PUBLIC
};

/*******************************************************************
 * Function Implementations
 ******************************************************************/

/*******************************************************************************
* Function Name: int main( void )
********************************************************************************/
int main(void)
{
    cy_rslt_t result ;

    /* Initialize the board support package */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);

    /* Initialize LED Pin */
    cyhal_gpio_init(CYBSP_USER_LED2,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    printf("**********Application Start*****************\n");

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}


/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
static wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_device_address_t bda = {0};
    wiced_bt_multi_adv_opcodes_t multi_adv_resp_opcode;
    uint8_t multi_adv_resp_status = 0;

    printf("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				printf( "Bluetooth Enabled\n" );

				/* Set the local BDA from the value in the configurator and print it */
				wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth Device Address: ");
				print_bd_address(bda);

                /* Start timer */
                if (wiced_init_timer(&app_timer, app_timer_callback, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
                    {
                        wiced_start_timer( &app_timer, 100 );
                    }

				/* Disable pairing */
				wiced_bt_set_pairable_mode( WICED_FALSE, WICED_FALSE );

				/* Setup Eddystone advertisement packets and begin multi-advertising for URL */
				app_set_advertisement_data_url();
            	wiced_set_multi_advertisement_params( ADV_INSTANCE_URL, &adv_parameters );
            	wiced_start_multi_advertisements( MULTI_ADVERT_START, ADV_INSTANCE_URL );

				/* Setup Eddystone advertisement packets and begin multi-advertising for UID */
				app_set_advertisement_data_uid();
            	wiced_set_multi_advertisement_params( ADV_INSTANCE_UID, &adv_parameters );
            	wiced_start_multi_advertisements( MULTI_ADVERT_START, ADV_INSTANCE_UID );

				/* Setup Eddystone advertisement packets and begin multi-advertising for TLM */
				app_set_advertisement_data_tlm();
            	wiced_set_multi_advertisement_params( ADV_INSTANCE_TLM, &adv_parameters );
            	wiced_start_multi_advertisements( MULTI_ADVERT_START, ADV_INSTANCE_TLM );

	            result = WICED_BT_SUCCESS;
			}
			else
			{
				printf( "Failed to initialize Bluetooth controller and stack\n" );
			}
			break;

        case BTM_MULTI_ADVERT_RESP_EVENT:

            /* Multi ADV Response */
            multi_adv_resp_opcode = p_event_data->ble_multi_adv_response_event.opcode;
            multi_adv_resp_status = p_event_data->ble_multi_adv_response_event.status;

            if (SET_ADVT_PARAM_MULTI == multi_adv_resp_opcode)
            {
                if(WICED_SUCCESS == multi_adv_resp_status)
                {
                    printf("Multi ADV Set Param Event Status: SUCCESS\n");
                }
                else
                {
                    printf("Multi ADV Set Param Event Status: FAILED\n");
                }
            }
            else if (SET_ADVT_DATA_MULTI == multi_adv_resp_opcode)
            {
                if(WICED_SUCCESS == multi_adv_resp_status)
                {
                    printf("Multi ADV Set Data Event Status: SUCCESS\n");
                }
                else
                {
                    printf("Multi ADV Set Data Event Status: FAILED\n");
                }
            }
            else if (SET_ADVT_ENABLE_MULTI == multi_adv_resp_opcode)
            {  
                if(WICED_SUCCESS == multi_adv_resp_status)
                {
                    printf("Multi ADV Start Event Status: SUCCESS\n");
					/* Advetising started, so turn LED on */
                    cyhal_gpio_write(CYBSP_USER_LED2,CYBSP_LED_STATE_ON);
                }
                else
                {
                    printf("Multi ADV Start Event Status: FAILED\n");
                }
            }

            if(WICED_SUCCESS == multi_adv_resp_status)
            {
                result = WICED_BT_SUCCESS;
            }
			else
			{
                /* Fail, so turn LED off */
                cyhal_gpio_write(CYBSP_USER_LED2,CYBSP_LED_STATE_OFF);
            }

            break;

		default:
			break;
    }

    return result;
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data_url( void )
********************************************************************************/
void app_set_advertisement_data_url( void )
{
	/* Setup URL multi-advertising packet */
	uint8_t url_packet[] =
	{
			/* Flags field */
			2, /* Field size is the packet type (1 byte) + flag settings (1 byte) */
			BTM_BLE_ADVERT_TYPE_FLAG,
			BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED,

			/* Service UUID field */
			3, /* Field size is the packet type (1 byte) +  16 bit UUID (2 bytes) */
			BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE,
			EDDY_UUID_LSB, EDDY_UUID_MSB,

			/* Service Data field */
			15, /* Field size is the packet type (1 byte) + 16 bit UUID (2 bytes) + URL - (12 bytes) */
			BTM_BLE_ADVERT_TYPE_SERVICE_DATA,
			EDDY_UUID_LSB, EDDY_UUID_MSB,
			EDDY_URL,
			EDDY_TX_PWR,
			EDDY_HTTPS_WWW,
			'i', 'n', 'f', 'i', 'n', 'e', 'o', 'n',
			EDDY_DOTCOM
	};

	uint8_t packet_len = sizeof(url_packet);

	wiced_set_multi_advertisement_data( url_packet, packet_len, ADV_INSTANCE_URL );
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data_uid( void )
********************************************************************************/
void app_set_advertisement_data_uid( void )
{
	/* Setup UID multi-advertising packet */
	uint8_t uid_packet[] =
	{
			/* Flags field */
			2, /* Field size is the packet type (1 byte) + flag settings (1 byte) */
			BTM_BLE_ADVERT_TYPE_FLAG,
			BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED,

			/* Service UUID field */
			3, /* Field size is the packet type (1 byte) +  16 bit UUID (2 bytes) */
			BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE,
			EDDY_UUID_LSB, EDDY_UUID_MSB,

			/* Service Data field */
			23, /* Field size is the packet type (1 byte) +  16 bit UUID (2 bytes) + data - (20 bytes) */
			BTM_BLE_ADVERT_TYPE_SERVICE_DATA,
			EDDY_UUID_LSB, EDDY_UUID_MSB,
			EDDY_UID,
			EDDY_TX_PWR,
			'k', 'e', 'y', 0, 0, 0, 0, 0, 0, 0, /* 10-byte Namespace */
			0, 0, 0, 0, 0, 0, /* 6 byte Instance is all 0's */
			0, 0, /* RFU bytes */
	};

	uint8_t packet_len = sizeof(uid_packet);

	wiced_set_multi_advertisement_data( uid_packet, packet_len, ADV_INSTANCE_UID );
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data_tlm( void )
********************************************************************************/
void app_set_advertisement_data_tlm( void )
{
	/* Setup TLM multi-advertising packet */
	uint8_t tlm_packet[] =
	{
			/* Flags field */
			2, /* Field size is the packet type (1 byte) + flag settings (1 byte) */
			BTM_BLE_ADVERT_TYPE_FLAG,
			BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED,

			/* Service UUID field */
			3, /* Field size is the packet type (1 byte) +  16 bit UUID (2 bytes) */
			BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE,
			EDDY_UUID_LSB, EDDY_UUID_MSB,

			/* Service Data field */
			17, /* Field size is the packet type (1 byte) +  16 bit UUID (2 bytes) + data - (14 bytes) */
			BTM_BLE_ADVERT_TYPE_SERVICE_DATA,
			EDDY_UUID_LSB, EDDY_UUID_MSB,
			EDDY_TLM,
			EDDY_TLM_UNENCRYPTED,
			0, 0, /* Battery voltage is all 0's */
			0, 0, /* Temperature is all 0's */
			0, 0, 0, 0, /* Advertising count is all 0's */
			(uint8_t) ((uptime >> 24) & 0x000000FF), /* uptime MSB */
			(uint8_t) ((uptime >> 16) & 0x000000FF), /* uptime MSB - 1 */
			(uint8_t) ((uptime >> 8) & 0x000000FF),  /* uptime MSB - 2 */
			(uint8_t) ((uptime >> 0) & 0x000000FF),  /* uptime LSB */
	};

	uint8_t packet_len = sizeof(tlm_packet);

	wiced_set_multi_advertisement_data( tlm_packet, packet_len, ADV_INSTANCE_TLM );
}


/*******************************************************************************
* Function Name: app_timer_callback(WICED_TIMER_PARAM_TYPE count )
********************************************************************************/
void app_timer_callback(WICED_TIMER_PARAM_TYPE count )
{
    uptime++;
    
    /* Call the function to update the TLM packet's value with the latest uptime */
    app_set_advertisement_data_tlm();
}
