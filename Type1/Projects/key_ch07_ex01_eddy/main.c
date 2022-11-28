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

/* Eddystone definitions */
#define EDDY_UUID_LSB	0xAA
#define EDDY_UUID_MSB	0xFE
#define EDDY_URL		0x10
#define EDDY_HTTPS_WWW	0x01
#define EDDY_DOTCOM		0x00
#define EDDY_TX_PWR		0xF0

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* Function to setup advertisement packet data */
void app_set_advertisement_data( void );

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

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

	/* Initialize pin to indicate advertising */
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

				/* Disable pairing */
				wiced_bt_set_pairable_mode( WICED_FALSE, WICED_FALSE );

				/* Setup Eddystone advertisement packet and begin advertising */
                app_set_advertisement_data();
                wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH, 0, NULL );

	            result = WICED_BT_SUCCESS;
			}
			else
			{
				printf( "Failed to initialize Bluetooth controller and stack\n" );
			}
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
            if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF ) /* Advertising is stopped - LED OFF */
			{
            	cyhal_gpio_write(CYBSP_USER_LED2,CYBSP_LED_STATE_OFF);
			}
            else /* Advertising is on - LED ON */
            {
                cyhal_gpio_write(CYBSP_USER_LED2,CYBSP_LED_STATE_ON);
            }
            result = WICED_BT_SUCCESS;
			break;

		default:
			break;
    }

    return result;
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data( void )
********************************************************************************/
void app_set_advertisement_data( void )
{
    #define NUM_FIELDS (3)

    uint8_t adv_data_flags[]     = { BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED };
    uint8_t adv_data_service[]   = { EDDY_UUID_LSB, EDDY_UUID_MSB };
    uint8_t adv_data_eddy_url[]  =
    {
    	EDDY_UUID_LSB, EDDY_UUID_MSB,
		EDDY_URL,
		EDDY_TX_PWR,
		EDDY_HTTPS_WWW,
        'i', 'n', 'f', 'i', 'n', 'e', 'o', 'n',
		EDDY_DOTCOM
     };

    wiced_bt_ble_advert_elem_t adv_packet_data[] =
        {
            /* Advertisement Element for Flags field */
            {
                .advert_type = BTM_BLE_ADVERT_TYPE_FLAG,
                .len = sizeof( adv_data_flags ),
                .p_data = adv_data_flags
            },

            /* Advertisement Element for Service UUID field */
            {
                .advert_type = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE,
                .len = sizeof( adv_data_service ),
                .p_data = adv_data_service
            },

            /* Advertisement Element for Service Data field */
            {
                .advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA,
                .len = sizeof( adv_data_eddy_url ),
                .p_data = adv_data_eddy_url
            },

        };

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data( NUM_FIELDS, adv_packet_data );
}
