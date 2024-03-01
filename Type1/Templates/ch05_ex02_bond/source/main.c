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
#include <queue.h>

/* btstack */
#include "wiced_bt_stack.h"

/* App utilities */
#include "app_bt_utils.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"

/* Include header file from BSP's bluetooth directory */
#include "cybsp_bt_config.h"

/* Include headers for KV Store library to store bonding information in flash */
#include "mtb_kvstore.h"
#include "app_kv-store_common.h"
/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#ifndef CYBSP_USER_LED2
#define CYBSP_USER_LED2 P10_0
#endif


#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)
#define GPIO_INTERRUPT_PRIORITY (7u)

#define LED_ON_DUTY (0)
#define LED_OFF_DUTY (100)
#define LED_BLINK_DUTY (50)
#define LED_BONDING_FREQ (2)
#define LED_BONDED_FREQ (5)

/* Typdef for function used to free allocated buffer to stack */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t app_bt_server_event_handler           (wiced_bt_gatt_event_data_t *p_data);

static wiced_bt_gatt_status_t app_bt_write_handler                  (wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler          (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler    (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler  (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);
/* Task to handle notifications and button press ISR */
static void notify_task(void * arg);
static void app_button_isr(void* handler_arg, cyhal_gpio_event_t event);

/* Tasks to handle UART */
static void rx_cback(void *handler_arg, cyhal_uart_event_t event); /* Callback for data received from UART */
static void uart_task(void *pvParameters);

static void print_array(void * to_print, uint16_t len);

/* Helper functions to find GATT database handles and allocate/free buffers for GATT operations */
static gatt_db_lookup_table_t 	*app_bt_find_by_handle(uint16_t handle);
static uint8_t 					*app_bt_alloc_buffer(uint16_t len);
static void 					 app_bt_free_buffer(uint8_t *p_data);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

cyhal_pwm_t status_pwm_obj;

uint16_t connection_id = 0;

/* Global variable for notify task handle and uart task handle */
TaskHandle_t NotifyTaskHandle = NULL;
TaskHandle_t UartTaskHandle = NULL;

/* Queue Handle */
QueueHandle_t xUARTQueue = 0;

#define BONDED WICED_TRUE
#define BONDING WICED_FALSE
bool bond_mode = BONDING;		// State of the peripheral - bonded or bonding

/* Bonding that goes into NV memory - we need remote keys, cccd value, and local identity keys */
wiced_bt_device_link_keys_t 	link_keys;
uint8_t							cccd[2];
wiced_bt_local_identity_keys_t 	identity_keys;

/* kv-store block device and kv-store object */
mtb_kvstore_bd_t  block_device;
mtb_kvstore_t  kvstore_obj;
uint32_t data_size = 0;	/* Size of data to read from kvstore */

/* Structure for GPIO interrupt */
cyhal_gpio_callback_data_t app_button_isr_data =
{
    .callback     = app_button_isr,
    .callback_arg = NULL
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
    cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    /* Initialize LED Pin */
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize PWM for connection status LED */
    cyhal_pwm_init(&status_pwm_obj, CYBSP_USER_LED2, NULL);
    cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_OFF_DUTY, LED_BONDING_FREQ);
    cyhal_pwm_start(&status_pwm_obj);

    /* Initialize the user button and configure a falling edge interrupt */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP,
                    CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &app_button_isr_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY, true);

    printf("**********Application Start*****************\n");

    /* Initialize the NV storage */
    app_kvstore_init_api(&block_device);			/* Map low level flash functions to KV store API */
    app_kvstore_init_bd();							/* Initialize flash */
    uint32_t  start_addr, length;
    app_kvstore_get_params(&length, &start_addr);	/* Initialize KV Store library */
    result = mtb_kvstore_init(&kvstore_obj, start_addr, length, &block_device);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to initialize kv-store \n");
        CY_ASSERT(0);
    }

	/* Zero out the bonding info in RAM to start */
	memset( &link_keys, 0, sizeof(link_keys) );
	memset( cccd, 0, sizeof(cccd) );
	memset( &identity_keys, 0, sizeof(identity_keys) );

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Start task to handle Counter notifications */
    xTaskCreate (notify_task, "NotifyTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY,
                 &NotifyTaskHandle);

 	/* Setup UART user input interface */
    xUARTQueue = xQueueCreate( 10, sizeof(uint8_t) );
    cyhal_uart_register_callback(&cy_retarget_io_uart_obj, rx_cback, NULL); /* Register UART Rx callback */
    cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY, 1, TRUE); /* Enable Rx interrupt */
    xTaskCreate (uart_task, "UartTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &UartTaskHandle);

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

    cy_rslt_t kv_rslt;

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

			    /* Check for existing data in KV store for a bonded device */
				kv_rslt = mtb_kvstore_read(&kvstore_obj, "kv_link_keys", NULL, NULL);
			    if (kv_rslt == CY_RSLT_SUCCESS) /* Data exists, so read in bonded device data */
			    {
			        data_size = sizeof(link_keys);
			    	mtb_kvstore_read(&kvstore_obj, "kv_link_keys", (uint8_t *)&link_keys, &data_size);

			        /* Add device to the address resolution database */
					wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );

			        bond_mode = BONDED; /* We have bonding information already, so don't go into bonding mode */

			    }

				printf("Bonding info: \n");
				printf("Remote BDA: ");
				print_array(&link_keys.bd_addr, sizeof(link_keys.bd_addr));
				printf("CCCD: ");
				print_array(cccd, sizeof(cccd));
				printf("Link Key Struct Contents: ");
				print_array(&link_keys, sizeof(link_keys));

				/* Register GATT callback and initialize database*/
				wiced_bt_gatt_register( app_bt_gatt_event_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );

				/* Enable pairing */
				wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE );

				/* Set advertisement packet and begin advertising */
				wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
				                                        cy_bt_adv_packet_data);
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

	            result = WICED_BT_SUCCESS;
			}
			else
			{
				printf( "Failed to initialize Bluetooth controller and stack\n" );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 							// Pairing Complete event
		    printf( "Pairing Complete %d.\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason );
	        if ( p_event_data->pairing_complete.pairing_complete_info.ble.reason == WICED_BT_SUCCESS ) /* Bonding successful */
	        {
		        	bond_mode = BONDED; // remember that the device is now bonded
                    result = WICED_BT_SUCCESS;
            }
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
	        printf( "Encryption Status event: Remote BDA ");
	        print_bd_address(p_event_data->encryption_status.bd_addr);
	        printf(" res %d\n", p_event_data->encryption_status.result );

	        /* Connection has been encrypted and we are already bonded meaning that we have correct/paired device restore values in the NV memory */
	        if( bond_mode == BONDED)
	    	{
	    		/* Set CCCD value from the value that was previously saved in the MV memory */
	    		app_psoc_button_count_client_char_config[0] = cccd[0];
	    		app_psoc_button_count_client_char_config[1] = cccd[1];
	    		printf("Restored existing CCCD info from NV memory\n");
	    	}
			result = WICED_BT_SUCCESS;
			break;

		case BTM_SECURITY_REQUEST_EVT: 							// Security access
	    	if( bond_mode == BONDING )
	    	{
	    		wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
	    		result = WICED_BT_SUCCESS;
	    	}
	    	else
	    	{
	    		printf("Security Request Denied - not in bonding mode\n");
	    		result = WICED_BT_FAILED_ON_SECURITY;
	    	}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys for paired device
			printf( "Paired Device Key Update\n");
			memcpy(&link_keys, &(p_event_data->paired_device_link_keys_update), sizeof( wiced_bt_device_link_keys_t));
			kv_rslt = mtb_kvstore_write(&kvstore_obj, "kv_link_keys", (uint8_t *)&link_keys, sizeof(link_keys));
			if(CY_RSLT_SUCCESS == kv_rslt)
			{
				printf( "Keys saved to NV memory for BDA ");
				print_bd_address(link_keys.bd_addr);

				print_array(&(p_event_data->paired_device_link_keys_update), sizeof( wiced_bt_device_link_keys_t ));

				/* Store starting CCCD value */
				kv_rslt = mtb_kvstore_write(&kvstore_obj, "kv_cccd", (uint8_t *)cccd, sizeof(cccd));

				result = WICED_BT_SUCCESS;
			}
			else
			{
				printf("NV memory write error: %ld\n", kv_rslt);
				result = WICED_BT_ERROR;
			}


			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 			// Retrieve saved link keys
	        /* If there is no stored bonded device info, we must return an error to cause the stack to generate keys. After generating keys
	         * the stack will call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that the keys can be stored */
			kv_rslt = mtb_kvstore_read(&kvstore_obj, "kv_link_keys", NULL, NULL);
		    if (CY_RSLT_SUCCESS == kv_rslt) /* Data exists, so provide it to the stack */
		    {
		    	data_size = sizeof(link_keys);
		    	mtb_kvstore_read(&kvstore_obj, "kv_link_keys", (uint8_t *)&link_keys, &data_size);
		    	data_size = sizeof(cccd);
		        mtb_kvstore_read(&kvstore_obj, "kv_cccd", (uint8_t *)cccd, &data_size);

		        /* Provide keys to stack */
				memcpy(&(p_event_data->paired_device_link_keys_request), &(link_keys), sizeof(wiced_bt_device_link_keys_t));
                wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
				printf("Link keys are available:");
				print_array(&(link_keys), sizeof(wiced_bt_device_link_keys_t));

				result = WICED_BT_SUCCESS;
		    }
			else
			{
				printf("New link keys need to be generated by the stack\n");
				result = WICED_BT_ERROR;
			}
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			printf( "Local Identity Key Update\n" );
			memcpy(&identity_keys, &(p_event_data->local_identity_keys_update), sizeof( wiced_bt_local_identity_keys_t));
			kv_rslt = mtb_kvstore_write(&kvstore_obj, "kv_identity_keys", (uint8_t *)&identity_keys, sizeof(identity_keys));
			if(CY_RSLT_SUCCESS == kv_rslt)
			{
				printf( "Local identity Keys saved to NV memory:");
				print_array(&identity_keys, sizeof( identity_keys ));
			}
			else
			{
				printf("NV memory write error: %ld\n", kv_rslt);
			}
			result = WICED_BT_SUCCESS;
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 				// Read keys from NV Memory
			printf( "Local Identity Key Request\n" );
	        /* If the keys are not in NV memory, return an error to cause the stack to generate
	         *  keys. The stack will call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT
	         * so the keys can be stored */
            kv_rslt = mtb_kvstore_read(&kvstore_obj, "kv_identity_keys", NULL, NULL);
            if (kv_rslt != CY_RSLT_SUCCESS)
            {
                printf("New Keys need to be generated! \n");

                result = WICED_BT_ERROR; /* This causes new keys to be generated */
            }
            else
            {
                printf("Identity keys are available in the database.\n");
                data_size = sizeof(identity_keys);
                mtb_kvstore_read(&kvstore_obj, "kv_identity_keys", (uint8_t *)&identity_keys, &data_size);
                memcpy(&(p_event_data->local_identity_keys_request),
                       &(identity_keys), sizeof(wiced_bt_local_identity_keys_t));
                printf("Local identity keys read from Flash: \n");
                print_array(&identity_keys, sizeof(wiced_bt_local_identity_keys_t));

                result = WICED_BT_SUCCESS; /* Keys don't need to be generated */

            }
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
            if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF ) /* Advertising stopped */
			{
            	if(0 == connection_id) /* not connected  - LED off */
				{
            	    cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_OFF_DUTY, LED_BONDING_FREQ);
				}
            	else /* connected - LED on */
				{
            	    cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_ON_DUTY, LED_BONDING_FREQ);
				}
			}
            else /* Advertising is on - LED blinking */
            {
            	if( bond_mode == BONDED ) /* Device is bonded */
            	{
            		cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_BLINK_DUTY, LED_BONDED_FREQ);
            	}
            	else /* Device is not bonded */
            	{
    				cyhal_pwm_set_duty_cycle( &status_pwm_obj, LED_BLINK_DUTY, LED_BONDING_FREQ);
            	}
            }
            result = WICED_BT_SUCCESS;
			break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Connection parameter update status:%d, "
                   "Connection Interval: %d, "
                   "Connection Latency: %d, "
                   "Connection Timeout: %d\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            /* Print the updated BLE physical link*/
            printf("Selected TX PHY - %dM\n Selected RX PHY - %dM\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

		default:
			break;
    }

    return result;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_event_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_connect_event_handler(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        status = app_bt_server_event_handler(p_event_data);
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
            {
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            }
            status = WICED_BT_GATT_SUCCESS;
        }
        break;

    default:
    	printf( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_connect_event_handler
 *
 * Handles GATT connection status changes.
 *
 * Param:	p_conn_status  Pointer to data that has connection details
 * Return:	wiced_bt_gatt_status_t
 * See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
           	printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
           	print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id );

			/* Handle the connection */
			connection_id = p_conn_status->conn_id;
        }
        else
        {
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

			/* Handle the disconnection */
            connection_id = 0;

            /* Reset the CCCD value so that CCCD will be off if a new device is bonded */
            memset( &link_keys.bd_addr, 0, sizeof( wiced_bt_device_address_t ) );
            app_psoc_button_count_client_char_config[0] = 0;
            app_psoc_button_count_client_char_config[1] = 0;

			/* Restart the advertisements */
			wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_server_event_handler
 *
 * Invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs in GATT Event callback.
 *
 * Param:	p_data   				Pointer to BLE GATT request data
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_server_event_handler(wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t   *p_att_req = &p_data->attribute_request;

    switch (p_att_req->opcode)
    {

		case GATT_REQ_READ: /* Attribute read notification (attribute value internally read from GATT database) */
		case GATT_REQ_READ_BLOB:
			status = app_bt_gatt_req_read_handler(p_att_req->conn_id, p_att_req->opcode,
																	   &p_att_req->data.read_req, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_BY_TYPE:
			status = app_bt_gatt_req_read_by_type_handler(p_att_req->conn_id, p_att_req->opcode,
																   &p_att_req->data.read_by_type, p_att_req->len_requested);
			break;

		case GATT_REQ_READ_MULTI:
		case GATT_REQ_READ_MULTI_VAR_LENGTH:
			status = app_bt_gatt_req_read_multi_handler(p_att_req->conn_id, p_att_req->opcode,
															  &p_att_req->data.read_multiple_req, p_att_req->len_requested);
			break;

		case GATT_REQ_WRITE:
		case GATT_CMD_WRITE:
		case GATT_CMD_SIGNED_WRITE:
			status = app_bt_write_handler(p_data);
			if ((p_att_req->opcode == GATT_REQ_WRITE) && (status == WICED_BT_GATT_SUCCESS))
			{
				wiced_bt_gatt_write_req_t *p_write_request = &p_att_req->data.write_req;
				wiced_bt_gatt_server_send_write_rsp(p_att_req->conn_id, p_att_req->opcode, p_write_request->handle);
			}
			break;

		case GATT_REQ_PREPARE_WRITE:
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_EXECUTE_WRITE:
			wiced_bt_gatt_server_send_execute_write_rsp(p_att_req->conn_id, p_att_req->opcode);
			status = WICED_BT_GATT_SUCCESS;
			break;

		case GATT_REQ_MTU:
			/* Application calls wiced_bt_gatt_server_send_mtu_rsp() with the desired mtu */
			status = wiced_bt_gatt_server_send_mtu_rsp(p_att_req->conn_id,
													   p_att_req->data.remote_mtu,
													   wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
			break;

		case GATT_HANDLE_VALUE_NOTIF: /* Stack has sent notification */
			break;

        case GATT_HANDLE_VALUE_IND: /* Stack has sent indication*/
			break;

		case GATT_HANDLE_VALUE_CONF: /* Stack has received confirmation from an indication*/
			break;

		default:
	    	printf( "Unhandled GATT Server Event: 0x%x (%d)\n", p_att_req->opcode, p_att_req->opcode );
			break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_write_handler
 *
 * Invoked when GATTS_REQ_TYPE_WRITE is received from the
 * client device. Handles "Write Requests" received from Client device.
 *
 * Param:	p_write_req   			Pointer to BLE GATT write request
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data)
{
	cy_rslt_t kv_rslt = 0;

	wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;

    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

     for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
     {
         /* Check for a matching handle entry */
         if (app_gatt_db_ext_attr_tbl[i].handle == p_write_req->handle)
         {
             /* Detected a matching handle in the external lookup table */
             if (app_gatt_db_ext_attr_tbl[i].max_len >= p_write_req->val_len)
             {
                 /* Value fits within the supplied buffer; copy over the value */
                 app_gatt_db_ext_attr_tbl[i].cur_len = p_write_req->val_len;
                 memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
                 memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

                 if (memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_write_req->p_val, app_gatt_db_ext_attr_tbl[i].cur_len) == 0)
                 {
                	 status = WICED_BT_GATT_SUCCESS;
                 }

                 switch ( p_write_req->handle )
                 {
                 	 // Add action when specified handle is written
                 	 case HDLC_PSOC_LED_VALUE:
                 		cyhal_gpio_write(CYBSP_USER_LED, app_psoc_led[0] == 0 );
                  		printf( "Turn the LED %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
                  		break;
 					case HDLD_PSOC_BUTTON_COUNT_CLIENT_CHAR_CONFIG:
                 		printf("Setting notify (0x%02x, 0x%02x)\n", p_write_req->p_val[0], p_write_req->p_val[1]);
                    	if ( p_write_req->val_len != 2 ) /* Check that exactly 2 bytes were sent since the CCCD is always 2 bytes */
 						{
 							return WICED_BT_GATT_INVALID_ATTR_LEN;
 						}

                     	/* Save value to NV memory */
                    	cccd[0] = p_write_req->p_val[0];
 						cccd[1] = p_write_req->p_val[1];
 						kv_rslt = mtb_kvstore_write(&kvstore_obj, "kv_cccd", (uint8_t *)cccd, sizeof(cccd));
 						if(CY_RSLT_SUCCESS == kv_rslt)
 						{
 							printf( "CCCD value saved to NV memory.\n" );
 						}
 						else
 						{
 							printf("NV memory write error: %ld\n", kv_rslt);
 						}
                     	break;
                 }
             }
             else
             {
                 /* Value to write will not fit within the table */
            	 status = WICED_BT_GATT_INVALID_ATTR_LEN;
                 printf("Invalid attribute length during GATT write\n");
             }
             break;
         }
     }
     if (WICED_BT_GATT_SUCCESS != status)
     {
         printf("GATT write failed: %d\n", status);
     }

     return status;
}


/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_handler
 *
 * This Function handles GATT read and read blob events
 *
 * Params: 	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return: 	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                              wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_len_to_copy, to_send;
    uint8_t *from;

    if ((puAttribute = app_bt_find_by_handle(p_read_req->handle)) == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

	switch ( p_read_req->handle )
	{
		// Add action when specified handle is read
		case HDLC_PSOC_LED_VALUE:
			printf( "LED is %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
			break;
	}

    to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);
    from = puAttribute->p_data + p_read_req->offset;
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_by_type_handler
 *
 * Process read-by-type request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t	BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                       wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = app_bt_find_by_handle(attr_handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used, len_requested - used, &pair_len,
                                                                attr_handle, puAttribute->cur_len, puAttribute->p_data);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

	switch ( p_read_req->s_handle )
	{
		// Add action when specified handle is read
		case HDLC_PSOC_LED_VALUE:
			printf( "LED is %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
			break;
	}

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used, p_rsp, (void *)app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_multi_handler
 *
 * Process write read multi request from peer device
 *
 * Params:	conn_id       			Connection ID
 * 			opcode        			BLE GATT request type opcode
 * 			p_read_req    			Pointer to read request containing the handle to read
 * 			len_requested 			Length of data requested
 * Return:	wiced_bt_gatt_status_t  BLE GATT status
*******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
                                                  wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    int used = 0;
    int xx;
    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
        if ((puAttribute = app_bt_find_by_handle(handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used, len_requested - used,
                                                        puAttribute->handle, puAttribute->cur_len, puAttribute->p_data);
            if (!filled)
            {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

	switch ( *p_read_req->p_handle_stream )
	{
		// Add action when specified handle is read
		case HDLC_PSOC_LED_VALUE:
			printf( "LED is %s\r\n", app_psoc_led[0] ? "ON" : "OFF" );
			break;
	}

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp, (void *)app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
* Function Name: app_bt_find_by_handle
*
* Finds attribute location by handle
*
* Param:  handle    				handle to look up
* Return: gatt_db_lookup_table_t   	pointer to location containing handle data
********************************************************************************/
static gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}


/*******************************************************************************
* Function Name: app_bt_alloc_buffer
*
* This Function allocates the buffer of requested length
*
* Param:  len			Length of buffer
* Return: uint8_t*      Pointer to allocated buffer
********************************************************************************/
static uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    return p;
}


/*******************************************************************************
* Function Name: app_bt_free_buffer
*
* This Function frees the buffer requested
*
* Param:  p_data		Pointer to buffer to be freed
********************************************************************************/
static void app_bt_free_buffer(uint8_t *p_data)
{
    if (p_data != NULL)
    {
        free(p_data);
    }
}


/*******************************************************************************
* Function Name: notify_task
*
* This Function handles sending notifications
*
********************************************************************************/
static void notify_task(void * arg)
{
    /* Notification values received from ISR */
    uint32_t ulNotificationValue;
    while(true)
    {
       /* Wait for the button ISR */
       ulNotificationValue = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

       /* If button was pressed increment value and check to see if a
        * BLE notification should be sent. If this value is not 1, then
        * it was not a button press (most likely a timeout) that caused
        * the event so we don't want to send a BLE notification. */
        if (ulNotificationValue == 1)
        {
            if( connection_id ) /* Check if we have an active
                                   connection */
            {
                /* Check to see if the client has asked for
                   notifications */
                 if( app_psoc_button_count_client_char_config[0] &
                     GATT_CLIENT_CONFIG_NOTIFICATION )
                 {
                     printf( "Notify button press count: (%d)\n",
                     		app_psoc_button_count[0] );
                     wiced_bt_gatt_server_send_notification( connection_id,
					HDLC_PSOC_BUTTON_COUNT_VALUE,
					app_psoc_button_count_len,
					app_psoc_button_count,
					NULL);
                 }
             }
         }
    }
}



/*******************************************************************************
* Function Name: app_button_isr
*
* This Function handles the button interrupts
*
********************************************************************************/
static void app_button_isr(void* handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(event);
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Increment button counter */
	app_psoc_button_count[0]++;

    /* Notify the counter task that the button was pressed */
    vTaskNotifyGiveFromISR( NotifyTaskHandle, &xHigherPriorityTaskWoken );

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
       Switch should be performed to ensure the interrupt returns
       directly to the highest priority task.  The macro used for this
       purpose is dependent on the port in use and may be called
       portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/*******************************************************************************
* Function Name: uart_task()
********************************************************************************
*
* Summary:
*   This function runs the UART task which processes the received commands via
*   Terminal.
*
* Parameters:
*   void *pvParameters                 Not used
*
* Return:
*   None
*
*******************************************************************************/
static void uart_task(void *pvParameters)
{
    uint8_t readbyte;

    for(;;)
    {
		/* Wait for a character to be sent from the UART ISR */
        if(pdPASS == xQueueReceive( xUARTQueue, &(readbyte), portMAX_DELAY))
        {
            switch (readbyte)
			{
            	case 'e':
					if(connection_id == 0) /* Only allowed when not connected */
                    {
                        printf( "Removing bonded device info\n");

                        /* Put into bonding mode and restart advertising */
                        bond_mode = BONDING;

                        /* Remove from the bonded device list */
                        wiced_bt_dev_delete_bonded_device(link_keys.bd_addr);
                        printf( "Removed host from bonded device list: ");
                        print_bd_address(link_keys.bd_addr);

                        /* Remove device from address resolution database */
                        wiced_bt_dev_remove_device_from_address_resolution_db (&(link_keys));
                        printf( "Removed device from address resolution database\n");

                        /* Clear bonding information and remove from NV memory */
                        memset( &link_keys, 0, sizeof(link_keys) );
                        memset( cccd, 0, sizeof(cccd));
                        mtb_kvstore_delete(&kvstore_obj, "kv_link_keys");
                        mtb_kvstore_delete(&kvstore_obj, "kv_cccd");

						if(wiced_bt_ble_get_current_advert_mode() == BTM_BLE_ADVERT_OFF)
						{
							wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
						}
						else
						{
							cyhal_pwm_set_duty_cycle(&status_pwm_obj, LED_BLINK_DUTY, LED_BONDING_FREQ);
						}
                    }
                    else
                    {
                        printf("Bonding information can only be deleted when disconnected\n");
                    }
					break;

            	default:
            		printf( "Invalid input");
            		break;
			}
        }
    }
}


/*******************************************************************************
* Function Name: rx_cback()
********************************************************************************
*
* Summary:
*   This function gets a character from the UART and sends it to the UART
*   task for processing
*
* Parameters:
*   void *handler_arg:                 Not used
*   cyhal_uart_event_t event:          Not used
*
* Return:
*   None
*
*******************************************************************************/
void rx_cback(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    uint8_t readbyte;
	cy_rslt_t status;
    BaseType_t xYieldRequired = pdFALSE;

    /* Read one byte from the buffer with a 1ms timeout */
    status = cyhal_uart_getc(&cy_retarget_io_uart_obj , &readbyte, 1);

    /* If a character was received, send it to the UART task */
	if(CY_RSLT_SUCCESS == status)
	{
    	xQueueSendFromISR( xUARTQueue, &readbyte, &xYieldRequired);
	}

	/* Yield current task if a higher priority task is now unblocked */
	portYIELD_FROM_ISR(xYieldRequired);
}


/*******************************************************************************
* Function Name: void print_array( void* to_print, uint16_t len )
********************************************************************************
* Summary:
*   This is a utility function that prints the specified number of values from memory
*
* Parameters:
*   void* to_print            	: Pointer to the location to print
*   uint16_t					: Number of bytes to print
*
* Return:
*  void
*
********************************************************************************/
static void print_array(void * to_print, uint16_t len)
{
	uint16_t counter;

	for( counter = 0; counter<len;counter++ )
	{
	   if( counter % 16 == 0 )
	   {
		   printf( "\n" );
	   }
		printf( "%02X ", *((uint8_t *)(to_print + counter)) );
	}
	printf( "\n" );

}
/* [] END OF FILE */
