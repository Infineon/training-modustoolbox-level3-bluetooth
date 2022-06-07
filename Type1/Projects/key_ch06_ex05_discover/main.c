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

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#define UART_INPUT true

#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

/* Typdef for function used to free allocated buffer to stack */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/* These are the UUIDs used in the peripheral. They are copied from the peripheral cycfg_gatt_db.h file */
#define __UUID_SERVICE_PSOC                          0x90, 0x86, 0x13, 0xFE, 0x44, 0x99, 0x07, 0x93, 0x67, 0x4B, 0x3F, 0x63, 0xFF, 0x8A, 0x9E, 0x9D
#define __UUID_CHARACTERISTIC_PSOC_LED               0xCD, 0x5B, 0xA7, 0xEB, 0x47, 0x68, 0x4C, 0x9E, 0xAB, 0x4C, 0x8B, 0xBA, 0xDA, 0xBD, 0x8A, 0x26
#define __UUID_CHARACTERISTIC_PSOC_BUTTON_COUNT      0xA6, 0xDB, 0x81, 0x59, 0x4D, 0x4F, 0x1E, 0x8B, 0x20, 0x4B, 0x4E, 0x6E, 0x83, 0xA0, 0x07, 0xFB
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);

/* Declaration for scan callback function */
void	scanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );

/* Function to write to an attribute */
void writeAttribute( uint16_t conn_id, uint16_t handle, uint16_t offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t* val );

/* Service discovery functions */
static void startServiceDiscovery( void );
static void startCharacteristicDiscovery( void );
static void startDescriptorDiscovery( void );

#if (UART_INPUT == true)
/* Tasks to handle UART */
static void rx_cback(void *handler_arg, cyhal_uart_event_t event); /* Callback for data received from UART */
static void uart_task(void *pvParameters);
#endif

/* Helper functions to allocate/free buffers for GATT operations */
static uint8_t 					*app_bt_alloc_buffer(uint16_t len);
static void 					app_bt_free_buffer(uint8_t *p_data);



/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id;

uint8_t ledValue;			// Current value of the LED characteristic

/* Enable RTOS aware debugging in OpenOCD */
volatile int uxTopUsedPriority;

/*UART task and Queue handles */
TaskHandle_t  UartTaskHandle = NULL;
QueueHandle_t xUARTQueue = 0;

/* Start/end handles for service group */
static const uint8_t serviceUUID[] = { __UUID_SERVICE_PSOC};
static uint16_t serviceStartHandle = 0x0001;
static uint16_t serviceEndHandle   = 0xFFFF;

/* Structure to hold handles of discovered characteristics: */
typedef struct {
uint16_t startHandle;
uint16_t endHandle;
uint16_t valHandle;
uint16_t cccdHandle;
} charHandle_t;

/* Start/end handles for each characteristic group */
static const uint8_t ledUUID[] = { __UUID_CHARACTERISTIC_PSOC_LED };
static charHandle_t  ledChar;
static const uint8_t counterUUID[] = { __UUID_CHARACTERISTIC_PSOC_BUTTON_COUNT };
static charHandle_t  counterChar;

/* Array of characteristic group handles to hold values temporarily as they are discovered */
#define MAX_CHARS_DISCOVERED (10)
static charHandle_t charHandles[MAX_CHARS_DISCOVERED];
static uint32_t charHandleCount;

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
				wiced_bt_set_local_bdaddr((uint8_t*)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth Device Address: ");
				print_bd_address(bda);

				/* Register GATT callback */
				wiced_bt_gatt_register( app_bt_gatt_event_callback );

				#if (UART_INPUT == true)
				/* Setup UART user input interface now that the stack is running */
			    xUARTQueue = xQueueCreate( 10, sizeof(uint8_t) );
			    cyhal_uart_register_callback(&cy_retarget_io_uart_obj, rx_cback, NULL); /* Register UART Rx callback */
			    cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY , 3, TRUE); /* Enable Rx interrupt */
			    xTaskCreate (uart_task, "UartTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &UartTaskHandle); /* Start task */
				uint8_t helpCommand = '?';
				xQueueSend( xUARTQueue, &helpCommand, 0); /* Print out list of commands */
				#endif

	            result = WICED_BT_SUCCESS;
			}
			else
			{
				printf( "Failed to initialize Bluetooth controller and stack\n" );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT:
			printf("Pairing complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.status);
			result = WICED_BT_SUCCESS;
			break;

		case BTM_ENCRYPTION_STATUS_EVT:
			printf("Encrypt status: %d\n", p_event_data->encryption_status.result);
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			result = WICED_BT_ERROR; // Return error since keys are not stored in EEPROM
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 				// Read keys from EEPROM
            /* This should return WICED_BT_SUCCESS if not using privacy. If RPA is enabled but keys are not
               stored in EEPROM, this must return WICED_BT_ERROR so that the stack will generate new privacy keys */
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			switch( p_event_data->ble_scan_state_changed )
			{
				case BTM_BLE_SCAN_TYPE_NONE:
					printf( "Scanning stopped.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					printf( "High duty scanning.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					printf( "Low duty scanning.\r\n" );
					break;
			}
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_CONNECTION_PARAM_UPDATE:
			result = WICED_BT_SUCCESS;
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

    case GATT_OPERATION_CPLT_EVT:
     	/* Look for any type of successful GATT completion */
    	if (p_event_data->operation_complete.status == WICED_BT_GATT_SUCCESS ||
			p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPTED_MITM ||
    		p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPTED_NO_MITM ||
			p_event_data->operation_complete.status == WICED_BT_GATT_NOT_ENCRYPTED)
    	{
    		printf("GATT operation completed successfully\n");
			status = WICED_BT_GATT_SUCCESS;
    		if(p_event_data->operation_complete.op == GATTC_OPTYPE_READ_HANDLE)
    		{
    			if(p_event_data->operation_complete.response_data.handle == ledChar.valHandle)
    			{
    				printf("LED value is: %d\n",ledValue);
    			}
    		}
    		else if (p_event_data->operation_complete.op == GATTC_OPTYPE_NOTIFICATION)
			{
    			if(p_event_data->operation_complete.response_data.handle == counterChar.valHandle)
    			{
    				printf("Count notification received: %d\n", *p_event_data->operation_complete.response_data.att_value.p_data);
    			}
			}
    	}
    	else
    	{
    		printf("GATT operation failed with status: %d\n", p_event_data->operation_complete.status);
    	}
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

    case GATT_DISCOVERY_RESULT_EVT:
    	//////////////// Services Discovery /////////////////
		if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_BY_UUID )
		{
			serviceStartHandle = p_event_data->discovery_result.discovery_data.group_value.s_handle;
			serviceEndHandle = p_event_data->discovery_result.discovery_data.group_value.e_handle;
			printf( "Discovered Service Start=0x%04X End=0x%04X\r\n", serviceStartHandle, serviceEndHandle );
		}

		//////////////// Characteristics Discovery /////////////////
		if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTICS )
		{
			charHandles[charHandleCount].startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
			charHandles[charHandleCount].valHandle =   p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
			charHandles[charHandleCount].endHandle = serviceEndHandle; /* Assume this is the last characteristic in the service so its end handle is at the end of the service group */

			printf( "Char Handle=0x%04X Value Handle=0x%04X ", charHandles[charHandleCount].startHandle, charHandles[charHandleCount].valHandle);

			if( charHandleCount != 0 ) /* If this isn't the first characteristic found, we need to reset the end handle of the previous characteristic */
			{
				charHandles[charHandleCount-1].endHandle = charHandles[charHandleCount].startHandle - 1;
			}
			charHandleCount += 1;

			if( charHandleCount > MAX_CHARS_DISCOVERED-1 )
			{
				printf( "This is really bad.. we discovered more characteristics than we can save\r\n" );
			}

			/* Look only for 16 byte UUIDs */
			if( p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.len == LEN_UUID_128)
			{
				if( memcmp( ledUUID, p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128, LEN_UUID_128 ) == 0 ) // If it is the LED characteristic
				{
					ledChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
					ledChar.valHandle =   p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
				}

				if( memcmp( counterUUID, p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128,LEN_UUID_128 ) == 0 ) // If it is the button count characteristic
				{
					counterChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
					counterChar.valHandle =  p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
				}

				printf( "UUUD: ");
				for (int i=0; i < p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.len; i++ ) // Dump the UUID bytes to the screen
				{
					printf( "%02X ", p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128[i] );
				}
			}
			printf( "\r\n" );
		}

		//////////////// Descriptors Discovery /////////////////
		if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
		{
			if( p_event_data->discovery_result.discovery_data.char_descr_info.type.uu.uuid16 == __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION )
			{
				counterChar.cccdHandle = p_event_data->discovery_result.discovery_data.char_descr_info.handle;

				/* Print out the handle and UUID of the CCCD */
				printf( "Char Descriptor Handle = 0x%04X, UUID: ", p_event_data->discovery_result.discovery_data.char_descr_info.handle);

				for( int i=0; i<p_event_data->discovery_result.discovery_data.char_descr_info.type.len; i++ )
				{
					/* We will use the uuid128 value from the union and just print out as many bytes as the len parameter
					 * indicates. This allows us to print any type of UUID */
					printf( "%02X ", p_event_data->discovery_result.discovery_data.char_descr_info.type.uu.uuid128[i] );
				}
				printf( "\r\n" );
			}
		}
    	break;

    case GATT_DISCOVERY_CPLT_EVT:
    	/* Once all characteristics are discovered... you need to setup the end handles */
    	if( p_event_data->discovery_complete.discovery_type == GATT_DISCOVER_CHARACTERISTICS )
    	{
    	  for( int i=0; i<charHandleCount; i++ )
    	  {
    	    if( charHandles[i].startHandle == ledChar.startHandle )
    	      ledChar.endHandle = charHandles[i].endHandle;
    	    if( charHandles[i].startHandle == counterChar.startHandle )
    	      counterChar.endHandle = charHandles[i].endHandle;
    	  }
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
           	/* Handle the connection */
            printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
           	print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id );
			
            connection_id = p_conn_status->conn_id;

    		/* Initiate pairing */
    		wiced_bt_dev_sec_bond(p_conn_status->bd_addr, p_conn_status->addr_type, BT_TRANSPORT_LE, 0, NULL);
        }
        else
        {
            /* Handle the disconnection */
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

			connection_id = 0;
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*******************************************************************************
* Function Name: void scanCallback(
* 					wiced_bt_ble_scan_results_t *p_scan_result,
* 					uint8_t *p_adv_data )
********************************************************************************/
void scanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
	#define MAX_ADV_NAME_LEN	(28) 		/* Maximum possible name length since flags take 3 bytes and max packet is 31. */
	#define SEARCH_DEVICE_NAME	"key_per"	/* Name of device to search for */

	uint8_t len;
	uint8_t *p_name = NULL;

	uint8_t dev_name[MAX_ADV_NAME_LEN];

	p_name = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &len );

	if( p_name && ( len == strlen(SEARCH_DEVICE_NAME) ) && (memcmp( SEARCH_DEVICE_NAME, p_name, len ) == 0) )
	{
		memcpy( dev_name, p_name, len);
		dev_name[len] = 0x00;	/* Null terminate the string */

		printf("Found Device \"%s\" with BD Address: ", dev_name);
		print_bd_address(p_scan_result->remote_bd_addr);

		/* Connect to peripheral and stop scanning*/
		wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);
		wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, scanCallback );
	}
}


/*******************************************************************************
* Function Name: void writeAttribute
********************************************************************************/
void writeAttribute( uint16_t conn_id, uint16_t handle, uint16_t offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t* val )
{
	if(conn_id && handle) /* Only write if we have a connection and the handle is defined */
	{
		/* Set up write parameters */
		wiced_bt_gatt_write_hdr_t write_params;
		write_params.handle = handle;
		write_params.offset = offset;
		write_params.len = len;
		write_params.auth_req = auth_req;

		/* Send the write command */
		wiced_bt_gatt_client_send_write ( connection_id, GATT_REQ_WRITE, &write_params, val, NULL);
	}
}

/*******************************************************************************
* Function Name: startServiceDiscovery
********************************************************************************/
static void startServiceDiscovery( void )
{
    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 0x0001;
    discovery_param.e_handle = 0xFFFF;
    discovery_param.uuid.len = LEN_UUID_128;
    memcpy( &discovery_param.uuid.uu.uuid128, serviceUUID, LEN_UUID_128 );

    wiced_bt_gatt_status_t status = wiced_bt_gatt_client_send_discover ( connection_id, GATT_DISCOVER_SERVICES_BY_UUID, &discovery_param );
    printf( "Started service discovery. Status: 0x%02X\r\n", status );
}

/*******************************************************************************
* Function Name: startCharacteristicDiscovery
********************************************************************************/
static void startCharacteristicDiscovery( void )
{
	charHandleCount = 0;

	wiced_bt_gatt_discovery_param_t discovery_param;
	memset( &discovery_param, 0, sizeof( discovery_param ) );
	discovery_param.s_handle = serviceStartHandle + 1;
	discovery_param.e_handle = serviceEndHandle;

	wiced_bt_gatt_status_t status = wiced_bt_gatt_client_send_discover( connection_id, GATT_DISCOVER_CHARACTERISTICS, &discovery_param );
	printf( "Started characteristic discover. Status: 0x%02X\r\n", status );
}

/*******************************************************************************
* Function Name: startDescriptorDiscovery
********************************************************************************/
static void startDescriptorDiscovery( void )
{
	printf( "Button Count Start Handle = 0x%04X End Handle=0x%04X\r\n", counterChar.startHandle, counterChar.endHandle );

	wiced_bt_gatt_discovery_param_t discovery_param;
	memset( &discovery_param, 0, sizeof( discovery_param ) );
	discovery_param.s_handle = counterChar.valHandle + 1; /* Use value handle plus 1 since the CCCD must be later in the group */
	discovery_param.e_handle = counterChar.endHandle;

	wiced_bt_gatt_status_t status = wiced_bt_gatt_client_send_discover( connection_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &discovery_param );
	printf( "Start char descriptor discovery. Status: 0x%02X\r\n", status );
}




#if (UART_INPUT == true)
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
				case 's':			// Turn on scanning
					wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, scanCallback );
					break;

				case 'S':			// Turn off scanning
					wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, scanCallback );
					break;

				case 'd':			// Disconnect
					wiced_bt_gatt_disconnect(connection_id);
					break;

				case 'n':			// Turn notifications on
					{
						uint8_t writeData[2] = {0};
						writeData[0] = GATT_CLIENT_CONFIG_NOTIFICATION; /* Values are sent little endian */
						writeAttribute( connection_id, counterChar.cccdHandle, 0, GATT_AUTH_REQ_NO_MITM, sizeof(uint16_t), writeData);
					}
					break;

				case 'N':			// Turn notifications off
					{
						uint8_t writeData[2] = {0};
						writeData[0] = GATT_CLIENT_CONFIG_NONE; /* Values are sent little endian */
						writeAttribute( connection_id, counterChar.cccdHandle, 0, GATT_AUTH_REQ_NO_MITM, sizeof(uint16_t), writeData);
					}
					break;

				case '0':			// LEDs off
				case '1':			// LEDs blue
				case '2':			// LEDs red
				case '3':			// LEDs blue+red
				case '4':			// LEDs green
				case '5':			// LEDs blue+green
				case '6':			// LEDs red+green
				case '7':			// LEDs white
					{
						uint8_t writeData[1];
						writeData[0] = readbyte-'0';
						writeAttribute( connection_id, ledChar.valHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint8_t), writeData);
					}
					break;

				case 'r':			// Read LED value
					wiced_bt_gatt_client_send_read_handle(connection_id, ledChar.valHandle, 0, &ledValue, sizeof(ledValue), GATT_AUTH_REQ_NONE);
					break;

				case 'q':			// Start service discovery
					startServiceDiscovery();
					break;

				case 'w':			// Start characteristic discovery
					startCharacteristicDiscovery();
					break;

				case 'e':			// Start descriptor discovery
					startDescriptorDiscovery();
					break;

				default:
					printf( "Unrecognized command\r\n" );
					/* No break - fall through and display help */

				case '?':			// Help
					printf( "Commands:\r\n" );
					printf( "\t%c\tHelp (this message)\r\n", '?' );
					printf( "\t%c\tStart scanning and connect\r\n", 's' );
					printf( "\t%c\tStop scanning\r\n", 'S' );
					printf( "\t%c\tDisconnect\r\n", 'd' );
					printf( "\t%c\tTurn on Button Count notifications\r\n", 'n' );
					printf( "\t%c\tTurn off Button Count notifications\r\n", 'N' );
					printf( "\t%s\tControl LED\r\n", "0...7" );
					printf( "\t%c\tRead LED value\r\n", 'r' );
					printf( "\t%c\tStart service discovery\r\n", 'q' );
					printf( "\t%c\tStart characteristic discovery\r\n", 'w' );
					printf( "\t%c\tStart descriptor discovery\r\n", 'e' );

					printf( "\r\n" );
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
#endif


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
/* [] END OF FILE */
