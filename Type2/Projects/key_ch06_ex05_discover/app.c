#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_rtos.h"
#include "cycfg.h"
#include "app_bt_utils.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_bt_gatt_util.h"

/* UART input is not used for exercises 1 and 2 */
#define UART_INPUT WICED_TRUE

/* UUIDs from peripheral */
#define __UUID_SERVICE_MYSVC                        0x83u, 0xC3u, 0x54u, 0x15u, 0xC7u, 0x55u, 0x38u, 0x89u, 0x2Bu, 0x4Du, 0x46u, 0xA2u, 0xABu, 0x8Cu, 0x2Bu, 0x87u
#define __UUID_CHARACTERISTIC_MYSVC_LED             0x64u, 0x8Au, 0xE5u, 0x4Cu, 0x1Fu, 0xE8u, 0xF5u, 0xACu, 0xBBu, 0x4Au, 0x44u, 0x92u, 0xE0u, 0x49u, 0x1Du, 0x98u
#define __UUID_CHARACTERISTIC_MYSVC_COUNTER         0xECu, 0x03u, 0x92u, 0x5Cu, 0xBCu, 0x20u, 0x01u, 0xA6u, 0x3Bu, 0x41u, 0x6Fu, 0x28u, 0xFBu, 0x7Bu, 0x02u, 0x26u
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_gatt_status_t	app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );

#if (UART_INPUT == WICED_TRUE)
void					uart_rx_callback( void *data );
#endif

/* Declaration for scan callback function */
void scanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

/* Function to write to an attribute */
void writeAttribute( uint16_t conn_id, uint16_t handle, uint16_t offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t* val );

void startServiceDiscovery( void );
void startCharacteristicDiscovery( void );
void startDescriptorDiscovery( void );

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id = 0;


uint16_t ledStatus = 0;

static const uint8_t serviceUUID[] = { __UUID_SERVICE_MYSVC };
static uint16_t serviceStartHandle = 0x0001;
static uint16_t serviceEndHandle   = 0xFFFF;

typedef struct {
uint16_t startHandle;
uint16_t endHandle;
uint16_t valHandle;
uint16_t cccdHandle;
} charHandle_t;

static const uint8_t ledUUID[] = { __UUID_CHARACTERISTIC_MYSVC_LED };
static charHandle_t  ledChar;
static const uint8_t counterUUID[] = { __UUID_CHARACTERISTIC_MYSVC_COUNTER };
static charHandle_t  counterChar;

#define MAX_CHARS_DISCOVERED (10)
static charHandle_t charHandles[MAX_CHARS_DISCOVERED];
static uint32_t charHandleCount;

/*******************************************************************************
* Function Name: void application_start( void )
********************************************************************************/
void application_start( void )
{
	#if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
		/* Select Debug UART setting to see debug traces on the appropriate port */
		wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
	#endif

	WICED_BT_TRACE( "**** App Start **** \r\n" );

	/* Initialize Stack and Register Management Callback */
	wiced_bt_stack_init( app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
	wiced_result_t status = WICED_BT_SUCCESS;

    WICED_BT_TRACE("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

	switch( event )
	{
		case BTM_ENABLED_EVT:								/* Bluetooth Controller and Host Stack Enabled */
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				/* Use Application Settings dialog to set BT_DEVICE_ADDRESS = random */
				wiced_bt_device_address_t bda;
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

				#if (UART_INPUT == WICED_TRUE)
				wiced_hal_puart_register_interrupt( uart_rx_callback );	// Enable receive interrupts on the PUART
				wiced_hal_puart_set_watermark_level( 1 );				// Interrupt up on each byte received
				wiced_hal_puart_enable_rx();
				WICED_BT_TRACE( "*******************************\r\n" );
				WICED_BT_TRACE( "Type '?' for a list of commands\r\n" );
				WICED_BT_TRACE( "*******************************\r\n" );
				#endif

				/* Register GATT callback */
				wiced_bt_gatt_register( app_bt_gatt_callback );
			}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			/* No keys stored so we need to return error to get Stack to generate them */
			status = WICED_BT_ERROR;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT:
			switch( p_event_data->ble_scan_state_changed )
			{
				case BTM_BLE_SCAN_TYPE_NONE:
					WICED_BT_TRACE( "Scanning stopped.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					WICED_BT_TRACE( "High duty scanning.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					WICED_BT_TRACE( "Low duty scanning.\r\n" );
					break;
			}
			break;

		default:
			break;
	}

	return status;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_callback( 
*					wiced_bt_gatt_evt_t event,
*					wiced_bt_gatt_event_data_t *p_event_data )
********************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	wiced_result_t status = WICED_BT_GATT_SUCCESS;

    wiced_bt_gatt_connection_status_t *p_conn = &p_event_data->connection_status;

	switch( event )
	{
		case GATT_CONNECTION_STATUS_EVT:
			if( p_conn->connected )
			{
				WICED_BT_TRACE( "GATT connect to: BDA %B, Connection ID %d\r\n",p_conn->bd_addr, p_conn->conn_id );

				/* Handle the connection */
				connection_id = p_conn->conn_id;

				/* Initiate pairing */
				wiced_bt_dev_sec_bond(p_conn->bd_addr, p_conn->addr_type, BT_TRANSPORT_LE, 0, NULL);
			}
			else
			{
				// Device has disconnected
				WICED_BT_TRACE("GATT disconnect from: BDA %B, Connection ID '%d', Reason '%s'\n", p_conn->bd_addr, p_conn->conn_id, get_bt_gatt_disconn_reason_name(p_conn->reason) );

				/* Handle the disconnection */
				connection_id = 0;
			}
			break;

	    case GATT_OPERATION_CPLT_EVT:
	     	/* Look for any type of successful GATT completion */
	    	if (p_event_data->operation_complete.status == WICED_BT_GATT_SUCCESS ||
				p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPED_MITM ||
	    		p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPED_NO_MITM ||
				p_event_data->operation_complete.status == WICED_BT_GATT_NOT_ENCRYPTED)
	    	{
	    		WICED_BT_TRACE("GATT operation completed successfully\n");
				status = WICED_BT_GATT_SUCCESS;
				if(p_event_data->operation_complete.op == GATTC_OPTYPE_READ) /* The operation was a read */
	    		{
	    			if(p_event_data->operation_complete.response_data.att_value.handle == ledChar.valHandle ) /* The LED value was read */
	    			{
	    				memcpy(&ledStatus, p_event_data->operation_complete.response_data.att_value.p_data, sizeof(uint8_t));
	    				WICED_BT_TRACE("LED value is: %d\n", ledStatus);
	    			}
	    		}

				if(p_event_data->operation_complete.op == GATTC_OPTYPE_NOTIFICATION) /* The operation was a notification */
				{
					/* Print notification value */
					WICED_BT_TRACE_ARRAY( p_event_data->operation_complete.response_data.att_value.p_data, p_event_data->operation_complete.response_data.att_value.len, "Notification Value: " );
				}
	    	}
	    	else
	    	{
	    		WICED_BT_TRACE("GATT operation failed with status: %d\n", p_event_data->operation_complete.status);
				status = WICED_BT_GATT_ERROR;
	    	}
	    	break;

		case GATT_DISCOVERY_RESULT_EVT:
			//////////////// Services Discovery /////////////////
			if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_BY_UUID )
	        {
				serviceStartHandle = GATT_DISCOVERY_RESULT_SERVICE_START_HANDLE( p_event_data );
				serviceEndHandle = GATT_DISCOVERY_RESULT_SERVICE_END_HANDLE( p_event_data );
				WICED_BT_TRACE( "Discovered Service Start=%X End=%X\r\n", serviceStartHandle, serviceEndHandle );
	        }


			//////////////// Characteristics Discovery /////////////////
			if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTICS )
			{
				charHandles[charHandleCount].startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
				charHandles[charHandleCount].valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
				charHandles[charHandleCount].endHandle = serviceEndHandle;

				WICED_BT_TRACE( "Char Handle=0x%X Value Handle=0x%X Len=%d ",
							charHandles[charHandleCount].startHandle,
							charHandles[charHandleCount].valHandle,
							GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data ),
							GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN( p_event_data ) );

				if( charHandleCount != 0 )
				{
					charHandles[charHandleCount-1].endHandle = charHandles[charHandleCount].endHandle - 1;
				}
				charHandleCount += 1;

				if( charHandleCount > MAX_CHARS_DISCOVERED-1 )
				{
					WICED_BT_TRACE( "This is really bad.. we discovered more characteristics than we can save\r\n" );
				}

				if( memcmp( ledUUID, GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128( p_event_data ), 16 ) == 0 ) // If it is the led Characteristic
				{
					ledChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle; // No macro for this unfortunately
					ledChar.valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
				}

				if( memcmp( counterUUID, GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128( p_event_data ),16 ) == 0 ) // If it is the button Characteristic
				{
					counterChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle; // No macro for this unfortunately
					counterChar.valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
				}

				for (int i=0; i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN( p_event_data ); i++ ) // Dump the bytes to the screen
				{
					WICED_BT_TRACE( "%02X ", GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128( p_event_data )[i] );
				}
				WICED_BT_TRACE( "\r\n" );
	        }

			//////////////// Descriptors Discovery /////////////////
			if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
			{
				WICED_BT_TRACE( "Char Descriptor Handle = %X Len=%d ", GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE( p_event_data ),
				GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN( p_event_data ) );

				for( int i=0; i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN( p_event_data ); i++ )
				{
					WICED_BT_TRACE( "%02X ", GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID128( p_event_data )[i] );
				}
				WICED_BT_TRACE( "\r\n" );

				if( GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID16( p_event_data ) == __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION )
				{
					counterChar.cccdHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE( p_event_data );
				}
			}
			break;

		case GATT_DISCOVERY_CPLT_EVT:
			// Once all characteristics are discovered... you need to setup the end handles
			if( p_event_data->discovery_complete.disc_type == GATT_DISCOVER_CHARACTERISTICS )
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
			WICED_BT_TRACE( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
			break;
	}

	return status;
}


/*******************************************************************************
* Function Name: scanCallback
********************************************************************************/
void scanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
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

		WICED_BT_TRACE("Found Device \"%s\" with BD Address: [%B]\n", dev_name, p_scan_result->remote_bd_addr);

		/* Connect to peripheral and stop scanning */
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
		wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + len-1);
		if ( p_write )
		{
			p_write->handle   = handle;
			p_write->offset   = offset;
			p_write->len      = len;
			p_write->auth_req = auth_req;
			memcpy(p_write->value, val, len);

			/* Send the write command to the Stack */
			wiced_bt_gatt_status_t status = wiced_bt_gatt_send_write ( conn_id, GATT_WRITE, p_write );

			WICED_BT_TRACE( "Send Write Status: 0x%X\r\n", status );

			wiced_bt_free_buffer( p_write );
		 }
	}
}


/*******************************************************************************
* Function Name: void startServiceDiscovery( void )
********************************************************************************/
void startServiceDiscovery( void )
{
    wiced_bt_gatt_discovery_param_t discovery_param;

    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 1;
    discovery_param.e_handle = 0xFFFF;
    discovery_param.uuid.len = 16;
    memcpy( &discovery_param.uuid.uu.uuid128, serviceUUID, 16 );

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover ( connection_id, GATT_DISCOVER_SERVICES_BY_UUID, &discovery_param );
    WICED_BT_TRACE( "Started Service Discovery 0x%X\r\n", status );
}


/*******************************************************************************
* Function Name: void startCharacteristicDiscovery( void )
********************************************************************************/
void startCharacteristicDiscovery()
{
    charHandleCount = 0;

    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = serviceStartHandle + 1;
    discovery_param.e_handle = serviceEndHandle;

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover( connection_id, GATT_DISCOVER_CHARACTERISTICS, &discovery_param );
    WICED_BT_TRACE( "Start char Discovery 0x%X\r\n", status );
}


/*******************************************************************************
* Function Name: void startDescriptorDiscovery( void )
********************************************************************************/
void startDescriptorDiscovery()
{
	WICED_BT_TRACE( "Counter Start Handle = %X End Handle=%X\r\n", counterChar.valHandle+1, counterChar.endHandle );

	wiced_bt_gatt_discovery_param_t discovery_param;
	memset( &discovery_param, 0, sizeof( discovery_param ) );
	discovery_param.s_handle = counterChar.valHandle+1;
	discovery_param.e_handle = counterChar.endHandle;

	wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover( connection_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &discovery_param );
	WICED_BT_TRACE( "Start char descriptor discovery 0x%X\r\n", status );
}


#if (UART_INPUT == WICED_TRUE)
/*******************************************************************************
* Function Name: void uart_rx_callback( void *data )
********************************************************************************/
void uart_rx_callback( void *data )
{
	uint8_t readbyte;

	/* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
	wiced_hal_puart_read( &readbyte );
	wiced_hal_puart_reset_puart_interrupt();

	switch( readbyte )
	{
		case 's':			/* Turn on scanning */
			wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, scanCallback);
			break;

		case 'S':			/* Turn off scanning */
			wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, scanCallback);
			break;

		case 'd':			/* Disconnect */
			wiced_bt_gatt_disconnect(connection_id);
			break;

		case '0':			/* LED off */
		case '1':			/* LED on */
			{
				uint8_t writeData[1];
				writeData[0] = readbyte-'0';
				writeAttribute(connection_id, ledChar.valHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint8_t), writeData);
			}
			break;

		case 'r':
			wiced_bt_util_send_gatt_read_by_handle(connection_id, ledChar.valHandle);
			break;

		case 'n': /* Turn on notifications */
			wiced_bt_util_set_gatt_client_config_descriptor(connection_id, counterChar.cccdHandle, GATT_CLIENT_CONFIG_NOTIFICATION);
			break;

		case 'N': /* Turn off notifications */
			wiced_bt_util_set_gatt_client_config_descriptor(connection_id, counterChar.cccdHandle, GATT_CLIENT_CONFIG_NONE);
			break;

		case 'q':
			startServiceDiscovery();
			break;

		case 'w':
			startCharacteristicDiscovery();
			break;

		case 'e':
			startDescriptorDiscovery();
			break;

		default:
			WICED_BT_TRACE( "Unrecognized command\r\n" );
			/* No break - fall through and display help */

		case '?':			/* Help */
			WICED_BT_TRACE( "Commands:\r\n" );
			WICED_BT_TRACE( "\t%c\tHelp (this message)\r\n", '?' );
			WICED_BT_TRACE( "\t%c\tStart scanning\r\n", 's' );
			WICED_BT_TRACE( "\t%c\tStop scanning\r\n", 'S' );
			WICED_BT_TRACE( "\t%c\tDisconnect\r\n", 'd' );
			WICED_BT_TRACE( "\t%c\tLED Off\r\n", '0' );
			WICED_BT_TRACE( "\t%c\tLED On\r\n", '1' );
			WICED_BT_TRACE( "\t%c\tGet LED state\r\n", 'r' );
			WICED_BT_TRACE( "\t%c\tTurn on Notifications\r\n", 'n' );
			WICED_BT_TRACE( "\t%c\tTurn off Notifications\r\n", 'N' );
			WICED_BT_TRACE( "\t%c\tStart Service Discovery\r\n", 'q' );
			WICED_BT_TRACE( "\t%c\tSttart Characteristic Discovery\r\n", 'w' );
			WICED_BT_TRACE( "\t%c\tStart Descriptor Discovery\r\n", 'e' );
			WICED_BT_TRACE( "\r\n" );
			break;
	}
}
#endif
