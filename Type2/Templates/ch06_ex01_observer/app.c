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


/* UART input is not used for exercises 1 and 2 */
#define UART_INPUT WICED_FALSE

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_gatt_status_t	app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );

#if (UART_INPUT == WICED_TRUE)
void					uart_rx_callback( void *data );
#endif

/* TODO add declaration for scan callback function */


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id = 0;


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

				/* TODO start scanning */


			}
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
			}
			else
			{
				WICED_BT_TRACE("GATT operation failed with status: %d\n", p_event_data->operation_complete.status);
				status = WICED_BT_GATT_ERROR;
			}
			break;

		default:
			WICED_BT_TRACE( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
			break;
	}

	return status;
}


/* TODO add scanCallback function */


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
			break;

		case 'S':			/* Turn off scanning */
			break;

		case '0':			/* LED off */
		case '1':			/* LED on */
			break;

		default:
			WICED_BT_TRACE( "Unrecognized command\r\n" );
			/* No break - fall through and display help */

		case '?':			/* Help */
			WICED_BT_TRACE( "Commands:\r\n" );
			WICED_BT_TRACE( "\t%c\tHelp (this message)\r\n", '?' );
			WICED_BT_TRACE( "\t%c\tStart scanning\r\n", 's' );
			WICED_BT_TRACE( "\t%c\tStop scanning\r\n", 'S' );
			WICED_BT_TRACE( "\r\n" );
			break;
	}
}
#endif
