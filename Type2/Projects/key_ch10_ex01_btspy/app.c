#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_rtos.h"
#include "cycfg.h"
#include "cycfg_gatt_db.h"
#include "app_bt_utils.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"

/* WICED HCI API */
#include "hci_control_api.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
 
/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* PWM configuration defines */
#define PWM_FREQUENCY	(1000)
#define PWM_MAX			(0xFFFF)
#define PWM_INIT		(PWM_MAX-999)
/* PWM compare value for blinking the LED with a 50% duty cycle */
#define LED_TOGGLE		(PWM_MAX-500)

/* Buffers for WICED HCI */
#define TRANS_UART_BUFFER_SIZE  1024
#define TRANS_UART_BUFFER_COUNT 2


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t	app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

static wiced_bt_gatt_status_t	app_gatt_get_value( wiced_bt_gatt_read_t *p_data );
static wiced_bt_gatt_status_t	app_gatt_set_value( wiced_bt_gatt_write_t *p_data );

void							app_set_advertisement_data( void );

void 							button_cback( void *data, uint8_t port_pin );

void 							app_btspy_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Add global variables */
uint16_t connection_id = 0;

/* Buffer pool for WICED HCI */
static wiced_transport_buffer_pool_t* transport_pool = NULL;

/* Transport Configuration for WICED HCI */
const wiced_transport_cfg_t  transport_cfg =
{
    .type                = WICED_TRANSPORT_UART,	/**< Wiced transport type. */
    .cfg.uart_cfg        =
    {
	.mode = WICED_TRANSPORT_UART_HCI_MODE,	/**<  UART mode, HCI or Raw */
	.baud_rate = HCI_UART_DEFAULT_BAUD	/**<  UART baud rate */
    },
    .rx_buff_pool_cfg    =
    {
    	.buffer_size = 0,		/**<  Rx Buffer Size */
		.buffer_count = 0		/**<  Rx Buffer Count */
    },
    .p_status_handler    = NULL,	/**< Wiced transport status handler.*/
    .p_data_handler      = NULL,	/**< Wiced transport receive data handler. */
    .p_tx_complete_cback = NULL	/**< Wiced transport tx complete callback. */
};


/*******************************************************************************
* Function Name: void application_start( void )
********************************************************************************/
void application_start( void )
{
	/* Initialize the transport configuration */
	    wiced_transport_init( &transport_cfg );

	/* Initialize Transport Buffer Pool */
	transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE,
	 TRANS_UART_BUFFER_COUNT );

    #if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
        /* Select Debug UART setting to see debug traces on the appropriate port */
        wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
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

    WICED_BT_TRACE("Bluetooth Management Event: 0x%x %s\r\n", event, get_bt_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				/* WICED HCI initialization */
				wiced_bt_dev_register_hci_trace(app_btspy_callback);

				/* Print out the local Bluetooth Device Address. The address type  is set in the makefile (BT_DEVICE_ADDRESS) */
				wiced_bt_device_address_t bda;
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

				/* Register GATT callback and initialize the GATT database */
				wiced_bt_gatt_register( app_gatt_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len );

				/* Enable/disable pairing */
				wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE );

				/* Configure the PWM and then disable it. */
				/* Routing of the PWM to the pin and starting ACLK will be done once advertising starts. */
				wiced_hal_pwm_start( PWM0, PMU_CLK, LED_TOGGLE, PWM_INIT, 0 );
				wiced_hal_pwm_disable(PWM0);

				/* Configure the button to trigger an interrupt when pressed */
				wiced_hal_gpio_configure_pin( USER_BUTTON1,
					( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ),
					GPIO_PIN_OUTPUT_HIGH );
				wiced_hal_gpio_register_pin_for_interrupt(USER_BUTTON1, button_cback, 0);

 				/* Create the packet and begin advertising */
				app_set_advertisement_data();
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;

			break;

		case BTM_PAIRING_COMPLETE_EVT: 						// Pairing Complete event
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
			break;

		case BTM_SECURITY_REQUEST_EVT: 						// Security access
			wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 		// Retrieval saved link keys
			status = WICED_BT_ERROR;
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 			// Read keys from NVRAM
			status = WICED_BT_ERROR;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
			WICED_BT_TRACE("Advertisement State Change: %s\r\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
            /* Turn LED ON, OFF, or blinking based on advertising and connection status */
			if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF ) /* Advertising is off */
			{
				if(connection_id == 0) /* Disconnected */
				{
					/* Adv off and disconnected - stop PWM and turn OFF the LED */
					wiced_hal_pwm_disable(PWM0);
					wiced_hal_aclk_disable(ACLK1);
					wiced_hal_gpio_select_function(LED1, WICED_GPIO);
					wiced_hal_gpio_configure_pin(LED1, GPIO_OUTPUT_ENABLE , LED_STATE_OFF);
				}
				else /* Connected */
				{
					/* Adv off and connected - stop PWM and turn ON the LED */
					wiced_hal_pwm_disable(PWM0);
					wiced_hal_aclk_disable(ACLK1);
					wiced_hal_gpio_select_function(LED1, WICED_GPIO);
					wiced_hal_gpio_configure_pin(LED1, GPIO_OUTPUT_ENABLE, LED_STATE_ON);
				}
			}
			else /* Advertising is on - blink the LED */
			{
				/* Start the PWM to blink the LED */
				wiced_hal_gpio_select_function(LED1, WICED_PWM0);
				wiced_hal_aclk_enable( PWM_FREQUENCY, ACLK1, ACLK_FREQ_24_MHZ );
				wiced_hal_pwm_enable(PWM0);
			}
			break;

		default:
			break;
    }

    return status;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_gatt_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    wiced_bt_gatt_connection_status_t *p_conn = &p_data->connection_status;
    wiced_bt_gatt_attribute_request_t *p_attr = &p_data->attribute_request;

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:					// Remote device initiates connect/disconnect
            if( p_conn->connected )
			{
				WICED_BT_TRACE( "GATT connect to: BDA %B, Connection ID %d\r\n",p_conn->bd_addr, p_conn->conn_id );
				
				/* Handle the connection */
				connection_id = p_conn->conn_id;
			}
			else
			{
				// Device has disconnected
				WICED_BT_TRACE("GATT disconnect from: BDA %B, Connection ID '%d', Reason '%s'\r\n", p_conn->bd_addr, p_conn->conn_id, get_bt_gatt_disconn_reason_name(p_conn->reason) );

				/* Handle the disconnection */
				connection_id = 0;

				/* Restart the advertisements */
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:					// Remote device initiates a GATT read/write
			switch( p_attr->request_type )
			{
				case GATTS_REQ_TYPE_READ:
					result = app_gatt_get_value( &(p_attr->data.read_req) );
					break;

				case GATTS_REQ_TYPE_WRITE:
					result = app_gatt_set_value( &(p_attr->data.write_req) );
					break;
            }
            break;

        default:
            break;
    }

    return result;
}


/*******************************************************************************
* Function Name: app_gatt_get_value(
* 					wiced_bt_gatt_read_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t	app_gatt_get_value( wiced_bt_gatt_read_t *p_data )
{
	uint16_t attr_handle = 	p_data->handle;
	uint8_t  *p_val = 		p_data->p_val;
	uint16_t *p_len = 		p_data->p_val_len;
	uint16_t  offset =		p_data->offset;

	int i = 0;
	int len_to_copy;

    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
    	// Search for a matching handle in the external lookup table
    	if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Start by assuming we will copy entire value */
    		len_to_copy = app_gatt_db_ext_attr_tbl[i].cur_len;

    		/* Offset is beyond the end of the actual data length, nothing to do*/
    		if ( offset >= len_to_copy)
    		{
    			return WICED_BT_GATT_INVALID_OFFSET;
    		}

    		/* Only need to copy from offset to the end */
    		len_to_copy = len_to_copy - offset;

    		/* Determine if there is enough space to copy the entire value.
    		 * If not, only copy as much as will fit. */
            if (len_to_copy > *p_len)
            {
            	len_to_copy = *p_len;
            }

			/* Tell the stack how much will be copied to the buffer and then do the copy */
			*p_len = len_to_copy;
			memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data + offset, len_to_copy);
			res = WICED_BT_GATT_SUCCESS;

            // Add code for any action required when this attribute is read
            switch ( attr_handle )
            {
				case HDLC_MYSVC_LED_VALUE:
					WICED_BT_TRACE( "LED is %s\r\n", app_mysvc_led[0] ? "ON" : "OFF" );
					break;
            }
			break; /* break out of for loop once matching handle is found */
       }
    }
    return res;
}


/*******************************************************************************
* Function Name: app_gatt_set_value(
*					wiced_bt_gatt_write_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t	app_gatt_set_value( wiced_bt_gatt_write_t *p_data )
{
	uint16_t attr_handle = 	p_data->handle;
	uint8_t  *p_val = 		p_data->p_val;
	uint16_t len = 			p_data->val_len;

	int i = 0;
    wiced_bool_t validLen = WICED_FALSE;

    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry and find is max available size
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            // Verify that size constraints have been met
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;

                // Add code for any action required when this attribute is written
                // For example you may need to write the value into NVRAM if it needs to be persistent
                switch ( attr_handle )
                {
                	case HDLC_MYSVC_LED_VALUE:
						wiced_hal_gpio_set_pin_output(LED2, !(app_mysvc_led[0]) );
						WICED_BT_TRACE( "Turn the LED %s\r\n", app_mysvc_led[0] ? "ON" : "OFF" );
						break;
                	case HDLD_MYSVC_COUNTER_CLIENT_CHAR_CONFIG:
                		WICED_BT_TRACE( "Setting notify (0x%02x, 0x%02x)\r\n", p_val[0], p_val[1] );
                		break;
                }
            }
            else
            {
                // Value to write does not meet size constraints
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break; /* break out of for loop once matching handle is found */
        }
    }

    return res;
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data( void )
********************************************************************************/
void app_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[2] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0;

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof( uint8_t );
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = app_gap_device_name_len;
    adv_elem[num_elem].p_data = app_gap_device_name;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );
}


void button_cback( void *data, uint8_t port_pin )
{
    app_mysvc_counter[0]++;

    if( connection_id )
    {
	if( app_mysvc_counter_client_char_config[0]
	& GATT_CLIENT_CONFIG_NOTIFICATION )
		{
			WICED_BT_TRACE( "Sending counter change notification (%d)\r\n",
						app_mysvc_counter[0] );
    	    wiced_bt_gatt_send_notification(
						connection_id,
						HDLC_MYSVC_COUNTER_VALUE,
						app_mysvc_counter_len,
						app_mysvc_counter );
		}
    }

    /* Clear the GPIO interrupt */
    wiced_hal_gpio_clear_pin_interrupt_status( USER_BUTTON1 );
}

/* WICED HCI stack callback function */
void app_btspy_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace( transport_pool, type, length, p_data );
}
