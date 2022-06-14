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
#include "wiced_hal_puart.h"
#include "wiced_hal_nvram.h"


/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
 
/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* PWM configuration defines */
#define PWM_FREQUENCY			(1000)
#define PWM_MAX					(0xFFFF)
#define PWM_INIT_BONDING		(PWM_MAX-999)
#define PWM_INIT_BONDED			(PWM_MAX-199)
/* PWM compare value for blinking the LED with a 50% duty cycle */
#define LED_TOGGLE_BONDING		(PWM_MAX-500)
#define LED_TOGGLE_BONDED		(PWM_MAX-100)


/* Bonding modes */
#define BONDING (0)
#define BONDED  (1)

/* NVRAM locations */
#define VSID_BONDINFO				(WICED_NVRAM_VSID_START)
#define VSID_LOCAL_IDENTITY_KEYS	(WICED_NVRAM_VSID_START+1)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t	app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

static wiced_bt_gatt_status_t	app_gatt_get_value( wiced_bt_gatt_read_t *p_data );
static wiced_bt_gatt_status_t	app_gatt_set_value( wiced_bt_gatt_write_t *p_data );

void							app_set_advertisement_data( void );

void 							button_cback( void *data, uint8_t port_pin );
void							rx_cback( void *data );

static void 					print_array(void * to_print, uint16_t len);


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id = 0;

uint8_t bond_mode = BONDING;		// State of the peripheral - bonded or bonding

/* Structure to store info that goes into NVRAM */
struct bondinfo
{
	uint8_t							cccd[2];
	wiced_bt_device_link_keys_t 	link_keys;
}  bondinfo;

wiced_bt_local_identity_keys_t 	identity_keys;

wiced_bt_device_address_t tempRemoteBDA;

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

    wiced_result_t nvram_status;

    WICED_BT_TRACE("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				/* Print out the local Bluetooth Device Address. The address type  is set in the makefile (BT_DEVICE_ADDRESS) */
				wiced_bt_device_address_t bda;
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

				/* Read data from nvram into the bondinfo structure in RAM */
				/* If no client has bonded previously, then the read will fail */
				memset( &bondinfo, 0, sizeof(bondinfo) );
				wiced_hal_read_nvram( VSID_BONDINFO, sizeof(bondinfo), (uint8_t*)&bondinfo, &nvram_status);
				if( nvram_status == WICED_BT_SUCCESS )
				{
					wiced_bt_dev_add_device_to_address_resolution_db ( &bondinfo.link_keys );
					WICED_BT_TRACE("\tBonding info found in NVRAM. Link key added to address resolution database\n" );
					bond_mode = BONDED; /* We have bonding information already, so don't go into bonding mode */
				}
				else
				{
					WICED_BT_TRACE("\tNo bonding info found in NVRAM\n" );
				}

				/* Print existing bonding info */
				WICED_BT_TRACE("Remote BDA: ");
				print_array(&bondinfo.link_keys.bd_addr, sizeof(bondinfo.link_keys.bd_addr));
				WICED_BT_TRACE("CCCD: ");
				print_array(&bondinfo.cccd, sizeof(bondinfo.cccd));
				WICED_BT_TRACE("Link Keys: ");
				print_array(&bondinfo.link_keys, sizeof(bondinfo.link_keys));

				/* Register GATT callback and initialize the GATT database */
				wiced_bt_gatt_register( app_gatt_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len );

				/* Enable/disable pairing */
				wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE );

				/* Configure the PWM and then disable it. */
				/* Routing of the PWM to the pin and starting ACLK will be done once advertising starts. */
				wiced_hal_pwm_start( PWM0, PMU_CLK, LED_TOGGLE_BONDING, PWM_INIT_BONDING, 0 );
				wiced_hal_pwm_disable(PWM0);

				/* Enable receive interrupts on the PUART */
				wiced_hal_puart_register_interrupt( rx_cback );
				/* Set watermark level to 1 to receive interrupt up on receiving each byte */
				wiced_hal_puart_set_watermark_level( 1 );
				wiced_hal_puart_enable_rx();

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

		case BTM_USER_CONFIRMATION_REQUEST_EVT:
			WICED_BT_TRACE("\r\n*************************\r\n" );
			WICED_BT_TRACE("* NUMERIC CODE = %06d *", p_event_data->user_confirmation_request.numeric_value );
			WICED_BT_TRACE("\r\n*************************\r\n\n" );
			WICED_BT_TRACE("* Press \"y\" if the numbers match, press \"n\" if they do not. *\n\r");

			/* Store the remote BDA so it can be used to send the response once the user presses 'y' or 'n'  */
			memcpy(&tempRemoteBDA, &(p_event_data->user_confirmation_request.bd_addr), sizeof(wiced_bt_device_address_t));

			status = WICED_BT_SUCCESS;
			break;

		case BTM_PASSKEY_NOTIFICATION_EVT:
			WICED_BT_TRACE("\r\n********************\r\n" );
			WICED_BT_TRACE("* PASSKEY = %06d *", p_event_data->user_passkey_notification.passkey );
			WICED_BT_TRACE("\r\n********************\r\n\n" );
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 						// Pairing Complete event
			if ( p_event_data->pairing_complete.pairing_complete_info.ble.reason == WICED_BT_SUCCESS ) /* Bonding successful */
			{
				bond_mode = BONDED; /* remember that the device is now bonded */
			}
			else
			{
				status = WICED_BT_ERROR;
			}
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
	        /* Connection has been encrypted to a device that was previously bonded meaning that at
	         * powerup, we already loaded the correct paired device info from NVRAM. So we can now
	         * restore the previously saved CCCD value for the device that just connected. */
	        if( bond_mode == BONDED)
	    	{
	    		/* Set CCCD value from the value that was previously saved in the NVRAM */
	    		app_mysvc_counter_client_char_config[0] = bondinfo.cccd[0];
	    		app_mysvc_counter_client_char_config[1] = bondinfo.cccd[1];
	    		WICED_BT_TRACE("Restored saved CCCD info\n");
	    	}
			break;

		case BTM_SECURITY_REQUEST_EVT: 						// Security access
	    	if( bond_mode == BONDING ) /* Only grant access if we are in bonding mode */
	    	{
	    		wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
	    	}
	    	else
	    	{
	    		WICED_BT_TRACE("Security Request Denied - not in bonding mode\n");
	    		status = WICED_BT_FAILED_ON_SECURITY;
	    	}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			memcpy(&(bondinfo.link_keys), &(p_event_data->paired_device_link_keys_update), sizeof(wiced_bt_device_link_keys_t));
			wiced_hal_write_nvram ( VSID_BONDINFO, sizeof(bondinfo), (uint8_t*)&bondinfo, &nvram_status );
			if(nvram_status == WICED_SUCCESS)
			{
				WICED_BT_TRACE( "\tBonding info saved to NVRAM %B result: %d \n\r", (uint8_t*)&(bondinfo.link_keys), nvram_status );
			}
			else
			{
				WICED_BT_TRACE("NVRAM Write Error: %d\n", nvram_status);
				status = WICED_BT_ERROR;
			}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 		// Retrieval saved link keys
			if( bond_mode == BONDED)
			{
				memcpy(&(p_event_data->paired_device_link_keys_request), &(bondinfo.link_keys), sizeof(wiced_bt_device_link_keys_t));
				wiced_bt_dev_add_device_to_address_resolution_db ( &bondinfo.link_keys );
				WICED_BT_TRACE("Link keys are available\n");
			}
			else /* keys not available, so we must return error to get the Stack to generate new keys */
			{
				status = WICED_BT_ERROR;
			}
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			memcpy(&(identity_keys), &(p_event_data->local_identity_keys_update), sizeof(wiced_bt_local_identity_keys_t));
			wiced_hal_write_nvram ( VSID_LOCAL_IDENTITY_KEYS, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)&identity_keys, &nvram_status );
			if(nvram_status == WICED_SUCCESS)
			{
				WICED_BT_TRACE( "\tLocal Identity Key saved to NVRAM \n\r");
			}
			else
			{
				WICED_BT_TRACE("NVRAM Write Error: %d\n", nvram_status);
				status = WICED_BT_ERROR;
			}
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 			// Read keys from NVRAM
			memset( &identity_keys, 0, sizeof(identity_keys) );
			wiced_hal_read_nvram( VSID_LOCAL_IDENTITY_KEYS, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)&identity_keys, &nvram_status);
			if( nvram_status == WICED_BT_SUCCESS )
			{
				WICED_BT_TRACE("\tIdentity keys found in NVRAM:\n" );
				print_array(&identity_keys, sizeof(identity_keys));

				/* Provide keys to the stack */
				memcpy(&(p_event_data->local_identity_keys_request), &identity_keys, sizeof(wiced_bt_local_identity_keys_t));

				/* Return success so that Stack will not generate new keys */
				status = WICED_BT_SUCCESS;
			}
			else
			{
				WICED_BT_TRACE("\tNo identity keys found in NVRAM\n" );
				/* Return error so that Stack will generate new keys */
				status = WICED_BT_ERROR;
			}
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
			WICED_BT_TRACE("Advertisement State Change: %s\n", get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
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
				if( bond_mode == BONDED )	/* Device is bonded - blink fast */
				{
					wiced_hal_gpio_select_function(LED1, WICED_PWM0);
					wiced_hal_aclk_enable( PWM_FREQUENCY, ACLK1, ACLK_FREQ_24_MHZ );
					wiced_hal_pwm_change_values( PWM0, LED_TOGGLE_BONDED, PWM_INIT_BONDED );
					wiced_hal_pwm_enable( PWM0 );
				}
				else 						/* Device is BONDING - blink slow */
				{
					wiced_hal_gpio_select_function(LED1, WICED_PWM0);
					wiced_hal_aclk_enable( PWM_FREQUENCY, ACLK1, ACLK_FREQ_24_MHZ );
					wiced_hal_pwm_change_values( PWM0, LED_TOGGLE_BONDING, PWM_INIT_BONDING );
					wiced_hal_pwm_enable( PWM0 );
				}
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
				WICED_BT_TRACE("GATT disconnect from: BDA %B, Connection ID '%d', Reason '%s'\n", p_conn->bd_addr, p_conn->conn_id, get_bt_gatt_disconn_reason_name(p_conn->reason) );

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
                		bondinfo.cccd[0] = p_val[0];
                		bondinfo.cccd[1] = p_val[1];

                		/* Save CCCD to NVRAM */
                		wiced_result_t temp_result;
                		wiced_hal_write_nvram( VSID_BONDINFO, sizeof(bondinfo), (uint8_t*)&(bondinfo), &temp_result );
                		WICED_BT_TRACE( "\tWrite CCCD value to NVRAM\n\r" );

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


/*******************************************************************************
* Function Name: void button_cback( void *data, uint8_t port_pin )
********************************************************************************/
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


/*******************************************************************************
* Function Name: void rx_cback( void *data )
********************************************************************************/
void rx_cback( void *data )
{
	uint8_t readbyte;
    wiced_result_t                  result;
    wiced_bt_device_link_keys_t     keys;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    /* Remove bonding info if the user sends 'e' */
    switch (readbyte)
    {
		case 'e':
			if(connection_id == 0) /* Only allow when device is not connected */
			{
				/* Put into bonding mode  */
				bond_mode = BONDING;

				/* Remove from the bonded device list */
				wiced_bt_dev_delete_bonded_device(bondinfo.link_keys.bd_addr);
				WICED_BT_TRACE( "Remove host %B from bonded device list \n\r", bondinfo.link_keys.bd_addr );

				/* Remove device from address resolution database */
				wiced_bt_dev_remove_device_from_address_resolution_db ( &bondinfo.link_keys );
				WICED_BT_TRACE( "Removed device from address resolution database\n");

				/* Clear bonding info structure */
				memset( &bondinfo, 0, sizeof(bondinfo) );

				/* Remove bonding information from NVRAM */
				wiced_hal_delete_nvram(VSID_BONDINFO, &result);

				if(wiced_bt_ble_get_current_advert_mode() == BTM_BLE_ADVERT_OFF)
				{
					wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
				}
				else
				{
					wiced_hal_pwm_change_values( PWM0, LED_TOGGLE_BONDING, PWM_INIT_BONDING );
				}
			}
			else
			{
				WICED_BT_TRACE( "Bonding information can only be deleted when disconnected\n");
			}
			break;

		case 'y': /* User has confirmed the numeric codes match */
			WICED_BT_TRACE( "Confirmation accepted\n");
			wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, tempRemoteBDA);
			break;

		case 'n': /* User has indicated the numeric codes do not match */
			WICED_BT_TRACE( "Confirmation rejected\n");
			wiced_bt_dev_confirm_req_reply( WICED_BT_ERROR, tempRemoteBDA);
			break;

		default:
			WICED_BT_TRACE( "Invalid Input\n");
			break;
	}
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
		   WICED_BT_TRACE( "\n" );
	   }
	   WICED_BT_TRACE( "%02X ", *((uint8_t *)(to_print + counter)) );
	}
	WICED_BT_TRACE( "\n" );

}
