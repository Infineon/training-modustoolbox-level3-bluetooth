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
#include "wiced_timer.h"

/*TODO: Add the include for the beacon library */


/* Time in ms for the timer interrupt */
#define SLEEP_100MS		(100)

/* Eddystone parameters */
#define EDDY_HTTPS_WWW	0x01
#define EDDY_DOTCOM		0x07
#define EDDY_TX_PWR		0xF0

#define UID_RANGE		0xF0

/* Allocate the multi-advertising instance numbers */
#define ADV_INST_URL		1
#define ADV_INST_UID		2
#define ADV_INST_TLM		3


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void							app_multi_advertise( void );

void							timer_cback( uint32_t data );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
wiced_timer_t my_timer;								// 0.1s timer

uint8_t url_packet[WICED_BT_BEACON_ADV_DATA_MAX];	// URL advertising packet
uint8_t tlm_packet[WICED_BT_BEACON_ADV_DATA_MAX];	// TLM advertising packet
uint8_t uid_packet[WICED_BT_BEACON_ADV_DATA_MAX];	// UID advertising packet

/* Configure mulit-advertising parameters - these will be used for all 3 Eddystone packets (URL, UID, TLM) */
wiced_bt_ble_multi_adv_params_t adv_parameters =
{
	.adv_int_min = 			/* TODO */,
	.adv_int_max = 			/* TODO */,
	.adv_type = 			/* TODO */,
	.channel_map = 			/* TODO */,
	.adv_filter_policy =	/* TODO */,
	.adv_tx_power = 		/* TODO */,
    .peer_bd_addr =         /* TODO */,
    .peer_addr_type =       /* TODO */,
    .own_bd_addr =          /* TODO */,
    .own_addr_type =        /* TODO */
};

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
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				/* Print out the local Bluetooth Device Address. The address type  is set in the makefile (BT_DEVICE_ADDRESS) */
				wiced_bt_device_address_t bda;
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

				/* Disable pairing */
				wiced_bt_set_pairable_mode( WICED_FALSE, WICED_FALSE );

	        	/* Setup and start advertising */
				app_multi_advertise();

	        	/* Start 0.1s timer */
	        	wiced_init_timer( &my_timer, timer_cback, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER );
	        	wiced_start_timer( &my_timer, SLEEP_100MS );
			}
			break;

		default:
			break;
    }

    return status;
}


/*******************************************************************************
* Function Name: void app_multi_advertise( void )
********************************************************************************/
void app_multi_advertise( void )
{
	uint8_t packet_len;

    uint8_t url[] = {'i', 'n', 'f', 'i', 'n', 'e', 'o', 'n', EDDY_DOTCOM, 0x00}; /* Name for cypress.com with null termination for the string added */
	uint8_t uid_namespace[] = { 'k', 'e', 'y', 0, 0, 0, 0, 0, 0, 0 };
	uint8_t uid_instance[] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };

    /* Set up a URL packet with max power, and implicit "https://www." prefix */
    wiced_bt_eddystone_set_data_for_url( adv_parameters.adv_tx_power, EDDYSTONE_URL_SCHEME_1, url, url_packet, &packet_len );
    wiced_set_multi_advertisement_data( url_packet, packet_len, ADV_INST_URL);	/* Setup advertising packet for instance 1 */
    wiced_set_multi_advertisement_params( ADV_INST_URL, &adv_parameters );		/* Setup advertising parameters for instance 1 */
    wiced_start_multi_advertisements( MULTI_ADVERT_START, ADV_INST_URL );		/* Start advertising instance 1 */

	/* TODO: Set up a UID packet with ranging_data = 0xF0, and the uid_namespace and uid_instance values declared above */
 

    /* TODO: Set up a TLM packet with the number of seconds and battery voltage, temperature, advert count = 0 */
 
}



/*******************************************************************************
* Function Name: void timer_cback( uint32_t *data )
********************************************************************************/
void timer_cback( uint32_t data )
{
	static uint32_t tenths = 0;
	uint8_t packet_len;

	/* Increment the tenths */
	tenths++;

	/* TODO: Reuse two lines of code from app_multi_advertise to re-generate the packet and re-set the advertising data */

}
