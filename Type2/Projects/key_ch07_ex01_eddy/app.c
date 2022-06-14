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
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void							app_set_advertisement_data( void );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/


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

				/* Create the packet and begin advertising */
				app_set_advertisement_data();
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
			break;

		default:
			break;
    }

    return status;
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

