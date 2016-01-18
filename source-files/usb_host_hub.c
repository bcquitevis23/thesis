/******************************************************************************

  USB Host Hub Device Driver

This is the Hub Class driver file for a USB Embedded Host device.
This file should be used in a project with usb_host.c to provided the USB
hardware interface.

To interface with usb_host.c, the routine USBHostHubClientInitialize() should be
specified as the Initialize() function, and USBHostHubClientEventHandler() should
be specified as the EventHandler() function in the usbClientDrvTable[] array
declared in usb_config.h.

Since hubs are performed with interrupt transfers, USB_SUPPORT_INTERRUPT_TRANSFERS
must be defined. 

This code is heavily based on Cypress's SH811 code for USB Hub Class. Major props 
to them for making the design of the code. Commands sent are based on USB 2.0
specs. Major props to Tsuneo from the Microchip Forums for bringing ideas on how
to make a hub class driver work.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

 File Name:       usb_host_hub.c
 Dependencies:    None
 Processor:       PIC24F/PIC32MX
 Compiler:        C30/C32
 Author:		  David Rigel Magsipoc
 Company:         USB Boys :D

Change History:
  Rev         Description
  ----------  ----------------------------------------------------------
  0.1		  September 30, 2010
			  First working version. Port polling for device attach and dettach 
			  working. Device enumeration is working already on the host.
  
*******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "USB\usb.h"
#include "usb_host_hub.h"
//#include "USB\usb_host_hub_port.h"

// *****************************************************************************
// *****************************************************************************
// Section: Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Max Number of Downstream Port Devices

This value represents the maximum number of attached devices this class driver
can support.  If the user does not define a value, it will be set to 1.
Currently this must be set to 1, due to limitations in the USB Host layer.
*/
#ifndef USB_MAX_DEVICES
	#define USB_MAX_DEVICES        5
#endif

// *****************************************************************************
/* Max Number of Hub Devices

This value represents the maximum number of attached devices this class driver
can support.  If the user does not define a value, it will be set to 1.
Currently this must be set to 1, due to limitations in the USB Host layer.
*/
#ifndef USB_MAX_HUB_DEVICES
	#define USB_MAX_HUB_DEVICES		1
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// Section: Interface and Protocol Constants
// *****************************************************************************

#define DEVICE_CLASS_HUB                							0x09    // Class code for Hubs.

#define STATUS_CHANGE_SIZE											0x02	// Size of the status change bits (1 + the number of ports).

#define DESCRIPTOR_STATUS_CHANGE									0x2900	// Descriptor type value of Status Change.

// These are definitions for different kinds of hubs

#define DEVICE_HUB_FULL_LOW_SPEED     								0x00    // Device protocol for full/low speed configured hub.
#define DEVICE_HUB_HIGH_SPEED_SINGLE_TT								0x01	// Device protocol for high speed single TT configured hub.
#define DEVICE_HUB_HIGH_SPEED_MULTIPLE_TT							0x02	// Device protocol for high speed multiple TT configured hub.

#define DEVICE_INTERFACE_SINGLE_TT									0x00	// Device interface protocol for single TT hub.
#define DEVICE_INTERFACE_MULTIPLE_TT								0x01	// Device interface protocol for multiple TT hub.
#define DEVICE_INTERFACE_MULTIPLE_TT_SECOND							0x02	// Device interface protocol for multiple TT hub used for the second interface descriptor.

// *****************************************************************************
// Section: State Machine Constants
// *****************************************************************************
#ifndef USB_ENABLE_TRANSFER_EVENT

	#define STATE_MASK												0x0F00		//
	#define SUBSTATE_MASK											0x00F0		//
	#define SUBSUBSTATE_MASK										0x000F		//

	#define NEXT_STATE												0x0100		//
	#define NEXT_SUBSTATE											0x0010		//
	#define NEXT_SUBSUBSTATE										0x0001		//
// *****************************************************************************
	#define STATE_DETACHED											0x0000		//
// *****************************************************************************
    #define STATE_INITIALIZE_DEVICE									0x0100		//
    #define SUBSTATE_WAIT_FOR_ENUMERATION       					0x0000		//
    #define STATE_DETACHED											0x0000		//
	#define SUBSTATE_DEVICE_ENUMERATED								0x0010		//
// *****************************************************************************
	#define STATE_GET_HUB_DESCRIPTORS								0x0200		//

	#define SUBSTATE_GET_STATUS_CHANGE_EP							0x0000		//
	#define SUBSUBSTATE_SEND_GET_STATUS_CHANGE_EP					0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_GET_STATUS_CHANGE_EP				0x0001		//
	#define SUBSUBSTATE_GET_STATUS_CHANGE_EP_COMPLETE				0x0002		//

	#define SUBSTATE_GET_STATUS										0x0010		//
	#define SUBSUBSTATE_SEND_GET_STATUS								0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_GET_STATUS							0x0001		//
	#define SUBSUBSTATE_GET_STATUS_COMPLETE							0x0002		//

	#define SUBSTATE_CLEAR_PORT_FEATURE								0x0020		//
	#define SUBSUBSTATE_SEND_CLEAR_PORT_FEATURE						0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_SEND_CLEAR_PORT_FEATURE			0x0001		//
	#define SUBSUBSTATE_SEND_CLEAR_PORT_FEATURE_COMPLETE			0x0002		//

	#define SUBSTATE_SET_PORT_FEATURE								0x0030		//
	#define SUBSUBSTATE_SEND_SET_PORT_FEATURE						0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_SEND_SET_PORT_FEATURE				0x0001		//
	#define SUBSUBSTATE_SEND_SET_PORT_FEATURE_COMPLETE				0x0002		//

	#define SUBSTATE_GET_PORT_STATUS								0x0040		//
	#define SUBSUBSTATE_SEND_GET_PORT_STATUS						0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_SEND_GET_PORT_STATUS				0x0001		//
	#define SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE				0x0002		//

	#define SUBSTATE_CLEAR_PORT_PRESENT_STATUS						0x0050		//
// *****************************************************************************
	#define STATE_RUNNING											0x0300		//

	#define SUBSTATE_PORT_TASK										0x0000		//
	#define SUBSTATE_WAITING_FOR_HUBCHANGE_EP						0x0010		//
	#define SUBSTATE_READ_HUBCHANGE_EP								0x0020		//
	#define SUBSTATE_READ_HUBCHANGE_EP_DONE							0x0030		//
	#define SUBSTATE_HOLDING										0x0040		//								
// *****************************************************************************
	#define STATE_HOLDING											0x0600		//
// *****************************************************************************


// *****************************************************************************
// Section: Downstream Port State Machine Constants
// *****************************************************************************

// *****************************************************************************
	#define STATE_GET_PORT_NUMBER									0x0000		//
// *****************************************************************************
	#define STATE_GET_INITIAL_STATUS								0x0100		//

	#define SUBSTATE_SEND_GET_STATUS								0x0000		//
	#define	SUBSTATE_WAIT_FOR_SEND_GET_STATUS						0x0010		//
	#define SUBSTATE_SEND_GET_STATUS_COMPLETE						0x0020		//
// *****************************************************************************
	#define STATE_DEVICE_PRESENT									0x0200		//

	#define SUBSTATE_SEARCH_UNUSED_USB_ADDRESS						0x0000		//

	#define SUBSTATE_CLEAR_C_PORT_CONNECTION						0x0010		//
	#define SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION				0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_CLEAR_C_PORT_CONNECTION			0x0001		//
	#define SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION_COMPLETE		0x0002		//

	#define SUBSTATE_CLEAR_FEAT_C_PORT_GET_STATUS					0x0020		//
	#define SUBSUBSTATE_SEND_CLEAR_FEAT_C_PORT_GET_STATUS			0x0000		//	
	#define SUBSUBSTATE_WAIT_FOR_CLEAR_FEAT_C_PORT_GET_STATUS		0x0001		//
	#define SUBSUBSTATE_SEND_CLEAR_FEAT_C_PORT_GET_STATUS_COMPLETE	0x0002		//

	#define SUBSTATE_SET_FEATURE_PORT_RESET							0x0030		//
	#define SUBSUBSTATE_SEND_SET_FEATURE_PORT_RESET					0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_SET_FEATURE_PORT_RESET				0x0001		//	
	#define SUBSUBSTATE_SEND_SET_FEATURE_PORT_RESET_COMPLETE		0x0002		//

	#define SUBSTATE_PORT_RESET_GET_STATUS							0x0040		//
	#define SUBSUBSTATE_SEND_PORT_RESET_GET_STATUS					0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_PORT_RESET_GET_STATUS				0x0001		//
	#define SUBSUBSTATE_SEND_PORT_RESET_GET_STATUS_COMPLETE			0x0002		//

	#define SUBSTATE_CLEAR_FEATURE_C_PORT_RESET						0x0050		//
	#define SUBSUBSTATE_SEND_CLEAR_FEATURE_C_PORT_RESET				0x0000		//	
	#define SUBSUBSTATE_WAIT_FOR_CLEAR_FEATURE_C_PORT_RESET			0x0001		//
	#define SUBSUBSTATE_SEND_CLEAR_FEATURE_C_PORT_RESET_COMPLETE	0x0002		//

	#define SUBSTATE_C_PORT_RESET_GET_STATUS						0x0060		//
	#define SUBSUBSTATE_SEND_C_PORT_RESET_GET_STATUS				0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_C_PORT_RESET_GET_STATUS			0x0001		//
	#define SUBSUBSTATE_SEND_C_PORT_RESET_GET_STATUS_COMPLETE		0x0002		//

	#define SUBSTATE_DETERMINE_DOWNSTREAM_SPEED						0x0070		//

	#define SUBSTATE_ENUMERATE_CURRENT_ATTACHED_DEVICE				0x0080		//
// *****************************************************************************
	#define STATE_DEVICE_NOT_PRESENT								0x0300		//

	#define SUBSTATE_CLEAR_C_PORT_CONNECTION_NP						0x0000		//
	#define SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION_NP				0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_CLEAR_C_PORT_CONNECTION_NP			0x0001		//
	#define SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION_COMPLETE_NP	0x0002		//
	#define SUBSUBSTATE_SEND_GET_PORT_STATUS_NP						0x0003		//
	#define SUBSUBSTATE_WAIT_FOR_GET_PORT_STATUS_NP					0x0004		//
	#define SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE_NP			0x0005		//

	#define SUBSTATE_CLEAR_C_PORT_ENABLE							0x0010		//
	#define SUBSUBSTATE_SEND_CLEAR_C_PORT_ENABLE					0x0000		//
	#define SUBSUBSTATE_WAIT_FOR_CLEAR_C_PORT_ENABLE				0x0001		//
	#define SUBSUBSTATE_SEND_CLEAR_C_PORT_ENABLE_2					0x0002		//
	#define SUBSUBSTATE_SEND_GET_PORT_STATUS_NP2					0x0003		//
	#define SUBSUBSTATE_WAIT_FOR_GET_PORT_STATUS_NP2				0x0004		//
	#define SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE_NP2			0x0005		//

	#define SUBSTATE_SEARCH_DETACHED_PORT							0x0020		//

#endif

// *****************************************************************************
// Section: Other Constants
// *****************************************************************************

#define	HUB_ADDR													0x01		// fixed hub address

#define USB_HUB_RESET												0xFF		// Device Request code to reset the device.
#define MARK_RESET_RECOVERY    										(0x0E)  	// Maintain with USB_HUB_DEVICE_INFO

/////////////MODIFICATION (1/7/2016)
//Feature Selector Values for Hub Class Specific Requests

//Set Hub Feature and Clear Hub Feature
#define C_HUB_LOCAL_POWER       0   //1 = There is a change in hub's local power
#define C_HUB_OVER_CURRENT      1   //1 = There is a change in hub's over-current status bit
//Set Port Feature and Clear Port Feature
//Port Status Bits
#define PORT_CONNECTION         0   //1 = there is a device connected to the port
#define PORT_ENABLE             1   //1 = port is enabled
#define PORT_SUSPEND            2   //1 = port is suspended
#define PORT_OVER_CURRENT       3   //1 = port is in over-current condition
#define PORT_RESET              4   //1 = when host wishes to reset the device
#define PORT_POWER              8   //1 = port is powered on
#define PORT_LOW_SPEED          9   //1 = low-speed device is attached
//Port Status Change Bits - 1 = there is a change in the port status bit
#define C_PORT_CONNECTION       16  //
#define C_PORT_ENABLE           17
#define C_PORT_SUSPEND          18
#define C_PORT_OVER_CURRENT     19
#define C_PORT_RESET            20
#define C_PORT_TEST             21
#define C_PORT_INDICATOR        22

//Hub Port Status Fields Bits
#define PORT_CONNECTION_BIT     (0b0000000000000001)
#define PORT_ENABLE_BIT         (0b0000000000000010)
#define PORT_SUSPEND_BIT        (0b0000000000000100)
#define PORT_OVER_CURRENT_BIT   (0b0000000000001000)
#define PORT_RESET_BIT          (0b0000000000010000)
#define PORT_POWER_BIT          (0b0000000100000000)
#define PORT_LOW_SPEED_BIT      (0b0000001000000000)
#define PORT_HIGH_SPEED_BIT     (0b0000010000000000)
#define PORT_TEST_BIT           (0b0000100000000000)
#define PORT_INDICATOR_BIT      (0b0001000000000000)

//Hub Port Change Fields Bits
#define C_PORT_CONNECTION_BIT   (0b0000000000000001)
#define C_PORT_ENABLE_BIT       (0b0000000000000010)
#define C_PORT_SUSPEND_BIT      (0b0000000000000100)
#define C_PORT_OVER_CURRENT_BIT (0b0000000000001000)
#define C_PORT_RESET_BIT        (0b0000000000010000)
/////////////////END OF MODIFICATION

//******************************************************************************
//******************************************************************************
// Section: Data Structures
//******************************************************************************
//******************************************************************************

// *****************************************************************************
/* USB Hub Device Information

   This structure is used to hold information of all the interfaces in a device that is unique
*/

typedef struct _USB_HUB_INTERFACE_DETAILS
{
	struct _USB_HUB_INTERFACE_DETAILS		*next;							// Pointer to the next interface
	WORD					sizeofStatusChangeDesc;							// Size of Status Change descriptor of a particular interface.
	WORD					endpointMaxDataSize;							// Max packet size.
	BYTE					endpointIN;										// Status Change endpoint.
	BYTE					interfaceNumber;								// Interface number, 2 if multiple transaction translator.
	BYTE					endpointPollInterval;							// interval for polling.
} USB_HUB_INTERFACE_DETAILS;

/* USB Hub Device Information

This structure is used to hold all the information about an attached hub device.
*/

typedef struct _USB_HUB_DEVICE_INFO
{
	BYTE				bNumberPort;							// Number of downstream ports in a the hub.
	BYTE				deviceAddress;          				// Address of the device on the bus.
    BYTE				clientDriverID;							// Client driver ID for device requests.
	BYTE				errorCode;             					// Error code of last error.
	WORD				state;                  				// State machine state of the device.
	WORD				portstate;								// State machine state of the downstream ports. 
   	WORD				returnState;        				    // State to return to after performing error handling.
    	union
    	{
        	struct
        	{
            		BYTE         	bfDirection     : 1;    	// Direction of current transfer (0=OUT, 1=IN).
            		BYTE        	bfReset         : 1;    	// Flag indicating to perform Hub Reset.
            		BYTE        	bfClearDataIN   : 1;    	// Flag indicating to clear the IN endpoint.
            		BYTE            bfClearDataOUT  : 1;    	// Flag indicating to clear the OUT endpoint.
        	};
        	BYTE                   	val;
    	}                         	flags;
	BYTE*				Status;									// For Status data buffer
	BYTE*				StatusChange;   						// Pointer to Status Change Endpoint.
	BYTE*				HubDevice;								// Pointer for Port Speed and attached devices.
	BYTE				Speed;									// Speed compatibility of the hub.
	BYTE				TTArchitecture;							// Transaction Translator architecture of the hub.
	BYTE				MaxPower;								// The maximum amount of bus power the hub will consume.
	BYTE				endpointIN;								// Hubchange endpoint of the hub.
	BYTE				endpointDATA;           				// Endpoint to use for the current transfer.
	DWORD				bytesTransferred;						// Number of bytes transferred to/from the user's data buffer.
	BYTE				MaxPacketSize;							//
	BYTE				bInterval;								//
	BYTE				noOfinterfaces;							// Total number of interface in the device.
	BYTE				port;									// Current Port that is configured
	BYTE				interface;								// Interface number of the current transfer.
} USB_HUB_DEVICE_INFO;

/* USB Hub Device Information

This structure is used to hold the port status and port change bits returned by a hub device.
*/

typedef struct _USB_PORT_STATUS
{
	BYTE				wPortStatus_Lo;
	BYTE				wPortStatus_Hi;
	BYTE				wPortChange_Lo;
	BYTE				wPortChange_Hi;
} USB_PORT_STATUS;

/* USB Hub Device Information

This structure holds the data regarding the attached devices on a hub.
*/

typedef struct _USB_HUB_DEVICE										// USB Address #0(enum), #1(Hub), #2..#5(device behind hub)
{																	// [X] = device address, valid #2..#5 only, #0 is used during enum
	BYTE				bPortPresent[USB_MAX_DEVICES];				// '1' present, '0' absent
	BYTE				bPortNumber[USB_MAX_DEVICES];				// contain port number at which device is attached 
	BYTE				bPortSpeed[USB_MAX_DEVICES]; 				// '1' LowSpeed, '0' FullSpeed
} USB_HUB_DEVICE;


//******************************************************************************
//******************************************************************************
// Section: Local Prototypes
//******************************************************************************
//******************************************************************************

void _USBHostHub_ResetStateJump( BYTE i );

//******************************************************************************
//******************************************************************************
// Section: Macros
//******************************************************************************
//******************************************************************************

#define _USBHostHub_LockDevice(x)			{                                               			\
                                                        deviceInfoHub[i].errorCode  = x;                \
                                                        deviceInfoHub[i].state      = STATE_HOLDING;    \
						  					}

#ifndef USB_ENABLE_TRANSFER_EVENT
	#define _USBHostHub_SetNextState()                	{ deviceInfoHub[i].state = (deviceInfoHub[i].state & STATE_MASK) + NEXT_STATE; }
	#define _USBHostHub_SetNextSubState()             	{ deviceInfoHub[i].state = (deviceInfoHub[i].state & (STATE_MASK | SUBSTATE_MASK)) + NEXT_SUBSTATE; }
	#define _USBHostHub_SetNextSubSubState()          	{ deviceInfoHub[i].state += NEXT_SUBSUBSTATE; }
	#define _USB_HostHubPort_SetNextState()           	{ deviceInfoHub[i].portstate = (deviceInfoHub[i].portstate & STATE_MASK) + NEXT_STATE; }
	#define _USB_HostHubPort_SetNextSubState()        	{ deviceInfoHub[i].portstate = (deviceInfoHub[i].portstate & (STATE_MASK | SUBSTATE_MASK)) + NEXT_SUBSTATE; }
	#define _USB_HostHubPort_SetNextSubSubState()     	{ deviceInfoHub[i].portstate += NEXT_SUBSUBSTATE; }
	#define _USBHostHub_SetStartingSubSubState()		{ deviceInfoHub[i].state = (deviceInfoHub[i].state & (STATE_MASK | SUBSTATE_MASK)); }
	#define _USB_HostHubPort_SetStartingSubSubState()	{ deviceInfoHub[i].portstate = (deviceInfoHub[i].portstate & (STATE_MASK | SUBSTATE_MASK)); }
	#define _USBHostHubPort_SetDevicePresentState()   	{ deviceInfoHub[i].portstate	= STATE_DEVICE_PRESENT; }
	#define _USBHostHubPort_SetDeviceNotPresentState()  { deviceInfoHub[i].portstate	= STATE_DEVICE_NOT_PRESENT; }

	#define _USBHostHub_TerminateReadTransfer( error )  {																							\
                                                            deviceInfoHub[i].errorCode    = error;													\
                                                            deviceInfoHub[i].state        = STATE_RUNNING | SUBSTATE_WAITING_FOR_HUBCHANGE_EP;		\
                                                        }
	#define _USBHostHub_TerminateTransfer( error )		{																							\
															deviceInfoHub[i].errorCode  = error;													\
															deviceInfoHub[i].state      = STATE_RUNNING;											\
														}

	#define _USB_SetNextState()                			{ state = (state & STATE_MASK) + NEXT_STATE; }
	#define _USB_SetNextSubState()             			{ state = (state & (STATE_MASK | SUBSTATE_MASK)) + NEXT_SUBSTATE; }
	#define _USB_SetNextSubSubState()          			{ state += NEXT_SUBSUBSTATE; }

#else
  #ifdef USB_HUB_ENABLE_TRANSFER_EVENT
    #define _USBHostHub_TerminateTransfer( error )  	{																			\
                                                    	    deviceInfoHub[i].errorCode  = error;									\
                                                    	    deviceInfoHub[i].state      = STATE_RUNNING;							\
 														}
  #else
    #define _USBHostHub_TerminateTransfer( error )  	{																			\
															deviceInfoHub[i].errorCode  = error;									\
															deviceInfoHub[i].state      = STATE_RUNNING;							\
														}
  #endif
#endif


//******************************************************************************
//******************************************************************************
// Section: Hub Host Global Variables
//******************************************************************************
//******************************************************************************

static USB_HUB_INTERFACE_DETAILS*	pInterfaceDetails 		= NULL;
static USB_HUB_INTERFACE_DETAILS*	pCurrInterfaceDetails 	= NULL;
static USB_HUB_DEVICE_INFO         	deviceInfoHub[USB_MAX_HUB_DEVICES] __attribute__ ((aligned));
BOOL								StatChangeIndicator = 0;				// Indicator whenever there is an event in the hub.
BYTE								CurrentPort;
BYTE								DevAddr;								// Address for newly attached devices.
BYTE								*pHubDevice			= NULL;				// Pointer to hub downstream devices.
BYTE								*pStat				= NULL;				// HUB ports status.
WORD								state;


// *****************************************************************************
// *****************************************************************************
// Application Callable Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    BOOL USBHostHubDeviceDetect( BYTE deviceAddress )

  Description:
    This function determines if a Hub device is attached and ready to use.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress  - Address of the attached device.

  Return Values:
    TRUE   -  Hub present and ready
    FALSE  -  Hub not present or not ready

  Remarks:
    This function replaces the USBHostHub_ApiDeviceDetect() function.
*******************************************************************************/
BOOL USBHostHubDeviceDetect( BYTE deviceAddress )
{
    BYTE    i;
 
	DBPRINTF("USBHostHubDeviceDetect (BYTE deviceAddress = %x)\n", deviceAddress);   
    // Find the correct device.
    for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].deviceAddress != deviceAddress); i++);
    if (i == USB_MAX_HUB_DEVICES)
    {
        return FALSE;
    }
    if ((USBHostHubDeviceStatus(i) == USB_HUB_NORMAL_RUNNING) &&
        (deviceAddress == 0))
    {
        return TRUE;
    }
    
    return FALSE;
}

/*******************************************************************************
  Function:
    BYTE    USBHostHubDeviceStatus( BYTE deviceAddress )

  Summary:

  Description:
    This function determines the status of a hub device.

  Preconditions:  None

  Parameters:
    BYTE deviceAddress - address of device to query

  Return Values:
    USB_HUB_DEVICE_NOT_FOUND           -  Illegal device address, or the
                                          device is not an Hub
    USB_HUB_INITIALIZING               -  Hub is attached and in the
                                          process of initializing
    USB_PROCESSING_REPORT_DESCRIPTOR   -  Hub device is detected and status
										  endpoint is being read.
    USB_HUB_NORMAL_RUNNING             -  Hub Device is running normal,
                                          ready to send and receive reports
    USB_HUB_DEVICE_HOLDING             -
    USB_HUB_DEVICE_DETACHED            -  Hub detached.

  Remarks:
    None
*******************************************************************************/
BYTE USBHostHubDeviceStatus( BYTE deviceAddress )
{
    BYTE    i;
    BYTE    status;

	DBPRINTF("USBHostHubDeviceStatus(BYTE deviceAddress = %x)\n", deviceAddress);

    // Find the correct device.
    for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].deviceAddress != deviceAddress); i++);
    if (i == USB_MAX_HUB_DEVICES)
    {
        return USB_HUB_DEVICE_NOT_FOUND;
    }

    status = USBHostDeviceStatus( i );
	DBPRINTF("Hub Status = %x\n", status);
    if (status != USB_DEVICE_ATTACHED)
    {
        return status;
    }
    else
    {
		// The device is attached and done enumerating.  We can get more specific now.
		#ifndef USB_ENABLE_TRANSFER_EVENT
			switch (deviceInfoHub[i].state & STATE_MASK)
			{
				case STATE_INITIALIZE_DEVICE:
					return USB_HUB_INITIALIZING;
					break;
				case STATE_GET_HUB_DESCRIPTORS:
					return USB_PROCESSING_HUB_DESCRIPTORS;
					break;
				case STATE_RUNNING:
					return USB_HUB_NORMAL_RUNNING;
					break;
				case STATE_HOLDING:
					return USB_HUB_DEVICE_HOLDING;
					break;
	
				default:
					return USB_HUB_DEVICE_DETACHED;
					break;
			}
		#endif
    }
}

/*******************************************************************************
  Function:
    BYTE USBHostHubResetDevice( BYTE deviceAddress )

  Summary:
    This function starts a Hub reset.

  Description:
    This function starts a Hub reset.  A reset can be
    issued only if the device is attached and not being initialized.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress - Device address

  Return Values:
    USB_SUCCESS                 - Reset started
    USB_HUB_DEVICE_NOT_FOUND    - No device with specified address
    USB_HUB_ILLEGAL_REQUEST     - Device is in an illegal state for reset

  Remarks:
    None
*******************************************************************************/
BYTE USBHostHubResetDevice( BYTE deviceAddress )
{
    BYTE    i;

    // Find the correct device.
    for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].deviceAddress != deviceAddress); i++);
    if (i == USB_MAX_HUB_DEVICES)
    {
        return USB_HUB_DEVICE_NOT_FOUND;
    }

    #ifndef USB_ENABLE_TRANSFER_EVENT
    if (((deviceInfoHub[i].state & STATE_MASK) != STATE_DETACHED) &&
        ((deviceInfoHub[i].state & STATE_MASK) != STATE_INITIALIZE_DEVICE))
    #else
    if ((deviceInfoHub[i].state != STATE_DETACHED) &&
        (deviceInfoHub[i].state != STATE_INITIALIZE_DEVICE))
    #endif
    {
        deviceInfoHub[i].flags.val |= MARK_RESET_RECOVERY;
        deviceInfoHub[i].flags.bfReset = 1;
        #ifndef USB_ENABLE_TRANSFER_EVENT
            deviceInfoHub[i].returnState = STATE_RUNNING | SUBSTATE_WAITING_FOR_HUBCHANGE_EP;
        #else
            deviceInfoHub[i].returnState = STATE_RUNNING;
        #endif

        _USBHostHub_ResetStateJump( i );
        return USB_SUCCESS;
    }
    
    return USB_HUB_ILLEGAL_REQUEST;
}



/****************************************************************************
  Function:
    BOOL USBHostHubInitialize( BYTE address, DWORD flags, BYTE clientDriverID )

  Summary:
    This function is the initialization routine for this client driver.

  Description:
    This function is the initialization routine for this client driver.  It
    is called by the host layer when the USB device is being enumerated.  For
    a hub device, we need to make sure that we have room for a new
    device, and that the device has at least one endpoint called status change.

  Precondition:
    None

  Parameters:
    BYTE address        - Address of the new device
    DWORD flags			- Initialization flags
    BYTE clientDriverID - ID to send when issuing a Device Request via
                            USBHostIssueDeviceRequest(), USBHostSetDeviceConfiguration(),
                            or USBHostSetDeviceInterface().

  Return Values:
    TRUE   - We can support the device.
    FALSE  - We cannot support the device.

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostHubInitialize( BYTE address, DWORD flags, BYTE clientDriverID )
{
	BYTE   				*descriptor;
	BYTE    			device;
    BYTE    			i;
	BYTE				endpointIN;
	BYTE				bDeviceProtocol;	// Speed compatibility and transaction translator design (single or multiple).
	BYTE				bInterfaceProtocol;	// Determines the Transaction Translator architecture (single or multiple).	
	BYTE 				numofinterfaces		= 0;	
	USB_HUB_INTERFACE_DETAILS	*pNewInterfaceDetails	= NULL;


//	DBPRINTF( "\nHub: USBHubClientInitialize(0" );

	// Find the free slot in the table.  If we cannot find one, kick off the device.
	for (device = 0; (device < USB_MAX_HUB_DEVICES) && (deviceInfoHub[device].deviceAddress != 0); device++);
	if (device == USB_MAX_HUB_DEVICES)
	{
		// Kick off the device
		return FALSE;
    }

	descriptor = USBHostGetDeviceDescriptor( address );
	descriptor += 5;

	// Save the value of bDeviceProtocol to determine the speed and Transaction Translator architecture
	bDeviceProtocol = *descriptor;
	descriptor = USBHostGetCurrentConfigurationDescriptor( address );					// edit mo yung usbhostgetcurrentconfiguration descriptor para maka support ng maraming devices
	i = 0;

	// Find the next interface descriptor.
	while (i < ((USB_CONFIGURATION_DESCRIPTOR *)descriptor)->wTotalLength)
	{
		// See if we are pointing to an interface descriptor.
		if (descriptor[i+1] == USB_DESCRIPTOR_INTERFACE)
		{
			// See if the interface is a Hub interface
			if (descriptor[i+5] == DEVICE_CLASS_HUB)
			{
//				DBPRINTF("\ncheck size %x", sizeof(USB_HUB_INTERFACE_DETAILS) );
				if ((pNewInterfaceDetails = (USB_HUB_INTERFACE_DETAILS*)malloc(sizeof(USB_HUB_INTERFACE_DETAILS))) == NULL)
				{
					// Hub: Out of memory for interface details
					return FALSE;
				}
				numofinterfaces++ ;

				// Create new entry into interface list
				if(pInterfaceDetails == NULL)
				{
					pInterfaceDetails = pNewInterfaceDetails;
					pCurrInterfaceDetails = pNewInterfaceDetails;
					pInterfaceDetails->next = NULL;
				}
				// Interface Descriptors will only be two when it supports multiple transaction translator
				else
				{
					pCurrInterfaceDetails->next             = pNewInterfaceDetails;
					pCurrInterfaceDetails                   = pNewInterfaceDetails;
					pCurrInterfaceDetails->next             = NULL;
				}
				// Hard coded to 4 usb ports
				deviceInfoHub[device].bNumberPort = 4;
				deviceInfoHub[i].port=1;											// Initialize port to the first port
				pCurrInterfaceDetails->interfaceNumber = descriptor[i+2];
				// Get the value of the bInterfaceProtocol
				bInterfaceProtocol = descriptor[i+7];
				// Check the speed compatibility of the hub
				if ( (bDeviceProtocol  == DEVICE_HUB_FULL_LOW_SPEED) && 			// If it is a full/low Speed Hub.
				   (bInterfaceProtocol == DEVICE_INTERFACE_SINGLE_TT) )				// If it has a single transaction translator.
				{
					deviceInfoHub[device].Speed = DEVICE_HUB_FULL_LOW_SPEED;
					deviceInfoHub[device].TTArchitecture = DEVICE_INTERFACE_SINGLE_TT;
				}
				else if ( (bDeviceProtocol == DEVICE_HUB_HIGH_SPEED_SINGLE_TT) &&		// If it is a high Speed Hub.
						  (bInterfaceProtocol == DEVICE_INTERFACE_SINGLE_TT) )			// If it has a single transaction translator.
				{
					deviceInfoHub[device].Speed = DEVICE_HUB_HIGH_SPEED_SINGLE_TT;
					deviceInfoHub[device].TTArchitecture = DEVICE_INTERFACE_SINGLE_TT;
				}
// Multiple TT Hubs are not yet supported
//				else if ( (bDeviceProtocol == DEVICE_HUB_HIGH_SPEED_MULTIPLE_TT) &&		// If it is a high Speed Hub.
//					( (bInterfaceProtocol == DEVICE_INTERFACE_MULTIPLE_TT) ||
//					  (bInterfaceProtocol == DEVICE_INTERFACE_MULTIPLE_TT_SECOND) ) 	// If it has a multiple transaction translator architecture.
//				{
//					deviceInfoHub[device].Speed = DEVICE_HUB_HIGH_SPEED_MULTIPLE_TT;
//					deviceInfoHub[device].TTArchitecture = DEVICE_INTERFACE_MULTIPLE_TT;
//					// find next interface descriptor for the next interface descriptor
//				}	// Doesn't support multiple TT architecture yet
				else
				{
					//errorCode = // insert error here, incorrect protocol values
					return FALSE;
				}
				// Look for bulk IN endpoints. (Status Change endpoint)
				endpointIN  = 0;
				// Scan for endpoint descriptors.
				i += descriptor[i];
				pCurrInterfaceDetails->sizeofStatusChangeDesc = descriptor[i];
				while (descriptor[i+1] == USB_DESCRIPTOR_ENDPOINT)
				{
					if (descriptor[i+3] == 0x03) // Interrupt
					{
						if (((descriptor[i+2] & 0x80) == 0x80) && (endpointIN == 0))
						{
							endpointIN = descriptor[i+2];
						}
						i += descriptor[i];
					}
					if ( endpointIN != 0 )
					{
						// Initialize the device information.
						deviceInfoHub[device].deviceAddress = address;
                        deviceInfoHub[device].clientDriverID   = clientDriverID;
						pCurrInterfaceDetails->endpointIN = endpointIN;
						pCurrInterfaceDetails->endpointPollInterval = descriptor[i+6];
                        USBHostSetNAKTimeout( address, endpointIN,  1, USB_NUM_BULK_NAKS );
						#ifndef USB_ENABLE_TRANSFER_EVENT
							deviceInfoHub[device].state = STATE_INITIALIZE_DEVICE;
						#endif
					}
				return TRUE;
				}
			}
		}

    	// Jump to the next descriptor in this configuration.
		i += descriptor[i];
	}
	// This configuration is not valid for a Hub device.
	return FALSE;
}



/*******************************************************************************
  Function:
     void USBHostHubTasks( void )

  Summary:
    This function performs the maintenance tasks required by Hub class

  Description:
    This function performs the maintenance tasks required by the Hub
    class.  If transfer events from the host layer are not being used, then
    it should be called on a regular basis by the application.  If transfer
    events from the host layer are being used, this function is compiled out,
    and does not need to be called.

  Precondition:
    USBHostHubInitialize() has been called.

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
*******************************************************************************/
void USBHostHubTasks( void )
{
#ifndef USB_ENABLE_TRANSFER_EVENT
	DWORD   byteCount;
   	BYTE    errorCode;
   	BYTE    i;

	DBPRINTF("USBHostHubTasks(deviceInfoHub[i].deviceAddress = deviceInfoHub[%x", i);
	DBPRINTF("].%x)\n", deviceInfoHub[i].deviceAddress);

	for (i=0; i<USB_MAX_HUB_DEVICES; i++)
   	{
       	if (deviceInfoHub[i].deviceAddress != 0) /* device address updated by lower layer */
		{
      		switch (deviceInfoHub[i].state & STATE_MASK)
			{
      			case STATE_DETACHED:
					// No device attached.
	  			break;

	    		case STATE_INITIALIZE_DEVICE:
	           		switch (deviceInfoHub[i].state & SUBSTATE_MASK)
              		{
               			case SUBSTATE_WAIT_FOR_ENUMERATION:
                  			if (USBHostDeviceStatus( i ) == USB_DEVICE_ATTACHED)
							{
								_USBHostHub_SetNextSubState();
								pCurrInterfaceDetails = pInterfaceDetails; // assign current interface to top of list
							}
                   			break;

						case SUBSTATE_DEVICE_ENUMERATED:
                   			_USBHostHub_SetNextState();
                   			break;

						default :
                   			break;
					}
					break;
			
    			case STATE_GET_HUB_DESCRIPTORS:
           			switch (deviceInfoHub[i].state & SUBSTATE_MASK)
					{
						case SUBSTATE_GET_STATUS_CHANGE_EP:
							switch (deviceInfoHub[i].state & SUBSUBSTATE_MASK)
							{
								case SUBSUBSTATE_SEND_GET_STATUS_CHANGE_EP:
									// If we are currently sending a token, we cannot do anything.
									if (U1CONbits.TOKBUSY)
										break;

									if(pCurrInterfaceDetails != NULL) // end of interface list
									{
										if(pCurrInterfaceDetails->sizeofStatusChangeDesc !=0) // Interface must have status change descriptor
										{
											if((deviceInfoHub[i].StatusChange = (BYTE *)malloc(pCurrInterfaceDetails->sizeofStatusChangeDesc)) == NULL)
											{
												_USBHostHub_LockDevice( USB_MEMORY_ALLOCATION_ERROR );
												break;
											}
											// send getdescriptor() request
											if ( !USBHostIssueDeviceRequest(deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_DEVICE,
											    USB_REQUEST_GET_DESCRIPTOR, DESCRIPTOR_STATUS_CHANGE, 0, 0x47,
											    deviceInfoHub[i].StatusChange, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
											{
												_USBHostHub_SetNextSubSubState();
											}
											else
											{
												free(deviceInfoHub[i].StatusChange);
											}
										}
									}
									else
									{
										//insert lock device here
									}
									break;
	
								case SUBSUBSTATE_WAIT_FOR_GET_STATUS_CHANGE_EP:
									if ( USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
									{
										if(errorCode)
										{
											/* Set error code */
    	                            		_USBHostHub_LockDevice( errorCode );
										}
										else
										{
    	                            		// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
    	                   					USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
										}
										_USBHostHub_SetNextSubSubState();
									}
									break;
	
								case SUBSUBSTATE_GET_STATUS_CHANGE_EP_COMPLETE:
									_USBHostHub_SetNextSubState();
									break;
							}
							break;
	
						case SUBSTATE_GET_STATUS:
							switch (deviceInfoHub[i].state & SUBSUBSTATE_MASK)
							{
								case SUBSUBSTATE_SEND_GET_STATUS:
									// If we are currently sending a token, we cannot do anything.
									if (U1CONbits.TOKBUSY)
										break;
									if((deviceInfoHub[i].Status = (BYTE *)malloc(sizeof(WORD))) == NULL )
									{
										_USBHostHub_LockDevice( USB_MEMORY_ALLOCATION_ERROR );
										break;
									}
									// send getstatus() request
									if (!USBHostIssueDeviceRequest(deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE,
									    USB_REQUEST_GET_STATUS, 0, 0, 0x02,
									    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
									{
										_USBHostHub_SetNextSubSubState();
									}
									else
									{
										free(deviceInfoHub[i].Status);
									}
									break;
	
								case SUBSUBSTATE_WAIT_FOR_GET_STATUS:
									if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
									{
										if(errorCode)
										{
											/* Set error code */
											_USBHostHub_LockDevice( errorCode );
										}
									}
									else
									{
										// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
										USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
	          						}
									_USBHostHub_SetNextSubSubState();
									break;

								case SUBSUBSTATE_GET_STATUS_COMPLETE:
									_USBHostHub_SetNextSubState();
									break;
							}
							break;

						case SUBSTATE_CLEAR_PORT_FEATURE:
							// Clear Feature: C_PORT_CONNECTION
							switch(deviceInfoHub[i].state & SUBSUBSTATE_MASK)
							{
								case SUBSUBSTATE_SEND_CLEAR_PORT_FEATURE:
									// If we are currently sending a token, we cannot do anything.
									if (U1CONbits.TOKBUSY)
										break;
									// Send clear port feature : C PORT CONNECTION.
									if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
									    USB_REQUEST_CLEAR_FEATURE, C_PORT_CONNECTION, deviceInfoHub[i].port, 0,
									    0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
									{
										_USBHostHub_SetNextSubSubState();
									}
									break;

								case SUBSUBSTATE_WAIT_FOR_SEND_CLEAR_PORT_FEATURE:
									if ( USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
									{
										if(errorCode)
										{
											/* Set error code */
											_USBHostHub_LockDevice( errorCode );
										}
									}
									else
									{
    	                           		// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
										USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
									}
									_USBHostHub_SetNextSubSubState();
									break;

								case SUBSUBSTATE_SEND_CLEAR_PORT_FEATURE_COMPLETE:
									if(deviceInfoHub[i].port == deviceInfoHub[i].bNumberPort)
									{
										deviceInfoHub[i].port=1;
										_USBHostHub_SetNextSubState();
									}
									else
									{
										deviceInfoHub[i].port+=1;
										_USBHostHub_SetStartingSubSubState();
									}
									break;
							}
							break;

						case SUBSTATE_SET_PORT_FEATURE:
							// Set Feature: Port Power
							switch(deviceInfoHub[i].state & SUBSUBSTATE_MASK)
							{
								case SUBSUBSTATE_SEND_SET_PORT_FEATURE:
									// Send set port feature : POWER ON to turn on the hub.
									// If we are currently sending a token, we cannot do anything.
									if (U1CONbits.TOKBUSY)
										break;
									if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
									    USB_REQUEST_SET_FEATURE, PORT_POWER, deviceInfoHub[i].port, 0,
									    0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
									{
										_USBHostHub_SetNextSubSubState();
									}
									break;

								case SUBSUBSTATE_WAIT_FOR_SEND_SET_PORT_FEATURE:
									if ( USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
									{
										if(errorCode)
										{
											/* Set error code */
											_USBHostHub_LockDevice( errorCode );
										}
									}
									else
									{
										// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
										USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
	          						}
									_USBHostHub_SetNextSubSubState();
									break;

								case SUBSUBSTATE_SEND_SET_PORT_FEATURE_COMPLETE:
									if(deviceInfoHub[i].port == deviceInfoHub[i].bNumberPort)
									{
										deviceInfoHub[i].port=1;
										_USBHostHub_SetNextSubState();
									}
									else
									{
										deviceInfoHub[i].port+=1;
										_USBHostHub_SetStartingSubSubState();
									}
									break;
							}
							break;	

						case SUBSTATE_GET_PORT_STATUS:
							switch(deviceInfoHub[i].state & SUBSUBSTATE_MASK)
							{
								case SUBSUBSTATE_SEND_GET_PORT_STATUS:
									// Send get port status on each port
									// If we are currently sending a token, we cannot do anything.
									if (U1CONbits.TOKBUSY)
										break;
									if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
									    USB_REQUEST_GET_STATUS, 0, deviceInfoHub[i].port, 4,
									    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
									{
										_USBHostHub_SetNextSubSubState();
									}
									break;

								case SUBSUBSTATE_WAIT_FOR_SEND_GET_PORT_STATUS:
									if ( USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
									{
										if(errorCode)
										{
											/* Set error code */
											_USBHostHub_LockDevice( errorCode );
										}
									}
									else
									{
										// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
										USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
									}
									_USBHostHub_SetNextSubSubState();
									break;
	
								case SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE:
									if(deviceInfoHub[i].port == deviceInfoHub[i].bNumberPort)
									{
										deviceInfoHub[i].port=1;
										_USBHostHub_SetNextSubState();
									}
									else
									{
										deviceInfoHub[i].port+=1;
										_USBHostHub_SetStartingSubSubState();
									}
									break;
							}
							break;

						case SUBSTATE_CLEAR_PORT_PRESENT_STATUS:

							if((deviceInfoHub[i].HubDevice = (BYTE *)malloc(pCurrInterfaceDetails->sizeofStatusChangeDesc)) == NULL)
							{
								_USBHostHub_LockDevice( USB_MEMORY_ALLOCATION_ERROR );
								break;
							}
							pHubDevice = deviceInfoHub[i].HubDevice;

							for( deviceInfoHub[i].port=2; deviceInfoHub[i].port<=deviceInfoHub[i].bNumberPort + 1; deviceInfoHub[i].port++ )
							{
								((USB_HUB_DEVICE *)pHubDevice)->bPortPresent[deviceInfoHub[i].port] = 0;
								((USB_HUB_DEVICE *)pHubDevice)->bPortNumber[deviceInfoHub[i].port] = 0;
							}
							deviceInfoHub[i].StatusChange[0] = 0;
							_USBHostHub_SetNextState();
							break;
					}
					break;
	
				case STATE_RUNNING:
					switch (deviceInfoHub[i].state & SUBSTATE_MASK)
						{
							case SUBSTATE_HOLDING:
								break;

							case SUBSTATE_PORT_TASK:
								for(deviceInfoHub[i].port = 1;deviceInfoHub[i].port < USB_MAX_DEVICES; deviceInfoHub[i].port++)
								{
									if ( ((USB_HUB_DEVICE *)pHubDevice)->bPortPresent[deviceInfoHub[i].port + 1] )	//nakaconnect device
									{
										do
										{
											// do port enumeration task
											( deviceInfoHub[i].port );
											// Ayusin to for error later on
											if (USBHostDeviceStatus(deviceInfoHub[i].port) == USB_HOLDING_OUT_OF_MEMORY)
												break;
										}while ( USBHostDeviceStatus(deviceInfoHub[i].port) !=	USB_DEVICE_ATTACHED );		// na enumerate na ba?
									}
								}
//								DBPRINTF("\nK");
								_USBHostHub_SetNextSubState();
								break;

							case SUBSTATE_WAITING_FOR_HUBCHANGE_EP:
								errorCode = USBHostHubTransfer( deviceInfoHub[i].deviceAddress, deviceInfoHub[i].endpointIN, STATUS_CHANGE_SIZE, deviceInfoHub[i].StatusChange);
								if (errorCode)
								// State change is handled by the USBHostHubTransfer function 
								{
									_USBHostHub_TerminateTransfer( errorCode );
								}
								break;
	
							case SUBSTATE_READ_HUBCHANGE_EP:;
								if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, deviceInfoHub[i].endpointDATA, &errorCode, &byteCount ))
								{
									if (errorCode)
									{
										DBPRINTF("errorCode2 is = %x\r\n",errorCode);
										if(USB_ENDPOINT_STALLED == errorCode)
										{
											USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, deviceInfoHub[i].endpointDATA );
											deviceInfoHub[i].returnState = STATE_RUNNING | SUBSTATE_WAITING_FOR_HUBCHANGE_EP;
											deviceInfoHub[i].flags.bfReset = 1;
											_USBHostHub_ResetStateJump( i );
										}
										else
										{
											_USBHostHub_TerminateReadTransfer(errorCode);
										}
									}
									else
									{
										// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
										USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, deviceInfoHub[i].endpointDATA );
//										DBPRINTF("\nStatus Change 1 = %x\r\n",(deviceInfoHub[i].StatusChange[0]&0x1F));
										_USBHostHub_SetNextSubState();
									}
								}
								break;

							case SUBSTATE_READ_HUBCHANGE_EP_DONE:
								/* Next transfer */
								if( deviceInfoHub[i].StatusChange[0]&0x1F )					// 0x1F 4 port bits + 1 hub bit
								{
									StatChangeIndicator = 1;
									deviceInfoHub[i].portstate = STATE_GET_PORT_NUMBER;
									#define USB_PORT_TASK_ENABLE
								}
								while(StatChangeIndicator)
								{
									USBPortInitialize();
									DelayMs(1);
								}
		 						deviceInfoHub[i].state = STATE_RUNNING | SUBSTATE_PORT_TASK;
								break;
						}
						break;
	
				case STATE_HOLDING:
					DBPRINTF("Current Port state is %x\r\n",deviceInfoHub[i].portstate);
					break;
			}
		}
	}
#endif
}


/*******************************************************************************
  Function:
    void  USBPortInitialize( void )


  Summary:
    This function performs the appropriate tasks to be done when a change in
	the hubchange endpoint is detected.

  Description:
    This function performs the appropriate tasks to be done when a change in
	the hubchange endpoint is detected. The port number is detected and the port
	resets. After this is done, the device is ready for enumeration.

  Preconditions:
    deviceInfoHub[i].StatusChange[0] should have a value

  Parameters:
    None

  Return Values:
    None

  Remarks:
    None
*******************************************************************************/

void USBPortInitialize( void )
{
#ifdef USB_PORT_TASK_ENABLE

	BYTE				PortSelect			= 0x02;
	BYTE				i					= 0;
	BYTE				j;
	DWORD   			byteCount;
	BYTE				errorCode;

	// Always use USB address #1 (the hub itself) for hub communication
	switch (deviceInfoHub[i].portstate & STATE_MASK)
	{
		case STATE_GET_PORT_NUMBER:
			for( j=0, CurrentPort = 0; j < deviceInfoHub[i].bNumberPort; j++ )
			{
				if(!(deviceInfoHub[i].StatusChange[0]&PortSelect))
					PortSelect <<= 1;
				else
				{
					switch(deviceInfoHub[i].StatusChange[0]&PortSelect)			// If more ports, increase the number of cases, for this code, only 4 downstream ports
					{
						case 0x02:
							CurrentPort = 0x01;
							break;

						case 0x04:
							CurrentPort = 0x02;
							break;

						case 0x08:
							CurrentPort = 0x03;
							break;

						case 0x10:
							CurrentPort = 0x04;
							break;

						default:
							CurrentPort = (deviceInfoHub[i].StatusChange[1] << 1) | deviceInfoHub[i].StatusChange[0];
							break;
					}
					deviceInfoHub[i].StatusChange[0] = 0;
					_USB_HostHubPort_SetNextState();
					break;
				}
			}
			break;

		case STATE_GET_INITIAL_STATUS:
			switch (deviceInfoHub[i].portstate & SUBSTATE_MASK)
			{
				case SUBSTATE_SEND_GET_STATUS:
					// Send get port status request
					pStat = deviceInfoHub[i].Status;									// keep current wPortStatus & wPortChange

					// If we are currently sending a token, we cannot do anything.
					if (U1CONbits.TOKBUSY)
						break;
					
					if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
					    USB_REQUEST_GET_STATUS, 0, CurrentPort, 4,
					    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
					{
						_USB_HostHubPort_SetNextSubState();
					}
					break;

				case SUBSTATE_WAIT_FOR_SEND_GET_STATUS:
					if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
					{
						if(errorCode)
						{
							/* Set error code */
							_USBHostHub_LockDevice( errorCode );
						}
					}
					else
					{
						// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
						USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
					}
					_USB_HostHubPort_SetNextSubState();
					break;

				case SUBSTATE_SEND_GET_STATUS_COMPLETE:
					if( ( (( USB_PORT_STATUS *)pStat)->wPortStatus_Lo)&&0x01 )
					{
						_USBHostHubPort_SetDevicePresentState();				// device connected
					}
					else
					{
						_USBHostHubPort_SetDeviceNotPresentState();				// device disconnected
					}
					break;
			}
			break;

		case STATE_DEVICE_PRESENT:
			switch (deviceInfoHub[i].portstate & SUBSTATE_MASK)
			{
				case SUBSTATE_SEARCH_UNUSED_USB_ADDRESS:
					// We will set the device address to the specified port number so we will
					// not search for unusewd USB address, we will do that instead.
					DevAddr = CurrentPort + 1;

					if( DevAddr == (USB_MAX_DEVICES + 1) )
					{
						// Open port not found
          				DevAddr = 0;								// open port not found (shouldn't happen)
						_USBHostHub_LockDevice( errorCode );		// insert error here
					}
					_USB_HostHubPort_SetNextSubState();
					break;

				case SUBSTATE_CLEAR_C_PORT_CONNECTION:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION:
							// Send Clear_Feature(C_PORT_CONNECTION) on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_CLEAR_FEATURE, C_PORT_CONNECTION, CurrentPort, 0,
							    0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_CLEAR_C_PORT_CONNECTION:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								if(errorCode)
								{
									/* Set error code */
									_USBHostHub_LockDevice( errorCode );
								}
								else
								{
									// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
									USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
								}
							}
							_USB_HostHubPort_SetNextSubSubState();
							break;

						case SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION_COMPLETE:
							_USB_HostHubPort_SetNextSubState();
							break;
					}
					break;

				case SUBSTATE_CLEAR_FEAT_C_PORT_GET_STATUS:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_CLEAR_FEAT_C_PORT_GET_STATUS:
							// Send GetPortStatus() on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_GET_STATUS, 0, CurrentPort, 0x04,
							    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )			// baguhin mo ang Status
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_CLEAR_FEAT_C_PORT_GET_STATUS:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								if(errorCode)
								{
									/* Set error code */
									_USBHostHub_LockDevice( errorCode );
								}
								else
								{
									// Clear the STALL.  Since it is EP0, we do not have to clear the stall.
									USBHostClearEndpointErrors( deviceInfoHub[i].deviceAddress, 0 );
								}
							}
							_USB_HostHubPort_SetNextSubSubState();
							break;

						case SUBSUBSTATE_SEND_CLEAR_FEAT_C_PORT_GET_STATUS_COMPLETE:
                            //MODIFICATION (1/7/2016)
							//if( !((( ((USB_PORT_STATUS *)deviceInfoHub[i].Status))->wPortChange_Lo) && USB_PORT_STAT_C_CONNECTION) )
							if( !((( ((USB_PORT_STATUS *)deviceInfoHub[i].Status))->wPortChange_Lo) && PORT_CONNECTION) )
			
                            {
								_USB_HostHubPort_SetNextSubState();
							}
							else
							{
								_USB_HostHubPort_SetStartingSubSubState();
							}
							break;
					}
					break;

				case SUBSTATE_SET_FEATURE_PORT_RESET:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_SET_FEATURE_PORT_RESET:
							// Send Set_Feature(PORT_RESET) on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
										  USB_REQUEST_SET_FEATURE, PORT_RESET, CurrentPort, 0,
									  	  0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_SET_FEATURE_PORT_RESET:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_SET_FEATURE_PORT_RESET_COMPLETE:
							_USB_HostHubPort_SetNextSubState();
							break;
					}
					break;

				case SUBSTATE_PORT_RESET_GET_STATUS:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_PORT_RESET_GET_STATUS:
							// Send GetPortStatus() on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_GET_STATUS, 0, CurrentPort, 4,
							    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_PORT_RESET_GET_STATUS:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_PORT_RESET_GET_STATUS_COMPLETE:
							//MODIFICATIONN(1/7/2016)
                            //if( (( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && USB_PORT_STAT_C_RESET) )
							if( (( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && PORT_RESET) )
                            {
								_USB_HostHubPort_SetNextSubState();
							}
							else
							{
								_USB_HostHubPort_SetStartingSubSubState();
							}
							break;

					}
					break;

				case SUBSTATE_CLEAR_FEATURE_C_PORT_RESET:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_CLEAR_FEATURE_C_PORT_RESET:
							// Send Clear_Feature(C_PORT_RESET) on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_CLEAR_FEATURE, C_PORT_RESET, CurrentPort, 0,
							    0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_CLEAR_FEATURE_C_PORT_RESET:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_CLEAR_FEATURE_C_PORT_RESET_COMPLETE:
							_USB_HostHubPort_SetNextSubState();
							break;
					}
					break;

				case SUBSTATE_C_PORT_RESET_GET_STATUS:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_C_PORT_RESET_GET_STATUS:
							// Send GetPortStatus() on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_GET_STATUS, 0, CurrentPort, 4,
							    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_C_PORT_RESET_GET_STATUS:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_C_PORT_RESET_GET_STATUS_COMPLETE:
                            //MODIFICATIONN(1/7/2016)
                            //if( (( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && USB_PORT_STAT_C_RESET) )
							if( (( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && PORT_RESET) )
                            {
								_USB_HostHubPort_SetNextSubState();
							}
							else
							{
								_USB_HostHubPort_SetStartingSubSubState();
							}
							break;
					}
					break;
					
				case SUBSTATE_DETERMINE_DOWNSTREAM_SPEED:
					// Check bit 9 of wPortStatus for port speed
					if (( ((USB_PORT_STATUS *)pStat)->wPortStatus_Hi)&0x02)
					{
						// Device is High Speed
						((USB_HUB_DEVICE *)pHubDevice)->bPortSpeed[0] = ((USB_HUB_DEVICE *)pHubDevice)->bPortSpeed[DevAddr] = 1;			// '1' - lowspeed, USB address 0 must also be set
					}
					else
					{
						// Device is Low Speed
						((USB_HUB_DEVICE *)pHubDevice)->bPortSpeed[0] = ((USB_HUB_DEVICE *)pHubDevice)->bPortSpeed[DevAddr] = 0;
					}
					_USB_HostHubPort_SetNextSubState();
					break;

				case SUBSTATE_ENUMERATE_CURRENT_ATTACHED_DEVICE:
					((USB_HUB_DEVICE *)pHubDevice)->bPortNumber[DevAddr] = CurrentPort;		// Save port number used in this address
					((USB_HUB_DEVICE *)pHubDevice)->bPortPresent[DevAddr] = 1;				// set USB device present on this address
					StatChangeIndicator = 0;
					// insert event here
//					DBPRINTF("\nPort Initialized\n");
					break;
			}
			break;

		case STATE_DEVICE_NOT_PRESENT:
			switch (deviceInfoHub[i].portstate & SUBSTATE_MASK)
			{
				case SUBSTATE_CLEAR_C_PORT_CONNECTION_NP:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION_NP:
							// Send Clear_Feature(C_PORT_CONNECTION) on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_CLEAR_FEATURE, C_PORT_CONNECTION, CurrentPort, 0,
							    0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_CLEAR_C_PORT_CONNECTION_NP:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_CLEAR_C_PORT_CONNECTION_COMPLETE_NP:
							_USB_HostHubPort_SetNextSubSubState();
							break;

						case SUBSUBSTATE_SEND_GET_PORT_STATUS_NP:
							// Send GetPortStatus() on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_GET_STATUS, 0, CurrentPort, 4,
							    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_GET_PORT_STATUS_NP:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE_NP:
							//MODIFICATION(1/7/2016)
                            //if( !(( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && USB_PORT_STAT_C_CONNECTION) )
							if( !(( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && PORT_CONNECTION) )
							{
								_USB_HostHubPort_SetNextSubState();
							}
							else
							{
								deviceInfoHub[i].portstate = STATE_DEVICE_NOT_PRESENT | SUBSTATE_CLEAR_C_PORT_CONNECTION_NP | SUBSUBSTATE_SEND_GET_PORT_STATUS_NP;
							}
							break;
					}
					break;

				case SUBSTATE_CLEAR_C_PORT_ENABLE:
					switch (deviceInfoHub[i].portstate & SUBSUBSTATE_MASK)
					{
						case SUBSUBSTATE_SEND_CLEAR_C_PORT_ENABLE:
							// Send Clear_Feature(C_PORT_ENABLE) on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_CLEAR_FEATURE, C_PORT_ENABLE, CurrentPort, 0,
							    0, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_CLEAR_C_PORT_ENABLE:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_CLEAR_C_PORT_ENABLE_2:
							_USB_HostHubPort_SetNextSubSubState();
							break;

						case SUBSUBSTATE_SEND_GET_PORT_STATUS_NP2:
							// Send GetPortStatus() on the current port.
							// If we are currently sending a token, we cannot do anything.
							if (U1CONbits.TOKBUSY)
								break;
							if( !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER,
							    USB_REQUEST_GET_STATUS, 0, CurrentPort, 4,
							    deviceInfoHub[i].Status, USB_DEVICE_REQUEST_GET, deviceInfoHub[i].clientDriverID) )
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_WAIT_FOR_GET_PORT_STATUS_NP2:
							if (USBHostTransferIsComplete( deviceInfoHub[i].deviceAddress, 0, &errorCode, &byteCount ))
							{
								_USB_HostHubPort_SetNextSubSubState();
							}
							break;

						case SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE_NP2:
							//MODIFICATION(1/7/2016)
                            //if( !(( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && USB_PORT_STAT_C_ENABLE) )
							if( !(( ((USB_PORT_STATUS *)pStat)->wPortChange_Lo) && PORT_ENABLE) )
							{
								_USB_HostHubPort_SetNextSubState();
							}
							else
							{
								deviceInfoHub[i].portstate = STATE_DEVICE_NOT_PRESENT | SUBSTATE_CLEAR_C_PORT_ENABLE | SUBSUBSTATE_SEND_GET_PORT_STATUS_COMPLETE_NP2;
							}
							break;
					}
					break;

				case SUBSTATE_SEARCH_DETACHED_PORT:

					for(DevAddr=HUB_ADDR + 1; DevAddr<USB_MAX_DEVICES + 1; DevAddr++)
					{
						if(((USB_HUB_DEVICE *)pHubDevice)->bPortNumber[DevAddr] == CurrentPort)
							break;
					}
					if(DevAddr == USB_MAX_DEVICES + 1)
					{
						DevAddr = 0;
					}
					if(DevAddr)
					{
						((USB_HUB_DEVICE *)pHubDevice)->bPortPresent[DevAddr] = 0;
						((USB_HUB_DEVICE *)pHubDevice)->bPortNumber[DevAddr] = 0;
					}
					StatChangeIndicator = 0;
					// Insert Event here
					//USBHostDetachDevice(DevAddr);
					DBPRINTF("\nDevice Disconnected\n");
					break;
			}
			break;
	}
#endif
}

/*******************************************************************************
  Function:
  BYTE  USBHostHubTransfer( BYTE deviceAddress, BYTE interfaceNum, WORD size
                        BYTE *data)

  Summary:
    This function starts a Hub transfer.

  Description:
    This function starts a Hub transfer. A read/write wrapper is provided in
    application interface file to access this function. We use this to get the
    status change endpoint.

  Preconditions:
    None

  Parameters:
    BYTE deviceAddress      - Device address
    BYTE interfaceNum       - Interface number of the device
    BYTE size               - Byte size of the data buffer
    BYTE *data              - Pointer to the data buffer

  Return Values:
    USB_SUCCESS                 - Request started successfully
    USB_HUB_DEVICE_NOT_FOUND    - No device with specified address
    USB_HUB_DEVICE_BUSY         - Device not in proper state for
                                  performing a transfer
    Others                      - Return values from USBHostIssueDeviceRequest(),
                                    USBHostRead(), and USBHostWrite()

  Remarks:
    None
*******************************************************************************/

BYTE USBHostHubTransfer( BYTE deviceAddress, BYTE interfaceNum, WORD size, BYTE *data )
{
	BYTE    i;
 	BYTE    errorCode;
	BYTE	direction = 1;				// set direction to read always since hub operation always reads

	// Find the correct device.
	for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].deviceAddress != deviceAddress); i++);
		if (i == USB_MAX_HUB_DEVICES)
    		{
        		return USB_HUB_DEVICE_NOT_FOUND;
    		}
	pCurrInterfaceDetails = pInterfaceDetails;

	while((pCurrInterfaceDetails != NULL) && (pCurrInterfaceDetails->interfaceNumber != interfaceNum))
	{
		pCurrInterfaceDetails = pCurrInterfaceDetails->next;
	}
    // Make sure the device is in a state ready to read/write.
    #ifndef USB_ENABLE_TRANSFER_EVENT
		if ((deviceInfoHub[i].state != (STATE_RUNNING)) &&
		   (deviceInfoHub[i].state & SUBSTATE_MASK) != (SUBSTATE_WAITING_FOR_HUBCHANGE_EP))
	#else
		if (deviceInfoHub[i].state != STATE_RUNNING)
	#endif
		{
			return USB_HUB_DEVICE_BUSY;
		}

	// Initialize the transfer information.
	deviceInfoHub[i].bytesTransferred  = 0;
	deviceInfoHub[i].errorCode         = USB_SUCCESS;
	deviceInfoHub[i].StatusChange	   = data;
	deviceInfoHub[i].endpointDATA      = pCurrInterfaceDetails->endpointIN;
	deviceInfoHub[i].interface         = interfaceNum;

	if (direction) // IN, if the device is a hub, it would be impossible to not meet this condition because Hubs don't have out endpoints
	{
		deviceInfoHub[i].endpointDATA  = pCurrInterfaceDetails->endpointIN;
	}    
	#ifdef DEBUG_MODE
//      UART2PrintString( "Data EP: " );
//      UART2PutHex( deviceInfoHub[i].endpointDATA );
//      UART2PrintString( "\r\n" );
	#endif

	if(direction)
	{
		
		errorCode = USBHostRead( deviceInfoHub[i].deviceAddress, deviceInfoHub[i].endpointDATA,
		    	                 deviceInfoHub[i].StatusChange, size );
		if (!errorCode)
       	{
        	#ifndef USB_ENABLE_TRANSFER_EVENT
               	deviceInfoHub[i].state  = STATE_RUNNING | SUBSTATE_READ_HUBCHANGE_EP;
			#else
               	deviceInfoHub[i].state  = SUBSTATE_READ_HUBCHANGE_EP;
			#endif
        }
        else
        {
			deviceInfoHub[i].errorCode    = errorCode;
			deviceInfoHub[i].state        = STATE_RUNNING;
		}
	}
	return errorCode;
}

/*******************************************************************************
  Function:
    BOOL USBHostubTransferIsComplete( BYTE deviceAddress,
                        BYTE errorCode, DWORD byteCount )

  Summary:
    This function indicates whether or not the last transfer is complete.

  Description:
    This function indicates whether or not the last transfer is complete.
    If the functions returns TRUE, the returned byte count and error
    code are valid. Since only one transfer can be performed at once
    and only one endpoint can be used, we only need to know the
    device address.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress  - Device address
    BYTE *errorCode     - Error code from last transfer
    DWORD *byteCount    - Number of bytes transferred

  Return Values:
    TRUE    - Transfer is complete, errorCode is valid
    FALSE   - Transfer is not complete, errorCode is not valid
*******************************************************************************/

BOOL USBHostHubTransferIsComplete( BYTE deviceAddress, BYTE *errorCode, BYTE *byteCount )
{
    BYTE    i;

     // Make sure a valid device is being requested.
    if ((deviceAddress == 0) || (deviceAddress > 127))
    {
        *errorCode = USB_HUB_DEVICE_NOT_FOUND;
        *byteCount = 0;
        return TRUE;
    }
    // Find the correct device.
    for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].deviceAddress != deviceAddress); i++);
    if ((i == USB_MAX_HUB_DEVICES) || (deviceInfoHub[i].state == STATE_DETACHED))
    {
        *errorCode = USB_HUB_DEVICE_NOT_FOUND;
        *byteCount = 0;
        return TRUE;
    }


    #ifndef USB_ENABLE_TRANSFER_EVENT
    	if ( (deviceInfoHub[i].state == (STATE_RUNNING | SUBSTATE_HOLDING)) ||
			((deviceInfoHub[i].state & STATE_MASK) == STATE_HOLDING))
	#else
    	if ((deviceInfoHub[i].state == STATE_RUNNING) ||
			(deviceInfoHub[i].state == STATE_HOLDING))
    #endif
    {
    	*byteCount = deviceInfoHub[i].bytesTransferred;
    	*errorCode = deviceInfoHub[i].errorCode;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/****************************************************************************
  Function:
    BYTE USBHostHubResetDeviceWithWait( BYTE deviceAddress  )

  Description:
    This function resets a Hub device, and waits until the reset is complete.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress  - Address of the device to reset.

  Return Values:
    USB_SUCCESS                 - Reset successful
    USB_HUB_RESET_ERROR         - Error while resetting device
    Others                      - See return values for USBHostHubResetDevice()
                                    and error codes that can be returned
                                    in the errorCode parameter of
                                    USBHostHubTransferIsComplete();
                                    
  Remarks:
    None
  ***************************************************************************/

BYTE USBHostHubResetDeviceWithWait( BYTE deviceAddress, BYTE i  )
{
    BYTE    byteCount;
    BYTE    errorCode;


    errorCode = USBHostHubResetDevice( deviceAddress );
    if (errorCode)
    {
        return errorCode;
    }
    do
    {
		USBHostTasks(deviceInfoHub[i].port);
        errorCode = USBHostHubDeviceStatus( deviceAddress );
    } while (errorCode == USB_HUB_RESETTING_DEVICE);

    if (USBHostHubTransferIsComplete( deviceAddress, &errorCode, (BYTE*)&byteCount ))
    {
        return errorCode;
    }
    return USB_HUB_RESET_ERROR;
}



// *****************************************************************************
// *****************************************************************************
// Section: Host Stack Interface Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    BOOL USBHostHubEventHandler( BYTE address, USB_EVENT event,
                        void *data, DWORD size )

  Precondition:
    The device has been initialized.

  Summary:
    This function is the event handler for this client driver.

  Description:
    This function is the event handler for this client driver.  It is called
    by the host layer when various events occur. Not much event occurs on a
	hub that is not handled by hubtasks so leave this as is unless there are
	a few more events not covered.

  Parameters:
    BYTE address    - Address of the device
    USB_EVENT event - Event that has occurred
    void *data      - Pointer to data pertinent to the event
    DWORD size       - Size of the data

  Return Values:
    TRUE   - Event was handled
    FALSE  - Event was not handled

  Remarks:
    None
*******************************************************************************/

BOOL USBHostHubEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size )
{
	BYTE    i;
	switch (event)
	{
		case EVENT_NONE:             // No event occured (NULL event)
			return TRUE;
			break;
	
		case EVENT_DETACH:           // USB cable has been detached (data: BYTE, address of device)
			#ifdef DEBUG_MODE
				UART2PrintString( "HUB: Detach\r\n" );
			#endif
			// Find the device in the table.  If found, clear the important fields.
	        for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].deviceAddress != address); i++);
	       	if (i < USB_MAX_HUB_DEVICES)
			{
				// Notify that application that the device has been detached.
				USB_HOST_APP_EVENT_HANDLER( address, EVENT_DETACH,&deviceInfoHub[i].deviceAddress, sizeof(BYTE) );
	 			/* Free the memory used by the HID device */
//edit				_USBHostHub_FreeRptDecriptorDataMem(address);
				DBPRINTF("\nDetached yung hub?\n");
				deviceInfoHub[i].deviceAddress   = 0;
				deviceInfoHub[i].state  = STATE_DETACHED;
			}
			return TRUE;
	        break;
		case EVENT_TRANSFER:         // A USB transfer has completed
/*
        	#if defined( USB_ENABLE_TRANSFER_EVENT )
        		#ifdef DEBUG_MODE
               		UART2PrintString( "HID: transfer event\r\n" );
               	#endif
			for (i=0; (i<USB_MAX_HUB_DEVICES) && (deviceInfoHub[i].ID.deviceAddress != address); i++) {}
				if (i == USB_MAX_HUB_DEVICES)
                	{
                		#ifdef DEBUG_MODE
                			UART2PrintString( "HID: Unknown device\r\n" );
                		#endif
                		return FALSE;
                	}
			switch (deviceInfoHub[i].state)
                	{
						case STATE_WAIT_FOR_HUBCHANGE_EP:
							#ifdef DEBUG_MODE
								UART2PrintString( "HID: Got Report Descriptor\r\n" );
							#endif
*/
		case STATE_RUNNING:	

        case EVENT_RESUME:           // Device-mode resume received
        case EVENT_SUSPEND:          // Device-mode suspend/idle event received
        case EVENT_RESET:            // Device-mode bus reset received
        case EVENT_STALL:            // A stall has occured
            return TRUE;
            break;

        default:
            return FALSE;
            break;
    }
    return FALSE;
}


// *****************************************************************************
// *****************************************************************************
// Internal Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    void _USBHostHub_ResetStateJump( BYTE i )

  Summary:

  Description:
    This function determines which portion of the reset processing needs to
    be executed next and jumps to that state.

Precondition:
    The device information must be in the deviceInfoHub array.

  Parameters:
    BYTE i  - Index into the deviceInfoHub structure for the device to reset.

  Returns:
    None

  Remarks:
    None
*******************************************************************************/
void _USBHostHub_ResetStateJump( BYTE i )
{
    BYTE    errorCode;

    if (deviceInfoHub[i].flags.bfReset)
    {
         errorCode = !USBHostIssueDeviceRequest( deviceInfoHub[i].deviceAddress, USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_INTERFACE,
                        USB_HUB_RESET, 0, deviceInfoHub[i].interface, 0, NULL, USB_DEVICE_REQUEST_SET, deviceInfoHub[i].clientDriverID );
        if (errorCode)
        {
            _USBHostHub_TerminateTransfer( USB_HUB_RESET_ERROR );
        }
        else
        {
			deviceInfoHub[i].state = SUBSTATE_READ_HUBCHANGE_EP_DONE;
        }
    }
    else
    {
        if (!deviceInfoHub[i].errorCode)
		{
			USB_HOST_APP_EVENT_HANDLER(deviceInfoHub[i].deviceAddress, EVENT_HUB_RESET, NULL, 0 );
		}
        else
        {
			USB_HOST_APP_EVENT_HANDLER(deviceInfoHub[i].deviceAddress, EVENT_HUB_RESET_ERROR, NULL, 0 );
        }        

        deviceInfoHub[i].state = deviceInfoHub[i].returnState;
    }
}

//drm :D
