/******************************************************************************
  File Information:
 
[ash] - add something here
[nyd] - not yet done



This code is heavily based on Cypress's SH811 code for USB Hub Class. Major props 
to them for making the design of the code.


Author          Date    Comments
--------------------------------------------------------------------------------
DRM          15-Oct-2007 First release

******************************************************************************/
//DOM-IGNORE-END

//DOM-IGNORE-BEGIN
#ifndef _USB_HOST_HUB_H_
#define _USB_HOST_HUB_H_
//DOM-IGNORE-END

extern BYTE CurrentPort;

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// Section: Hub Class Error Codes
// *****************************************************************************

#define HUB_COMMAND_PASSED                  0x00    // Transfer was successful. Returned in dCSWStatus.
#define HUB_COMMAND_FAILED                  0x01    // Transfer failed. Returned in dCSWStatus.
#define HUB_PHASE_ERROR                     0x02    // Transfer phase error. Returned in dCSWStatus.

#define USB_HUB_ERROR                       USB_ERROR_CLASS_DEFINED             // Error code offset.

#define USB_HUB_OUT_OF_MEMORY               (USB_HUB_ERROR | 0x01)              // No dynamic memory is available.
#define USB_HUB_DEVICE_NOT_FOUND            (USB_HUB_ERROR | 0x02)              // Device with the specified address is not available.
#define USB_HUB_DEVICE_BUSY                 (USB_HUB_ERROR | 0x03)              // A transfer is currently in progress.
#define USB_HUB_RESET_ERROR                 (USB_HUB_ERROR | 0x0A)              // An error occurred while resetting the device.
#define USB_HUB_ILLEGAL_REQUEST             (USB_HUB_ERROR | 0x0B)              // Cannot perform requested operation.

// *****************************************************************************
// Section: Additional return values for USBHostHubDeviceStatus (see USBHostDeviceStatus also)
// *****************************************************************************

#define USB_HUB_DEVICE_DETACHED             0x50    // Device is detached.
#define USB_HUB_INITIALIZING                0x51    // Device is initializing.
#define USB_PROCESSING_HUB_DESCRIPTORS	    0x52    // Parser is processing report descriptor.
#define USB_HUB_NORMAL_RUNNING              0x53    // Device is running and available for data transfers.
#define USB_HUB_DEVICE_HOLDING              0x54    // Device is holding due to error
#define USB_HUB_RESETTING_DEVICE            0x55    // Device is being reset.

// *****************************************************************************
// Section: Hub Event Definition
// *****************************************************************************

// If the application has not defined an offset for HID events, set it to 0.
#ifndef EVENT_HUB_OFFSET
    #define EVENT_HUB_OFFSET    0
#endif

    // No event occured (NULL event)
#define EVENT_HUB_NONE						EVENT_HUB_BASE + EVENT_HUB_OFFSET + 0   
    // A Report Descriptor has been parsed.  The returned data pointer is NULL.
    // The application must collect details, or simply return TRUE if the 
    // application is already aware of the data format.
#define EVENT_HUB_READ_DONE					EVENT_HUB_BASE + EVENT_HUB_OFFSET + 1   
    // A Hub Read transfer has completed.  The returned data pointer is the 
	// status change bits of the hub indicating if there is a device that 
	// attached or detached.  
#define EVENT_HUB_RESET						EVENT_HUB_BASE + EVENT_HUB_OFFSET + 2
	// 
#define EVENT_HUB_ATTACH					EVENT_HUB_BASE + EVENT_HUB_OFFSET + 3   
    // A Hub device has attached.  The returned data pointer points to a
    // USB_HUB_DEVICE structure.
#define EVENT_HUB_DETACH                    EVENT_HUB_BASE + EVENT_HUB_OFFSET + 4  
    // A Hub device has detached.  The returned data pointer points to a
    // byte with the previous address of the detached device.
#define EVENT_HUB_DEVICE_ATTACH				EVENT_HUB_BASE + EVENT_HUB_OFFSET + 5  
    // A device was attached to a hub. Device enumeration will be done to
	// the attached device.
#define EVENT_HUB_DEVICE_DETACHED			EVENT_HUB_BASE + EVENT_HUB_OFFSET + 6  
    // A device was detached from the hub. The data pointer points to a byte
	// with the previous address of the detached device.
#define EVENT_HUB_RESET_ERROR               EVENT_HUB_BASE + EVENT_HUB_OFFSET + 10   


// *****************************************************************************
// Section: Hub Event Definition
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Functions Prototype
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    BOOL USBHostHubDeviceDetect( BYTE deviceAddress )

  Description:
    This function determines if a HID device is attached and ready to use.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress  - Address of the attached device.

  Return Values:
    TRUE   -  HID present and ready
    FALSE  -  HID not present or not ready

  Remarks:
    This function replaces the USBHostHID_ApiDeviceDetect() function.
*******************************************************************************/

BOOL USBHostHubDeviceDetect( BYTE deviceAddress );

/*******************************************************************************
  Function:
    BYTE    USBHostHubDeviceStatus( BYTE deviceAddress )

  Summary:

  Description:
    This function determines the status of a HID device.

  Preconditions:  None

  Parameters:
    BYTE deviceAddress - address of device to query

  Return Values:
    USB_HID_DEVICE_NOT_FOUND           -  Illegal device address, or the
                                          device is not an HID
    USB_HID_INITIALIZING               -  HID is attached and in the
                                          process of initializing
    USB_PROCESSING_REPORT_DESCRIPTOR   -  HID device is detected and report
                                          descriptor is being parsed
    USB_HID_NORMAL_RUNNING             -  HID Device is running normal,
                                          ready to send and receive reports
    USB_HID_DEVICE_HOLDING             -
    USB_HID_DEVICE_DETACHED            -  HID detached.

  Remarks:
    None
*******************************************************************************/

BYTE    USBHostHubDeviceStatus( BYTE deviceAddress );


#define USBHostHubRead( deviceAddress,reportid,interface,size,data) \
         USBHostHubTransfer( deviceAddress,1,size,data)

/*******************************************************************************
  Function:
    BYTE USBHostHIDResetDevice( BYTE deviceAddress )

  Summary:
    This function starts a HID  reset.

  Description:
    This function starts a HID reset.  A reset can be
    issued only if the device is attached and not being initialized.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress - Device address

  Return Values:
    USB_SUCCESS                 - Reset started
    USB_MSD_DEVICE_NOT_FOUND    - No device with specified address
    USB_MSD_ILLEGAL_REQUEST     - Device is in an illegal state for reset

  Remarks:
    None
*******************************************************************************/

/*******************************************************************************
  Function:
    BYTE USBHostHubResetDevice( BYTE deviceAddress )

  Summary:
    This function starts a HID  reset.

  Description:
    This function starts a HID reset.  A reset can be
    issued only if the device is attached and not being initialized.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress - Device address

  Return Values:
    USB_SUCCESS                 - Reset started
    USB_MSD_DEVICE_NOT_FOUND    - No device with specified address
    USB_MSD_ILLEGAL_REQUEST     - Device is in an illegal state for reset

  Remarks:
    None
*******************************************************************************/

BYTE USBHostHubResetDevice( BYTE deviceAddress );

/****************************************************************************
  Function:
    BYTE USBHostHubResetDeviceWithWait( BYTE deviceAddress  )

  Description:
    This function resets a HID device, and waits until the reset is complete.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress  - Address of the device to reset.

  Return Values:
    USB_SUCCESS                 - Reset successful
    USB_HID_RESET_ERROR         - Error while resetting device
    Others                      - See return values for USBHostHIDResetDevice()
                                    and error codes that can be returned
                                    in the errorCode parameter of
                                    USBHostHIDTransferIsComplete();
                                    
  Remarks:
    None
  ***************************************************************************/

BYTE USBHostHubResetDeviceWithWait( BYTE deviceAddress, BYTE i  );

/****************************************************************************
  Function:
    BOOL EnumerateDeviceStatus( BYTE PortNumber )

  Summary:
    This function checks if the status of the enumerations process.

  Description:
    This function checks if the status of the enumerations process.  If the 
	device is successfully enumerated or there is an error, it will return 
	TRUE, else false.

  Preconditions:
    None

  Parameters:
    BYTE PortNumber  - PortNumber of the hub

  Return Values:
    USB_DEVICE_ATTACHED                 - Device is attached and running
    USB_DEVICE_DETACHED                 - No device is attached
    USB_DEVICE_ENUMERATING              - Device is enumerating
    USB_HOLDING_OUT_OF_MEMORY           - Not enough heap space available
    USB_HOLDING_UNSUPPORTED_DEVICE      - Invalid configuration or
                                            unsupported class
    USB_HOLDING_UNSUPPORTED_HUB         - Hubs are not supported
    USB_HOLDING_INVALID_CONFIGURATION   - Invalid configuration requested
    USB_HOLDING_PROCESSING_CAPACITY     - Processing requirement excessive
    USB_HOLDING_POWER_REQUIREMENT       - Power requirement excessive
    USB_HOLDING_CLIENT_INIT_ERROR       - Client driver failed to initialize
    USB_DEVICE_SUSPENDED                - Device is suspended
    Other                               - Device is holding in an error
                                            state. The return value
                                            indicates the error.

  Remarks:
    None
  ***************************************************************************/

BOOL EnumerateDeviceStatus ( BYTE PortNumber );

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
    DWORD flags         - Initialization flags
    BYTE clientDriverID - ID to send when issuing a Device Request via
                            USBHostIssueDeviceRequest(), USBHostSetDeviceConfiguration(),
                            or USBHostSetDeviceInterface().

  Return Values:
    TRUE   - We can support the device.
    FALSE  - We cannot support the device.

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostHubInitialize( BYTE address, DWORD flags, BYTE clientDriverID );

/*******************************************************************************
  Function:
     void USBHostHubTasks( void )

  Summary:
    This function performs the maintenance tasks required by HID class

  Description:
    This function performs the maintenance tasks required by the HID
    class.  If transfer events from the host layer are not being used, then
    it should be called on a regular basis by the application.  If transfer
    events from the host layer are being used, this function is compiled out,
    and does not need to be called.

  Precondition:
    USBHostHIDInitialize() has been called.

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
*******************************************************************************/

void USBHostHubTasks( void );

/*******************************************************************************
  Function:
    void USBPortInitialize( void )


  Summary:
    This function performs the enumeration of the downstream ports of a USB
    hub class device.

  Description:
    This function performs the enumeration of the downstream ports of a USB
    hub class device.
  Preconditions:
    None

  Parameters:
    
    None

  Return Values:
    None

  Remarks:
    None
*******************************************************************************/

void USBPortInitialize( void );

/*******************************************************************************
  Function:
	BYTE USBHostHubTransfer( BYTE deviceAddress, BYTE interfaceNum, 
							 WORD size, BYTE data );

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
    BYTE direction          - 1=read, 0=write
    BYTE interfaceNum       - Interface number of the device
    BYTE size               - Byte size of the data buffer
    BYTE *data              - Pointer to the data buffer

  Return Values:
    USB_SUCCESS                 - Request started successfully
    USB_HID_DEVICE_NOT_FOUND    - No device with specified address
    USB_HID_DEVICE_BUSY         - Device not in proper state for
                                  performing a transfer
    Others                      - Return values from USBHostIssueDeviceRequest(),
                                    USBHostRead(), and USBHostWrite()

  Remarks:
    None
*******************************************************************************/

BYTE USBHostHubTransfer( BYTE deviceAddress, BYTE interfaceNum, WORD size, BYTE *data );


/*******************************************************************************
  Function:
    BOOL USBHostubTransferIsComplete( BYTE deviceAddress,
                        BYTE *errorCode, DWORD *byteCount )

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

BOOL USBHostHubTransferIsComplete( BYTE deviceAddress, BYTE *errorCode, BYTE *byteCount );


/****************************************************************************
  Function:
    BYTE USBReturnCurrentPort( BYTE CurrentPort )

  Summary:
    This function returns the port number of the device.

  Description:
    This function returns the port number of the device to be called for USBHostHubDeviceDetect().

  Precondition:
    None

  Parameters:
    BYTE CurrentPort  - Port number of the device currently being configured

  Return Values:
	DeviceNumber - Port number of the device    

  Remarks:
    None
  ***************************************************************************/

BYTE USBReturnCurrentPort ( BYTE CurrentPort );

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
    by the host layer when various events occur.

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


BOOL USBHostHubEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size );




// *****************************************************************************
// *****************************************************************************
// Section: Legacy Macros 
// *****************************************************************************
// *****************************************************************************

    // This macro provides legacy support for an older API function.
#define USBHostHub_ApiDeviceDetect()                USBHostHubDeviceDetect( 1 )

    // This macro provides legacy support for an older API function.
#define USBHostHub_ApiGetReport( r, i, s, d )       USBHostHubRead( 1, r, i, s, d )

    // This macro provides legacy support for an older API function.
#define USBHostHub_ApiResetDevice()                 USBHostHubResetDeviceWithWait( 1 )

    // This macro provides legacy support for an older API function.
#define USBHostHub_ApiTransferIsComplete( e, c )    USBHostHubTransferIsComplete( 1, e, c )

#endif

