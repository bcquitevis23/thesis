/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.
 Software License Agreement:
 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.
 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
********************************************************************
 File Description:
 Change History:
  Rev   Description
  1.0   Initial release
  2.1   Updated for simplicity and to use common
                     coding style
********************************************************************/
#include "USB/usb.h"
#include "USB/usb_host_msd.h"
#include "USB/usb_host_msd_scsi.h"
#include "usb_host_hub.h"
#include "fatfs/ff.h"


// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************

#ifdef __C30__
    #define PLL_96MHZ_OFF   0xFFFF
    #define PLL_96MHZ_ON    0xF7FF


    // Configuration Bit settings  for an Explorer 16 with USB PICtail Plus
    //      Primary Oscillator:             HS
    //      Internal USB 3.3v Regulator:    Disabled
    //      IOLOCK:                         Set Once
    //      Primary Oscillator Output:      Digital I/O
    //      Clock Switching and Monitor:    Both disabled
    //      Oscillator:                     Primary with PLL
    //      USB 96MHz PLL Prescale:         Divide by 2
    //      Internal/External Switch Over:  Enabled
    //      WDT Postscaler:                 1:32768
    //      WDT Prescaler:                  1:128
    //      WDT Window:                     Non-window Mode
    //      Comm Channel:                   EMUC2/EMUD2
    //      Clip on Emulation Mode:         Reset into Operation Mode
    //      Write Protect:                  Disabled
    //      Code Protect:                   Disabled
    //      JTAG Port Enable:               Disabled

    #if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GB210__)
        _CONFIG2(FNOSC_PRIPLL & POSCMOD_HS & PLL_96MHZ_ON & PLLDIV_DIV2) // Primary HS OSC with PLL, USBPLL /2
        _CONFIG1(JTAGEN_OFF & FWDTEN_OFF & ICS_PGx2)   // JTAG off, watchdog timer off
    #elif defined(__PIC24FJ64GB004__)
        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ256GB106__)
        _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
        _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV3 & IOL1WAY_ON)
    #elif defined(__PIC24FJ256DA210__)
        _CONFIG1(FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
    #endif
#elif defined( __PIC32MX__ )
    #pragma config UPLLEN   = ON            // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_15        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLODIV = DIV_1         // PLL Output Divider
    #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
    #pragma config FWDTEN   = OFF           // Watchdog Timer
    #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
    //#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF           // CLKO Enable
    #pragma config POSCMOD  = HS            // Primary Oscillator
    #pragma config IESO     = OFF           // Internal/External Switch-over
    #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL        // Oscillator Selection
    #pragma config CP       = OFF           // Code Protect
    #pragma config BWP      = OFF           // Boot Flash Write Protect
    #pragma config PWP      = OFF           // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
    #pragma config DEBUG    = ON            // Background Debugger Enable

#else


    #error Cannot define configuration bits.

#endif
    
BOOL HubAttached;  // flag if hub device is attached
BYTE DeviceNumber;  
USB_EVENT event;
BYTE HubStatus;
BYTE driveNumber;
BYTE MSDAttached, MSD1Attached, MSD2Attached, MSD3Attached, MSD4Attached;	// MSD Device attached flag
BYTE MSD1Mounted, MSD2Mounted, MSD3Mounted, MSD4Mounted; // MSD Mount flag
BYTE deviceAddress;
BYTE volume;
FRESULT res;

#define USB_MAX_DEVICES 5
#define MAX_ALLOWED_CURRENT	(500)

int main(void)
{
    
    FATFS fatfs[_VOLUMES];
    
    #if defined(__PIC32MX__)
        {
            int  value;
    
            value = SYSTEMConfigWaitStatesAndPB( GetSystemClock() );
    
            // Enable the cache for the best performance
            CheKseg0CacheOn();
    
            INTEnableSystemMultiVectoredInt();
    
            value = OSCCON;
            while (!(value & 0x00000020))
            {
                value = OSCCON;    // Wait for PLL lock to stabilize
            }
        }
			DBINIT();

    #endif

	// Initialize variables
    HubAttached = FALSE;
	MSD1Attached = 0;
	MSD2Attached = 0;
	MSD3Attached = 0;
	MSD4Attached = 0;
	MSDAttached = 0;
	MSD1Mounted = 0;
	MSD2Mounted = 0;
	MSD3Mounted = 0;
	MSD4Mounted = 0;
    
    //Initialize the stack
    USBInitialize(0);
    
    #if defined(DEBUG_MODE)
        // PPS - Configure U2RX - put on pin 49 (RP10)
        RPINR19bits.U2RXR = 10;

        // PPS - Configure U2TX - put on pin 50 (RP17)
        RPOR8bits.RP17R = 5;

        UART2Init();
    #endif
	mJTAGPortEnable(1);

    while(1) {
        
        DBPRINTF("USB FILE TRANSFER HUB\n\n\n");

		DeviceNumber = 0;

        //USB stack process function
        USBTasks(DeviceNumber);
            
        //if no hub and msd devices are plugged in
        while (!USBHostHubDeviceDetect(1)) {
		//	DBPRINTF("deviceAddress = %x\n", deviceAddress);
            USBTasks(DeviceNumber);
        } 

        //if hub is plugged in
        if(USBHostHubDeviceDetect(1)) {
            HubAttached = TRUE;
            event = EVENT_HUB_ATTACH;
            DBPRINTF("Hub Device Attached\n");
                //Just sit here until the device is removed.
                while(HubAttached == TRUE) {
                    USBTasks(DeviceNumber);
					if (MSDAttached) {
					for (DeviceNumber = 1; DeviceNumber < USB_MAX_DEVICES; DeviceNumber ++) {					
						driveNumber = DeviceNumber - 1;
						volume = CurrentPort - 1;

						if (DeviceNumber == 1) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD1Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x\n", CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD1Mounted = 1;
						MSD1Attached = 1;
						} // if usbhostmsdscsi
						} // if devicenumber 1

						else if (DeviceNumber == 2) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD2Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x\n", CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD2Mounted = 1;
						MSD2Attached = 1;
						} // if usbhostmsdscsi
						} // if devicenumber 2

						else if (DeviceNumber == 3) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD3Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x Address %x\n", driveNumber, CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD3Mounted = 1;
						MSD3Attached = 1;
						} // if usbhostmsdscsi
						} // if devicenumber 3

						else if (DeviceNumber == 4) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD4Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x Address %x\n", driveNumber, CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD4Mounted = 1;
						MSD4Attached = 1;
						} // if usbhostmsdscsi
						} // if devicenumber 4

					} // for DeviceNumber
					DeviceNumber = 0;
					}
				} // while HubAttached
        } // while USBHostHubDeviceDetect 
    return 0;
} // while (1)
} // main


/****************************************************************************
  Function:
    BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event,
                void *data, DWORD size )

  Summary:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.

  Description:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.  If the application is able to handle the event, it
    returns TRUE.  Otherwise, it returns FALSE.

  Precondition:
    None

  Parameters:
    BYTE address    - Address of device where event occurred
    USB_EVENT event - Identifies the event that occured
    void *data      - Pointer to event-specific data
    DWORD size      - Size of the event-specific data

  Return Values:
    TRUE    - The event was handled
    FALSE   - The event was not handled

  Remarks:
    The application may also implement an event handling routine if it
    requires knowledge of events.  To do so, it must implement a routine that
    matches this function signature and define the USB_HOST_APP_EVENT_HANDLER
    macro as the name of that function.
  ***************************************************************************/

BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    switch( event )
    {
        case EVENT_VBUS_REQUEST_POWER:
            // The data pointer points to a byte that represents the amount of power
            // requested in mA, divided by two.  If the device wants too much power,
            // we reject it.
			if (((USB_VBUS_POWER_EVENT_DATA*)data)->current <= (MAX_ALLOWED_CURRENT / 2))
            {
                return TRUE;
            }
            else
            {
                DBPRINTF( "\n***** USB Error - device requires too much current *****\n" );
            }
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // Turn off Vbus power.
            // This means that the device was removed
            HubAttached = FALSE;
            return TRUE;
            break;
            
        case EVENT_HUB_ATTACH:
            // Hub device is attached
            HubAttached = TRUE;
            return TRUE;
            break;

		case EVENT_ATTACH:
			// USB device is attached
			DBPRINTF("MSD Device at %x\n", CurrentPort);
			MSDAttached = 1;
			return TRUE;
			break;

		case EVENT_DETACH:
			// USB device is detached
			DBPRINTF("MSD Device at %x Detached\n", CurrentPort);
			volume = CurrentPort - 1;
			switch (CurrentPort){
				case 1:
					MSD1Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Mounted\n", volume);
						MSD1Mounted = 0;
					}
					break;

				case 2:
					MSD2Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Mounted\n", volume);
						MSD2Mounted = 0;
					}
					break;

				case 3:
					MSD3Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Mounted\n", volume);
						MSD3Mounted = 0;
					}				
					break;

				case 4:
					MSD4Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Mounted\n", volume);
						MSD4Mounted = 0;
					}
					break;

				default:
					break;					

			}
			return TRUE;
			break;

        case EVENT_UNSUPPORTED_DEVICE:
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            //UART2PrintString( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            //UART2PrintString( "\r\n***** USB Error - client driver initialization error *****\r\n" );
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            //UART2PrintString( "\r\n***** USB Error - out of heap memory *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            //UART2PrintString( "\r\n***** USB Error - unspecified *****\r\n" );
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;
}

static unsigned int _excep_code; 
static unsigned int _excep_addr; 
// this function overrides the normal _weak_ generic handler 
void _general_exception_handler(void) 
{ 
 unsigned int mod_addr; 
 asm volatile("mfc0 %0,$13" : "=r" (_excep_code)); 
 asm volatile("mfc0 %0,$14" : "=r" (_excep_addr)); 
 // Skip instruction causing the exception 
 _excep_code = (_excep_code & 0x0000007C) >> 2; 
 mod_addr = _excep_addr + 4; 
 asm volatile("mtc0 %0,$14" :: "r" (mod_addr)); 
}
