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


#define UART_MODULE_ID UART2 // PIM is connected to Explorer through UART2 module
#define DESIRED_BAUDRATE (9600) //The desired BaudRate
    
BOOL HubAttached;  // flag if hub device is attached
BYTE DeviceNumber;  
USB_EVENT event;
BYTE HubStatus;
BYTE driveNumber;
BYTE MSDAttached, MSD1Attached, MSD2Attached, MSD3Attached, MSD4Attached;	// MSD Device attached flag
BYTE MSD1Mounted, MSD2Mounted, MSD3Mounted, MSD4Mounted; // MSD Mount flag
BYTE deviceAddress[_VOLUMES];
BYTE volume;
FRESULT res;
void BTINIT(void);
void GetBTCommand(const char choice);
UINT32 GetMenuChoice(void);
void SendDataBuffer(const char *buffer, UINT32 size);

#define USB_MAX_DEVICES 5
#define MAX_ALLOWED_CURRENT	(500)

int main(void)
{
    int i;
    FATFS fatfs[_VOLUMES];
    /**
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
	****/

	BTINIT();
	DBINIT();

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

//        UART2Init();
    #endif
	mJTAGPortEnable(1);

    while(1) {
        
        DBPRINTF("USB FILE TRANSFER HUB\n\n\n");

		DeviceNumber = 0;
		for (i=0; i<_VOLUMES;i++) {
			deviceAddress[i] = 0;
		}

        //USB stack process function
        USBTasks(0);
            
        //if no hub and msd devices are plugged in
        while (!USBHostHubDeviceDetect(1)) {
		//	DBPRINTF("deviceAddress = %x\n", deviceAddress);
            USBTasks(0);
        } 

        //if hub is plugged in
        if(USBHostHubDeviceDetect(1)) {
            HubAttached = TRUE;
            event = EVENT_HUB_ATTACH;
            DBPRINTF("Hub Device Attached\n");
                //Just sit here until the device is removed.
                while(HubAttached == TRUE) {
                    USBTasks(0);
					if (MSDAttached) {
					for (i = 0; i < _VOLUMES; i++) {					
						driveNumber = i;
						volume = CurrentPort - 1;

						if ((deviceAddress[i] != 0) && (i == volume)) {
						
						if (i == 0) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD1Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x\n", CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD1Mounted = 1;
						} // if usbhostmsdscsi
						} // if i == 0

						else if (i == 1) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD2Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x\n", CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD2Mounted = 1;
						} // if usbhostmsdscsi
						} // if i == 1

						else if (i == 2) {
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD3Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x Address %x\n", driveNumber, CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD3Mounted = 1;
						} // if usbhostmsdscsi
						} // if i == 2

						else if (i == 3)
						if(USBHostMSDSCSIMediaDetect(driveNumber) && MSD4Mounted == 0) {
						DBPRINTF("MSD Device Attached in Port %x Address %x\n", driveNumber, CurrentPort);
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: MSD Device Mounted\n", volume);
							} // if res
						MSD4Mounted = 1;
						} // if usbhostmsdscsi
						} // if i == 3
				
					} // for i
					i = 0;
					} // if msdattached
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
			// USB device is detached
			DBPRINTF("MSD Device at %x Attached\n", CurrentPort);
			switch (CurrentPort){
				case 1:
					MSD1Attached = 1;
					break;

				case 2:
					MSD2Attached = 1;
					break;

				case 3:
					MSD3Attached = 1;
					break;

				case 4:
					MSD4Attached = 1;
					break;

				default:
					break;					

			}

				deviceAddress[CurrentPort] = CurrentPort + 1;
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
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD1Mounted = 0;
					}
					break;

				case 2:
					MSD2Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD2Mounted = 0;
					}
					break;

				case 3:
					MSD3Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD3Mounted = 0;
					}				
					break;

				case 4:
					MSD4Attached = 0;
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD4Mounted = 0;
					}
					break;

				default:
					break;					

			}

				deviceAddress[CurrentPort] = 0;

			if ((MSD1Attached == 0) && (MSD2Attached == 0) && (MSD3Attached == 0) && (MSD4Attached == 0)) {
				MSDAttached = 0;
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

void BTINIT (void) {
	#if defined (__32MX220F032D__) || defined (__32MX250F128D__)
    PPSInput(2,U2RX,RPB5); // Assign RPB5 as input pin for U2RX
    PPSOutput(4,RPB0,U2TX); // Set RPB0 pin as output for U2TX
    #elif defined (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)
    PPSInput(2,U1RX,RPF4); // Assign RPF4 as input pin for U1RX
    PPSOutput(2,RPF5,U1TX); // Set RPF5 pin as output for U1TX
    #endif

    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above.
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // Explorer-16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
    //mJTAGPortEnable(DEBUG_JTAGPORT_OFF);

    mPORTDClearBits(BIT_0); 		// Turn off RA7 on startup.
    mPORTDSetPinsDigitalOut(BIT_0);	// Make RA7 as output.

    // Configure UART module, set buad rate, turn on UART, etc.
    UARTConfigure(UART_MODULE_ID, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART_MODULE_ID, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART_MODULE_ID, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART_MODULE_ID, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART_MODULE_ID, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART_MODULE_ID), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART_MODULE_ID), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART_MODULE_ID), INT_SUB_PRIORITY_LEVEL_0);

    // Enable multi-vector interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

}

void GetBTCommand(const char choice) {

	switch(choice) {
		case 1:
			if (MSD1Attached) {
				SendDataBuffer("MSD 1 Attached\n", strlen("MSD 1 Attached\n"));
            }
			else {
				SendDataBuffer("MSD 1 Not Attached\n", strlen("MSD 1 Not Attached\n"));
			}
			break;

		case 2:
			if (MSD2Attached) {
				SendDataBuffer("MSD 2 Attached\n", strlen("MSD 2 Attached\n"));
			}
			else {
				SendDataBuffer("MSD 2 Not Attached\n", strlen("MSD 2 Not Attached\n"));
			}
			break;

		case 3:
			if (MSD3Attached) {
				SendDataBuffer("MSD 3 Attached\n", strlen("MSD 3 Attached\n"));
			}
			else {
				SendDataBuffer("MSD 3 Not Attached\n", strlen("MSD 3 Not Attached"));
			}
			break;

		case 4:
			if (MSD4Attached) {
				SendDataBuffer("MSD 4 Attached\n", strlen("MSD 4 Attached\n"));
			}
			else {
				SendDataBuffer("MSD 4 Not Attached\n", strlen("MSD 4 Not Attached\n"));
			}
			break;

		default:
			break;

	}

}

UINT32 GetMenuChoice(void)
{
    UINT8  menu_item;
	menu_item = 0;

    if(UARTReceivedDataIsAvailable(UART_MODULE_ID)) {

    menu_item = UARTGetDataByte(UART_MODULE_ID);

    menu_item -= '0';
	}
    return (UINT32)menu_item;
}

// *****************************************************************************
// void UARTTxBuffer(char *buffer, UINT32 size)
// *****************************************************************************
void SendDataBuffer(const char *buffer, UINT32 size)
{
    while(size)
    {
        while(!UARTTransmitterIsReady(UART_MODULE_ID))
            ;

        UARTSendDataByte(UART_MODULE_ID, *buffer);

        buffer++;
        size--;
    }

    while(!UARTTransmissionHasCompleted(UART_MODULE_ID))
        ;
}

// UART 2 interrupt handler, set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART_MODULE_ID)))
	{
            // Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART_MODULE_ID));

            // Echo what we just received.
            GetBTCommand(UARTGetDataByte(UART_MODULE_ID));

            
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART_MODULE_ID)) )
	{
            INTClearFlag(INT_SOURCE_UART_TX(UART_MODULE_ID));
	}
}
