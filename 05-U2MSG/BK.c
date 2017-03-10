//*****************************************************************************************************
//                                                                                                    
//   BK.c:  Tutorial Program for PIC32
//                                                                                                    
//   Written by: Barry P. Keane, Ph.D.                                                                
//               Inotec, Incorporated                                                                
//               P.O. Box 1587                                                                        
//               Clemson, SC  29633                                                                  
//               Ph: (864) 882-1463
//               email: inotec@ieee.org
//
//                                                                                                    
//    Copyright © 2017 Inotec, Inc.
//
//    Written for PIC32 using C32 compiler 
//		Target HW: DM32                                        
//                                                                                                  
//*****************************************************************************************************
//    Journal: 
//
// 2-25-17		05-U2MSG:  Adding in routine to send a message string out the serial port.
//            In C, a string is an array of ASCII characters terminated by a NULL (last byte = 0).  The
//						NULL terminator is the only way that C knows where the string ends.  This is a dangerous
//						thing in C because an accidentally unterminated string may cause a routine to process
//						array elements that are out of bounds, so beware.  In our main loop, we will see if
//						there is data to send (U2_Send()).  If so, we send out one character per pass through
//						the main loop.
//
// 1-25-17		04-U2BUFF: Adding in buffered I/O using circular buffer.  1st Data byte received is put in
//            U2_RxBuffer[0], next in U2_RxBuffer[1] etc up to U2_RxBuffer[1023], then the next byte "rolls
//						over" and goes in U2_RxBuffer[0]...etc.  We keep in and out pointers, U2_RxBufIn and U2_RxBufOut
//						The "in" pointer always points to the next location to save a byte, while the "out" pointer
//            points to the last byte taken out.  When they are equal, there is no data that has not been processed.
//						We use a defined constant, U2_MAX_RX_BUF_LEN for the buffer length.  This is defined in BK.h.  Thus
//						we can easily change the size of the buffer for different projects.  When transmitting large files
//						over a modem, the in and out pointers could get far apart, so a big buffer is needed.  For our test
//            a very small buffer would work.  When we increment the pointers, we do a logical and ( &= ) of the 
//						pointer after incrementing.  This will give us a roll over at the max buffer size.  
//
//  1-25-17		03-U2 Adding in serial port U2 for hyperterminal
//						Connect terminal (like Hyperterminal or RealTerm). at 19,200 BAUD N,8,1 NO FLOW CONTROL
//						ASCII code of received character will be incremented by 1 and echoed back.
//						Refer to PIC32 schematic, Search for J19.  At J19, we can put in jumpers that
//						allow us to use this serial port to communicate with RS232, RS485 or USB 
//						devices.  For now we will just use RS232, so make sure that the 2 jumpers are
//						in 5-6 and 7-8 (the middle positions).  Also, we need to make a standard RS232
//						cable.  With no handshaking, we just need to connect pins 2,3 and 5 on the DE-9S
//						connector.  The cable needs to connect to J11 on the DM32 as follows:
//						Signal		J11	Color		DE-9S
//						TX(out)		4		RED			2			PC input
//						RX(in)		1		WHT			3			PC output
//						GND				5		YEL			5			GND 
//						Handshaking not needed here, but for later would be...
//						RTS(out)	3		BLU			8			PC input
//						CTS(in)		2		BLK			7			PC output
//
//						The connector on the PC has male pins (DE9-P), therefore the mating connector for 
//						our cable is female DE-9S (Socket)//						
//
//	1-24-17		02_TIMERS: Shows use of utility timers by adding RED_LED_Timer and YEL_LED_Timer
//						This program will do the following:
//						1) Turn on RED_LED for 1 sec when SW1 is pressed
//						2) Turn on YEL_LED for 2 sec when SW2 is released
//						To do this, we have to:
//						1) Add timer variables to BK.h
//						2) Add timer decrement code to the utility interrupt
//						3) Initialize the timer when LED is turned on
//						4) Turn off the LED when its timer counts down to 0.  This can be
//						   done in the main program or in the interrupt						
//
//	 1-23-17	01-LEDSW: Beginning tutorial on PIC32 for Brian
//						This program will do the following:
//						1) Turn on RED_LED when SW1 is pressed
//						2) Turn on YEL_LED when SW2 is released
//						3) Turn off the above when SW3 is pressed
//						4) Toggles GRN_LED once per second in Timer1 interrupt
//
//************************************************************************
//
//   Configuration Bits:  The following #pragma statements set up the 
//   hardware by causing actual hardware configuration bits to be 
//   programmed into the chip. 
//
//************************************************************************
#pragma config FPLLIDIV = DIV_2    	// 8 MHz osc div by 2 = 4 The boards crystal oscillator is 8 MHz determined by the
																		// 8 MHz crystal on the board.  A "Phased Locked Loop Input Divider" on the chip
																		// will first divide this by 2 giving 4 MHz
#pragma config FPLLMUL = MUL_20    	// x20 = 80MHz		Then a PLL multiplier will multiply this by 20 giving
																		// a basic oscillator frequency of 80 MHz
#pragma config FPLLODIV = DIV_1    	// 1:1 still 80 MHz  A PLL output divider can divide this down, but we
																		// leave it at 80 MHz by making this 1:1 
#pragma config FNOSC = PRIPLL				// Sets us up to use the above PLL output as the primary oscillator
																		// There can be a secondary oscillator that we can switch to, typically
																		// for low power but we're not using it.
#pragma config POSCMOD = XT					// Determined by the type of crystal we use on the board
#pragma config FPBDIV = DIV_2      	// Peripheral Bus Clock 40 MHz This determines the speed of the peripheral
																	 	// bus (where devices like serial UARTs, Timers, etc operate.  It usually
																		// can't run at the full oscillator speed.  We divide by 2 and run the peripheral
																		// bus at 40 MHz
#pragma config DEBUG = ON						// Adds some overhead for us to be able to do debugging inside IDE
#pragma config FWDTEN = OFF					// Turns off "Watchdog Timer"  We'll look at this later
#pragma config FCKSM = CSDCMD      	// Clock switching and monitoring disabled  We would enable this if we
																		// wanted to switch to a secondary oscillator and run at low power
#pragma config CP = OFF            	// Code protection off for now  Would prevent people from reading our code
																		// with their programmer, but we turn protection off			
#pragma config BWP = OFF           	// Boot flash write enabled  We could protect this for a "bootloader"
																		// but don't worry about it for now.
#pragma config PWP = OFF           	// Flash protect off:  We can prevent writing to flash but won't worry 
																		// about it for now
#pragma config OSCIOFNC = ON        // CLKO Enable
#pragma config IESO     = OFF       // Internal/External Switch-over
#pragma config FSOSCEN  = OFF       // Secondary Oscillator Enable
//#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select  These pins could alternatively be used on a different board design
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select  These are the pins that we use for programming chip



//************************************************************************
//
//   Include Files
//
//************************************************************************

#include<p32xxxx.h>					// Includes a lot of specific definitions (registers, ports etc) for
														// the chip that we are using  Chip us also be selected in IDE under Configure-Select Device
														// It gets installed with MPLAB IDE

#include<plib.h>						// Peripheral library functions that we can use.  I don't use them a lot but
														// usually do my own.  
														// This also gets installed with MPLAB IDE

#include "BK.h"							// This is our main "include" file that defines constants, variables, macros, etc
														// so that this doesn't clutter up the .c file


int main()									// All c programs have to have a main() function.  After startup, which initializes variables, etc, the
{														// program will start here.

	char TestChar;						// Use this variable for characters received, define as "char" (8-bits)

  #define FOSC 80000000			// We are defining our own constants here this is the clock speed 80,000,000 Hz
  #define FPB 40000000			// This is the peripheral bus speed 40,000,000 Hz
  
	// System initialization of general hardware and globals...I always call it SysInit();
	SysInit();  

  SYSTEMConfigPerformance(FOSC);    
	//  From DiJasio...
	//•Make sure to restore the OSC_PB_DIV setting immediately after the SYSTEMConfigPerformance() call:
	// For performance, this is important as it optimizes speed, makes it probably 10x faster than if you don't do it.  The reason is that 
	// the default is for all the racey stuff to be off (pipeline, cache, memory wait states, etc. - this makes a big difference!
	mOSCSetPBDIV( OSC_PB_DIV_2 );
	// I found this in the DiJasio book and realized that if you don't do this, the performance call above will override the FPBDIV configuration
	// setting and screw up peripheral timing 

	// Unitilize UART2.  We will use this to communicate over an RS232 serial port with either a computer running a terminal program
	// (Hyperterminal, Realterm) or some other device such as our front panel device (ELK204-7T-TC1-MT)  
	#define BR2 19200            			// Baud rate for LCDMO (Matrix Orbital Display)
  U2_Init();

  // Use muliple vectored interrupts:  This sets up the vectored interrupt handling for our program.  We will look at interrupts later
  INTEnableSystemMultiVectoredInt();
  INTEnableInterrupts();

  // Init Timer 1 for 1 msec utility interrupt
  InitTimer1();   

	// Configure UART2 Serial RX Interrupt for devices such as PC Hyperterminal
	// This will give us a receive interrupt anytime we get a character on the U2 serial port.  It is set up here for Priority 1, but this is
	// arbitrary for our trivial test setup.  With multiple interrupts, some may be more time critical, in which case we would make those
	// higher priority than less "stressful" interrupts.
	ConfigIntUART2(UART_INT_PR1 | UART_RX_INT_EN);

	// Main Loop:  All embedded systems (that run inside a device) must have a program that runs all the time, thus the infinite loop
	// Inside the loop we check for and and handle events, periodically sample I/O's, see if we get messages or need to send
	// messages.
  while (1)
  {
		// U2_Send() will send out any queued character in the buffer
		U2_Send();
		
			
    // IF SW1 pressed THEN   **************************************************SW1 Test
    if ( SW1PressedFlag && (!SW1ActionTaken_Flag) )
    {
    
      // Take action here..
			


			// Set action taken flag so this won't keep happening
      SW1ActionTaken_Flag = TRUE;
    
      
    // ELSE IF SW released THEN
    }
    else if ( !SW1PressedFlag && (SW1ActionTaken_Flag) )
    {

      // Do this after action taken...

      SW1ActionTaken_Flag = FALSE;
      
    // END IF
    }

    // IF SW2 pressed THEN   **************************************************SW2 Test
    if ( SW2PressedFlag && (!SW2ActionTaken_Flag) )
    {
      
      // Take action here...


			SW2ActionTaken_Flag = TRUE;
      
    // ELSE IF SW released THEN
    }
    else if ( !SW2PressedFlag && (SW2ActionTaken_Flag) )
    {

      // Do this after action taken...
      
      SW2ActionTaken_Flag = FALSE;

		// END IF
		}


    // IF SW3 pressed THEN   **************************************************SW3 Test
    if ( SW3PressedFlag && (!SW3ActionTaken_Flag) )
    {
      
      // Take action here...



      SW3ActionTaken_Flag = TRUE;

    // ELSE IF SW released THEN
    }
    else if ( !SW3PressedFlag && (SW3ActionTaken_Flag) )
    {

      // Do this after action taken...
      
      SW3ActionTaken_Flag = FALSE;


		// END IF
		}  
  }// END WHILE (Main Loop)
}


//************************************************************************
//
//      Function: SysInit() System Initialization
//
//      This function initializes the hardware and global variables.
//
//************************************************************************

/*
PSEUDOCODE:

*/

void SysInit(void) 
{

  unsigned int i;

  //-------------------------------------------------
  // Initialize I/O's
  //-------------------------------------------------
 
  // Set up LED outputs
  // The _TRIS stuff determines whether a particular chip pin is an output or input
  _TRISD13 = 0; // This makes PorD bit 13 an output Look at the schematic or pin diagram to see what pin number this is
  _TRISD12 = 0; // PortD bit12
  _TRISD3 = 0;  // PortD bit 3
  
  // NOTE:  Pins default to inputs so you don't have to do _TRISSxx = 1; to make them inputs    
  
	GRN_LED = LED_OFF;			// In the .h file, we have defined GRN_LED to be _LATD13 "latch D13"  We write to latch D13 to turn OFF/ON the GRN LED
	YEL_LED = LED_OFF;			// In BK.h we have defined LED_OFF to be 1 because the DM32 is wired to turn off the LED when it is high
  RED_LED = LED_OFF;
     
  // Set up Switch Inputs 
  _TRISD4 = 1;	// Not really necessary as input is default
  _TRISD5 = 1;
  _TRISD6 = 1;
  

  //-------------------------------------------------
  // Initialize global variables
  //-------------------------------------------------
	
	// NOTE:  I use a lot of global variables.  This is not considered good form in other types of programming, but in lean, time critical 
	// control code it is very appropriate and the most efficient means of communicating from one section of code to another without 
	// a lot of time consuming and unnecessary passing of parameters.  For example, in the interrupt we set SW1PressedFlag, which we can 
	// test in the main program.  This is fine because its value is never changed except in one place - the interrupt.

  //************************************************************************************** Misc Variables:
	GenExceptCnt = 0;  

  SecsCounter = 0;
	msecTimer = 0;
  
  SW1dbTimer = 0;
  SW1PressedFlag = FALSE;
  SW1ActionTaken_Flag = FALSE;
  SW2dbTimer = 0;
  SW2PressedFlag = FALSE;
  SW2ActionTaken_Flag = FALSE;
  SW3dbTimer = 0;
  SW3PressedFlag = FALSE;
  SW3ActionTaken_Flag = FALSE;  
}


//************************************************************************
//
//      Function: InitTimer1()
//
//      This timer is set up to generate a 1 msec interrupt.  This interrupt is
//			useful for many things, but primarily it drives counters that we use
//			to time intervals with 1 msec resolution.
//
//			NOTE: Timer1 is a Type A counter T1CON has 2 bits to select
//			      prescaller (Timer2 has 3 bits)
//
//************************************************************************
void InitTimer1(void)
{

  unsigned int test;
 
  TMR1 = 0;		  // clear the timer

  // TIMER RELATED:
	// FPB is 40,000,000
  // FPB period is 1/40,000,000

	// Timer1 counts PB pulses through prescaler
	// Time per count, with 8:1 prescaler is .2 usec
	// Make prescaler 8:1 by setting TCKPS<1:0> (2 bits) below to 01
	// PR1 is the desired preset for Timer1
	// For 1000 usec, this is (1000 usec/.2usec) - 1 = 5000 -  1 = 4999
  PR1 = 4999;
		
	// Configure Timer module
  // T1CON:  1000 0000 0001 0000
  //         |||       | ||  ||...TCS         Source 0=internal
  //         |||       | ||  |....TSYNC       External Clock Sync
  //         |||       | ||         
  //         |||       | ||
  //         |||       | ||.......TCKPS<1:0>: Timer Input Clock prescaler Select bits 
  //         |||       | |........"  "  "     00=1:1  01=1:8  10=1:64  11=1:256
  //         |||       |..........TGATE:      Gated Time Accumulation Enable bit 0=disabled                  
  //         |||..................TSIDL       Stop in Idle Mode bit (N/A)
  //         ||...................FRZ:        Freeze in Debug Exception Mode bit
  //         |....................TON         1 Starts the timer	
  T1CON =  0b1000000000010000;	// enabled, prescaler 1:64
	
  // Init the Timer 1 Interrupt, clear the flag, enable the source, set priority
	// These are compiler library functions
  mT1SetIntPriority(1);			// Priority is not critical here, but this woulc be a pretty important interrupt in a more complicated structure
  mT1ClearIntFlag();				// This clears the interrupt flag so that we will not immediately get a timer interrupt
  mT1IntEnable(1);					// This enables the timer interrupt, otherwise no interrupts occur
}


//--------------------------------------------------------------------------------------------------------------------
// UART2 Functions  Serial I/O such as PC connection via Hyperterminal
//--------------------------------------------------------------------------------------------------------------------


//************************************************************************
//
//      Function: U2_Init()
//
//      Initializes UART2 (RS-232) 
//
//			We are setting up for NO FLOW CONTROL meaning that the hardware is
//      connected with just 2 wires, RX and TX.  
//
//			In a situation where we use a device such as a modem that has limitations
//      on how much data flow it can handle, we use hardware flow control, sometimes
//			called hardware handshaking.  In this case we connect RTS and CTS signals.  This
//			way, either receiving device can signal that it "can't take any more data for now"
//      using its handshaking output.  The transmitting device will see this and hold
//      transmission until the receiving device indicates that it is again able to
//      take more data.
//
//			The BRG (baud rate generator) is programmed with U2BRG as calculated below
//			based on the desired baud rate (BR2) and the frequency of the peripheral bus (FPB).
//
//			The extensive comments below are taken from the device reference manual
//			and put in the documentation for convenience.  We can make changes here
//			for future projects without having to look at the manual.
//
//			We set this up to give us an interrupt on receipt of a character (RX Interrupt)
//
//			We do not use TX interrupts here, as we just send out one character
//			each time through the main loop IF the transmitter is not busy sending
//      the previous character. 
//		
//
//************************************************************************
void U2_Init(void)
{

  // Calculate Baud Rate period (Assumes BRGH is set to 1 below for high speed)
  U2BRG = (int) FPB/( (int)4*BR2) - 1;  	// U2BRG is a chip register that must be set up from this calculation in order for the
  																				// serial port to operate at the correct baud rate (BR2)
    

  //   UxMODE: UART Mode Register
  //  ------------------------------------------------------------------  ---------------------------------------------------------------  
  //  |  ON  |  FRZ  |  USIDL  |  IREN | RTSMD |   -   |   UEN<1:0>    |  | WAKE | LPBACK | ABAUD | RXINV |  BRGH | PDSEL<1:0> |  STSEL |  
  //  ------------------------------------------------------------------  ---------------------------------------------------------------
  //UxMODE = 0x8008;0xC000
  //UxMODE 	= 0b1000000000001000;    // Do this to test with handshaking off
	U2MODE	= 0b1000000000001000;
  //          ||  | ||    |.......BRGH  Depending on the speed, this might have to be 0 for slow baud rates
  //          ||  | ||............Set to 10 to use RX,TX,CTS,RTS  Set to 00 for no handshaking
  //          ||  |...............Set to 0 if RTS used for Flow Control
  //          ||..................Freeze in Debug Exception mode
  //          |...................UART Enabled

  /*
  bit 15
  ON: UART Enable bit
   1 = UART is enabled. UART pins are controlled by UART as defined by UEN<1:0> and UTXEN control bits.
   0 = UART is disabled. UART pins are controlled by corresponding PORT, LAT, and TRIS bits.
  bit 14 FRZ: Freeze in Debug Exception Mode bit
    1 = Freeze operation when CPU is in Debug Exception mode
    0 = Continue operation when CPU is in Debug Exception mode
  bit 13 USIDL: Stop in Idle Mode bit
   1 = Discontinue operation when device enters Idle mode
   0 = Continue operation in Idle mode
  bit 12 IREN: IrDA Encoder and Decoder Enable bit
    1 = IrDA is enabled
    0 = IrDA is disabled
  bit 11 RTSMD: Mode Selection for UxRTS Pin bit
    1 = UxRTS pin is in simplex mode
    0 = UxRTS pin is in flow control mode
  bit 10 unused
  bit 8-9  UEN
   11 = UxTX, UxRX, and UxBCLK pins are enabled and used; UxCTS pin is controlled by port latches
   10 = UxTX, UxRX, UxCTS, and UxRTS pins are enabled and used
   01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin is controlled by port latches
   00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/UxBCLK pins are controlled by port latches
  bit 7 WAKE: Enable Wake-up on Start bit Detect During Sleep Mode bit
   1 = Wake-up enabled
   0 = Wake-up disabled
  bit 6 LPBACK: UART Loopback Mode Select bit
   1 = Enable Loopback mode
   0 = Loopback mode is disabled
  bit 5 ABAUD: Auto Baud Enable bit
   1 = Input to Capture module from UxRX pin
   0 = Input to Capture module from ICx pin
  bit 4 RXINV: Receive Polarity Inversion bit
  1 = UxRX idle state is ‘0’
  0 = UxRX idle state is ‘1
  bit 3 BRGH: High Baud Rate Enable bit
  1 = High speed mode – 4x baud clock enabled
  0 = Standard speed mode – 16x baud clock enabled
  bit 2-1 PDSEL<1:0>: Parity and Data Selection bits
   11 = 9-bit data, no parity
   10 = 8-bit data, odd parity
   01 = 8-bit data, even parity
   00 = 8-bit data, no parity  This is the most common selection
  bit 0 STSEL: Stop Selection bit
   1 = 2 Stop bits
   0 = 1 Stop bit  	This is the most common selection
  */


  //   UXSTA: UARTX Status and Control Register

  //  NOTE: Upper 16 bits can be used for automatic address detection, but are not used at this time

  //  ------------------------------------------------------------------  ---------------------------------------------------------------  
  //  | UTXISEL<1:0> | UTXINV | URXEN | UTXBRK | UTXEN | UTXBF |  TRMT |  |URXISEL<1:0>| ADDEN | RIDLE |  PERR  | FERR  | OERR  | URXDA |  
  //  ------------------------------------------------------------------  ---------------------------------------------------------------
  //U1STA = 0x1510;
  //          0001010100010000
  U2STA	=   0b0001010100010000;
  //             | | |   |....Receiver is Idle
  //             | | |........Transmit shift register is empty and transmit buffer is empty (the last transmission has completed)
  //             | |..........Tx Enable
  //             |............Rx Enable 
  /*
  bit 15-14 UTXISEL<1:0>: Tx Interrupt Mode Selection bits
   11 = Reserved, do not use
   10 = Interrupt is generated when the Transmit buffer becomes empty
   01 = Interrupt is generated when all characters are transmitted
   00 = Interrupt is generated when the Transmit buffer contains at least one empty space
  bit 13 UTXINV: Transmit Polarity Inversion bit
   If IrDA mode is disabled (i.e., IREN (UxMOD<12>) is ‘0’)
   1 = UxTX idle state is ‘0’
   0 = UxTX idle state is ‘1’
   If IrDA mode is enabled (i.e., IREN (UxMOD<12>) is ‘1’)
   1 = IrDA encoded UxTX Idle state is ‘1’
   0 = IrDA encoded UxTX Idle state is ‘0’
  bit 12 URXEN: Receiver Enable bit
   1 = UARTx receiver is enabled, UxRX pin controlled by UARTx (if ON = 1)
   0 = UARTx receiver is disabled, the UxRX pin is ignored by the UARTx module. UxRX pin controlled by PORT  
  bit 11 UTXBRK: Transmit Break bit
   1 = UxTX pin is driven low, regardless of transmitter state
   0 = UxTX pin operates normally
  bit 10 UTXEN: Transmit Enable bit
   1 = UART transmitter enabled, UxTX pin controlled by UART (if UARTEN = 1)
   0 = UART transmitter disabled, any pending transmission is aborted and buffer is reset. UxTX pin controlled
       by PORT.
  bit 9 UTXBF: Transmit Buffer Full Status bit (Read Only)
   1 = Transmit buffer is full
   0 = Transmit buffer is not full, at least one more data word can be written
  bit 8 TRMT: Transmit Shift Register is Empty bit (Read Only)
   1 = Transmit shift register is empty and transmit buffer is empty (the last transmission has completed)
   0 = Transmit shift register is not empty, a transmission is in progress or queued in the transmit buffer
  bit 7-6 URXISEL<1:0>: Receive Interrupt Mode Selection bit
   11 =Interrupt flag bit is set when Receive Buffer is full (i.e., has 4 data characters)
   10 =Interrupt flag bit is set when Receive Buffer is 3/4 full (i.e., has 3 data characters)
   0x =Interrupt flag bit is set when a character is received
  bit 5 ADDEN: Address Character Detect (bit 8 of received data = 1)
   1 = Address Detect mode enabled. If 9-bit mode is not selected, this control bit has no effect.
   0 = Address Detect mode disabled
  bit 4 RIDLE: Receiver Idle bit (Read Only)
   1 = Receiver is Idle
   0 = Data is being received
  bit 3 PERR: Parity Error Status bit (Read Only)
   1 = Parity error has been detected for the current character
   0 = Parity error has not been detected
  bit 2 FERR: Framing Error Status bit (Read Only)
   1 = Framing Error has been detected for the current character
   0 = Framing Error has not been detected
  bit 1 OERR: Receive Buffer Overrun Error Status bit (Read/Clear Only)
   1 = Receive buffer has overflowed
   0 = Receive buffer has not overflowed
  bit 0 URXDA: Receive Buffer Data Available bit (Read Only)
   1 = Receive buffer has data, at least one more character can be read
   0 = Receive buffer is empty
  */

	IEC1bits.U2RXIE = 1;					// Enable RX interrupt
  //NOTE: For U1...IEC0bits.U1RXIE = 1;
	//IPC8 |= 0b000000000000000000000000000ppp00;							// Must set priority to something
	IPC8 |= 	0b00000000000000000000000000000100;								
	//NOTE:  Use IPC6 for U1
	
  // To cause an interrupt when TX buffer is empty
	//IEC0bits.U1TXIE = 1;					// Enable TX interrupt.	
}


//************************************************************************
//
//      Function: U2_Flush()
//
//      Initializes UART2
//
//************************************************************************
void U2_Flush(void)
{
	int idx;

	// Clear the RX interrupt Flag
	INTClearFlag(INT_SOURCE_UART_RX(UART2));

	// Clear the TX interrupt Flag
	INTClearFlag(INT_SOURCE_UART_TX(UART2));

	// reset receive buffer's work indices
  U2_RxBufIn = U2_RxBufOut = U2_RxLen = 0;

	// reset transmit buffer's work indices
  U2_TxBufIn = U2_TxBufOut = U2_TxLen = 0;
	
	// reset each buffer's xfer character
  U2_RxChar = U2_TxChar = 0;

	// reset receive/transmit error flag
  U2_RxError = U2_TxError = 0;

	for(idx = 0; idx < U2_MAX_RX_BUF_LEN; idx++)  
		U2_RxBuffer[idx] = 0;

	for(idx = 0; idx < U2_MAX_TX_BUF_LEN; idx++)
		U2_TxBuffer[idx] = 0;

}


// -----------------------------------------------------------------------------
//	U2_Msg - Queues a character message string to UART
//
//  NOT BLOCKING - characters are sent by U2_Send() in main loop
// -----------------------------------------------------------------------------
void U2_Msg( char* msg)
{										// allocate static character
	static char ch;
	static int max_len;

  while( *msg )			// loop until *msg == '\0' the  end of the string
	{
    U2_Put( *msg++);	// send the character and point to the next one 
	}
}


// -----------------------------------------------------------------------------
// U2_Put
//
// Puts in Tx buffer and bumps pointer
// -----------------------------------------------------------------------------
void U2_Put( char ch)
{
	U2_TxBuffer[U2_TxBufIn] = ch;

	// advance and modulate buffer index
	U2_TxBufIn++; U2_TxBufIn &= (U2_MAX_TX_BUF_LEN-1);

	// is there a transmit buffer overrun?
	if(U2_TxBufIn == U2_TxBufOut) 
		U2_TxError = UART_OVERFLOW;
}


// -----------------------------------------------------------------------------
// U2_Send() - if required put out one character and get return
//
//	NOT BLOCKING - just sends one character per main loop
//
// -----------------------------------------------------------------------------
void U2_Send(void)
{
	char ch;

	// Handle the transmit stack one character at a time.
	if(U2_TxBufIn != U2_TxBufOut)  
	{
	  // get transmit character from buffer 	
	 	U2_TxChar = U2_TxBuffer[U2_TxBufOut];
	
	  // write character to the uart
		U2_TxTimer = 1000;
	
    while(!UARTTransmitterIsReady(UART2)&&U2_TxTimer);
		  UARTSendDataByte(UART2, U2_TxChar);
	
		// advance and modulate buffer index
		U2_TxBufOut++; U2_TxBufOut &= (U2_MAX_TX_BUF_LEN-1);

		// increment character transmitted counter
	  U2_TxLen++;
	}  
}


//************************************************************************
//
//      Function: U2_ProcKeyEntry
//
//      A single character is received for each key pressed
//
//************************************************************************
void U2_ProcKeyEntry(void)
{

	char CharRecd;

	// Get character from buffer	
	CharRecd = U2_RxBuffer[U2_RxBufOut];

	// advance and modulate buffer index
	U2_RxBufOut++; U2_RxBufOut &= (U2_MAX_RX_BUF_LEN-1);

	if ( CharRecd == 'A')
	{
		// Top Left Key pressed...
		BREAK();
	}
	else if ( CharRecd == 'B')
	{	
		// Up Arrow Key pressed...
		BREAK();
	}
	else if ( CharRecd == 'C')
	{	
		// Right Arrow Key pressed...
		BREAK();
	}
	else if ( CharRecd == 'D')
	{	
		// Left Arrow Key pressed...
		BREAK();
	}
	else if ( CharRecd == 'E')
	{	
		// Enter Key pressed...
		BREAK();
	}
	else if ( CharRecd == 'G')
	{	
		// Bottom Left Key pressed...
		BREAK();
	}
	else if ( CharRecd == 'H')
	{	
		// Down Arrow Key pressed...
		BREAK();
	}

}




//************************************************************************
//
//      Function: msecDelay(n)
//
//      This "convenience function provides a delay (blocking) of n milliseconds 
//      within a program.  Do not use if multi-millisecond blocking is
//			problematic.  It uses Timer1
//
//************************************************************************
void msecDelay(unsigned int n)
{
  msecTimer = n;
  while (msecTimer);
}


//--------------------------------------------------------------------------------------------------------------------
// Interrupts:  Many different sources can cause an interrupt on the PIC32.  When this happens, the main program is
// paused and execution comes here to one of these interrupt service routines.  At the end of the routine, execution
// returns to where we left off in the main program.  We keep the interrupt very small, no blocking or waiting so
// that the main program loops fast and is responsive.
//
//--------------------------------------------------------------------------------------------------------------------



//************************************************************************
//
//      Function: TIMER1 Interrupt Service Routine
//
//      This timer is set to interrupt every 1 msec.  
//
//************************************************************************
void __ISR( _TIMER_1_VECTOR, ipl1) T1InterruptHandler( void)

{
	
  //-------------------------------------------------
	// Millisecond timers:
  //-------------------------------------------------

  // Decrement the utility millisecond timer to zero, then turn OFF
  if (RED_LED_Timer)
	{
    RED_LED_Timer--;
		if ( !RED_LED_Timer )
			RED_LED = LED_OFF;
	}

  // Decrement the utility millisecond timer to zero
  if (YEL_LED_Timer)
	{
    YEL_LED_Timer--;
		if ( !YEL_LED_Timer )
			YEL_LED = LED_OFF;
	}

  // Decrement the utility millisecond timer to zero
  if (msecTimer)
     msecTimer--;

	// These timers can be disabled... Set the timer to TIMER_DISABLE (Defined as 0xFFFFFFFF) and they will not operate
	if ( (TestTimer) && (TestTimer < TIMER_DISABLE) )
		TestTimer--;


	// NOTE: We are not using either of the above timers yet	

  //-------------------------------------------------
	// Seconds counter:
  //-------------------------------------------------

  ++SecsCounter;

  if ( SecsCounter == 1000 )
  {

    SecsCounter = 0;
		
    // Handle any 1 sec events or counters...
	
		GRN_LED = !GRN_LED;					// The ! will toggle the value, so the GRN_LED will toggle once per second

  }


  //-------------------------------------------------
	// Switch test and debounce section:
  //-------------------------------------------------

  //-------------------------------------------------SW1
  // IF SW1 NOT pressed THEN
  if ( SW1 != SW_PRESSED ) 
  {
    // Clear debounce timer
    SW1dbTimer = 0;

    // Clear the SW pressed flag
    SW1PressedFlag = FALSE;

  // Else
  }  
  else
  {

    // Increment db timer (up to max)
    if ( SW1dbTimer < 255 )
      ++SW1dbTimer;

  // END IF
  }

  //-------------------------------------------------SW2
  // IF SW2 NOT pressed THEN
  if ( SW2 != SW_PRESSED ) 
  {
    // Clear debounce timer
    SW2dbTimer = 0;

    // Clear the SW pressed flag
    SW2PressedFlag = FALSE;

  // Else
  }  
  else
  {

    // Increment db timer (up to max)
    if ( SW2dbTimer < 255 )
      ++SW2dbTimer;
  
  // END IF
  }

  //-------------------------------------------------SW3
  // IF SW3 NOT pressed THEN
  if ( SW3 != SW_PRESSED ) 
  {
    // Clear debounce timer
    SW3dbTimer = 0;

    // Clear the SW pressed flag
    SW3PressedFlag = FALSE;

  // Else
  }  
  else
  {

    // Increment db timer (up to max)
    if ( SW3dbTimer < 255 )
      ++SW3dbTimer;

  // END IF
  }	

  // If a Timer has reached debounce time THEN set its flag

  //-------------------------------------------------SW1
  if ( SW1dbTimer > DB_TIME )
  {

    // Set SW pressed flag
    SW1PressedFlag = TRUE;

  // END IF
  }

  //-------------------------------------------------SW2
  if ( SW2dbTimer > DB_TIME )
  {

    // Set SW pressed flag
    SW2PressedFlag = TRUE;

  // END IF
  }

  //-------------------------------------------------SW3
  if ( SW3dbTimer > DB_TIME )
  {	
    // Set SW pressed flag
    SW3PressedFlag = TRUE;
	}

	
	// Clear the interrupt flag
	_T2IF = 0;
	
	// Clear the interrupt flag	
	IFS0bits.T1IF = 0;							//		Use this for Timer2_T2IF = 0;


} // END Timer Interrupt


// -----------------------------------------------------------------------------
// UART2 interrupt handler
// it is set at priority level 2
//
// UART2 is for PC/Bluetooth
// -----------------------------------------------------------------------------
void __ISR(_UART2_VECTOR, ipl2) u2_int_handler(void)
{
	// Is this an RX interrupt?  Should be since we only enabled RX interrupts

	if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
	{
		// Clear the RX interrupt Flag
		INTClearFlag(INT_SOURCE_UART_RX(UART2));

		// reads UART#2 receiver until empty
		while (U2STAbits.URXDA)
		{	
			// fetch next character from UART#2
			U2_RxChar = UARTGetDataByte(UART2);

			// count a character as received
			U2_RxLen++;

			// store char into receiver buffer
			U2_RxBuffer[U2_RxBufIn] = U2_RxChar;

			// advance and modulate buffer index
			U2_RxBufIn++; U2_RxBufIn &= (U2_MAX_RX_BUF_LEN-1);

			// buffer overflow trumps all errors
			if (U2_RxBufIn == U2_RxBufOut) 
				U2_RxError = UART_OVERFLOW;   

			// IF this is CR, set message received flag
			if ( U2_RxChar == CR )
				U2_MsgRecd = TRUE;										
		}									
	}
	
	// Is this a TX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_TX(UART2)))
	{
		// Clear the TX interrupt Flag
		INTClearFlag(INT_SOURCE_UART_TX(UART2));
	}
}


//************************************************************************
//
//      Function: General Exception Handler
//
//      Control branches here on all exceptions.  The following code
//      gets the code and address for diagnostics.
//			
//			We can get here when an array gets out of bounds
//			I accidentally tried to read flash memory locations
// 			using "physical" memory instead of "virtual" memory and that 
//      causes a general exception (good way to get an exception
//      on purpose for testing.
//
//		 	In a real system we can use the watchdog timer (WDT) to cause
// 			a reset when we hang here.  This can be tested on reset so that
// 			we know that it happened.
//
//************************************************************************
void _general_exception_handler(unsigned cause, unsigned status)     
{

	char StrToWrite[100];
  int *p;
  
	excep_code = (cause & 0x0000007C) >> 2;
  excep_addr = __builtin_mfc0(_CP0_EPC, _CP0_EPC_SELECT);
  if ((cause & 0x80000000) != 0)
		excep_addr += 4;	
	
	// Write code and address to flash just below checksum  
  p = (int*) 0x1d07FFF4;
  NVMWriteWord((void*)(p), excep_code);
  p = (int*) 0x1d07FFF8;
  NVMWriteWord((void*)(p), excep_addr);
  
	
  // Try to save info to DL file...
  
	// Attempt to open file for append    
  //FS_DLWritePointer = FSfopen (DL_FileName, "a");
  
  // Write excep_code and excep_addr to DL file
	//sprintf(StrToWrite, "\r\nECODE: %X  EADDR: %X\r\n", excep_code,excep_addr);
	
	// Write line to file
	//FSfprintf ( FS_DLWritePointer, StrToWrite );

  // Close file
  //FSfclose (FS_DLWritePointer);

	// Hang here
	while (1); 
} 



