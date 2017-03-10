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
/* 2-1-17			07-STATEMACH:
Very powerful topic: State Machines.  State machines, sometimes called sequential state machines give us
a simple, fast and efficient way of multitasking.  The main loop has to be non blocking.  The CPU can't 
wait in the main loop for the UART to finish sending a character or the user to press a key, etc.
If so, nothing else can happen.  Instead the main loop will check each time through to see if the state 
of a machine has changed, for example, we were waiting for a key press and we got one.  Then the state 
machine code will go to the next state. 

Check out StateMach.flo in the FLO folder.  It is overkill to use a state machine for such a simple 
thing, but it illustrates the point.  While the main program is toggling the YEL and RED LEDs, the
state machine will handle a menu where the user can change LEDs and flash times.

Take a look at ftp.flo and polling.flo to see what more complex state machines look like.  The locomotive
project requires running both of these simultaneously when we have to FTP a file to the server
and still keep polling for data.


*/


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

	// Initialize LCD and send sign-on
	LCDMO_Init();
	LCDMO_Color(1,255,1);
	LCDMO_ClearScreen();
	LCDMO_UnderlineOFF();
	LCDMO_BlinkOFF();
	U2_Msg("Set Contrast");					// Use UP/DOWN arrows to adjust contract during 5 sec startup
	StartUpTimer = 5000;						// Sets the startup time to 5 sec
																	// StartUpTimer has to be added to msec timers in timer interrupt


	// Let's still do this:  Allow contrast adjustment for 5 sec before starting main loop
  while (StartUpTimer)
  {
		// If a character is in buffer then get it			// Buffer is overkill for LCDMO, but for consistency keep using it
		if ( U2_RxBufIn != U2_RxBufOut )
		{
			LCDMO_CharRecd = U2_RxBuffer[U2_RxBufOut];		// NOTE:  Made LCDMO_CharRecd a global
		}
		else
		{
			LCDMO_CharRecd = 0;
		}	

		// Allow contrast adjustment...
		if ( LCDMO_CharRecd && StartUpTimer )					// ( StartUpTimer must be > 0. Interrupt counts it down from 5,000)
		{
			if ( LCDMO_CharRecd == 'B' )									// We get this character when up arrow is pressed
				LCDMO_ContrastUp();
			else if ( LCDMO_CharRecd == 'H' )						// We get this character when down arrow is pressed
				LCDMO_ContrastDown();
			else if ( LCDMO_CharRecd == 'E' )						// Terminate StartUp timing if center key pressed
				StartUpTimer = 0;

			// Don't forget to bump pointer since we have read a character
			U2_RxBufOut++; U2_RxBufOut &= (U2_MAX_RX_BUF_LEN-1);   // advance and modulate buffer index

			// Clear character
			LCDMO_CharRecd = 0;
		}

		// U2_Send() will send out any queued character in the buffer
		U2_Send();

	// END WHILE (Startup Loop)
	}

	// After contrast adjust, display menu by setting starting state of state machine
	STM_CursorPos = 1;						// Start out on top line
	STM_State = STM_DISPL_MENU;		// Will start the state machine at displaying menu
	
	STM_ToggleTimer = TIMER_DISABLE;	// Disable toggling until user makes a selection

	// Finished with startup, now do main loop
	while(1)
	{
		// Keep checking for characters use global LCDMO_CharRecd
		if ( U2_RxBufIn != U2_RxBufOut )
		{
			LCDMO_CharRecd = U2_RxBuffer[U2_RxBufOut];

			// Don't forget to bump pointer since we have read a character
			U2_RxBufOut++; U2_RxBufOut &= (U2_MAX_RX_BUF_LEN-1);   // advance and modulate buffer index
					
			// NOTE:  State machine must clear character after responding

		}
		else
		{
			LCDMO_CharRecd = 0;					
		}	

		// U2_Send() will send out any queued character in the buffer
		U2_Send();

		// Going to move this out of the main loop now
		CheckSWs();		

		// Keep checking state machine for state changes
		STM_UpdateState();

		// Anytime STM_MenuSel is not zero, a command has been entered
		if ( STM_MenuSel )
			STM_ProcSel();			// Process the selection

		// If toggle timer has expired, reset it and toggle an LED
		if ( !STM_ToggleTimer )
		{
			// Reset timer to interval
			STM_ToggleTimer = STM_ToggleInt;

			// Toggle selected led		
			if ( STM_Color == RED )
				RED_LED = !RED_LED;
			else
				YEL_LED = !YEL_LED;
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
  _TRISD12 = 0; // PortD bit12  (YEL_LED)
  _TRISD3 = 0;  // PortD bit 3  (RED_LED)
  
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
	GenExceptCnt = 0;  		// NOTE:  The compiler makes all general variables 0 on startup, so we 
												//				Don't really need to do this.

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
//      Function: CheckSWs  Check for SW1, SW2, SW3 action
//
//      Moved to separate routine to unclutter the main loop
//
//************************************************************************
void CheckSWs(void)
{	
   // IF SW1 pressed THEN   **************************************************SW1 Test
   if ( SW1PressedFlag && (!SW1ActionTaken_Flag) )
   {
   
     // Take action here...

	


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
}


//************************************************************************
//
//      Function: InitTimer1()
//
//      This timer is set up to generate a 1 msec interrupt.  This interrupt is
//			useful for many things, but primarily it drives counters that we use
//			to time intervals with 1 msec resolution.
//			All the timers are flexible (and thus complicated) in how they
//			can be used.  No need to delve into that until you need something
//      more complicated.
//
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
//  This doesn't physically send bytes, it just puts the entire string
//  into the Tx buffer using U2_Put() below.
//
//  NOT BLOCKING - characters are sent by U2_Send() in main loop
//	NOTE:  Potential disaster if string is not terminated with null - it keeps 
//         going!
// -----------------------------------------------------------------------------
void U2_Msg( char* msg)			// msg is the string.  Here we use a pointer to it, *msg
{	
	while( *msg )			// loop until *msg == '\0' the  end of the string
	{
    U2_Put( *msg++);	// send the character and point to the next one 
	}
}


// -----------------------------------------------------------------------------
// U2_Put
//
// Puts a single character into the Tx buffer and bumps pointer
// -----------------------------------------------------------------------------
void U2_Put( char ch)
{
	U2_TxBuffer[U2_TxBufIn] = ch;

	// advance and modulate buffer index (rollover)
	U2_TxBufIn++; U2_TxBufIn &= (U2_MAX_TX_BUF_LEN-1);

	// is there a transmit buffer overrun?
	if(U2_TxBufIn == U2_TxBufOut) 						// We should never wrap all the way around to Output pointer
		U2_TxError = UART_OVERFLOW;
}


// -----------------------------------------------------------------------------
// U2_Send() - if required put out one character and get return
//
//	NOT BLOCKING - just sends one character per main loop if there is one to send
//
// -----------------------------------------------------------------------------
void U2_Send(void)
{
	char ch;

	// IF there is something to send THEN
	if(U2_TxBufIn != U2_TxBufOut)  
	{
	  // get transmit character from buffer 	
	 	U2_TxChar = U2_TxBuffer[U2_TxBufOut];
	
	  // Set a timer so we don't hang up forever on a UART or line problem
		U2_TxTimer = 1000;
	
		// Send the character
    while( !UARTTransmitterIsReady(UART2) && U2_TxTimer );  // Will exit here if timer is zero
		  UARTSendDataByte(UART2, U2_TxChar);
	
		// advance and modulate buffer index
		U2_TxBufOut++; U2_TxBufOut &= (U2_MAX_TX_BUF_LEN-1);		// Here we increment the OUTPUT pointer

		// increment character transmitted counter (We usually don't use this)
	  U2_TxLen++;
	}  
}


//************************************************************************
//
//      Function: U2_ProcKeyEntry
//
//      In using the LCDMO, we just get one character (key press) at a time
//			So here we check to see what was received.  We can use this to do
//			whatever things we want to do in a particular application.
//	
//			For now, I'm just putting in something arbitrary for up/down arrow
//			and clearing display on bottom left key
//
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

		LCDMO_GoTo(1,1);					// Go to first column of second line
		U2_Msg("UP PRESSED!         ");
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
		LCDMO_ClearScreen();

		BREAK();
	}
	else if ( CharRecd == 'H')
	{	
		// Down Arrow Key pressed...
		LCDMO_GoTo(2,1);					// Go to first column of first line
		U2_Msg("DOWN PRESSED!       ");				// NOTE: Good idea to write 20 characters to clear any leftover chars on line

		BREAK();
	}

}


//--------------------------------------------------------------------------------------------------------------------
// LCD routines (Matrix Orbital)
//--------------------------------------------------------------------------------------------------------------------

// NOTE:  To a fault, I always (almost) keep all my source in one file, in this case BK.c.  Also, just
//        one h file, BK.h.  A good argument could be made for keeping the LCDMO routines in a separate
//        file.  In practice, I find it's faster for me to just go to the last project where I used
//				it and cut and paste.  To each his own! 


//************************************************************************
//
//      Function: LCDMO_Init()
//
//      This function initializes the display.  Check the manual (LK204-7T-1U.pdf)
//      to get a picture of how to communicate/set up the display.
//
//
//************************************************************************
void LCDMO_Init(void)
{	
	unsigned char addr;
	char WorkStr[10];

	LCDMO_ContrastValue = 128;    					// Initialize contrast 
	LCDMO_Contrast(LCDMO_ContrastValue);

	// Auto scroll off
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x52;
	WorkStr[2] = 0;
	U2_Msg(WorkStr);

	// Init medium numbers, large numbers or bar graphs	(Only one can be used)
	WorkStr[0] = 0xFE;	
	//WorkStr[1] = 0x6D;   // Medium Numbers
	//WorkStr[1] = 0x6E;   // Large	Numbers
	WorkStr[1] = 0x68;	 // Horizontal Bar graphs
	//WorkStr[1] = 0x76;	 // Wide Vertical Bars
	WorkStr[2] = 0;
	U2_Msg(WorkStr);	
}


//************************************************************************
//
//      Function: LCDMO_putc()
//
//      This function sends a single character to the LCD Tx Buffer
//
//
//************************************************************************

int LCDMO_putc( unsigned char c)
{
	// Set up character bits and set data bit
	LCDMO_TxBuffer[LCDMO_TxBufIn] = c | LCDMO_DC_MASK;

	// advance and modulate buffer index
	LCDMO_TxBufIn++; LCDMO_TxBufIn &= (LCDMO_MAX_TX_BUF_LEN-1);
} 



//************************************************************************
//
//      Function: LCDMO_GoTo(line,col)
//
//      This function puts the lcd cursor at a particular location
//      on the screen.  Lines 1-2 Columns 1-16
//
//      Addresses (decimal):
//            Line 1 0
//						Line 2 16
//			Last screen location is 31
//
//************************************************************************
void LCDMO_GoTo( unsigned char line, unsigned char col)
{	

	unsigned char addr;
	char WorkStr[10];
									
	// First, limit ranges
	if ( line > 4 ) line = 4;
	if ( line < 1 ) line = 1;
	if ( col > 20 ) col = 20;
	if ( col < 1 ) col = 1;

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x47;
	WorkStr[2] = col;
	WorkStr[3] = line;
	WorkStr[4] = 0;

	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}



//************************************************************************
//
//      Function: LCDMO_ClearScreen()
//
//      This function clears the LCD screen
//
//************************************************************************
void LCDMO_ClearScreen(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x58;
	WorkStr[2] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);

}


//************************************************************************
//
//      Function: LCDMO_UnderlineON()
//
//************************************************************************
void LCDMO_UnderlineON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x4A;
	WorkStr[2] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);

}


//************************************************************************
//
//      Function: LCDMO_UnderlineOFF()
//
//************************************************************************
void LCDMO_UnderlineOFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x4B;
	WorkStr[2] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);

}


//************************************************************************
//
//      Function: LCDMO_BlinkON()
//
//************************************************************************
void LCDMO_BlinkON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x53;
	WorkStr[2] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_BlinkOFF()
//
//************************************************************************
void LCDMO_BlinkOFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x54;
	WorkStr[2] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_DSP_Brightness(Brightness)
//
//			0 - 255
//
//************************************************************************
void LCDMO_DSP_Brightness(unsigned char Brightness)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x99;
	WorkStr[2] = Brightness;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_KP_Brightness(Brightness)
//
//			Sets Keypad backlight brightness
//			0 - 255
//
//************************************************************************
void LCDMO_KP_Brightness(unsigned char Brightness)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x9C;
	WorkStr[2] = Brightness;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_Color( Red, Green, Blue) 
//
//			0 - 255
//
//************************************************************************
void LCDMO_Color( unsigned char Red, unsigned char Green, unsigned char Blue ) 
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x82;
	WorkStr[2] = Red;
	WorkStr[3] = Green;
	WorkStr[4] = Blue;
	WorkStr[5] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_1_ON() 
//
//			Turns GP Output 1 ON
//
//************************************************************************
void LCDMO_GPOut_1_ON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x57;
	WorkStr[2] = 1;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_1_OFF() 
//
//			Turns GP Output 1 OFF
//
//************************************************************************
void LCDMO_GPOut_1_OFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x56;
	WorkStr[2] = 1;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_2_ON() 
//
//			Turns GP Output 2 ON
//
//************************************************************************
void LCDMO_GPOut_2_ON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x57;
	WorkStr[2] = 2;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_2_OFF() 
//
//			Turns GP Output 2 OFF
//
//************************************************************************
void LCDMO_GPOut_2_OFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x56;
	WorkStr[2] = 2;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_LED1_OFF() 
//
//			Turns Led OFF
//
//************************************************************************
void LCDMO_LED1_OFF(void)
{
	LCDMO_GPOut_1_OFF();
	LCDMO_GPOut_2_OFF();
}


//************************************************************************
//
//      Function: LCDMO_LED1_YEL() 
//
//			Turns Led Yellow
//
//************************************************************************
void LCDMO_LED1_YEL(void)
{
	LCDMO_GPOut_1_ON();
	LCDMO_GPOut_2_ON();
}


//************************************************************************
//
//      Function: LCDMO_LED1_GRN() 
//
//			Turns Led Green
//
//************************************************************************
void LCDMO_LED1_GRN(void)
{
	LCDMO_GPOut_1_OFF();
	LCDMO_GPOut_2_ON();
}


//************************************************************************
//
//      Function: LCDMO_LED1_RED() 
//
//			Turns Led Red
//
//************************************************************************
void LCDMO_LED1_RED(void)
{
	LCDMO_GPOut_1_ON();
	LCDMO_GPOut_2_OFF();
}


//************************************************************************
//
//      Function: LCDMO_GPOut_3_ON() 
//
//			Turns GP Output 3 ON
//
//************************************************************************
void LCDMO_GPOut_3_ON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x57;
	WorkStr[2] = 3;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_3_OFF() 
//
//			Turns GP Output 3 OFF
//
//************************************************************************
void LCDMO_GPOut_3_OFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x56;
	WorkStr[2] = 3;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_4_ON() 
//
//			Turns GP Output 4 ON
//
//************************************************************************
void LCDMO_GPOut_4_ON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x57;
	WorkStr[2] = 4;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_4_OFF() 
//
//			Turns GP Output 4 OFF
//
//************************************************************************
void LCDMO_GPOut_4_OFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x56;
	WorkStr[2] = 4;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_LED2_OFF() 
//
//			Turns Led OFF
//
//************************************************************************
void LCDMO_LED2_OFF(void)
{
	LCDMO_GPOut_3_OFF();
	LCDMO_GPOut_4_OFF();
}


//************************************************************************
//
//      Function: LCDMO_LED2_YEL() 
//
//			Turns Led Yellow
//
//************************************************************************
void LCDMO_LED2_YEL(void)
{
	LCDMO_GPOut_3_ON();
	LCDMO_GPOut_4_ON();
}


//************************************************************************
//
//      Function: LCDMO_LED2_GRN() 
//
//			Turns Led Green
//
//************************************************************************
void LCDMO_LED2_GRN(void)
{
	LCDMO_GPOut_3_OFF();
	LCDMO_GPOut_4_ON();
}


//************************************************************************
//
//      Function: LCDMO_LED2_RED() 
//
//			Turns Led Red
//
//************************************************************************
void LCDMO_LED2_RED(void)
{
	LCDMO_GPOut_3_ON();
	LCDMO_GPOut_4_OFF();
}


//************************************************************************
//
//      Function: LCDMO_GPOut_5_ON() 
//
//			Turns GP Output 5 ON
//
//************************************************************************
void LCDMO_GPOut_5_ON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x57;
	WorkStr[2] = 5;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_5_OFF() 
//
//			Turns GP Output 5 OFF
//
//************************************************************************
void LCDMO_GPOut_5_OFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x56;
	WorkStr[2] = 5;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_6_ON() 
//
//			Turns GP Output 6 ON
//
//************************************************************************
void LCDMO_GPOut_6_ON(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x57;
	WorkStr[2] = 6;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_GPOut_6_OFF() 
//
//			Turns GP Output 6 OFF
//
//************************************************************************
void LCDMO_GPOut_6_OFF(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x56;
	WorkStr[2] = 6;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_LargeDigit(Col,Digit)
//
//			Write large digit to display
//
//************************************************************************
void LCDMO_LargeDigit(unsigned char col, unsigned char digit)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x23;
	WorkStr[2] = col;
	WorkStr[3] = digit;
	WorkStr[4] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_MediumDigit(Col,Digit)
//
//			Write medium digit to display
//
//************************************************************************
void LCDMO_MediumDigit(unsigned char line, unsigned char col, unsigned char digit)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x6F;
	WorkStr[2] = line;
	WorkStr[3] = col;
	WorkStr[4] = digit;
	WorkStr[5] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_HorizontalBarGraph(Line,Col,Direction, Length)
//
//			Write Bar Graph to display 
//      Direction: LCDMO_RIGHT or LCE_LEFT
//      Length: 0 to 100 (pixels)
//
//************************************************************************
void LCDMO_HorizontalBarGraph(unsigned char line, unsigned char col, unsigned char direction, unsigned char length)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x7C;
	WorkStr[2] = col;
	WorkStr[3] = line;
	WorkStr[4] = direction;  // (LCDMO_RIGHT or LCE_LEFT)
	WorkStr[5] = length;
	WorkStr[6] = 0;          // 0 TO 100
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}


//************************************************************************
//
//      Function: LCDMO_VerticalBarGraph(Col, Length)
//
//      Length: 0 to 32 (pixels)
//
//************************************************************************
void LCDMO_VerticalBarGraph(unsigned char col, unsigned char length)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x3D;
	WorkStr[2] = col;
	WorkStr[3] = length;
	WorkStr[4] = 0;          // 0 TO 100
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
}




//************************************************************************
//
//      Function: LCDMO_LED3_OFF() 
//
//			Turns Led OFF
//
//************************************************************************
void LCDMO_LED3_OFF(void)
{
	LCDMO_GPOut_5_OFF();
	LCDMO_GPOut_6_OFF();
}


//************************************************************************
//
//      Function: LCDMO_LED3_YEL() 
//
//			Turns Led Yellow
//
//************************************************************************
void LCDMO_LED3_YEL(void)
{
	LCDMO_GPOut_5_ON();
	LCDMO_GPOut_6_ON();
}


//************************************************************************
//
//      Function: LCDMO_LED3_GRN() 
//
//			Turns Led Green
//
//************************************************************************
void LCDMO_LED3_GRN(void)
{
	LCDMO_GPOut_5_OFF();
	LCDMO_GPOut_6_ON();
}


//************************************************************************
//
//      Function: LCDMO_LED3_RED() 
//
//			Turns Led Red
//
//************************************************************************
void LCDMO_LED3_RED(void)
{
	LCDMO_GPOut_5_ON();
	LCDMO_GPOut_6_OFF();
}


//************************************************************************
//
//      Function: LCDMO_ReadVersion()
//
//      This function requests version number from display
//
//************************************************************************
void LCDMO_ReadVersion(void)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x36;
	WorkStr[2] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);

}


//************************************************************************
//
//      Function: LCDMO_ContrastUp()
//
//      
//
//************************************************************************
void LCDMO_ContrastUp(void)
{
	if ( LCDMO_ContrastValue < 246 )
		LCDMO_ContrastValue += 10;
	LCDMO_Contrast(LCDMO_ContrastValue);
}


//************************************************************************
//
//      Function: LCDMO_ContrastDown()
//
//      
//
//************************************************************************
void LCDMO_ContrastDown(void)
{
	if ( LCDMO_ContrastValue > 10 )
		LCDMO_ContrastValue -= 10;
	LCDMO_Contrast(LCDMO_ContrastValue);
}


//************************************************************************
//
//      Function: LCDMO_Contrast(contrast)
//
//      Contrast setting 0 to 255
//
//************************************************************************
void LCDMO_Contrast(unsigned char contrast)
{

	char WorkStr[16];

	// Build message
	WorkStr[0] = 0xFE;
	WorkStr[1] = 0x50;
	WorkStr[2] = contrast;
	WorkStr[3] = 0;
	
	// Queue msg to LCD
	U2_Msg(WorkStr);
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


//************************************************************************
//
//      Function: STM_UpdateState()
//
//      This function runs the menu selection state machine.
//      
//
//************************************************************************
void STM_UpdateState(void)
{

	// Always do the following regardless of state

	// **** Put "always" tasks here (if any - I usually don't do anything here) ****
  

  // IF State is IDLE                              										****************** IDLE
  if ( STM_State == STM_IDLE )
  {
	
		// Do nothing - machine is disabled, not running


  // ELSE IF State is DISPL_MENU                           						****************** DISPL_MENU
	}
  else if ( STM_State == STM_DISPL_MENU )
	{	

		// Fill 4 lines with display menu
		LCDMO_GoTo(1,1);
		U2_Msg("RED 250ms  Press Up ");
		LCDMO_GoTo(2,1);
		U2_Msg("RED 500ms  or Down  ");
		LCDMO_GoTo(3,1);
		U2_Msg("YEL 1sec   Then     ");
		LCDMO_GoTo(4,1);
		U2_Msg("YEL 2sec   Select   ");

		// Go to Cursor Position and turn on cursor
		LCDMO_GoTo(STM_CursorPos,1);
		LCDMO_BlinkON();

		// Next State
		STM_State = STM_WAIT4_SEL;
		
  // ELSE IF State is WAIT4_SEL                            						****************** WAIT4_SEL
	}
  else if ( STM_State == STM_WAIT4_SEL )
	{	
	
		// IF Up Arrow move cursor
		if ( LCDMO_CharRecd == 'B' )
		{
			if ( STM_CursorPos > 1 )			// Go to new cursor position
				--STM_CursorPos;
			LCDMO_GoTo(STM_CursorPos,1);
			
			LCDMO_CharRecd = 0;						// Clear character		

		// ELSE IF Down Arrow move cursor
		}	
		else if ( LCDMO_CharRecd == 'H' )
		{
			if ( STM_CursorPos < 4 )			// Go to new cursor position
				++STM_CursorPos;
			LCDMO_GoTo(STM_CursorPos,1);
			
			LCDMO_CharRecd = 0;						// Clear character

		// ELSE IF Select Key pressed
		}	
		else if ( LCDMO_CharRecd == 'E' )
		{			
			LCDMO_CharRecd = 0;									// Clear character
			STM_MenuSel = STM_CursorPos;	// Cursor position indicates selection

			LCDMO_CharRecd = 0;						// Clear character

			// Re-display menu (Leave cursor at present position)
			STM_State = STM_DISPL_MENU;

		// END IF
		}

	// ELSE
	}
	else
	{	

		// Shouldn't get here - state was not defined, set to IDLE
		STM_State = 0;		

	// END IF
	}
}



//************************************************************************
//
//      Function: STM_ProcSel()
//
//      This function process the menu selection, carries out
//			the sommand
//      
//			NOTE:  Alternatively, could have passed the selection as an argument.
//						 Instead, we used a global, STM_MenuSel;
//************************************************************************
void STM_ProcSel(void)
{
	if ( STM_MenuSel == 1)
	{
		STM_Color = RED;							// We defined colors in BK.h
		STM_ToggleInt = 250;
	}
	if ( STM_MenuSel == 2)
	{
		STM_Color = RED;
		STM_ToggleInt = 500;
	}
	if ( STM_MenuSel == 3)
	{
		STM_Color = YEL;
		STM_ToggleInt = 1000;
	}
	if ( STM_MenuSel == 4)
	{
		STM_Color = YEL;
		STM_ToggleInt = 2000;	
	}

	// Set timer to interval (It was disabled initially)
	STM_ToggleTimer = STM_ToggleInt;

	// Clear menu selection so we don't keepdoing this
	STM_MenuSel = 0;
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


  // Decrement the utility millisecond timer to zero
  if ( msecTimer )
     msecTimer--;

	if ( StartUpTimer )
		StartUpTimer--;

	// These timers can be disabled... Set the timer to TIMER_DISABLE (Defined as 0xFFFFFFFF) and they will not operate
	if ( (TestTimer) && (TestTimer < TIMER_DISABLE) )
		TestTimer--;
	if ( (STM_ToggleTimer) && (STM_ToggleTimer < TIMER_DISABLE) )
		STM_ToggleTimer--;


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



