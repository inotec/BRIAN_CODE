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
//	 1-23-17	01-LEDSW Beginning tutorial on PIC32 for Brian
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

  // Use muliple vectored interrupts:  This sets up the vectored interrupt handling for our program.  We will look at interrupts later
  INTEnableSystemMultiVectoredInt();
  INTEnableInterrupts();

  // Init Timer 1 for 1 msec utility interrupt
  InitTimer1();   

	// Main Loop:  All embedded systems (that run inside a device) must have a program that runs all the time, thus the infinite loop
	// Inside the loop we check for and and handle events, periodically sample I/O's, see if we get messages or need to send
	// messages.
  while (1)
  {
		// Testing switch/pushbuttons:
		// The DM32 has 3 pushbutton switches.  We test each one here every time through the loop.  This is very handy for testing.
		// We can do something when a switch is pressed or when it is released
		// We use the SWnActionTaken_Flag to keep from doing the action over and over.
		// The SWnPressedFlag is periodically set or cleared in the utility interrupt, where we use timers to debounce the switches - Switches
		// do not have a clean on/off when pressed, but actually bounce for a few milliseconds.  When the bouncing stops, we setup the flag.
		//
		// I have fancier switch actions (double,triple click, press and hold etc.  These complicate the code, so we won't do that here.
		// I have started moving all the switch test code below to a separate function CheckSWs() to keep from cluttering up the main loop, but
		// I'll leave it in for now
				
    // IF SW1 pressed THEN   **************************************************SW1 Test
    if ( SW1PressedFlag && (!SW1ActionTaken_Flag) )
    {
    
      // Take action here..
			
			RED_LED = LED_ON;


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

			YEL_LED = LED_ON;
      
      SW2ActionTaken_Flag = FALSE;

		// END IF
		}


    // IF SW3 pressed THEN   **************************************************SW3 Test
    if ( SW3PressedFlag && (!SW3ActionTaken_Flag) )
    {
      
      // Take action here...

			RED_LED = LED_OFF;
			YEL_LED = LED_OFF;


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

  // Decrement the utility millisecond timer to zero
  if (msecTimer)
     msecTimer--;

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


