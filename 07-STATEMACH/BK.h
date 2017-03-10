//*************************************************************************
//       Project Header File
//                                                                                             
//   BK.h Tutorial Program for PIC32
//                                                                                                    
//   Written by: Barry P. Keane, Ph.D.                                                                
//               Inotec, Incorporated                                                                 
//               P.O. Box 1587                                                                        
//               Clemson, SC  29633                                                                  
//
//                                                                                                    
//    Copyright � 2017 Inotec, Inc. 
//    Source file: C:\BRIAN_CODE\BK.h 
//                                        
//                                                                                                  
//
/******************************************************************************************************/
/******** CONFIGURATION: THE FOLLOWING ITEMS NEED TO BE SET UP FOR THE SYSTEM BEING COMPILED***********/
/******************************************************************************************************/


const char Version[] = "1.00";


/******************************************************************************************************/
/********             END OF CONFIGURATION SECTION                                          ***********/
/******************************************************************************************************/



//************************************************************************
//
//   Constant Definitions
//
//************************************************************************

#define TIMER_DISABLE   0xFFFFFFFF  // This value will disable some timers

//************************************************************************
//
//   Hardware Definitions
//
//************************************************************************

// LED's
#define GRN_LED _LATD13		// On the PIC32 we can make a pin high by setting this value to 1	 
#define YEL_LED _LATD12 
#define RED_LED _LATD3

#define LED_ON	0					// The LEDs on DM32 are wired so that they come on when the pin is low
#define LED_OFF 1					// On another device the LEDs might be wired so that they come on when
													// the pin is high.  This way in the code the statement is always
													// RED_LED = LED_ON;
													// So, we don't have to remember what pin the RED led is connected to or
													// whether it is high active or low active
		
// Switches
#define SW1 _RD4    
#define SW2 _RD5 
#define SW3 _RD6  

#define SW_PRESSED 0	// PIC32 is wired such that pressing the SW makes the pin go low
											// Like with the LEDs we don't have to remember what the pin number is
											// or whether it is high active or low active
											
#define DB_TIME 50		// Debounce time for switches in msec

// UART Buffer Constants	  
#define U1_MAX_RX_BUF_LEN  1024
#define U1_MAX_TX_BUF_LEN  1024
#define U2_MAX_RX_BUF_LEN  1024
#define U2_MAX_TX_BUF_LEN  1024
#define U5_MAX_RX_BUF_LEN  1024
#define U5_MAX_TX_BUF_LEN  1024

// UART constants
#define UART_MAX_MSG                 16
#define UART_OVERRUN               0x02
#define UART_FRAME                 0x04
#define UART_PARITY                0x08
#define UART_EOF                   0xEF
#define UART_OVERFLOW              0xFF
#define UART_UPPERCASE_MASK        0xDF
#define UART_ESC                   0x1B

// ASCII codes etc	
#define CR 0x0D
#define LF 0x0A
#define DLE 0x10
#define ETX 0x03
#define CTRLZ 0x1B
#define ESC 0x1B

// LCDMO constants and macros
#define LCDMO_MAX_TX_BUF_LEN 512
#define LCDMO_E 				_LATD5					// High active
#define LCDMO_A0 				_LATB15				// Low for cmd high for data
#define LCDMO_PULSE() 	LCDMO_E = 1; Delayms( 1); LCDMO_E = 0;
#define SET_LATE(a) 	LATE &= 0xFF00; LATE |= a; 
#define LCDMO_CLEAR 		LCDMO_putCmd(0x01);

#define LCDMO_DC_MASK 0b10000000000000000000000000000000    // Set this bit in TX buffer to indicate data
#define LCDMO_RIGHT	0
#define LCDMO_LEFT	1
		
// Define colors
#define BRN	0
#define BLK 1
#define RED 2
#define ORN 3
#define YEL 4
#define GRN 5
#define BLU 6
#define VIO 7
#define GRY 8
#define WHT 9
		

		
		
//*************************************************************************
//                                                                        
//   Type Definitions                                    
//                                                                        
//*************************************************************************


//*************************************************************************
// Type/Structure Definitions


//*************************************************************************
//                                                                        
//   Global Constant String Definitions                                   
//                                                                        
//*************************************************************************


//*************************************************************************
// Function Prototypes         
// 
//*************************************************************************

//************************************************************************************** General system funcions:
void SysInit(void);
void CheckSWs(void);
void InitTimer1(void);
void msecDelay(unsigned int n);


//************************************************************************************** UART2 functions:
void U2_Init(void);
void U2_Flush(void);
void U2_Msg(char* msg);
void U2_Put(char ch);
void U2_Send(void);
void U2_ProcKeyEntry(void);

//************************************************************************************** LCD functions:
void LCDMO_Init(void);
void LCDMO_Send(void);
int LCDMO_putc( unsigned char c);
void LCDMO_Msg(const unsigned char* msg);
void LCDMO_GoTo( unsigned char line, unsigned char col);
void LCDMO_ClearScreen(void);
void LCDMO_UnderlineON(void);
void LCDMO_UnderlineOFF(void);
void LCDMO_BlinkON(void);
void LCDMO_BlinkOFF(void);
void LCDMO_DSP_Brightness( unsigned char brightness);
void LCDMO_KP_Brightness( unsigned char brightness);
void LCDMO_Color( unsigned char Red, unsigned char Green, unsigned char Blue); 
void LCDMO_GPOut_1_ON(void);
void LCDMO_GPOut_1_OFF(void);
void LCDMO_GPOut_2_ON(void);
void LCDMO_GPOut_2_OFF(void);
void LCDMO_GPOut_3_ON(void);
void LCDMO_GPOut_3_OFF(void);
void LCDMO_GPOut_4_ON(void);
void LCDMO_GPOut_4_OFF(void);
void LCDMO_GPOut_5_ON(void);
void LCDMO_GPOut_5_OFF(void);
void LCDMO_GPOut_6_ON(void);
void LCDMO_GPOut_6_OFF(void);
void LCDMO_LED1_OFF(void);
void LCDMO_LED1_YEL(void);
void LCDMO_LED1_GRN(void);
void LCDMO_LED1_RED(void);
void LCDMO_LED2_OFF(void);
void LCDMO_LED2_YEL(void);
void LCDMO_LED2_GRN(void);
void LCDMO_LED2_RED(void);
void LCDMO_LED3_OFF(void);
void LCDMO_LED3_YEL(void);
void LCDMO_LED3_GRN(void);
void LCDMO_LED3_RED(void);
void LCDMO_LargeDigit(unsigned char col, unsigned char digit);
void LCDMO_MediumDigit(unsigned char line, unsigned char col, unsigned char digit);
void LCDMO_HorizontalBarGraph(unsigned char line, unsigned char col, unsigned char direction, unsigned char length);	
void LCDMO_VerticalBarGraph(unsigned char col, unsigned char length);	
void LCDMO_ReadVersion(void);
void LCDMO_TestUpdateState();
void LCDMO_ContrastUp();
void LCDMO_ContrastDown();
void LCDMO_Contrast(unsigned char contrast);
void LCDMO_UpdateDisplay(void);

// State Machine Stuff:
void STM_UpdateState(void);
void STM_ProcSel(void);

//*************************************************************************
// Define Global Variables         
// 
//*************************************************************************


//************************************************************************************** General Variables:
// Testing...


// For exception handling...
UINT32 excep_code;
UINT32 excep_addr; 

char IntRunningFlag;

int GenExceptCnt;



//************************************************************************************** UART2 Variables:
int U2_RxBufIn;
int U2_RxBufOut;
int U2_RxLen;   
int U2_TxBufIn;
int U2_TxBufOut;
int U2_TxLen;   
char U2_RxChar;
char U2_TxChar;
char U2_RxError;
char U2_TxError;
char U2_RxBuffer[U2_MAX_RX_BUF_LEN];
char U2_TxBuffer[U2_MAX_TX_BUF_LEN];
char U2_MsgRecd;							// Interrupt sets this true when <CR> is received
unsigned char U2_StreamFlag;


//************************************************************************************** Timer Variables:
unsigned int	SW1dbTimer;
unsigned char SW1PressedFlag;
unsigned char SW1ActionTaken_Flag;
unsigned int	SW1HoldTime;
unsigned int 	SW2dbTimer;
unsigned char SW2PressedFlag;
unsigned char SW2ActionTaken_Flag;
unsigned int	SW2HoldTime;
unsigned int	SW3dbTimer;
unsigned char SW3PressedFlag;
unsigned char SW3ActionTaken_Flag;
unsigned int 	SW3HoldTime;
unsigned int U1_TxTimer;
unsigned int U2_TxTimer;
unsigned int U2_StreamTimer;
unsigned int U5_TxTimer;

//************************************************************************************** LCDMO Variables:
int LCDMO_TxBufIn;
int LCDMO_TxBufOut;
int LCDMO_TxChar;
int LCDMO_TxBuffer[LCDMO_MAX_TX_BUF_LEN];
int LCDMO_Busy;
char LCDMO_CharRecd;
unsigned char LCDMO_Page;
unsigned int LCDMO_TestState;
unsigned char LCDMO_ContrastValue;


// msec Timing:
unsigned int ThirdMsecCtr;
unsigned int TestTimer;
unsigned int msecTimer;
unsigned int FourthSecCtr;
unsigned int StartUpTimer;
unsigned int LCDMO_SendTimer;

// sec Timing:
unsigned int SecsCounter;

// Aging timers

// State Machine Stuff:
unsigned char STM_State;
#define STM_IDLE				0					// Set up the allowed states for state machine
#define STM_DISPL_MENU 	1					// i.e., give each a unique number
#define STM_WAIT4_SEL		2
#define STM_ISSUE_CMD		3
		
unsigned char STM_CursorPos;
unsigned char STM_MenuSel;
unsigned int STM_Color;
unsigned int STM_ToggleInt;		// Reset interval for toggle timer
unsigned int STM_ToggleTimer;	// Actual toggle timer (will count msecs)

		
				
//*************************************************************************
// Define Global Arrays
// 
//************************************************************************* 


//************************************************************************
// Macro Definitions
//                          
//*************************************************************************

#define Quote() { strcat (StrToWrite, """\x22""" ); }      // Will concatenate a quote to the strint StrToWrite

#define BREAK() { Nop(); Nop(); Nop(); }        // Used for debugger break points 
#define Break() { Nop(); Nop(); Nop(); }        // Used for debugger break points 
#define break() { Nop(); Nop(); Nop(); }        // Used for debugger break points 

// Shortcuts (not available in C32 compiler)
#define _T2IE IEC0bits.T2IE
#define _T2IF IFS0bits.T2IF
#define _T2IP IPC2bits.T2IP
		



