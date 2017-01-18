// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include "stacks.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "chip.h"
//#include "uart.h"
#include "utils.h"
#include "common.h"
#include "platform_conf.h"

#define SYSTICKHZ             10

/* Structure for initial base clock states */
struct CLK_BASE_STATES {
	CHIP_CGU_BASE_CLK_T clk;	/* Base clock */
	CHIP_CGU_CLKIN_T clkin;	/* Base clock source, see UM for allowable souorces per base clock */
	bool autoblock_enab;/* Set to true to enable autoblocking on frequency change */
	bool powerdn;		/* Set to true if the base clock is initially powered down */
};

/* Initial base clock states are mostly on */
STATIC const struct CLK_BASE_STATES InitClkStates[] = {

	/* Ethernet Clock base */
	{CLK_BASE_PHY_TX, CLKIN_ENET_TX, true, false},
	{CLK_BASE_PHY_RX, CLKIN_ENET_TX, true, false},

	/* Clocks derived from dividers */
	{CLK_BASE_USB1, CLKIN_IDIVD, true, true}
};

/* Set up and initialize clocking prior to call to main */
void Board_SetupClocking(void)
{
	int i;

	/* Enable Flash acceleration and setup wait states */
	Chip_CREG_SetFlashAcceleration(MAX_CLOCK_FREQ);

	/* Setup System core frequency to MAX_CLOCK_FREQ */
	Chip_SetupCoreClock(CLKIN_CRYSTAL, MAX_CLOCK_FREQ, true);

	/* Setup system base clocks and initial states. This won't enable and
	   disable individual clocks, but sets up the base clock sources for
	   each individual peripheral clock. */
	for (i = 0; i < (sizeof(InitClkStates) / sizeof(InitClkStates[0])); i++) {
		Chip_Clock_SetBaseClock(InitClkStates[i].clk, InitClkStates[i].clkin,
								InitClkStates[i].autoblock_enab, InitClkStates[i].powerdn);
	}

	/* Reset and enable 32Khz oscillator */
	LPC_CREG->CREG0 &= ~((1 << 3) | (1 << 2));
	LPC_CREG->CREG0 |= (1 << 1) | (1 << 0);
}

// ****************************************************************************
// Platform initialization

int platform_init()
{
  // Set up microcontroller system and SystemCoreClock variable
  //Chip_SystemInit();
  Board_SetupClocking();
 
  // DeInit NVIC and SCBNVIC
  //NVIC_DeInit();
  //NVIC_SCBDeInit();
  
  // Configure the NVIC Preemption Priority Bits:
  // two (2) bits of preemption priority, six (6) bits of sub-priority.
  // Since the Number of Bits used for Priority Levels is five (5), so the
  // actual bit number of sub-priority is three (3)
  NVIC_SetPriorityGrouping(0x05);
 
  //  Set Vector table offset value
#if (__RAM_MODE__==1)
  //NVIC_SetVTOR(0x10000000);
#else
  //NVIC_SetVTOR(0x00000000);
#endif

  // Enable SysTick
  SysTick_Config( platform_s_cpu_get_frequency() / SYSTICKHZ );
  
  cmn_platform_init();
  
  return PLATFORM_OK;
}

// *****************************************************************************
// CPU specific functions

u32 platform_s_cpu_get_frequency()
{
  SystemCoreClockUpdate();
  return SystemCoreClock;
}

void stm32_cpu_reset()
{
  NVIC_SystemReset();
}

/*
// ****************************************************************************
// PIO functions

// Array with register addresses
typedef volatile unsigned int* vu_ptr;

static const vu_ptr pio_m0s_regs[] = { &MODE0S_0, &MODE0S_1, &MODE0S_2, &MODE0S_3, &MODE0S_4, &MODE0S_5, &MODE0S_6, &MODE0S_7 };
static const vu_ptr pio_m0c_regs[] = { &MODE0C_0, &MODE0C_1, &MODE0C_2, &MODE0C_3, &MODE0C_4, &MODE0C_5, &MODE0C_6, &MODE0C_7 };
static const vu_ptr pio_m1s_regs[] = { &MODE1S_0, &MODE1S_1, &MODE1S_2, &MODE1S_3, &MODE1S_4, &MODE1S_5, &MODE1S_6, &MODE1S_7 };
static const vu_ptr pio_m1c_regs[] = { &MODE1C_0, &MODE1C_1, &MODE1C_2, &MODE1C_3, &MODE1C_4, &MODE1C_5, &MODE1C_6, &MODE1C_7 };
static const vu_ptr pio_m0_regs[] = { &MODE0_0, &MODE0_1, &MODE0_2, &MODE0_3, &MODE0_4, &MODE0_5, &MODE0_6, &MODE0_7 };
static const vu_ptr pio_m1_regs[] = { &MODE1_0, &MODE1_1, &MODE1_2, &MODE1_3, &MODE1_4, &MODE1_5, &MODE1_6, &MODE1_7 };
static const vu_ptr pio_pin_regs[] = { &PINS_0, &PINS_1, &PINS_2, &PINS_3, &PINS_4, &PINS_5, &PINS_6, &PINS_7 };

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1;
  
  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      *pio_m0_regs[ port ] = pinmask;        
      break;
      
    case PLATFORM_IO_PIN_SET:
      *pio_m0s_regs[ port ] = pinmask;    
      break;
      
    case PLATFORM_IO_PIN_CLEAR:
      *pio_m0c_regs[ port ] = pinmask;
      break;
      
    case PLATFORM_IO_PORT_DIR_OUTPUT:
      *pio_m1_regs[ port ] = 0xFFFFFFFF;
      break;
      
    case PLATFORM_IO_PIN_DIR_OUTPUT:
      *pio_m1s_regs[ port ] = pinmask;
      break;
      
    case PLATFORM_IO_PORT_DIR_INPUT:
      *pio_m1_regs[ port ] = 0;
      *pio_m0_regs[ port ] = 0;
      break;
      
    case PLATFORM_IO_PIN_DIR_INPUT:
      *pio_m1c_regs[ port ] = pinmask;
      *pio_m0c_regs[ port ] = pinmask;
      break;
            
    case PLATFORM_IO_PORT_GET_VALUE:
      retval = *pio_pin_regs[ port ];
      break;
      
    case PLATFORM_IO_PIN_GET:
      retval = *pio_pin_regs[ port ] & pinmask ? 1 : 0;
      break;
      
    default:
      retval = 0;
      break;
  }
  return retval;  
}
*/

// ****************************************************************************
// UART

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
	// RXD pin setup for UART
	uint8_t rxPort;
	uint8_t rxPin;
	uint16_t rxModeFunc;
	// TXD pin setup for UART
	uint8_t txPort;
	uint8_t txPin;
	uint16_t txModeFunc;
	LPC43XX_IRQn_Type UARTx_IRQn;
	u32 actualBaud = 0;
	LPC_USART_T *pUART = LPC_USART0;
	
	// TODO:  select pUART based on id
	( void )id;
	

	// Determine pin mux setting for specified U(S)ART
	if(LPC_USART0 == pUART)
	{
		// Configure P6_4 for TXD, P2_1 for RXD
		txPort = 6;
		txPin = 4;
		txModeFunc = SCU_MODE_FUNC2;
		rxPort = 2;
		rxPin = 1;
		rxModeFunc = SCU_MODE_FUNC1;
		UARTx_IRQn = USART0_IRQn;
	}
	else if(LPC_UART1 == pUART)
	{
		// Configure p1_13 for TXD, P1_14 for RXD
		txPort = 1;
		txPin = 13;
		txModeFunc = SCU_MODE_FUNC1;
		rxPort = 1;
		rxPin = 14;
		rxModeFunc = SCU_MODE_FUNC1;
		UARTx_IRQn = UART1_IRQn;
	}
	else if(LPC_USART2 == pUART)
	{
		// Configure P2_10 for TXD, P2_11 for RXD
		txPort = 2;
		txPin = 10;
		txModeFunc = SCU_MODE_FUNC2;
		rxPort = 2;
		rxPin = 11;
		rxModeFunc = SCU_MODE_FUNC2;
		UARTx_IRQn = USART2_IRQn;
	}
	else if(LPC_USART3 == pUART)
	{
		// Configure P2_3 for TXD,  P2_4 for RXD
		txPort = 2;
		txPin = 3;
		txModeFunc = SCU_MODE_FUNC2;
		rxPort = 2;
		rxPin = 4;
		rxModeFunc = SCU_MODE_FUNC2;
		UARTx_IRQn = USART3_IRQn;
	}
	else
	{
		// Invalid UART passed.
		// Null it to cause the initialization code
		// to be skipped.
		//assert(false);
		pUART = 0;
	}
	if(pUART)
	{
		// Set pin muxes for TXD/RXD
		Chip_SCU_PinMuxSet(txPort, txPin, (SCU_MODE_INACT | txModeFunc));
		Chip_SCU_PinMuxSet(rxPort, rxPin, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | rxModeFunc));
		// Enable FIFOs and disable interrupts
		Chip_UART_Init(pUART);
		// Set baud rate for 31250 (MIDI)
		actualBaud = Chip_UART_SetBaudFDR(pUART, baud);
		// 8N1
		Chip_UART_ConfigData(pUART, UART_LCR_WLEN8 | UART_LCR_PARITY_DIS | UART_LCR_SBS_1BIT);
		// Transmit immediately when data is available
		Chip_UART_TXEnable(pUART);

		//RingBuffer_Init(&s_txring, s_txbuff, 1, UART_SRB_SIZE);

		// preemption = 1, sub-priority = 1
		//NVIC_SetPriority(UARTx_IRQn, 1);
		//NVIC_EnableIRQ(UARTx_IRQn);
	}

  return actualBaud;
}

void platform_s_uart_send( unsigned id, u8 data )
{
  Chip_UART_SendBlocking(LPC_USART0, &data, 1);
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  u8 buffer;

  if( timeout == 0 )
  {
    if ( Chip_UART_Read(LPC_USART0, &buffer, 1) == 0 )
      return -1;
    else
      return ( int )buffer;
  }

  Chip_UART_ReadBlocking(LPC_USART0, &buffer, 1);
  return ( int )buffer;
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  return PLATFORM_ERR;
}

/*

// ****************************************************************************
// Timer

static const vu_ptr tmr_load[] = { &T0LOAD, &T1LOAD };
static const vu_ptr tmr_value[] = { &T0VALUE, &T1VALUE };
static const vu_ptr tmr_ctrl[] = { &T0CTRL, &T1CTRL };
static const vu_ptr tmr_clr[] = { &T0CLR, &T1CLR };
static const unsigned tmr_prescale[] = { 1, 16, 256, 1 };

// Helper: get timer clock
static u32 platform_timer_get_clock( unsigned id )
{
  return MAIN_CLOCK / tmr_prescale[ ( *tmr_ctrl[ id ] >> 2 ) & 0x03 ];
}

// Helper: set timer clock
static u32 platform_timer_set_clock( unsigned id, u32 clock )
{
  unsigned i, mini = 0;
  
  for( i = 0; i < 3; i ++ )
    if( ABSDIFF( clock, MAIN_CLOCK / tmr_prescale[ i ] ) < ABSDIFF( clock, MAIN_CLOCK / tmr_prescale[ mini ] ) )
      mini = i;
  *tmr_ctrl[ id ] = ( *tmr_ctrl[ id ] & ~0xB ) | ( mini << 2 );
  return MAIN_CLOCK / tmr_prescale[ mini ];
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
  u32 freq;
  u64 final;
  u32 mask = ( id == 0 ) ? ( 1 << 5 ) : ( 1 << 6 );
    
  freq = platform_timer_get_clock( id );
  final = ( ( u64 )delay_us * freq ) / 1000000;
  if( final > 0xFFFFFFFF )
    final = 0xFFFFFFFF;
  *tmr_ctrl[ id ] &= 0x7F;
  *tmr_load[ id ] = final;
  *tmr_clr[ id ] = 0;
  *tmr_ctrl[ id ] |= 0x80;
  while( ( INT_PENDING & mask ) == 0 );
}
      
timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;
  
  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      *tmr_ctrl[ id ] &= 0x7F;
      *tmr_load[ id ] = 0xFFFFFFFF;
      *tmr_ctrl[ id ] |= 0x80;    
      res = 0xFFFFFFFF;
      break;
      
    case PLATFORM_TIMER_OP_READ:
      res = *tmr_value[ id ];
      break;
      
    case PLATFORM_TIMER_OP_SET_CLOCK:
      res = platform_timer_set_clock( id, data );
      break;
      
    case PLATFORM_TIMER_OP_GET_CLOCK:
      res = platform_timer_get_clock( id );
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      res = 0xFFFFFFFF;
      break;
  }
  return res;
}

*/

// ****************************************************************************
// "Dummy" timer functions

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
 return 0;
}

timer_data_type platform_timer_read_sys()
{
  return 0; //cmn_systimer_get();
}

u64 platform_timer_sys_raw_read()
{
  return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
  SysTick->CTRL &= ~( 1 << SysTick_CTRL_TICKINT_Pos );
}

void platform_timer_sys_enable_int()
{
  SysTick->CTRL |= 1 << SysTick_CTRL_TICKINT_Pos;
}


