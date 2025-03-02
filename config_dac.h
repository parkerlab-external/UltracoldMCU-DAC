#include "commons.h"
#include "driverlib/watchdog.h"
#include "driverlib/rom.h"

#define N_SLOT          4
#define N_CHAN          4
#define EXTRA_BITS 16
#define MY_WATCHDOG_TIMER 80000000
#define MAX_POS 1000

#define INIT_WAIT 1000
#define WAIT_FOR_SYNC 20
#define SSI_FREQ 20000000
#define WAIT_RACK_CLK_LOW 1

#define RESET_BASE  GPIO_PORTB_BASE
#define RESET_PIN   GPIO_PIN_4

#define SYNC1_BASE  GPIO_PORTD_BASE
#define SYNC1_PIN   GPIO_PIN_1

#define SYNC2_BASE  GPIO_PORTE_BASE
#define SYNC2_PIN   GPIO_PIN_4

#define RACK_CLK_BASE    GPIO_PORTE_BASE
#define RACK_CLK_PIN GPIO_PIN_5

#define START_BASE  GPIO_PORTA_BASE
#define START_PIN   GPIO_PIN_2

#define RACK_UART_PORT GPIO_PORTC_BASE
#define RACK_UART_RX_PIN GPIO_PIN_4
#define RACK_UART_TX_PIN GPIO_PIN_5
#define RACK_UART_BASE UART1_BASE

#define CLEAR_MODE 0
