//*****************************************************************************
//
// pulse_count_timer.c - Time pulses for counting decays and other optical phenomena.
//
// 
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

/***************************/
/* Port and pin parameters */
/***************************/
#define MAIN_PORT_CHOICE D
#define TRIG_PIN 0
/* LIGHT_PIN has to be pin 2, and LIGHT_OFF_PIN has to be 1 for leaving the AOM ON all the time. */
#define LIGHT_PIN 2
#define LIGHT_OFF_PIN 1
#define LASER_PIN 3

#define ALWAYS_ON 1

/* complicated macros to apply ports and pins */
#define CHOOSE_PORT_INNER(PORT_LETTER) SYSCTL_PERIPH_GPIO ## PORT_LETTER
#define CHOOSE_PORT_BASE_INNER(PORT_LETTER) GPIO_PORT ## PORT_LETTER ## _BASE
#define CHOOSE_PORT_MID(PORT_TO_USE) CHOOSE_PORT_INNER(PORT_TO_USE)
#define CHOOSE_PORT_BASE_MID(PORT_TO_USE) CHOOSE_PORT_BASE_INNER(PORT_TO_USE)

#define GPIO_PIN_NUM_RAW(PIN_NUMBER) GPIO_PIN_ ## PIN_NUMBER
#define GPIO_PIN_NUM(PIN_NUMBER) GPIO_PIN_NUM_RAW(PIN_NUMBER)

/* Standard ports and pins */
#define MAIN_PORT CHOOSE_PORT_MID(MAIN_PORT_CHOICE)
#define MAIN_PORT_BASE CHOOSE_PORT_BASE_MID(MAIN_PORT_CHOICE)

/**********************/
/* UART Configuration */
/**********************/
#define UART_CHOICE 0
#define UART_CHOICE_PORT A
#define UART_RX 0
#define UART_TX 1
#define UART_RX_CFG GPIO_PA0_U0RX
#define UART_TX_CFG GPIO_PA1_U0TX

#define CHOOSE_UART_INNER(UART_NUM) SYSCTL_PERIPH_UART ## UART_NUM
#define CHOOSE_UART_BASE_INNER(UART_NUM) UART ## UART_NUM ## _BASE
#define CHOOSE_UART_MID(UART_NUM) CHOOSE_UART_INNER(UART_NUM)
#define CHOOSE_UART_BASE_MID(UART_NUM) CHOOSE_UART_BASE_INNER(UART_NUM)
#define CHOOSE_UART_INT_INNER(UART_NUM) INT_UART ## UART_NUM
#define CHOOSE_UART_INT_MID(UART_NUM) CHOOSE_UART_INT_INNER(UART_NUM)

#define UART_PORT CHOOSE_PORT_MID(UART_CHOICE_PORT)
#define UART_PORT_BASE CHOOSE_PORT_BASE_MID(UART_CHOICE_PORT)
#define MY_UART_PERIPH CHOOSE_UART_MID(UART_CHOICE)
#define MY_UART_BASE CHOOSE_UART_BASE_MID(UART_CHOICE)
#define MY_UART_INT CHOOSE_UART_INT_MID(UART_CHOICE)

/*****************************************/
/* Sample rate and triggering parameters */
/*****************************************/
#define TIMER_FREQ 1980

/* all times must be at least 2 to allow the trigger to reset */
#define COUNT_TIME 2
#define DWELL_TIME 4

#define TRIGGER_CYCLE (COUNT_TIME + DWELL_TIME)

#define LIGHT_OFF_TRIGS 333
#define LIGHT_ON_TRIGS 333

#define LIGHT_OFF_TIME  (LIGHT_OFF_TRIGS*TRIGGER_CYCLE)
#define LIGHT_ON_TIME   (LIGHT_ON_TRIGS*TRIGGER_CYCLE + COUNT_TIME)
#define LIGHT_CYCLE (LIGHT_OFF_TIME + LIGHT_ON_TIME)

/* Note: total running time is (LCM(TRIGGER_CYCLE, LIGHT_CYCLE)+1)/TIMER_FREQ */

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

uint32_t g_ui32TrigCount = 0;
uint32_t g_ui32LightCount = 0;
uint32_t volatile g_ui32Running = 0;

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32SysClock;

    //
    // Set the system clock to run at 80Mhz off PLL with external crystal as
    // reference.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                SYSCTL_OSC_MAIN);
    ui32SysClock = SysCtlClockGet();

    /* GPIO Enabling */
    SysCtlPeripheralEnable(MAIN_PORT);
    SysCtlPeripheralEnable(UART_PORT);
    while(!SysCtlPeripheralReady(MAIN_PORT) || !SysCtlPeripheralReady(UART_PORT)) {}

    /* GPIO Pin Config */
    GPIOPinTypeGPIOOutput(MAIN_PORT_BASE, GPIO_PIN_NUM(TRIG_PIN) | GPIO_PIN_NUM(LIGHT_PIN) | GPIO_PIN_NUM(LASER_PIN) | GPIO_PIN_NUM(LIGHT_OFF_PIN));
    GPIOPinConfigure(UART_RX_CFG);
    GPIOPinConfigure(UART_TX_CFG);
    GPIOPinTypeUART(UART_PORT_BASE, GPIO_PIN_NUM(UART_TX) | GPIO_PIN_NUM(UART_RX));

    /* Default state write */
    GPIOPinWrite(MAIN_PORT_BASE, GPIO_PIN_NUM(TRIG_PIN) | GPIO_PIN_NUM(LIGHT_PIN) | GPIO_PIN_NUM(LASER_PIN) | GPIO_PIN_NUM(LIGHT_OFF_PIN), GPIO_PIN_NUM(LIGHT_PIN) | GPIO_PIN_NUM(LASER_PIN));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Enable the UART */
    SysCtlPeripheralEnable(MY_UART_PERIPH);
    UARTClockSourceSet(MY_UART_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(MY_UART_BASE, ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_ZERO));
    /* Empty UART Buffer and enable UART interrupts*/
    while(UARTCharsAvail(MY_UART_BASE)){
        UARTCharGetNonBlocking(MY_UART_BASE);
    }
    UARTFIFOLevelSet(MY_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTIntEnable(MY_UART_BASE, UART_INT_RX);
    UARTIntClear(MY_UART_BASE, UARTIntStatus(MY_UART_BASE, true));

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClock/TIMER_FREQ); // clocks per sec/ cycles per sec = clocks per cycle

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);

    IntPrioritySet(INT_TIMER0A, 0);
    IntPrioritySet(MY_UART_INT, 0xE0);
    IntEnable(INT_TIMER0A);
    IntEnable(MY_UART_INT);
    IntMasterEnable();

    while(true){}
}

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if(g_ui32Running == 0) return; /* Don't run if we haven't received a UART command */

    uint32_t ui32PortState = 0;

#if ALWAYS_ON
    if(g_ui32LightCount >= LIGHT_OFF_TIME)
    {
        ui32PortState |= GPIO_PIN_NUM(LASER_PIN);
    }
    ui32PortState |= GPIO_PIN_NUM(LIGHT_PIN);
#else
    if(g_ui32LightCount >= LIGHT_OFF_TIME)
    {
        ui32PortState |= (GPIO_PIN_NUM(LIGHT_PIN) | GPIO_PIN_NUM(LASER_PIN));
    }
    else
    {
        ui32PortState |= GPIO_PIN_NUM(LIGHT_OFF_PIN);
    }
#endif

    if((g_ui32TrigCount == 0) || (g_ui32TrigCount == COUNT_TIME))
    {
        ui32PortState |= GPIO_PIN_NUM(TRIG_PIN);
    }

    GPIOPinWrite(MAIN_PORT_BASE, GPIO_PIN_NUM(TRIG_PIN) | GPIO_PIN_NUM(LIGHT_PIN) | GPIO_PIN_NUM(LASER_PIN) | GPIO_PIN_NUM(LIGHT_OFF_PIN), ui32PortState);

    g_ui32LightCount += 1;
    g_ui32TrigCount += 1;

    if(g_ui32LightCount >= LIGHT_CYCLE) g_ui32LightCount -= LIGHT_CYCLE;
    if(g_ui32TrigCount >= TRIGGER_CYCLE) g_ui32TrigCount -= TRIGGER_CYCLE;

    if((g_ui32LightCount == 0) && (g_ui32TrigCount == 0)) /* stop when completely done */
    {
        g_ui32Running = 0;
    }
}

void UARTHandler(void)
{
    UARTIntClear(MY_UART_BASE, UARTIntStatus(MY_UART_BASE, true));

    /* Read everything that's available */
    while(UARTCharsAvail(MY_UART_BASE)){
        UARTCharGetNonBlocking(MY_UART_BASE);
    }

    g_ui32Running = 1;
}
