//*****************************************************************************
//
// dacout_launchpad.c
//
// Launchpad code for DAC out boxes
//
//*****************************************************************************
#include "data_io_dac.h"

inline void read_absl_val(int i_slot, int32_t *p_absl_val) {
    int32_t val_i = g_table_poly_val[i_slot][g_chkpt_pos[i_slot]];
    switch (g_table_poly_dur[i_slot][g_chkpt_pos[i_slot]]) {
        case -1 : *p_absl_val = 0;      advance_chkpt_pos(i_slot)  ; break;
        case -2 : *p_absl_val = val_i;  advance_chkpt_pos(i_slot)  ; break;
        default : *p_absl_val += val_i; advance_frame_pos(i_slot); break;
    }
    recede_chkpt_pos(i_slot);
}

void UARTHandler() {
    bool keep_going = true;
    uint32_t char_prefix = 0;
    int32_t pos, i_slot, i_chan;
    int g_range_mode_buff[N_CHAN];
    g_checksum_uart = 0;

    for(i_chan = 0; i_chan < N_CHAN; ++i_chan)
        g_range_mode_buff[i_chan] = g_range_mode[i_chan];

    UARTIntClear(RACK_UART_BASE, UARTIntStatus(RACK_UART_BASE, true));
    char_prefix = readu8_uart();
    keep_going = (char_prefix == 0xAA);
    while ( keep_going ) {
        char_prefix = readu8_uart();
        if ( char_prefix == 'U' ) { // Update one channel's arrays to have new values
            // Receive channel number and position
            i_slot = readu8_uart();
            pos = readi16_uart();
            pos = (pos < 0) ? 0 : ((pos > MAX_POS) ? MAX_POS - 1 : pos);
            // Receive and update increment durations and values
            g_table_poly_dur[i_slot][pos] = readi32_uart();
            g_table_poly_val[i_slot][pos] = readi32_uart();
        }
        else if ( char_prefix == 'R' ) { // Reset the array index
            i_slot = readu8_uart();
            g_chkpt_pos[i_slot] = readi16_uart();
        }
        else if ( char_prefix == 'Z' ) { // Make a given channel output its minimum voltage and set the arrays to zero.
            i_slot = readu8_uart();
            g_absl_val_cache[i_slot] = 0;
            for(i_slot=0; i_slot < MAX_POS; i_slot += 1){
                g_table_poly_dur[i_slot][i_slot] = 0;
                g_table_poly_val[i_slot][i_slot] = 0;
            }
        }
        else if ( char_prefix == 'E' ) { // Change the effective array length
            i_slot = readu8_uart();
            g_n_chkpt[i_slot] = readi16_uart();
        }
        else if(char_prefix == 'G'){ // Change the channel range
            i_slot = readu8_uart();
            g_range_mode_buff[i_slot] = readu8_uart();
        }
        else if(char_prefix == 'T'){ // Break the UART Interrupt
            GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_TX_PIN);
            SysCtlDelay(INIT_WAIT);
            // Update all the channel ranges at the end
            for(i_slot = 0; i_slot < N_CHAN; i_slot++){
                if(g_range_mode[i_slot] != g_range_mode_buff[i_slot]) {
                    g_range_mode[i_slot] = g_range_mode_buff[i_slot];
                    set_range_mode_ssi(i_slot);
                }
            }
            write_checksum_uart();
            while(UARTBusy(RACK_UART_BASE));
            //SysCtlDelay(INIT_WAIT);
            GPIOPinTypeGPIOInput(RACK_UART_PORT, RACK_UART_TX_PIN);
            keep_going = false;
        }
    }
}

void GPIOIntHandler(void)
{
    int i_slot;
    GPIOIntClear(RACK_CLK_BASE,0x1FF); // clear all interrupts
    if(!GPIOPinRead(RACK_CLK_BASE, RACK_CLK_PIN)) // ignore false triggers
        return;
    WatchdogReloadSet(WATCHDOG0_BASE, MY_WATCHDOG_TIMER);

    int32_t active_now = GPIOPinRead(START_BASE, START_PIN);

    // SSI updates goes before fetching
    // write every frame so the writing noise would not change between slow ramps and steady hold
    write_absl_val_ssi(0, 0x12); // Channel 1 Update
    write_absl_val_ssi(1, 0x11); // Channel 2 Update
    write_absl_val_ssi(2, 0x14); // Channel 3 Update
    write_absl_val_ssi(3, 0x18); // Channel 4 Update

    if(active_now) {
        for(i_slot = 0; i_slot <= 3; i_slot++)
            read_absl_val(i_slot, &g_absl_val_cache[i_slot]);
    } else {
        for(i_slot = 0; i_slot <= 3; i_slot++){
            g_absl_val_cache[i_slot] = g_table_poly_val[i_slot][0];
            clear_pos(i_slot);
        }
    }
    if(UARTCharsAvail(RACK_UART_BASE)){
        UARTHandler();
    }
}

int
main(void)
{
    // Set the system clock to run at 80Mhz off PLL with external crystal as
    // reference.

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);

    // Enable the peripherals we will use
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    WatchdogReloadSet(WATCHDOG0_BASE, MY_WATCHDOG_TIMER);
    WatchdogResetEnable(WATCHDOG0_BASE);
    WatchdogStallEnable(WATCHDOG0_BASE);
    WatchdogEnable(WATCHDOG0_BASE);

    // Configure pins
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(RESET_BASE,RESET_PIN);
    GPIOPinTypeGPIOOutput(SYNC1_BASE, SYNC1_PIN);
    GPIOPinTypeGPIOOutput(SYNC2_BASE, SYNC2_PIN);
    GPIOPinTypeGPIOInput(RACK_CLK_BASE, RACK_CLK_PIN);
    GPIOPinTypeGPIOInput(START_BASE, START_PIN);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);

    // Enable the UART
    GPIOPinConfigure(GPIO_PC4_U1RX);
	GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_RX_PIN);
    GPIOPinTypeGPIOInput(RACK_UART_PORT, RACK_UART_TX_PIN);
    // 9 Bit Stuff
    UART9BitAddrSet(RACK_UART_BASE, UART_ADDRESS, 0xFF);
    UART9BitEnable(RACK_UART_BASE);
    UARTConfigSetExpClk(RACK_UART_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                    UART_CONFIG_PAR_ZERO));

    // Write the default pin state
    GPIOPinWrite(RESET_BASE,RESET_PIN,RESET_PIN);// (RESET_BAR - set high)
    GPIOPinWrite(SYNC1_BASE,SYNC1_PIN,SYNC1_PIN);// (SYNC1_BAR - set high)
    //PE4,PE5 (SYNC2_BAR, RACK_CLK_BAR - set high)
    GPIOPinWrite(SYNC2_BASE,SYNC2_PIN,SYNC2_PIN);

    // Setup SSI1
    SSIIntClear(SSI1_BASE,SSI_TXEOT);
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(),
            SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, SSI_FREQ, 8);
    SSIEnable(SSI1_BASE);

    // Wait for all the pins to be what they should
    SysCtlDelay(INIT_WAIT);

    set_range_mode_ssi(0);
    set_range_mode_ssi(1);
    set_range_mode_ssi(2);
    set_range_mode_ssi(3);

    // Set up interrupts
    IntMasterDisable();
    GPIOIntEnable(RACK_CLK_BASE,RACK_CLK_PIN);
    GPIOIntTypeSet(RACK_CLK_BASE,RACK_CLK_PIN,GPIO_RISING_EDGE);
    IntEnable(INT_GPIOE);

    while(UARTCharsAvail(RACK_UART_BASE))
        UARTCharGet(RACK_UART_BASE);

    // 9 Bit Stuff
    UARTFIFOLevelSet(RACK_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTIntEnable(RACK_UART_BASE, UART_INT_9BIT);
    UARTIntClear(RACK_UART_BASE, UARTIntStatus(RACK_UART_BASE, true));

    GPIOIntClear(RACK_CLK_BASE,0x1FF);
    IntMasterEnable();

    while(true)
    {

    }
}
