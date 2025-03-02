#include "config_dac.h"
#include "config_device.h"

bool  g_to_update = false;
// Global Variable Declaration
int32_t g_absl_val_cache[N_CHAN] = { 0, 0, 0, 0 };
int g_frame_pos[N_SLOT] = { 0, 0, 0, 0 }; // counting index
int32_t g_chkpt_pos[N_SLOT] = { 0, 0, 0, 0 }; // array index

// A list of durations (in units of cycles) for increment addition
// Note: if dt_i = -1, then the command resets g_absl_val_cache to zero
//       if dt_i = -2, then the command sets g_absl_val_cache to a val_i
int32_t g_table_poly_dur[N_SLOT][MAX_POS] = {{-2}, {-2}, {-2}, {-2}};
// During each dt_i, the corresponding val_i is constantly added to the voltage (modulo max voltage)
int32_t g_table_poly_val[N_SLOT][MAX_POS] = {{0x00008000 }, {0x00008000}, {0x00008000}, {0x00008000}};

int32_t g_n_chkpt[N_SLOT] = {2,2,2,2}; // effective array length
int32_t g_range_mode[N_CHAN] = {5,5,5,5}; // initial ranges
uint32_t g_checksum_uart = 0; // UART error detection

int32_t readu8_uart(){ // make a 16 bit number from 8 bit reads
    int32_t val1;
    val1 = UARTCharGet(RACK_UART_BASE);
    g_checksum_uart += val1;
    return (val1);
}

int32_t readi16_uart(){ // make a 16 bit number from 8 bit reads
    int32_t val1, val2;
    val1 = UARTCharGet(RACK_UART_BASE);
    val2 = UARTCharGet(RACK_UART_BASE);
    g_checksum_uart += val1 + val2;
    return (val1 << 8 | val2);
}

int32_t readi32_uart(){ // make a 32 bit number from 8 bit reads
    int32_t val1, val2, val3, val4;
    val1 = UARTCharGet(RACK_UART_BASE);
    val2 = UARTCharGet(RACK_UART_BASE);
    val3 = UARTCharGet(RACK_UART_BASE);
    val4 = UARTCharGet(RACK_UART_BASE);
    g_checksum_uart += val1 + val2 + val3 + val4;
    return( val1 << 24| val2 << 16 | val3 << 8 | val4 );
}

inline int32_t UARTCharGetAndLog(uint32_t uart_base) {
    int32_t val_char;
    val_char = UARTCharGet(uart_base);
#ifdef UART_BASE_DEBUG
    if (uart_base == UART_BASE_DEBUG) {
        g_debug_vals[g_debug_pos] = val_char;
        g_debug_pos++;
        if (g_debug_pos >= 500)
            g_debug_pos = 0;
    }
#endif
    return val_char;
}

inline int32_t UARTCharGetWithWait(uint32_t uart_base) {
    int a = 100000;
    while ( !UARTCharsAvail(uart_base) && (a > 0 )) {
        a--;
        if (a == 0) return -1;
    }
    return (UARTCharGetAndLog(uart_base));
}

inline void clear_pos(int i_slot) {
    g_chkpt_pos[i_slot]   = 0;
    g_frame_pos[i_slot] = 0;
}

inline void advance_chkpt_pos(int i_slot) {
    g_chkpt_pos[i_slot] += 1;
    g_frame_pos[i_slot] = 0;
}

inline void recede_chkpt_pos(int i_slot) {
    if ( g_chkpt_pos[i_slot] >= g_n_chkpt[i_slot] )
        g_chkpt_pos[i_slot] = g_n_chkpt[i_slot] - 1;
}

inline void advance_frame_pos(int i_slot) {
    g_frame_pos[i_slot] += 1;
    // Move to next command in the cycle
    if ( g_frame_pos[i_slot] >=
         g_table_poly_dur[i_slot][g_chkpt_pos[i_slot]] ) {
        g_chkpt_pos[i_slot] += 1;
        g_frame_pos[i_slot] = 0;
    }
}

inline void write_checksum_uart() {
    int i_tx;
    for(i_tx = 0; i_tx < N_SLOT; i_tx++)
        UARTCharPut(RACK_UART_BASE, g_checksum_uart >> ((3 - i_tx) * 8));
}

static inline void write_absl_val_ssi(int i_chan, uint32_t leading_chan)
{
    // Clear all interrupts if CLEAR_MODE is 1
    if(CLEAR_MODE)
        GPIOIntClear(RACK_CLK_BASE,0x1FF);

    GPIOPinWrite(SYNC2_BASE,SYNC2_PIN,0);
    SSIDataPutNonBlocking(SSI1_BASE, leading_chan);
    SSIDataPutNonBlocking(SSI1_BASE, g_absl_val_cache[i_chan] >> 8 +EXTRA_BITS);
    SSIDataPutNonBlocking(SSI1_BASE, g_absl_val_cache[i_chan] >> EXTRA_BITS);

    READY_SSI
    SysCtlDelay(1);
    GPIOPinWrite(SYNC2_BASE,SYNC2_PIN,SYNC2_PIN);
}

// Initialize the output drivers, or the range modes
// [?] what's the model anyway?
// AD5770 Ranges
// 0-5V		0101
// 0-10V	0110
// +/- 5V	0111
// +/- 10V	1000
// 
void set_range_mode_ssi(int i_chan)
{
    int w1, w2;
    w1 = (i_chan <<  5) | ((g_range_mode[i_chan] >> 1) & 0x7);
    w2 = (g_range_mode[i_chan] << 7) | 0x20;
    // Initialize Channel X Output Amplifier to range Y (Address 0bZZZ, see datasheet for AD5750)
    // format: ZZZ-(00)-YYYY-(0100000)
    GPIOPinWrite(SYNC1_BASE,SYNC1_PIN,0);
    SSIDataPutNonBlocking(SSI1_BASE,w1); //X-X-X-0-0-Y-Y-Y
    SSIDataPutNonBlocking(SSI1_BASE,w2); //Y-0-1-0-0-0-0-0
    SysCtlDelay(WAIT_FOR_SYNC);
    GPIOPinWrite(SYNC1_BASE,SYNC1_PIN,SYNC1_PIN);
}

void WDogIntHandler(void)
{
    // Toggle the red LED if we get get a watch trigger, indicating a clock error.
    WatchdogIntClear(WATCHDOG0_BASE);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}
