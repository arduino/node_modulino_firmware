typedef struct {
    uint32_t bsrr;
    uint32_t moder;
} LedMatrixLutEntry;
    
/* 
LUT to map an LED pixel index (idx) to the actual GPIOA pin
pairs with their corresponding BSRR and MODER values.
The calculation for each entry is:

// Table mapping LED indices to logical pin pairs
// These are not the actual GPIO pin numbers
static const uint8_t pins[][2] = {
    { 7, 3 },{ 3, 7 },{ 7, 4 },{ 4, 7 },{ 3, 4 },{ 4, 3 },{ 7, 8 },{ 8, 7 },{ 3, 8 },{ 8, 3 }, // 0 - 9    
    { 4, 8 },{ 8, 4 },{ 7, 0 },{ 0, 7 },{ 3, 0 },{ 0, 3 },{ 4, 0 },{ 0, 4 },{ 8, 0 },{ 0, 8 }, // 10 - 19
    { 7, 6 },{ 6, 7 },{ 3, 6 },{ 6, 3 },{ 4, 6 },{ 6, 4 },{ 8, 6 },{ 6, 8 },{ 0, 6 },{ 6, 0 }, // 20 - 29
    { 7, 5 },{ 5, 7 },{ 3, 5 },{ 5, 3 },{ 4, 5 },{ 5, 4 },{ 8, 5 },{ 5, 8 },{ 0, 5 },{ 5, 0 }, // 30 - 39
    { 6, 5 },{ 5, 6 },{ 7, 1 },{ 1, 7 },{ 3, 1 },{ 1, 3 },{ 4, 1 },{ 1, 4 },{ 8, 1 },{ 1, 8 }, // 40 - 49
    { 0, 1 },{ 1, 0 },{ 6, 1 },{ 1, 6 },{ 5, 1 },{ 1, 5 },{ 7, 2 },{ 2, 7 },{ 3, 2 },{ 2, 3 }, // 50 - 59
    { 4, 2 },{ 2, 4 },{ 8, 2 },{ 2, 8 },{ 0, 2 },{ 2, 0 },{ 6, 2 },{ 2, 6 },{ 5, 2 },{ 2, 5 }, // 60 - 69
    { 1, 2 },{ 2, 1 },{ 7, 10 },{ 10, 7 },{ 3, 10 },{ 10, 3 },{ 4, 10 },{ 10, 4 },{ 8, 10 },{ 10, 8 }, // 70 - 79
    { 0, 10 },{ 10, 0 },{ 6, 10 },{ 10, 6 },{ 5, 10 },{ 10, 5 },{ 1, 10 },{ 10, 1 },{ 2, 10 },{ 10, 2 }, // 80 - 89
    { 7, 9 },{ 9, 7 },{ 3, 9 },{ 9, 3 },{ 4, 9 },{ 9, 4 } // 90 - 95
};

// LUT to map logical pin numbers to actual GPIOA pin numbers (e.g. index 1 is PA3)
// This allows to remap logical pin numbers to physical pins easily for different board layouts.
static const uint8_t gpio_pin_lut[] = { 0, 3, 1, 5, 8, 7, 2, 4, 6, 11, 12 };

// Find the pair of GPIOA pins for the given LED index
uint8_t p1 = gpio_pin_lut[pins[idx][0]];
uint8_t p2 = gpio_pin_lut[pins[idx][1]];

// Calculate BSRR and MODER values to set those pins to output high
GPIOA->BSRR |= (1 << p1 | 1 << (p2 + 16));
GPIOA->MODER |= (1 << (p1 * 2) | 1 << (p2 * 2));
*/

static const LedMatrixLutEntry led_lut[96] = {
    { 0x00200010, 0x00000500 }, // LED 0: 7(PA4) -> 3(PA5)
    { 0x00100020, 0x00000500 }, // LED 1: 3(PA5) -> 7(PA4)
    { 0x01000010, 0x00010100 }, // LED 2: 7(PA4) -> 4(PA8)
    { 0x00100100, 0x00010100 }, // LED 3: 4(PA8) -> 7(PA4)
    { 0x01000020, 0x00010400 }, // LED 4: 3(PA5) -> 4(PA8)
    { 0x00200100, 0x00010400 }, // LED 5: 4(PA8) -> 3(PA5)
    { 0x00400010, 0x00001100 }, // LED 6: 7(PA4) -> 8(PA6)
    { 0x00100040, 0x00001100 }, // LED 7: 8(PA6) -> 7(PA4)
    { 0x00400020, 0x00001400 }, // LED 8: 3(PA5) -> 8(PA6)
    { 0x00200040, 0x00001400 }, // LED 9: 8(PA6) -> 3(PA5)
    { 0x00400100, 0x00011000 }, // LED 10: 4(PA8) -> 8(PA6)
    { 0x01000040, 0x00011000 }, // LED 11: 8(PA6) -> 4(PA8)
    { 0x00010010, 0x00000101 }, // LED 12: 7(PA4) -> 0(PA0)
    { 0x00100001, 0x00000101 }, // LED 13: 0(PA0) -> 7(PA4)
    { 0x00010020, 0x00000401 }, // LED 14: 3(PA5) -> 0(PA0)
    { 0x00200001, 0x00000401 }, // LED 15: 0(PA0) -> 3(PA5)
    { 0x00010100, 0x00010001 }, // LED 16: 4(PA8) -> 0(PA0)
    { 0x01000001, 0x00010001 }, // LED 17: 0(PA0) -> 4(PA8)
    { 0x00010040, 0x00001001 }, // LED 18: 8(PA6) -> 0(PA0)
    { 0x00400001, 0x00001001 }, // LED 19: 0(PA0) -> 8(PA6)
    { 0x00040010, 0x00000110 }, // LED 20: 7(PA4) -> 6(PA2)
    { 0x00100004, 0x00000110 }, // LED 21: 6(PA2) -> 7(PA4)
    { 0x00040020, 0x00000410 }, // LED 22: 3(PA5) -> 6(PA2)
    { 0x00200004, 0x00000410 }, // LED 23: 6(PA2) -> 3(PA5)
    { 0x00040100, 0x00010010 }, // LED 24: 4(PA8) -> 6(PA2)
    { 0x01000004, 0x00010010 }, // LED 25: 6(PA2) -> 4(PA8)
    { 0x00040040, 0x00001010 }, // LED 26: 8(PA6) -> 6(PA2)
    { 0x00400004, 0x00001010 }, // LED 27: 6(PA2) -> 8(PA6)
    { 0x00040001, 0x00000011 }, // LED 28: 0(PA0) -> 6(PA2)
    { 0x00010004, 0x00000011 }, // LED 29: 6(PA2) -> 0(PA0)
    { 0x00800010, 0x00004100 }, // LED 30: 7(PA4) -> 5(PA7)
    { 0x00100080, 0x00004100 }, // LED 31: 5(PA7) -> 7(PA4)
    { 0x00800020, 0x00004400 }, // LED 32: 3(PA5) -> 5(PA7)
    { 0x00200080, 0x00004400 }, // LED 33: 5(PA7) -> 3(PA5)
    { 0x00800100, 0x00014000 }, // LED 34: 4(PA8) -> 5(PA7)
    { 0x01000080, 0x00014000 }, // LED 35: 5(PA7) -> 4(PA8)
    { 0x00800040, 0x00005000 }, // LED 36: 8(PA6) -> 5(PA7)
    { 0x00400080, 0x00005000 }, // LED 37: 5(PA7) -> 8(PA6)
    { 0x00800001, 0x00004001 }, // LED 38: 0(PA0) -> 5(PA7)
    { 0x00010080, 0x00004001 }, // LED 39: 5(PA7) -> 0(PA0)
    { 0x00800004, 0x00004010 }, // LED 40: 6(PA2) -> 5(PA7)
    { 0x00040080, 0x00004010 }, // LED 41: 5(PA7) -> 6(PA2)
    { 0x00080010, 0x00000140 }, // LED 42: 7(PA4) -> 1(PA3)
    { 0x00100008, 0x00000140 }, // LED 43: 1(PA3) -> 7(PA4)
    { 0x00080020, 0x00000440 }, // LED 44: 3(PA5) -> 1(PA3)
    { 0x00200008, 0x00000440 }, // LED 45: 1(PA3) -> 3(PA5)
    { 0x00080100, 0x00010040 }, // LED 46: 4(PA8) -> 1(PA3)
    { 0x01000008, 0x00010040 }, // LED 47: 1(PA3) -> 4(PA8)
    { 0x00080040, 0x00001040 }, // LED 48: 8(PA6) -> 1(PA3)
    { 0x00400008, 0x00001040 }, // LED 49: 1(PA3) -> 8(PA6)
    { 0x00080001, 0x00000041 }, // LED 50: 0(PA0) -> 1(PA3)
    { 0x00010008, 0x00000041 }, // LED 51: 1(PA3) -> 0(PA0)
    { 0x00080004, 0x00000050 }, // LED 52: 6(PA2) -> 1(PA3)
    { 0x00040008, 0x00000050 }, // LED 53: 1(PA3) -> 6(PA2)
    { 0x00080080, 0x00004040 }, // LED 54: 5(PA7) -> 1(PA3)
    { 0x00800008, 0x00004040 }, // LED 55: 1(PA3) -> 5(PA7)
    { 0x00020010, 0x00000104 }, // LED 56: 7(PA4) -> 2(PA1)
    { 0x00100002, 0x00000104 }, // LED 57: 2(PA1) -> 7(PA4)
    { 0x00020020, 0x00000404 }, // LED 58: 3(PA5) -> 2(PA1)
    { 0x00200002, 0x00000404 }, // LED 59: 2(PA1) -> 3(PA5)
    { 0x00020100, 0x00010004 }, // LED 60: 4(PA8) -> 2(PA1)
    { 0x01000002, 0x00010004 }, // LED 61: 2(PA1) -> 4(PA8)
    { 0x00020040, 0x00001004 }, // LED 62: 8(PA6) -> 2(PA1)
    { 0x00400002, 0x00001004 }, // LED 63: 2(PA1) -> 8(PA6)
    { 0x00020001, 0x00000005 }, // LED 64: 0(PA0) -> 2(PA1)
    { 0x00010002, 0x00000005 }, // LED 65: 2(PA1) -> 0(PA0)
    { 0x00020004, 0x00000014 }, // LED 66: 6(PA2) -> 2(PA1)
    { 0x00040002, 0x00000014 }, // LED 67: 2(PA1) -> 6(PA2)
    { 0x00020080, 0x00004004 }, // LED 68: 5(PA7) -> 2(PA1)
    { 0x00800002, 0x00004004 }, // LED 69: 2(PA1) -> 5(PA7)
    { 0x00020008, 0x00000044 }, // LED 70: 1(PA3) -> 2(PA1)
    { 0x00080002, 0x00000044 }, // LED 71: 2(PA1) -> 1(PA3)
    { 0x10000010, 0x01000100 }, // LED 72: 7(PA4) -> 10(PA12)
    { 0x00101000, 0x01000100 }, // LED 73: 10(PA12) -> 7(PA4)
    { 0x10000020, 0x01000400 }, // LED 74: 3(PA5) -> 10(PA12)
    { 0x00201000, 0x01000400 }, // LED 75: 10(PA12) -> 3(PA5)
    { 0x10000100, 0x01010000 }, // LED 76: 4(PA8) -> 10(PA12)
    { 0x01001000, 0x01010000 }, // LED 77: 10(PA12) -> 4(PA8)
    { 0x10000040, 0x01001000 }, // LED 78: 8(PA6) -> 10(PA12)
    { 0x00401000, 0x01001000 }, // LED 79: 10(PA12) -> 8(PA6)
    { 0x10000001, 0x01000001 }, // LED 80: 0(PA0) -> 10(PA12)
    { 0x00011000, 0x01000001 }, // LED 81: 10(PA12) -> 0(PA0)
    { 0x10000004, 0x01000010 }, // LED 82: 6(PA2) -> 10(PA12)
    { 0x00041000, 0x01000010 }, // LED 83: 10(PA12) -> 6(PA2)
    { 0x10000080, 0x01004000 }, // LED 84: 5(PA7) -> 10(PA12)
    { 0x00801000, 0x01004000 }, // LED 85: 10(PA12) -> 5(PA7)
    { 0x10000008, 0x01000040 }, // LED 86: 1(PA3) -> 10(PA12)
    { 0x00081000, 0x01000040 }, // LED 87: 10(PA12) -> 1(PA3)
    { 0x10000002, 0x01000004 }, // LED 88: 2(PA1) -> 10(PA12)
    { 0x00021000, 0x01000004 }, // LED 89: 10(PA12) -> 2(PA1)
    { 0x08000010, 0x00400100 }, // LED 90: 7(PA4) -> 9(PA11)
    { 0x00100800, 0x00400100 }, // LED 91: 9(PA11) -> 7(PA4)
    { 0x08000020, 0x00400400 }, // LED 92: 3(PA5) -> 9(PA11)
    { 0x00200800, 0x00400400 }, // LED 93: 9(PA11) -> 3(PA5)
    { 0x08000100, 0x00410000 }, // LED 94: 4(PA8) -> 9(PA11)
    { 0x01000800, 0x00410000 }, // LED 95: 9(PA11) -> 4(PA8)
};


#define NUM_MATRIX_LEDS 96
// Declare external flag from main.c
extern bool ledMatrixGrayscaleMode;

// A maximum of 48 bytes needed for 4-bit grayscale mode (96 LEDs * 4 bits = 384 bits = 48 bytes)
static uint8_t __attribute__((aligned)) framebuffer[NUM_MATRIX_LEDS / 2];

// Using static inline function with always_inline attribute
// to ensure the function is inlined for performance in ISR even with -Os optimization.
static inline void __attribute__((always_inline)) turnLed(uint8_t idx, bool on) {
    // Set all matrix pins to Input (Hi-Z) to prevent ghosting,
    // while preserving special function pins (SWD, UART, etc.).
    //
    // Mask 0xFC3C0000 details:
    // Bits 31-28 (0xF): Preserves PA15, PA14 (SWDCLK)
    // Bits 27-24 (0xC): Preserves PA13 (SWDIO), clears PA12
    // Bits 23-20 (0x3): Clears PA11, preserves PA10
    // Bits 19-16 (0xC): Preserves PA9, clears PA8
    // Bits 15-00 (0x0): Clears PA0-PA7
    //
    // Result: Clears PA12, PA11, PA8, PA0-PA7 (All Matrix Pins)
    // Preserves PA15, PA14, PA13, PA10, PA9 
    GPIOA->MODER &= 0xFC3C0000; 

    if (on) {
        // Set correct output levels BEFORE changing to output mode.
        // That way the levels are set while the pins are still in Input mode (Hi-Z), 
        // so it doesn't manifest on the pins before they are switched to Output mode.
        // When done in reverse order, the pins would drive whatever residual value
        // left in the output data register, causing ghosting.
        GPIOA->BSRR = led_lut[idx].bsrr;
        GPIOA->MODER |= led_lut[idx].moder;
    }
}

void writeMatrix(uint8_t* buf) {
    if(ledMatrixGrayscaleMode){
        memcpy(framebuffer, buf, NUM_MATRIX_LEDS/2);
    } else {
        // Monochrome mode, each bit represents one LED
        memcpy(framebuffer, buf, NUM_MATRIX_LEDS/8);
    }
}

void TIM3_IRQHandler() {
    // Clear Update Interrupt Flag at the start to avoid ghost interrupts 
    // since clearing the flag might take several clock cycles to propagate.
    // Direclty assigning the SR register instead of calling HAL_TIM_IRQHandler(&htim3) for efficiency.    
    // We use direct assignment (=) rather than RMW (&=) to avoid 
    // accidentally clearing other flags that might have triggered during the RMW operation. 
    // Writing 0 clears the bit, writing 1 has no effect (rc_w0).
    // We skip "if (TIM3->SR & UIF)" because TIM3 only triggers this one 
    // interrupt type (Update) in our config. Checking costs unnecessary CPU cycles.
    TIM3->SR = ~TIM_SR_UIF;

    // Optimization: Skipped read-back of SR register ((void)TIM3->SR).
    // The ISR is sufficiently long (>70 cycles) and clocks are 1:1 synchronous,
    // guaranteeing the write buffer flushes before the ISR exits.
    // If it were shorter, ISR might be fast enough to exit before the generic write buffer 
    // commits the write to the peripheral, causing the NVIC to re-fire the interrupt

    // Since these variables are static and local to the ISR, and not modified 
    // by any other code or hardware, they do not need to be volatile.
    // Optimization: Group variables in a struct to allow base-relative addressing
    // saving register re-loads.
    static struct {
        uint8_t i_isr;
        uint8_t pwm_counter;
        uint8_t row; 
        uint8_t col; 
    } state = {0, 0, 0, 0};

    if(ledMatrixGrayscaleMode){
        // Horizontal layout mapping with 4-bit packed storage
        // i_isr corresponds to the LED index in row-major order.
        
        uint8_t byte_idx = state.i_isr / 2;
        // Host sends data as High Nibble (Even LED) then Low Nibble (Odd LED)
        // E.g. 0xF0 means LED 0 = 15, LED 1 = 0.
        bool is_even_led = (state.i_isr % 2 == 0);
        uint8_t nibble = is_even_led ? (framebuffer[byte_idx] >> 4) : (framebuffer[byte_idx] & 0x0F);
        
        // PWM logic: compare brightness against a rolling counter.
        // pwm_counter cycles 0..15 (16 levels)
        uint8_t brightness = nibble; 
        
        bool on = (brightness > state.pwm_counter);
        turnLed(state.i_isr, on);
        
        // Increment LED index
        state.i_isr++;
        if (state.i_isr >= NUM_MATRIX_LEDS) {
            state.i_isr = 0;
            // Increment PWM cycle (0-15)
            state.pwm_counter = (state.pwm_counter + 1) & 0x0F; 
        }
    } else {
        // Vertical layout mapping:
        // The framebuffer is organized as 12 bytes, where each byte represents a column.
        // i_isr corresponds to the LED index in row-major order (0-11 is row 0, 12-23 is row 1, etc).
        
        // Resync logic in case i_isr was reset externally or on mode switch (though i_isr is static)
        // Since i_isr corresponds to row*12 + col, checking for 0 is safe synchronization.
        if (state.i_isr == 0) {
            state.row = 0;
            state.col = 0;
        }

        turnLed(state.i_isr, ((framebuffer[state.col] & (1 << state.row)) != 0));

        // Increment logic matching (i_isr / 12) / (i_isr % 12)
        state.col++;
        if (state.col >= 12) {
            state.col = 0;
            state.row++;
            // No need to reset row here as i_isr reset handles it
        }

        state.i_isr++;
        if (state.i_isr >= NUM_MATRIX_LEDS) {
            state.i_isr = 0;
            // row/col will be reset at start of next call
        }
    }
}
