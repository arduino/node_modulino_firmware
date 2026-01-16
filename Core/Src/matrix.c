// Table mapping LED indices to GPIOA pin pairs
static const uint8_t pins[][2] = {
    { 7, 3 }, // 0
    { 3, 7 },
    { 7, 4 },
    { 4, 7 },
    { 3, 4 },
    { 4, 3 },
    { 7, 8 },
    { 8, 7 },
    { 3, 8 },
    { 8, 3 },
    { 4, 8 }, // 10
    { 8, 4 },
    { 7, 0 },
    { 0, 7 },
    { 3, 0 },
    { 0, 3 },
    { 4, 0 },
    { 0, 4 },
    { 8, 0 },
    { 0, 8 },
    { 7, 6 }, // 20
    { 6, 7 },
    { 3, 6 },
    { 6, 3 },
    { 4, 6 },
    { 6, 4 },
    { 8, 6 },
    { 6, 8 },
    { 0, 6 },
    { 6, 0 },
    { 7, 5 }, // 30
    { 5, 7 },
    { 3, 5 },
    { 5, 3 },
    { 4, 5 },
    { 5, 4 },
    { 8, 5 },
    { 5, 8 },
    { 0, 5 },
    { 5, 0 },
    { 6, 5 }, // 40
    { 5, 6 },
    { 7, 1 },
    { 1, 7 },
    { 3, 1 },
    { 1, 3 },
    { 4, 1 },
    { 1, 4 },
    { 8, 1 },
    { 1, 8 },
    { 0, 1 }, // 50
    { 1, 0 },
    { 6, 1 },
    { 1, 6 },
    { 5, 1 },
    { 1, 5 },
    { 7, 2 },
    { 2, 7 },
    { 3, 2 },
    { 2, 3 },
    { 4, 2 },
    { 2, 4 },
    { 8, 2 },
    { 2, 8 },
    { 0, 2 },
    { 2, 0 },
    { 6, 2 },
    { 2, 6 },
    { 5, 2 },
    { 2, 5 },
    { 1, 2 },
    { 2, 1 },
    { 7, 10 },
    { 10, 7 },
    { 3, 10 },
    { 10, 3 },
    { 4, 10 },
    { 10, 4 },
    { 8, 10 },
    { 10, 8 },
    { 0, 10 },
    { 10, 0 },
    { 6, 10 },
    { 10, 6 },
    { 5, 10 },
    { 10, 5 },
    { 1, 10 },
    { 10, 1 },
    { 2, 10 },
    { 10, 2 },
    { 7, 9 },
    { 9, 7 },
    { 3, 9 },
    { 9, 3 },
    { 4, 9 },
    { 9, 4 },
  };

// LUT to map logical pin numbers to actual GPIOA pin numbers
static const uint8_t pin_lut[] = { 0, 3, 1, 5, 8, 7, 2, 4, 6, 11, 12 };

#define NUM_MATRIX_LEDS 96
// Declare external flag from main.c
extern bool ledMatrixGrayscaleMode;

static uint8_t __attribute__((aligned)) framebuffer[NUM_MATRIX_LEDS / 2];

static inline void turnLed(int idx, bool on) {
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
        // Optimized pin lookup from static const table
        uint8_t p1 = pin_lut[pins[idx][0]];
        uint8_t p2 = pin_lut[pins[idx][1]];

        // Set correct output levels BEFORE changing to output mode.
        // That way the levels are set while the pins are still in Input mode (Hi-Z), 
        // so it doesn't manifest on the pins before they are switched to Output mode.
        // When done in reverse order, the pins would drive whatever residual value
        // left in the output data register, causing ghosting.
        GPIOA->BSRR |= (1 << p1 | 1 << (p2 + 16));
        GPIOA->MODER |= (1 << (p1 * 2) | 1 << (p2 * 2));
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

    static volatile int i_isr = 0;
    static volatile uint8_t pwm_counter = 0;

    if(ledMatrixGrayscaleMode){
        // Horizontal layout mapping with 4-bit packed storage
        // i_isr corresponds to the LED index in row-major order.
        
        int byte_idx = i_isr / 2;
        // Host sends data as High Nibble (Even LED) then Low Nibble (Odd LED)
        // E.g. 0xF0 means LED 0 = 15, LED 1 = 0.
        bool is_even_led = (i_isr % 2 == 0);
        uint8_t nibble = is_even_led ? (framebuffer[byte_idx] >> 4) : (framebuffer[byte_idx] & 0x0F);
        
        // PWM logic: compare brightness against a rolling counter.
        // pwm_counter cycles 0..15 (16 levels)
        uint8_t brightness = nibble; 
        
        bool on = (brightness > pwm_counter);
        turnLed(i_isr, on);
        
        // Increment LED index
        i_isr++;
        if (i_isr >= NUM_MATRIX_LEDS) {
            i_isr = 0;
            // Increment PWM cycle (0-15)
            pwm_counter = (pwm_counter + 1) & 0x0F; 
        }
    } else {
        // Vertical layout mapping:
        // The framebuffer is organized as 12 bytes, where each byte represents a column.
        // i_isr corresponds to the LED index in row-major order (0-11 is row 0, 12-23 is row 1, etc).
        
        // Optimization: Use stateful counters to avoid costly division/modulo by 12 in ISR
        static uint8_t row = 0;
        static uint8_t col = 0;

        // Resync logic in case i_isr was reset externally or on mode switch (though i_isr is static)
        // Since i_isr corresponds to row*12 + col, checking for 0 is safe synchronization.
        if (i_isr == 0) {
            row = 0;
            col = 0;
        }

        turnLed(i_isr, ((framebuffer[col] & (1 << row)) != 0));

        // Increment logic matching (i_isr / 12) / (i_isr % 12)
        col++;
        if (col >= 12) {
            col = 0;
            row++;
            // No need to reset row here as i_isr reset handles it
        }

        i_isr++;
        if (i_isr >= NUM_MATRIX_LEDS) {
            i_isr = 0;
            // row/col will be reset at start of next call
        }
    }
}
