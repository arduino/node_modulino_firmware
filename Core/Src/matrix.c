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


int idxToPin(int idx) {
    switch (idx) {
        case 0: return 0;
        case 1: return 3;
        case 2: return 1;
        case 3: return 5;
        case 4: return 8;
        case 5: return 7;
        case 6: return 2;
        case 7: return 4;
        case 8: return 6;
        case 9: return 11;
        case 10: return 12;
    }
    return -1;
}

#define NUM_MATRIX_LEDS 96
static uint8_t __attribute__((aligned)) framebuffer[NUM_MATRIX_LEDS];

static void turnLed(int idx, bool on) {
    GPIOA->MODER = 0;

    if (on) {
        GPIOA->MODER |= (1 << (idxToPin(pins[idx][0]) * 2) | 1 << (idxToPin(pins[idx][1]) * 2));
        GPIOA->BSRR |= (1 << (idxToPin(pins[idx][0])) | 1 << (idxToPin(pins[idx][1]) + 16));
    }
}

void writeMatrix(uint32_t* buf) {
    memcpy(framebuffer, (uint8_t*)buf, NUM_MATRIX_LEDS);
}

void TIM3_IRQHandler() {
    static volatile int i_isr = 0;
    static volatile uint8_t pwm_counter = 0;

    // Horizontal layout mapping:
    // The framebuffer is organized as 8 rows * 12 columns bytes, where each byte represents an LED's brightness.
    // i_isr corresponds to the LED index in row-major order (0-11 is row 0, 12-23 is row 1, etc).
    // So the mapping is direct.
    
    // Simple software PWM
    uint8_t brightness = framebuffer[i_isr];

    // PWM logic: compare brightness against a rolling counter.
    // To maintain a reasonable refresh rate with 96 LEDs, we reduce the color depth resolution.
    // Incrementing pwm_counter by 32 gives 8 levels of brightness (256/32 = 8).
    // Total steps per full update cycle = 96 LEDs * 8 levels = 768 interrupts.
    // At e.g. 24kHz interrupt rate, that's ~31Hz refresh rate.
    
    bool on = (brightness > pwm_counter);
    turnLed(i_isr, on);
    
    i_isr = (i_isr + 1);
    if (i_isr >= NUM_MATRIX_LEDS) {
        i_isr = 0;
        pwm_counter = (pwm_counter + 32) % 256; 
    }
    
    HAL_TIM_IRQHandler(&htim3);
}
