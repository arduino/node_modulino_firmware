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
static volatile bool matrix_started = false;

static void turnLed(int idx, bool on) {
    GPIOA->MODER = 0;

    if (on) {
        GPIOA->MODER |= (1 << (idxToPin(pins[idx][0]) * 2) | 1 << (idxToPin(pins[idx][1]) * 2));
        GPIOA->BSRR |= (1 << (idxToPin(pins[idx][0])) | 1 << (idxToPin(pins[idx][1]) + 16));
    }
}

void writeMatrix(uint32_t* buf) {
    memcpy(framebuffer, (uint8_t*)buf, NUM_MATRIX_LEDS/2);
    matrix_started = true;
}

void TIM3_IRQHandler() {
    if (!matrix_started) {
        HAL_TIM_IRQHandler(&htim3);
        return;
    }

    static volatile int i_isr = 0;
    static volatile uint8_t pwm_counter = 0;

    // Horizontal layout mapping with 4-bit packed storage
    // i_isr corresponds to the LED index in row-major order.
    
    int byte_idx = i_isr / 2;
    int is_high_nibble = i_isr % 2;
    uint8_t nibble = is_high_nibble ? (framebuffer[byte_idx] >> 4) : (framebuffer[byte_idx] & 0x0F);
    
    // Simple software PWM
    // Input is 4-bit (0-15). Logic uses 8 levels (0-7) to maintain refresh rate.
    uint8_t brightness = nibble >> 1; // Map 0-15 -> 0-7

    // PWM logic: compare brightness against a rolling counter.
    // pwm_counter cycles 0..7
    // Total steps per full update cycle = 96 LEDs * 8 levels = 768 interrupts.
    
    bool on = (brightness > pwm_counter);
    turnLed(i_isr, on);
    
    i_isr = (i_isr + 1);
    if (i_isr >= NUM_MATRIX_LEDS) {
        i_isr = 0;
        pwm_counter = (pwm_counter + 1) % 8; 
    }
    
    HAL_TIM_IRQHandler(&htim3);
}
