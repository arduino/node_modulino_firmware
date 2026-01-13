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
static uint8_t __attribute__((aligned)) framebuffer[NUM_MATRIX_LEDS / 8];

static void turnLed(int idx, bool on) {
    GPIOA->MODER = 0; // Set all pins to hi-Z mode

    if (on) {
        // Set correct output levels BEFORE changing to output mode.
        // That way the levels are set while the pins are still in Input mode (Hi-Z), 
        // so it doesn't manifest on the pins before they are switched to Output mode.
        // When done in reverse order, the pins would drive whatever residual value
        // left in the output data register, causing ghosting.
        GPIOA->BSRR = (1 << (idxToPin(pins[idx][0])) | 1 << (idxToPin(pins[idx][1]) + 16));
        GPIOA->MODER |= (1 << (idxToPin(pins[idx][0]) * 2) | 1 << (idxToPin(pins[idx][1]) * 2));
    }
}

static uint32_t reverse(uint32_t x)
{
    x = ((x >> 1) & 0x55555555u) | ((x & 0x55555555u) << 1);
    x = ((x >> 2) & 0x33333333u) | ((x & 0x33333333u) << 2);
    x = ((x >> 4) & 0x0f0f0f0fu) | ((x & 0x0f0f0f0fu) << 4);
    x = ((x >> 8) & 0x00ff00ffu) | ((x & 0x00ff00ffu) << 8);
    x = ((x >> 16) & 0xffffu) | ((x & 0xffffu) << 16);
    return x;
}

void writeMatrix(uint32_t* buf) {
    memcpy(framebuffer, (uint32_t*)buf, NUM_MATRIX_LEDS/8);
}

void TIM3_IRQHandler() {
    static volatile int i_isr = 0;
    turnLed(i_isr, ((framebuffer[i_isr >> 3] & (1 << (i_isr % 8))) != 0));
    i_isr = (i_isr + 1) % NUM_MATRIX_LEDS;
    HAL_TIM_IRQHandler(&htim3);
}
