#include "motors.h"
#include <string.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

// Internal State
static volatile MotorMode currentMode = MOTOR_MODE_DC;
static volatile int32_t stepper_steps_remaining = 0;
static volatile bool stepper_direction = true; // true = forward
static volatile uint8_t stepper_phase = 0; // 0-7 for 8-step sequence
static volatile bool stepper_half_step = false; // Default Full Step (Two-Phase On)
static volatile bool motor_busy = false;
static uint8_t motor_decay_mode = 0;
static volatile uint8_t stepper_release_delay_ms = 0;
static volatile bool stepper_settle_pending = false;
static volatile bool stepper_release_pending = false;
static volatile uint16_t stepper_period_ticks = 0;
static volatile uint16_t stepper_release_delay_ticks = 0;
// stepper_settle_pending and stepper_release_pending share TIM3 as a tiny
// one-shot scheduler: first for bridge settle, later for delayed coil release.

// Current Sensing State
static uint16_t motor_current_a = 0;
static uint16_t motor_current_b = 0;
static uint32_t last_current_read_time = 0;
static uint16_t motor_current_baseline_a = 0;
static uint16_t motor_current_baseline_b = 0;
static bool motor_current_baseline_valid = false;

// HFS pin state (PC15 -> MAX22211 HFS). false = low (KISEN=7500, full range).
static bool motor_hfs_enabled = false;

// Track latest commanded DC speeds so baseline is only learned at true idle.
static int16_t motor_speed_cmd_a = 0;
static int16_t motor_speed_cmd_b = 0;

/**
 * @brief  Reads the current sense value from a specific ADC channel.
 * @param  channel The ADC channel to read.
 * @retval 12-bit ADC reading, averaged over several samples.
 * @note   Used in: Motor_Update() to sample ADC values for A and B motor currents independently, 
 *         which is necessary to maintain an adaptive baseline and calculate the effective motor current.
 */
static uint16_t ReadCurrentSenseChannel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t accumulated = 0;
    uint8_t valid_samples = 0;

    // With ADC scan-direction-forward mode, explicitly clear the other ISEN
    // channel so the conversion starts from the intended single channel.
    sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
    sConfig.Rank = ADC_RANK_NONE;
    sConfig.Channel = (channel == ADC_CHANNEL_4) ? ADC_CHANNEL_5 : ADC_CHANNEL_4;
    (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.Channel = channel;
    sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0;
    }

    // Discard the first conversion after a channel switch so the ADC sampling
    // capacitor has time to settle on the new ISEN node.
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        (void)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    for (uint8_t sample_index = 0; sample_index < 4; sample_index++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            accumulated += HAL_ADC_GetValue(&hadc1);
            valid_samples++;
        }
        HAL_ADC_Stop(&hadc1);
    }

    if (valid_samples == 0) {
        return 0;
    }

    return (uint16_t)(accumulated / valid_samples);
}

// Pin Definitions
#define PIN_DIN1A   GPIO_PIN_0  // PA0
#define PIN_DIN2A   GPIO_PIN_1  // PA1
#define PIN_DIN1B   GPIO_PIN_2  // PA2
#define PIN_DIN2B   GPIO_PIN_3  // PA3
#define PIN_ISENA   GPIO_PIN_4  // PA4, Current Sense A
#define PIN_ISENB   GPIO_PIN_5  // PA5, Current Sense B
#define PIN_ENA     GPIO_PIN_7  // PA7, Bridge A Enable
#define PIN_ENB     GPIO_PIN_8  // PA8, Bridge B Enable
#define PIN_DECAY1  GPIO_PIN_11 // PA11
#define PIN_DECAY2  GPIO_PIN_12 // PA12

#define AF_DIN1A    GPIO_AF5_TIM1 // PA0 -> TIM1_CH1
#define AF_DIN2A    GPIO_AF5_TIM1 // PA1 -> TIM1_CH2
#define AF_DIN1B    GPIO_AF5_TIM1 // PA2 -> TIM1_CH3
#define AF_DIN2B    GPIO_AF5_TIM1 // PA3_ALT1 -> TIM1_CH4

#define PORT_BRIDGE GPIOA
#define STEPPER_SETTLE_TICKS 20U // 2.0ms at 0.1ms/tick

/**
 * @brief  Converts a release delay in milliseconds to TIM3 timer ticks.
 * @param  delay_ms The delay in milliseconds.
 * @retval Converted duration in TIM3 ticks.
 * @note   Used in: Motor_CommandStepper() to convert the user's release delay parameter 
 *         into hardware timer ticks for the one-shot release scheduler.
 */
static uint16_t Stepper_ReleaseDelayMsToTicks(uint8_t delay_ms) {
    // TIM3 tick is 0.1ms, so 1ms = 10 ticks.
    return (uint16_t)delay_ms * 10U;
}

/**
 * @brief  Enters a critical section, disabling interrupts.
 * @retval Original primask state to be used with Motor_ExitCritical.
 * @note   Used in: Motor_CommandStepper() to prevent stepper timer IRQs (TIM3) from 
 *         firing while modifying shared multithreading variables like stepper direction/speed.
 */
static uint32_t Motor_EnterCritical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief  Exits a critical section, restoring interrupt state.
 * @param  primask The primask state previously returned by Motor_EnterCritical.
 * @note   Used in: Motor_CommandStepper() to safely restore default system interrupt 
 *         capabilities after modifying stepper state variables.
 */
static void Motor_ExitCritical(uint32_t primask) {
    __set_PRIMASK(primask);
}

/**
 * @brief  Disables the stepper motor driver outputs to remove holding torque.
 * @note   Used in: TIM3_IRQHandler() to disable outputs dynamically when the delayed 
 *         release timer successfully expires.
 */
static void Stepper_ReleaseOutputs(void) {
    HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN1A | PIN_DIN2A | PIN_DIN1B | PIN_DIN2B, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_ENA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_ENB, GPIO_PIN_RESET);
}

/**
 * @brief  Enables the stepper motor driver outputs.
 * @note   Used in: Motor_SetMode() and Motor_CommandStepper() to awaken the H-bridges 
 *         so they can begin driving coils aggressively for movement or holding torque.
 */
static void Stepper_EnableOutputs(void) {
    HAL_GPIO_WritePin(GPIOA, PIN_ENA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, PIN_ENB, GPIO_PIN_SET);
}

// Helper: Configure GPIO pins
/**
 * @brief  Configures a GPIO pin as a standard push-pull output.
 * @param  pin The GPIO_PIN_x to be configured on PORTA.
 * @note   Used in: Motor_SetMode() to assign H-bridge pins to manual GPIO outputs 
 *         when switching from DC mode (PWM) to Stepper mode (bit-banged).
 */
static void ConfigPin_Output(uint32_t pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  Configures a GPIO pin for alternate function PWM output.
 * @param  pin The GPIO_PIN_x to configure on PORTA.
 * @param  alternate The alternate function macro (e.g., GPIO_AF5_TIM1).
 * @note   Used in: Motor_SetMode() to map hardware timer (TIM1) signals to the pins 
 *         for smooth hardware-level DC motor control instead of Stepper outputs.
 */
static void ConfigPin_PWM(uint32_t pin, uint32_t alternate) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = alternate;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Bipolar Stepper Sequence (8-Step, Half-Stepping base)
// 0: A+, B0
// 1: A+, B+ (Full Step 1)
// 2: A0, B+
// 3: A-, B+ (Full Step 2)
// 4: A-, B0
// 5: A-, B- (Full Step 3)
// 6: A0, B-
// 7: A+, B- (Full Step 4)
/**
 * @brief  Applies the chosen phase logically defining H-bridge coil states.
 * @param  phase The 0-7 phase configuration reflecting the internal stepping state machine.
 * @note   Used in: Motor_CommandStepper() and TIM3_IRQHandler() to actually transition 
 *         the physical motor hardware accurately following the stepper rotation sequences.
 */
static void Stepper_ApplyPhase(uint8_t phase) {
    // Phase mapping to Pins
    // A+ = DIN1A:H, DIN2A:L
    // A- = DIN1A:L, DIN2A:H
    // A0 = DIN1A:L, DIN2A:L (Brake/Off) -> Ideally use EN=L for Hi-Z but Brake is OK for holding
    
    // Reset all to Low first (Brake state)
    HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN1A | PIN_DIN2A | PIN_DIN1B | PIN_DIN2B, GPIO_PIN_RESET);

    switch (phase & 0x07) {
        case 0: // A+, B0
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN1A, GPIO_PIN_SET);
            break;
        case 1: // A+, B+
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN1A | PIN_DIN1B, GPIO_PIN_SET);
            break;
        case 2: // A0, B+
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN1B, GPIO_PIN_SET);
            break;
        case 3: // A-, B+
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN2A | PIN_DIN1B, GPIO_PIN_SET);
            break;
        case 4: // A-, B0
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN2A, GPIO_PIN_SET);
            break;
        case 5: // A-, B-
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN2A | PIN_DIN2B, GPIO_PIN_SET);
            break;
        case 6: // A0, B-
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN2B, GPIO_PIN_SET);
            break;
        case 7: // A+, B-
            HAL_GPIO_WritePin(PORT_BRIDGE, PIN_DIN1A | PIN_DIN2B, GPIO_PIN_SET);
            break;
    }
}

/**
 * @brief  Initializes peripheral clocks, GPIOs, ADC currents, and hardware timers.
 * @note   Used in: Application boot (main logic/system init) to ready everything 
 *         required strictly for motor control before handling commands.
 */
void Motor_Init(void) {
    // 1. Enable Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE(); // For DC Motor PWM
    __HAL_RCC_TIM3_CLK_ENABLE(); // For Stepper Timing
    
    // 2. Configure Pin Constants (Enable & Decay)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PIN_ENA | PIN_ENB | PIN_DECAY1 | PIN_DECAY2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Initial State: Disabled, Fast Decay (Low)
    HAL_GPIO_WritePin(GPIOA, PIN_ENA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_ENB, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_DECAY1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_DECAY2, GPIO_PIN_RESET);

    // 3. Configure ADC Current Sense Pins (PA4, PA5)
    __HAL_RCC_ADC_CLK_ENABLE();
    GPIO_InitStruct.Pin = PIN_ISENA | PIN_ISENB;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 4a. Configure HFS pin (PC15) as output, default low (full current range)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

    // 4. Initialize ADC1
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    HAL_ADC_Init(&hadc1);
    
    HAL_ADCEx_Calibration_Start(&hadc1);

    // 5. Setup TIM3 for Stepper Interrupts (Base initialization)
    // Clock is 12MHz (HSI/4)
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 1200 - 1; // 12MHz / 1200 = 10kHz tick (0.1ms)
    htim3.Init.Period = 100; // Default period
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim3);
    
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0); // Lower priority than I2C/Comms
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    // 6. Configure TIM1 for DC Motor PWM (20kHz)
    // Clock is 12MHz. Target 20kHz.
    // ARR = (12,000,000 / 20,000) - 1 = 599
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = (12000000 / 20000) - 1; 
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    // Configure each PWM channel (required for output to work)
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // Start at 0% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
    
    // Configure Break/Dead-Time (required for TIM1 advanced timer functionality)
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    // 7. Default Mode Set
    Motor_SetMode(MOTOR_MODE_DC);
}

/**
 * @brief  Sets the overall operating mode (DC driving or Stepper mode).
 * @param  mode The active MotorMode selection.
 * @note   Used in: Motor_Init() and Motor_HandleCommand() to dynamically re-allocate 
 *         the driver pins between PWM generation and direct GPIO phase toggling based on needs.
 */
void Motor_SetMode(MotorMode mode) {
    // Disable outputs during switch
    HAL_GPIO_WritePin(GPIOA, PIN_ENA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_ENB, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim3);

    currentMode = mode;

    if (mode != MOTOR_MODE_DC) {
        motor_speed_cmd_a = 0;
        motor_speed_cmd_b = 0;
    }

    if (mode == MOTOR_MODE_DC) {
        // Configure pins as Alternate Function for TIM1 PWM
        ConfigPin_PWM(PIN_DIN1A, AF_DIN1A);
        ConfigPin_PWM(PIN_DIN2A, AF_DIN2A);
        ConfigPin_PWM(PIN_DIN1B, AF_DIN1B);
        ConfigPin_PWM(PIN_DIN2B, AF_DIN2B);

        // Enable Drivers
        Stepper_EnableOutputs();

        // Start PWMs with 0 duty cycle
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    } else {
        // MOTOR_MODE_STEPPER
        // Configure Pins as GPIO Output
        ConfigPin_Output(PIN_DIN1A);
        ConfigPin_Output(PIN_DIN2A);
        ConfigPin_Output(PIN_DIN1B);
        ConfigPin_Output(PIN_DIN2B);
        
        // Enable Drivers
        Stepper_EnableOutputs();
        
        // Start TIM3 for step timing
        // Timer will be running but period updated by command
        // HAL_TIM_Base_Start_IT(&htim3); // Only start when moving
    }
}

/**
 * @brief  Sets relative DC speed mapping to hardware PWM ratio.
 * @param  motor Target motor (MOTOR_A or MOTOR_B).
 * @param  speed Input relative speed scalar (-32767 to 32767).
 * @note   Used in: Motor_HandleCommand() to parse host requests for DC rotation 
 *         and smoothly adjust active hardware timers on demand.
 */
void Motor_SetDCSpeed(uint8_t motor, int16_t speed) {
    if (currentMode != MOTOR_MODE_DC) return;

    // Constrain speed
    if (speed > MOTOR_DC_SPEED_MAX) speed = MOTOR_DC_SPEED_MAX;
    if (speed < -MOTOR_DC_SPEED_MAX) speed = -MOTOR_DC_SPEED_MAX;

    uint32_t ch_fwd, ch_rev;
    uint32_t en_pin;
    
    // Map channels
    // Motor A: CH1 (DIN1), CH2 (DIN2)
    // Motor B: CH3 (DIN3), CH4 (DIN4)
    if (motor == MOTOR_A) {
        ch_fwd = TIM_CHANNEL_1;
        ch_rev = TIM_CHANNEL_2;
        en_pin = PIN_ENA;
        motor_speed_cmd_a = speed;
    } else {
        ch_fwd = TIM_CHANNEL_3;
        ch_rev = TIM_CHANNEL_4;
        en_pin = PIN_ENB;
        motor_speed_cmd_b = speed;
    }

    uint32_t duty;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    if (speed >= 0) {
        duty = ((uint32_t)speed * period) / MOTOR_DC_SPEED_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, ch_fwd, duty);
        __HAL_TIM_SET_COMPARE(&htim1, ch_rev, 0);
    } else {
        duty = ((uint32_t)(-speed) * period) / MOTOR_DC_SPEED_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, ch_fwd, 0);
        __HAL_TIM_SET_COMPARE(&htim1, ch_rev, duty);
    }

    // Keep inactive bridge in Hi-Z instead of brake mode to reduce coupling
    // into current-sense telemetry when only one motor is running.
    HAL_GPIO_WritePin(GPIOA, en_pin, (speed == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  Adjusts the base operational driving frequency (PWM carrier).
 * @param  frequency Desired frequency in Hz (200 - 60k).
 * @note   Used in: Motor_HandleCommand() so users can dynamically shift motor PWM tone 
 *         for varying load characteristics or acoustic requirements.
 */
void Motor_SetDCFrequency(uint16_t frequency) {
    if (currentMode != MOTOR_MODE_DC) return;
    
    // Limit frequency to hardware capabilities (12MHz clock, 16-bit ARR, PSC=0)
    // Min: 12M / 65536 ~= 184 Hz
    // Max: Arbitrary reasonable limit (e.g. 100kHz)
    if (frequency < 200) frequency = 200;
    if (frequency > 60000) frequency = 60000;

    uint32_t arr = (12000000 / frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
}


/**
 * @brief  Commands the stepper system to step a specified layout.
 * @param  steps Amount to turn; negative represents backwards.
 * @param  speed Time scalar affecting phase activation gaps.
 * @param  releaseDelayMs Delay after completing sequences before killing bridge outputs.
 * @note   Used in: Motor_HandleCommand() to translate a host request for stepped 
 *         movement into physical hardware timer parameters effectively queuing motion.
 */
void Motor_CommandStepper(int32_t steps, uint16_t speed, uint8_t releaseDelayMs) {
    if (currentMode != MOTOR_MODE_STEPPER) return;

    uint32_t primask = Motor_EnterCritical();
    HAL_TIM_Base_Stop_IT(&htim3);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    stepper_settle_pending = false;
    stepper_release_pending = false;
    stepper_period_ticks = speed;
    stepper_release_delay_ms = releaseDelayMs;
    stepper_release_delay_ticks = Stepper_ReleaseDelayMsToTicks(releaseDelayMs);
    stepper_direction = (steps > 0);
    
    if (steps == 0 || speed == 0) {
        stepper_steps_remaining = 0;
        motor_busy = false;
        if (stepper_release_delay_ticks > 0) {
            // No movement requested: keep current phase briefly energized,
            // then release outputs after the configured delay.
            __HAL_TIM_SET_AUTORELOAD(&htim3, stepper_release_delay_ticks);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            stepper_release_pending = true;
            HAL_TIM_Base_Start_IT(&htim3);
        } else {
            // Zero-delay means hold torque at the current phase.
            Stepper_EnableOutputs();
            Stepper_ApplyPhase(stepper_phase);
        }
        Motor_ExitCritical(primask);
        return;
    }

    bool outputs_were_released =
        (HAL_GPIO_ReadPin(GPIOA, PIN_ENA) == GPIO_PIN_RESET) &&
        (HAL_GPIO_ReadPin(GPIOA, PIN_ENB) == GPIO_PIN_RESET);
    Stepper_EnableOutputs();

    // In full-step mode, ensure we start from a two-phase-on state.
    // Normalize once here so each ISR tick advances by a uniform phase delta.
    if (!stepper_half_step && ((stepper_phase & 0x01U) == 0U)) {
        stepper_phase = (uint8_t)(stepper_phase + (stepper_direction ? 1 : -1));
    }
    Stepper_ApplyPhase(stepper_phase);

    stepper_steps_remaining = (steps > 0) ? steps : -steps;

    if (outputs_were_released) {
        // If outputs were released, wait one short settle interval before the
        // first step so bridge current can stabilize.
        stepper_settle_pending = true;
        uint16_t first_interval = STEPPER_SETTLE_TICKS;
        __HAL_TIM_SET_AUTORELOAD(&htim3, first_interval);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        motor_busy = true;
        HAL_TIM_Base_Start_IT(&htim3);
        Motor_ExitCritical(primask);
        return;
    }

    // Execute the first step immediately so short moves do not wait one period
    // before any motion starts.
    uint8_t increment = stepper_half_step ? 1 : 2;
    if (stepper_direction) {
        stepper_phase += increment;
    } else {
        stepper_phase -= increment;
    }
    Stepper_ApplyPhase(stepper_phase);
    stepper_steps_remaining--;

    if (stepper_steps_remaining == 0) {
        motor_busy = false;
        if (stepper_release_delay_ticks > 0) {
            // Single-step move completed immediately; arm delayed release.
            stepper_release_pending = true;
            __HAL_TIM_SET_AUTORELOAD(&htim3, stepper_release_delay_ticks);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            HAL_TIM_Base_Start_IT(&htim3);
        }
        Motor_ExitCritical(primask);
        return;
    }
    
    // Set timer period based on 'speed'
    // This is raw, user should convert RPM/PPS to period ticks
    // Assuming 'speed' is period in 0.1ms units (1 to 65535)
    
    __HAL_TIM_SET_AUTORELOAD(&htim3, speed);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    motor_busy = true;
    HAL_TIM_Base_Start_IT(&htim3);
    Motor_ExitCritical(primask);
}

/**
 * @brief  Receives raw I2C command arrays mapping directly out to motor functionality.
 * @param  buffer Byte data indicating command category and arguments.
 * @note   Used in: Primary external comms parser to easily funnel host-level routines 
 *         into direct action upon driver variables.
 */
void Motor_HandleCommand(const uint8_t *buffer) {
    if (buffer == NULL) {
        return;
    }

    // Handle Mode Switch
    if (buffer[0] == CMD_MOTOR_MODE) {
        Motor_SetMode(buffer[1] == 1 ? MOTOR_MODE_STEPPER : MOTOR_MODE_DC);
    }
    // Handle DC Motor Speed Command [ 'S', speedA_L, speedA_H, speedB_L, speedB_H ]
    // Speeds are signed 16-bit full scale: -32767..32767.
    else if (buffer[0] == CMD_MOTOR_SPEED_DC) {
        int16_t speedA, speedB;
        memcpy(&speedA, &buffer[1], 2);
        memcpy(&speedB, &buffer[3], 2);
        Motor_SetDCSpeed(MOTOR_A, speedA);
        Motor_SetDCSpeed(MOTOR_B, speedB);
    }
    // Handle DC Motor Frequency Command [ 'F', freq_L, freq_H ]
    else if (buffer[0] == CMD_MOTOR_FREQ_DC) {
        uint16_t freq;
        memcpy(&freq, &buffer[1], 2);
        Motor_SetDCFrequency(freq);
    }
    // Handle Stepper Command [ 'G', steps_L, steps_H, steps_LL, steps_HH, speed_L, speed_H, releaseDelayMs ]
    else if (buffer[0] == CMD_MOTOR_STEPPER) {
        int32_t steps;
        uint16_t speed;
        memcpy(&steps, &buffer[1], 4);
        memcpy(&speed, &buffer[5], 2);
        Motor_CommandStepper(steps, speed, buffer[7]);
    }
    // Handle Decay Mode [ 'T', mode ]
    else if (buffer[0] == CMD_MOTOR_DECAY) {
        Motor_SetDecay(buffer[1]);
    }
    // Handle Step Mode [ 'H', mode (0=Full, 1=Half) ]
    else if (buffer[0] == CMD_MOTOR_STEP_MODE) {
        Motor_SetStepMode(buffer[1]);
    }
    // Handle HFS (Half Full-Scale) [ 'X', 0=low/full-range, 1=high/half-range ]
    else if (buffer[0] == CMD_MOTOR_HFS) {
        Motor_SetHFS(buffer[1] != 0);
    }
}

/**
 * @brief  Sets step logic type (Half vs Full).
 * @param  halfStep True uses half steps.
 * @note   Used in: Motor_HandleCommand() to allow shifting between step resolution 
 *         vs driver torque behavior dynamically.
 */
void Motor_SetStepMode(bool halfStep) {
    stepper_half_step = halfStep;
}

/**
 * @brief  Overrides the active chip decay mechanisms via discrete lines.
 * @param  decayMode Fast vs slow/mixed mapped directly to driver configuration pins.
 * @note   Used in: Motor_HandleCommand() to address acoustic or ripple needs dynamically 
 *         from user input.
 */
void Motor_SetDecay(uint8_t decayMode) {
    motor_decay_mode = decayMode & 0x03;
    HAL_GPIO_WritePin(GPIOA, PIN_DECAY1, (motor_decay_mode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, PIN_DECAY2, (motor_decay_mode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Checks if active scheduled stepping motions exist.
 * @retval Return true if stepper remains busy moving.
 * @note   Used in: Motor_GetStatusFlags() for pushing real-time status details 
 *         so host controllers can poll to avoid over-commanding.
 */
bool Motor_IsBusy(void) {
    return motor_busy;
}

/**
 * @brief  Checks current internal mode representation.
 * @retval MotorMode value defining DC or Stepper activity logic.
 * @note   Used in: Motor_GetStatusFlags() returning the hardware intent representation 
 *         to standard interrogation telemetry packets.
 */
MotorMode Motor_GetMode(void) {
    return currentMode;
}

/**
 * @brief  Reports standard or half stepping flag configuration.
 * @retval Stepping boolean.
 * @note   Used in: Motor_GetStatusFlags() mapping logic rules out towards 
 *         live user diagnostic checks.
 */
bool Motor_GetStepMode(void) {
    return stepper_half_step;
}

/**
 * @brief  Reports current decay setting limit bits.
 * @retval Saved decay mode numeric selection.
 * @note   Used in: Motor_GetStatusFlags() passing localized configuration 
 *         out toward the reporting framework.
 */
uint8_t Motor_GetDecay(void) {
    return motor_decay_mode;
}

/**
 * @brief  Processes baseline stabilization and continuous sampling.
 * @note   Used in: Central application main loop routines iteratively processing 
 *         all module telemetry updates on ~100Hz pacing to maintain stability.
 */
void Motor_Update(void) {
    uint32_t now = HAL_GetTick();
    if (now - last_current_read_time > 10) { // Read every 10ms (100Hz)
        last_current_read_time = now;

        uint16_t raw_a = ReadCurrentSenseChannel(ADC_CHANNEL_4);
        uint16_t raw_b = ReadCurrentSenseChannel(ADC_CHANNEL_5);

        bool dc_idle = (currentMode == MOTOR_MODE_DC) &&
                       (motor_speed_cmd_a == 0) &&
                       (motor_speed_cmd_b == 0);

        if (!motor_current_baseline_valid) {
            motor_current_baseline_a = raw_a;
            motor_current_baseline_b = raw_b;
            motor_current_baseline_valid = true;
        } else if (dc_idle) {
            // Slow baseline adaptation while idle to track ADC/offset drift.
            motor_current_baseline_a = (uint16_t)(((uint32_t)motor_current_baseline_a * 7U + raw_a) / 8U);
            motor_current_baseline_b = (uint16_t)(((uint32_t)motor_current_baseline_b * 7U + raw_b) / 8U);
        }

        motor_current_a = (raw_a > motor_current_baseline_a) ? (raw_a - motor_current_baseline_a) : 0;
        motor_current_b = (raw_b > motor_current_baseline_b) ? (raw_b - motor_current_baseline_b) : 0;
    }
}

/**
 * @brief  Fetches internally logged real current minus calibration baselines.
 * @param  motor Denotes channel (MOTOR_A or MOTOR_B).
 * @retval Processed unit value for sensed motor activity limits.
 * @note   Used in: Motor_PopulateTelemetry() formatting actual reading outputs 
 *         into the telemetry stream upon register polling.
 */
uint16_t Motor_GetCurrent(uint8_t motor) {
    return (motor == MOTOR_A) ? motor_current_a : motor_current_b;
}

/**
 * @brief  Drives Half Full-Scale operational adjustments physically impacting max ranges.
 * @param  enable Defines behavior intent across ranges.
 * @note   Used in: Motor_HandleCommand() bridging host HFS toggles firmly 
 *         down into exact GPIO output representations.
 */
void Motor_SetHFS(bool enable) {
    motor_hfs_enabled = enable;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Queries actual Half Full-Scale status toggles.
 * @retval Returning active HFS state.
 * @note   Used in: Motor_GetStatusFlags() for comprehensive telemetry diagnostics reporting.
 */
bool Motor_GetHFS(void) {
    return motor_hfs_enabled;
}

/**
 * @brief  Returns parameter indicating whether late motor suspension commands remain queued.
 * @retval True if motor expects auto-shutdown upon movement resolution.
 * @note   Used in: Motor_GetStatusFlags() offering detailed execution checks 
 *         for external polling interfaces.
 */
bool Motor_GetReleaseOnComplete(void) {
    return stepper_release_delay_ms > 0;
}

/**
 * @brief  Aggregates multiple internal statuses (Busy, Mode, HFS, Decay, etc) into a minimal binary array format.
 * @retval Compiled binary 8-bit mapping flag representing active statuses.
 * @note   Used in: Motor_PopulateTelemetry() efficiently passing state blocks 
 *         instead of returning distinct register checks for every variable.
 */
uint8_t Motor_GetStatusFlags(void) {
    return (Motor_IsBusy() ? 0x01U : 0x00U)
         | ((Motor_GetMode() == MOTOR_MODE_STEPPER) ? 0x02U : 0x00U)
         | (Motor_GetStepMode() ? 0x04U : 0x00U)
         | (Motor_GetHFS() ? 0x08U : 0x00U)
         | ((uint8_t)((Motor_GetDecay() & 0x03U) << 4U))
         | (Motor_GetReleaseOnComplete() ? 0x40U : 0x00U);
}

/**
 * @brief  Fills the given array completely defining internal states and active sensed values.
 * @param  buffer Target destination array memory.
 * @note   Used in: Communication modules managing external bus (I2C) readout requirements 
 *         to correctly return local stats per node query frames.
 */
void Motor_PopulateTelemetry(uint8_t *buffer) {
    if (buffer == NULL) {
        return;
    }

    uint16_t current_a = Motor_GetCurrent(MOTOR_A);
    uint16_t current_b = Motor_GetCurrent(MOTOR_B);

    buffer[0] = (uint8_t)(current_a & 0xFFU);
    buffer[1] = (uint8_t)((current_a >> 8) & 0xFFU);
    buffer[2] = (uint8_t)(current_b & 0xFFU);
    buffer[3] = (uint8_t)((current_b >> 8) & 0xFFU);
    buffer[4] = Motor_GetStatusFlags();
}

// ISR Callback for Stepper
// Using ifdef here is redundant but allows to compile this file
// as part of the monolithic firmware without linker issues, if desired.
#ifdef MODULINO_MOTORS_BUILD
/**
 * @brief  Hardware recurring timer driving Stepper outputs rhythmically over phase segments.
 * @note   Used in: STM32 HAL interrupt vector mapping natively routing TIM3 intervals 
 *         safely across defined logic stepping sequences to manage motor positions correctly.
 */
void TIM3_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

            if (stepper_release_pending) {
                // Delayed-release one-shot expired: disable both bridges and
                // leave coils floating to remove holding torque.
                stepper_release_pending = false;
                HAL_TIM_Base_Stop_IT(&htim3);
                Stepper_ReleaseOutputs();
                return;
            }

            if (stepper_settle_pending) {
                // First wake-up after outputs were re-enabled. Switch TIM3 to
                // the requested step period and emit the first real step now.
                stepper_settle_pending = false;
                __HAL_TIM_SET_AUTORELOAD(&htim3, stepper_period_ticks);
                __HAL_TIM_SET_COUNTER(&htim3, 0);

                if (currentMode == MOTOR_MODE_STEPPER && stepper_steps_remaining > 0) {
                    uint8_t increment = stepper_half_step ? 1 : 2;

                    if (stepper_direction) {
                        stepper_phase += increment;
                    } else {
                        stepper_phase -= increment;
                    }

                    Stepper_ApplyPhase(stepper_phase);
                    stepper_steps_remaining--;

                    if (stepper_steps_remaining == 0) {
                        motor_busy = false;
                        if (stepper_release_delay_ticks > 0) {
                            // Move ended inside settle path; keep timer alive
                            // for one more one-shot interval before releasing.
                            stepper_release_pending = true;
                            __HAL_TIM_SET_AUTORELOAD(&htim3, stepper_release_delay_ticks);
                            __HAL_TIM_SET_COUNTER(&htim3, 0);
                        } else {
                            // No delayed release requested; stop periodic IRQ.
                            HAL_TIM_Base_Stop_IT(&htim3);
                        }
                    }
                }
                return;
            }
            
            if (currentMode == MOTOR_MODE_STEPPER && stepper_steps_remaining > 0) {
                // Increment based on Step Mode
                uint8_t increment = stepper_half_step ? 1 : 2;

                if (stepper_direction) {
                    stepper_phase += increment;
                } else {
                    stepper_phase -= increment;
                }

                Stepper_ApplyPhase(stepper_phase);
                stepper_steps_remaining--;
                
                if (stepper_steps_remaining == 0) {
                    motor_busy = false;
                    if (stepper_release_delay_ticks > 0) {
                        // Main stepping path completion: convert periodic timer
                        // into a final one-shot release delay.
                        stepper_release_pending = true;
                        __HAL_TIM_SET_AUTORELOAD(&htim3, stepper_release_delay_ticks);
                        __HAL_TIM_SET_COUNTER(&htim3, 0);
                    } else {
                        // Hold current phase energized when delay is zero.
                        HAL_TIM_Base_Stop_IT(&htim3);
                    }
                }
            }
        }
    }
}
#endif
