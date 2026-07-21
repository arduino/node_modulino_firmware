/**
 * This driver supports the MAX22211 Motor Driver in two modes:
 * 1. DC Motor Mode: Independent control of two DC motors with PWM speed.
 * 2. Stepper Motor Mode: Control of one Bipolar Stepper Motor with step/speed control.
 * 
 * The driver manages the hardware timer resources (TIM1 and TIM3)
 * on the STM32C011 microcontroller.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32c0xx_hal.h"


/** @brief Command ID for setting Motor Mode */
#define CMD_MOTOR_MODE        'M'

/** @brief Command ID for setting DC Motor Speed */
#define CMD_MOTOR_SPEED_DC    'S' 

/** @brief Command ID for setting DC Motor Frequency */
#define CMD_MOTOR_FREQ_DC     'F' 

/** @brief Command ID for Stepper Movement (Go) */
#define CMD_MOTOR_STEPPER     'G' 

/** @brief Command ID for setting Decay Mode */
#define CMD_MOTOR_DECAY       'T' 

/** @brief Command ID for setting Stepper Step Mode (Half/Full) */
#define CMD_MOTOR_STEP_MODE   'H' 

/** @brief Command ID for setting HFS (Half Full-Scale) pin */
#define CMD_MOTOR_HFS         'X'


/**
 * @brief Motor Operation Modes
 */
typedef enum {
    MOTOR_MODE_DC = 0,      /**< Independent control of two DC motors */
    MOTOR_MODE_STEPPER = 1  /**< Bipolar Stepper Motor control */
} MotorMode;

/** @brief Index for DC Motor A (Channel 1/2) */
#define MOTOR_A 0

/** @brief Index for DC Motor B (Channel 3/4) */
#define MOTOR_B 1

/** @brief Full-scale signed DC speed command carried by the I2C protocol. */
#define MOTOR_DC_SPEED_MAX 32767


/**
 * @brief Initialize the Motor Driver and associated peripherals.
 * @details Configures GPIOs, Timers (TIM1, TIM3), and sets initial state to Disabled.
 */
void Motor_Init(void);

/**
 * @brief Set the operating mode of the motor driver.
 * @param mode The desired mode (MOTOR_MODE_DC or MOTOR_MODE_STEPPER).
 * @details Switches pin configurations between PWM (for DC) and GPIO/Timer (for Stepper).
 *          Stops all running motors when switching.
 */
void Motor_SetMode(MotorMode mode);

/**
 * @brief Set the speed and direction of a DC motor.
 * @param motor The motor index (MOTOR_A or MOTOR_B).
 * @param speed Speed value from -32767 (Max Reverse) to +32767 (Max Forward). 0 is Stop.
 * @note Only active in MOTOR_MODE_DC.
 */
void Motor_SetDCSpeed(uint8_t motor, int16_t speed);

/**
 * @brief Set DC Motor PWM Frequency
 * @param frequency Frequency in Hz (e.g., 20000 for 20kHz)
 */
void Motor_SetDCFrequency(uint16_t frequency);


/**
 * @brief Command the Stepper Motor to move.
 * @param steps Number of steps to move. Positive for one direction, negative for the other.
 * @param speed Timer period for step interval (lower value = faster speed).
 *              Approx step interval: speed * 0.1 ms (based on TIM3 configuration).
 *              e.g. speed=100 -> 10ms per step, speed=10 -> 1ms per step.
 * @param releaseDelayMs Delay before coil release after move completion.
 *                       0 keeps holding torque, 1..255 releases after that many milliseconds.
 * @note Only active in MOTOR_MODE_STEPPER. This function is non-blocking (interrupt driven).
 */
void Motor_CommandStepper(int32_t steps, uint16_t speed, uint8_t releaseDelayMs);

/**
 * @brief Decode and execute a motor command frame received over I2C.
 * @param buffer Pointer to at least 8 bytes containing the command frame.
 */
void Motor_HandleCommand(const uint8_t *buffer);

/**
 * @brief Set the Decay Mode for the motor driver.
 * @param decayMode Bitmask configuration for DECAY pins (MAX22211):
 *                  - Bit 0: DECAY1
 *                  - Bit 1: DECAY2
 *                  
 *                  Modes:
 *                  - 0: Slow Decay
 *                  - 1: Mixed Decay (30% Fast)
 *                  - 2: Mixed Decay (60% Fast)
 *                  - 3: Fast Decay
 *                  
 *                  @note Default is 0 (Slow Decay).
 */
void Motor_SetDecay(uint8_t decayMode);

/**
 * @brief Set the Stepper Step Mode.
 * @param halfStep true for Half-Stepping, false for Full-Stepping.
 * @note Changes the stepping sequence logic.
 */
void Motor_SetStepMode(bool halfStep);

/**
 * @brief Check if the motor is currently busy.
 * @return true if the stepper motor is moving, false otherwise.
 * @note Primarily used for Stepper mode to check if a move command is complete.
 */
bool Motor_IsBusy(void);

/**
 * @brief Get the current motor operation mode.
 * @return MOTOR_MODE_DC or MOTOR_MODE_STEPPER.
 */
MotorMode Motor_GetMode(void);

/**
 * @brief Get the current step mode selection.
 * @return true when half-step mode is enabled, false for full-step mode.
 */
bool Motor_GetStepMode(void);

/**
 * @brief Get the current decay mode setting.
 * @return Decay mode value in range 0..3.
 */
uint8_t Motor_GetDecay(void);

/**
 * @brief Periodic update function.
 * @details Can be called in the main loop for housekeeping or status monitoring.
 *          In Motor Mode, this function also updates current readings from ADC.
 */
void Motor_Update(void);

/**
 * @brief Get the latest current reading for a DC Motor.
 * @param motor The motor index (MOTOR_A or MOTOR_B).
 * @return Raw ADC delta counts (0-4095). Convert to mA on the host using the
 *         MAX22211 KISEN value for the current HFS setting.
 */
uint16_t Motor_GetCurrent(uint8_t motor);

/**
 * @brief Set the HFS (Half Full-Scale) pin on the MAX22211.
 * @param enable true = HFS high (reduced current range, higher precision);
 *               false = HFS low (full current range, KISEN=7500).
 * @note The host library must use the matching KISEN value when converting
 *       raw ADC counts to milliamps. Read back the state with Motor_GetHFS().
 */
void Motor_SetHFS(bool enable);

/**
 * @brief Get the current HFS pin state.
 * @return true if HFS is high, false if HFS is low.
 */
bool Motor_GetHFS(void);

/**
 * @brief Get the current stepper release-on-complete behavior.
 * @return true if the stepper releases its coils after a move, false if it holds position.
 */
bool Motor_GetReleaseOnComplete(void);

/**
 * @brief Build the packed motor status flags byte used in telemetry.
 * @return Status flags packed as: bit0 busy, bit1 mode, bit2 step mode,
 *         bit3 HFS, bits4-5 decay, bit6 release-on-complete.
 */
uint8_t Motor_GetStatusFlags(void);

/**
 * @brief Populate motor telemetry payload bytes for I2C response.
 * @param buffer Pointer to at least 5 bytes where telemetry is written.
 *               Layout: [0..1] current A (LE), [2..3] current B (LE), [4] status flags.
 */
void Motor_PopulateTelemetry(uint8_t *buffer);

#endif // MOTORS_H
