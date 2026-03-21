/**
 ******************************************************************************
 * @file           : sts_servo_cmd.h
 * @brief          : STS Servo Command API
 * @author         : Grisham Balloo
 * @date           : 2026-03-21
 * @version        : 0.2.0
 ******************************************************************************
 * @details
 * Declares the high-level command API for Feetech STS servo control.
 * Built on top of the service layer primitives, this header exposes
 * motion control, torque management, and status query functions.
 *
 * All commands require an initialised sts_servo_t handle. See sts_servo.h
 * for bus and servo initialisation.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#pragma once

#include "sts_protocol.h"
#include "sts_servo.h"

/** * @name Hardware Constraints
 * @brief Physical limits specific to the Feetech STS servo model (STS3215).
 * @{ 
 */

#define STS_MAX_POSITION      4095U  /**< Maximum valid encoder position (12-bit resolution) */
#define STS_MAX_SPEED         3073U  /**< Maximum velocity limit in steps per second */
#define STS_MAX_ACCELERATION  150U   /**< Maximum acceleration/deceleration ramp */
#define STS_MAX_PWM           1000U  /**< Maximum open-loop PWM value */
#define STS_MAX_STEP          32767U /**< Maximum relative steps */
#define STS_MAX_TORQUE        1000U  /**< Maximum dynamic torque limit */

#define EEPROM_UNLOCK         0U     /**< EEPROM write protection disabled */
#define EEPROM_LOCK           1U     /**< EEPROM write protection enabled */

/** @} */

/**
 * @brief Defines the rotational direction of the servo.
 * @note In standard Feetech configuration, CCW is the positive/forward direction.
 */
typedef enum {
    STS_DIR_CCW = 0U, /**< Counter-Clockwise */
    STS_DIR_CW  = 1U  /**< Clockwise */
} sts_direction_t;


/**
 * @brief Enables or disables motor torque output.
 * @param servo Pointer to an initialised servo handle.
 * @param enable Non-zero to enable torque, zero to disable (free spin).
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTorqueEnable(sts_servo_t *servo, uint8_t enable);

/**
 * @brief Sets the operating mode of the servo.
 * @param servo Pointer to an initialised servo handle.
 * @param mode The desired operating mode (Position, Speed, PWM, Step).
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetOperatingMode(sts_servo_t *servo, sts_operating_mode_t mode);

/**
 * @brief Sets the absolute target position for the servo.
 * @param servo Pointer to an initialised servo handle.
 * @param position 0 to STS_MAX_POSITION.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTargetPosition(sts_servo_t *servo, uint16_t position);

/**
 * @brief Reads the current actual position from the servo.
 * @param servo Pointer to an initialised servo handle.
 * @param[out] position_out Pointer to store the 16-bit position.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_GetPresentPosition(sts_servo_t *servo, uint16_t *position_out);

/**
 * @brief Reads the current rotational speed of the servo.
 * @param servo Pointer to an initialised servo handle.
 * @param[out] speed_out Pointer to store the 16-bit speed value.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_GetPresentSpeed(sts_servo_t *servo, uint16_t *speed_out);

/**
 * @brief Sets the target speed and direction of the servo.
 * @note In Mode 1 (Speed) and 3 (Step), dictates movement vector. In Mode 0 (Position), acts as a speed limit.
 * @param servo Pointer to an initialized servo handle.
 * @param speed The target speed magnitude (0 to STS_MAX_SPEED).
 * @param dir The desired rotation direction (STS_DIR_CCW or STS_DIR_CW).
 * @return sts_result_t STS_OK on success, or speciifc error code.
 */
sts_result_t STS_SetTargetSpeed(sts_servo_t *servo, uint16_t speed, sts_direction_t dir);

/**
 * @brief Sets the acceleration profile for servo movements.
 * @param servo Pointer to an initialized servo handle.
 * @param acceleration The acceleration rate (0 to STS_MAX_ACCELERATION).
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTargetAcceleration(sts_servo_t *servo, uint8_t acceleration);

/**
 * @brief Universal target wrapper that routes the command based on current mode.
 * @note In Position mode, negative targets are automatically clamped to 0.
 * @param servo Pointer to an initialized servo handle.
 * @param target The desired target value. Bounds depend on active mode.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTarget(sts_servo_t *servo, int32_t target);

/**
 * @brief Commands the servo to move a relative number of steps.
 * @note Servo must be in Step Mode (Mode 3).
 * @param servo Pointer to an initialized servo handle.
 * @param steps Number of relative steps to execute (0 to STS_MAX_STEP).
 * @param dir The direction of rotation (STS_DIR_CW or STS_DIR_CCW).
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTargetStep(sts_servo_t *servo, uint16_t steps, sts_direction_t dir);

/**
 * @brief Sets the open-loop PWM duty cycle (effort) applied to motor coils.
 * @note Servo must be in PWM Mode (Mode 2).
 * @param servo Pointer to an initialized servo handle.
 * @param pwm Raw PWM effort value (0 to STS_MAX_PWM).
 * @param dir The direction to apply force (STS_DIR_CW or STS_DIR_CCW).
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTargetPWM(sts_servo_t *servo, uint16_t pwm, sts_direction_t dir);

/**
 * @brief Sets the dynamic (RAM) maximum torque limit.
 * @note Safe for continuous dynamic updates. Reverts to EEPROM default on power cycle.
 * @param servo Pointer to an initialized servo handle.
 * @param limit Max torque value (0 to STS_MAX_TORQUE). 0 disables torque.
 * @return sts_result_t STS_OK on success, or specifc error code.
 */
sts_result_t STS_SetTorqueLimit(sts_servo_t *servo, uint16_t limit);

/**
 * @brief Reads the current physical load/effort of the motor.
 * @param servo Pointer to an initialized servo handle.
 * @param[out] load_out Pointer to store the signed load value.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_GetPresentLoad(sts_servo_t *servo, int16_t *load_out);

/**
 * @brief Reads the current input voltage at the servo's power terminals.
 * @param servo Pointer to an initialized servo handle.
 * @param[out] voltage_out Pointer to store the voltage (in 0.1V units, e.g., 120 = 12.0V).
 * @return sts_result_t STS_OK on success, or speciifc error code.
 */
sts_result_t STS_GetPresentVoltage(sts_servo_t *servo, uint8_t *voltage_out);

/**
 * @brief Reads the internal temperature of the servo's MCU/Motor driver.
 * @param servo Pointer to an initialized servo handle.
 * @param[out] temp_out Pointer to store the temperature in degrees Celsius.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_GetPresentTemperature(sts_servo_t *servo, uint8_t *temp_out);

/**
 * @brief Reads the moving status flag to determine if the motor is currently in motion.
 * @param servo Pointer to an initialized servo handle.
 * @param[out] status_out Pointer to store the moving flag (1: Moving, 0: Target reached).
 * @return sts_result_t STS_OK on success, or speciifc error code.
 */
sts_result_t STS_GetMovingStatus(sts_servo_t *servo, uint8_t *status_out);

/**
 * @brief Toggles the EEPROM write protection.
 * @note Must be unlocked (0) before modifying registers 0x00 to 0x20, 
 * and immediately locked (1) afterward to prevent flash corruption.
 * @param servo Pointer to an initialized servo handle.
 * @param lock EEPROM_LOCK to protect, EEPROM_UNLOCK to unprotect.
 * @return sts_result_t STS_OK on success, or specific error code.
 */
sts_result_t STS_SetEEPROMLock(sts_servo_t *servo, uint8_t lock);

/**
 * @brief Assigns a new permanent ID to the servo.
 * @note EEPROM must be unlocked prior to calling. Changes take effect immediately.
 * @warning DANGER: Unlocks EEPROM. Repeated writes will PERMANENTLY DAMAGE the servo hardware.
 * @param servo Pointer to an initialized servo handle.
 * @param new_id The new ID (0 to STS_ID_BROADCAST_SYNC).
 * @return sts_result_t STS_OK on success, STS_ERR_INVALID_PARAM if ID is out of bounds.
 */
sts_result_t STS_SetID(sts_servo_t *servo, uint8_t new_id);