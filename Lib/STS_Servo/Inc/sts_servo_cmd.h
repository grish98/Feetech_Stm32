/**
 ******************************************************************************
 * @file           : sts_servo_cmd.h
 * @brief          : STS Servo Command API
 * @author         : Grisham Balloo
 * @date           : 2026-03-08
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

/** * @brief Maximum valid encoder position.
 * @details Represents a full 360-degree rotation (12-bit magnetic encoder resolution). 
 */
#define STS_MAX_POSITION        4095U 

/** * @brief Maximum velocity limit in steps per second.
 * @details Used in Speed/Step modes as the target velocity, and in Position mode as the speed limit. 
 */
#define STS_MAX_SPEED           3073U 

/** * @brief Maximum acceleration/deceleration ramp.
 * @details Dictates how aggressively the servo reaches its target speed (Start/Stop Acceleration). 
 */
#define STS_MAX_ACCELERATION    150U  

/** @} */

/**
 * @brief Defines the rotational direction of the servo.
 * Used primarily in Speed, PWM, and Step modes to dictate the 
 * movement vector. In standard Feetech configuration, CCW is 
 * considered the positive/forward direction.
 */
typedef enum {
    STS_DIR_CCW = 0U, /**< Counter-Clockwise  */
    STS_DIR_CW  = 1U  /**< Clockwise  */
} sts_direction_t;

/**
 * @brief Defines the active internal control algorithm (Operating Mode).
 * Changing the operating mode fundamentally alters how the servo responds 
 * to target commands. It rewires the internal control loop .
 */
typedef enum {
    STS_MODE_POSITION = 0U, /**< Absolute position control via PID. Target speed acts as a travel limit. */
    STS_MODE_SPEED    = 1U, /**< Continuous rotation velocity control. Absolute position commands are ignored. */
    STS_MODE_PWM      = 2U, /**< Open-loop voltage control. PID is disabled; servo acts as a standard DC motor. */
    STS_MODE_STEP     = 3U  /**< Relative position control. Moves a specified number of steps from the current location. */
} sts_operating_mode_t;

/**
 * @brief Enables or disables motor torque output.
 * @param servo  Pointer to an initialised servo handle.
 * @param enable Non-zero to enable torque, zero to disable (free spin).
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_SetTorqueEnable(sts_servo_t *servo, uint8_t enable);

/**
 * @brief Sets the operating mode of the servo (Position, Speed, PWM, Step).
 * @param servo Pointer to an initialised servo handle.
 * @param mode  The desired operating mode.
 * @return STS_OK on success.
 */
sts_result_t STS_SetOperatingMode(sts_servo_t *servo, sts_operating_mode_t mode);

/**
 * @brief Sets the target position for the servo.
 * @param servo  Pointer to an initialised servo handle.
 * @param position 0 to 4095 (valid range for STS3215/3032).
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_SetTargetPosition(sts_servo_t *servo, uint16_t position);

/**
 * @brief Reads the current actual position from the servo.
 * @param servo  Pointer to an initialised servo handle.
 * @param[out] position_out Pointer to store the 16-bit position.
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_GetPresentPosition(sts_servo_t *servo, uint16_t *position_out);

/**
 * @brief Reads the current rotational speed of the servo.
 * @param servo Pointer to an initialised servo handle.
 * @param[out] speed_out Pointer to store the 16-bit speed value.
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_GetPresentSpeed(sts_servo_t *servo, uint16_t *speed_out);

/**
 * @brief Sets the target speed and direction of the servo.
 * @note The behaviour of this command depends on the active operating mode.
 * In Mode 1 (Speed) and Mode 3 (Step), it dictates the movement vector.
 * In Mode 0 (Position), it acts as an upper speed limit for the trajectory.
 * @param servo Pointer to an initialised servo handle.
 * @param speed The target speed magnitude (0 to STS_MAX_SPEED).
 * @param dir   The desired rotation direction (STS_DIR_CCW or STS_DIR_CW).
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_SetTargetSpeed(sts_servo_t *servo, uint16_t speed, sts_direction_t dir);

/**
 * @brief Sets the acceleration profile for servo movements.
 * @note This dictates how aggressively the internal PID loop ramps up 
 * to the target speed. There is no physical accelerometer, so this 
 * is purely a profile configuration parameter.
 * @param servo Pointer to an initialised servo handle.
 * @param acceleration The acceleration rate (0 to STS_MAX_ACCELERATION).
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_SetTargetAcceleration(sts_servo_t *servo, uint8_t acceleration);
