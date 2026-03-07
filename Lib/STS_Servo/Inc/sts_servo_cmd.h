/**
 ******************************************************************************
 * @file           : sts_servo_cmd.h
 * @brief          : STS Servo Command API
 * @author         : Grisham Balloo
 * @date           : 2026-03-06
 * @version        : 0.1.0
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
#include "sts_servo.h"

/**
 * @brief Enables or disables motor torque output.
 * @param servo  Pointer to an initialised servo handle.
 * @param enable Non-zero to enable torque, zero to disable (free spin).
 * @return STS_OK on success, or specific sts_result_t error code.
 */
sts_result_t STS_SetTorqueEnable(sts_servo_t *servo, uint8_t enable);



