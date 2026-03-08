/**
 ******************************************************************************
 * @file           : sts_servo_cmd.c
 * @brief          : STS Servo Command Implementation
 * @author         : Grisham Balloo
 * @date           : 2026-03-08
 * @version        : 0.2.0
 ******************************************************************************
 * @details
 * Implements the high-level command set for Feetech STS servo control,
 * built on top of the STS_Write8, STS_Write16, STS_Read8, and STS_Read16
 * register access primitives.
 *
 * All commands are stateless and route through the service layer command
 * engine — no direct protocol framing occurs at this level.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "sts_servo_cmd.h"
#include "sts_servo.h"
#include "sts_registers.h"

#define STS_MAX_POSITION    4095U

sts_result_t STS_SetTorqueEnable(sts_servo_t *servo, uint8_t enable) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    uint8_t value = (enable != 0U) ? STS_TORQUE_ENABLE : STS_TORQUE_DISABLE;
    return STS_Write8(servo, STS_REG_TORQUE_ENABLE, value);
}

sts_result_t STS_SetTargetPosition(sts_servo_t *servo, uint16_t position) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (position > STS_MAX_POSITION) {
        return STS_ERR_INVALID_PARAM;
    }

    return STS_Write16(servo, STS_REG_GOAL_POSITION, position);
}

sts_result_t STS_GetPresentPosition(sts_servo_t *servo, uint16_t *position_out) {
    if (servo == NULL || position_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    return STS_Read16(servo, STS_REG_PRESENT_POSITION, position_out);
}