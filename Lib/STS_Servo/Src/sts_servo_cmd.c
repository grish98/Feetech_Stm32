/**
 ******************************************************************************
 * @file           : sts_servo_cmd.c
 * @brief          : STS Servo Command Implementation
 * @author         : Grisham Balloo
 * @date           : 2026-03-19
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


sts_result_t STS_SetTorqueEnable(sts_servo_t *servo, uint8_t enable) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    uint8_t value = (enable != 0U) ? STS_TORQUE_ENABLE : STS_TORQUE_DISABLE;
    return STS_Write8(servo, STS_REG_TORQUE_ENABLE, value);
}

sts_result_t STS_SetOperatingMode(sts_servo_t *servo, sts_operating_mode_t mode) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    
    if (mode > STS_MODE_STEP) {
        return STS_ERR_INVALID_PARAM;
    }
    
    return STS_Write8(servo, STS_REG_OPERATION_MODE, (uint8_t)mode);
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

sts_result_t STS_SetTargetSpeed(sts_servo_t *servo, uint16_t speed, sts_direction_t dir) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    
    uint16_t magnitude = speed & STS_SPEED_MAGNITUDE_MASK; 
    if (magnitude > STS_MAX_SPEED) {
        return STS_ERR_INVALID_PARAM;
    }
    
    uint16_t reg_val = magnitude | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT); 

    return STS_Write16(servo, STS_REG_GOAL_SPEED, reg_val);
}

sts_result_t STS_GetPresentSpeed(sts_servo_t *servo, uint16_t *speed_out) {
    if (servo == NULL || speed_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    return STS_Read16(servo, STS_REG_PRESENT_SPEED, speed_out);
}

sts_result_t STS_SetTargetAcceleration(sts_servo_t *servo, uint8_t acceleration) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (acceleration > STS_MAX_ACCELERATION) {
        return STS_ERR_INVALID_PARAM;
    }
    return STS_Write8(servo, STS_REG_ACCELERATION, acceleration);
}

