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
   
    if (speed > STS_MAX_SPEED) {
        return STS_ERR_INVALID_PARAM;
    }
    
    uint16_t reg_val = speed | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT); 
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

sts_result_t STS_SetTargetPWM(sts_servo_t *servo, uint16_t pwm, sts_direction_t dir) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    
    if (pwm > STS_MAX_PWM) {
        return STS_ERR_INVALID_PARAM;
    }
    
    uint16_t reg_val = pwm | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT); 
    return STS_Write16(servo, STS_REG_GOAL_SPEED, reg_val);
}

sts_result_t STS_SetTargetStep(sts_servo_t *servo, uint16_t steps, sts_direction_t dir) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    
    if (steps > STS_MAX_STEP) {
        return STS_ERR_INVALID_PARAM;
    }
    
    uint16_t reg_val = steps | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT); 
    return STS_Write16(servo, STS_REG_GOAL_POSITION, reg_val);
}

/**
 * @brief Universal target command that routes data based on the current operating mode.
 * @param servo Pointer to an initialised servo handle.
 * @param target The target value (Pos: 0 to 4095. Speed/PWM/Step: Negative to Positive max limits).
 * @return STS_OK on success, or specific error code.
 */
sts_result_t STS_SetTarget(sts_servo_t *servo, int32_t target) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    sts_direction_t dir = (target < 0) ? STS_DIR_CW : STS_DIR_CCW;
    uint32_t magnitude  = (uint32_t)((target < 0) ? -target : target);

    switch (servo->current_mode) {
        case STS_MODE_POSITION:
            if (target < 0) {
                target = 0;
            }
            return STS_SetTargetPosition(servo, (uint16_t)target);

        case STS_MODE_SPEED:
            return STS_SetTargetSpeed(servo, (uint16_t)magnitude, dir);

        case STS_MODE_PWM:
            return STS_SetTargetPWM(servo, (uint16_t)magnitude, dir);

        case STS_MODE_STEP:
            return STS_SetTargetStep(servo, (uint16_t)magnitude, dir);

        default:
            return STS_ERR_INVALID_PARAM;
    }
}