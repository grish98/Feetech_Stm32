/**
 ******************************************************************************
 * @file           : sts_servo_cmd.c
 * @brief          : STS Servo Command Implementation
 * @author         : Grisham Balloo
 ******************************************************************************
 * @details
 * Implements the high-level command set for Feetech STS servo control,
 * built on top of the STS_Write8, STS_Write16, STS_Read8, and STS_Read16
 * register access primitives.
 *
 * All commands are stateless and route through the service layer command
 * engine  no direct protocol framing occurs at this level.
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

sts_result_t STS_SetTargetPosition(sts_servo_t *servo, uint32_t position) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (position > STS_MAX_POSITION) {
        return STS_ERR_INVALID_PARAM;
    }

    return STS_Write16(servo, STS_REG_GOAL_POSITION, (uint16_t)position);
}

sts_result_t STS_GetPresentPosition(sts_servo_t *servo, uint16_t *position_out) {
    if (servo == NULL || position_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    return STS_Read16(servo, STS_REG_PRESENT_POSITION, position_out);
}

sts_result_t STS_SetTargetSpeed(sts_servo_t *servo, uint32_t speed, sts_direction_t dir) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (speed > STS_MAX_SPEED) {
        return STS_ERR_INVALID_PARAM;
    }

    uint16_t reg_val = (uint16_t)speed | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT);
    return STS_Write16(servo, STS_REG_GOAL_SPEED, reg_val);
}

sts_result_t STS_GetPresentSpeed(sts_servo_t *servo, int16_t *speed_out) {
    if (servo == NULL || speed_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    uint16_t raw_val = 0U;
    sts_result_t res = STS_Read16(servo, STS_REG_PRESENT_SPEED, &raw_val);

    if (res == STS_OK) {
        /* Encoder runs at 2× position resolution; divide raw ticks/s by 2 to get steps/s. */
        int16_t true_speed = (int16_t)((raw_val & STS_SPEED_MAGNITUDE_MASK) / 2U);
        if (raw_val & STS_SPEED_DIRECTION_BIT) {
            *speed_out = -true_speed;
        } else {
            *speed_out = true_speed;
        }
    }
    return res;
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

sts_result_t STS_SetTargetPWM(sts_servo_t *servo, uint32_t pwm, sts_direction_t dir) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (pwm > STS_MAX_PWM) {
        return STS_ERR_INVALID_PARAM;
    }

    uint16_t reg_val = (uint16_t)pwm | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT);
    return STS_Write16(servo, STS_REG_GOAL_SPEED, reg_val);
}

sts_result_t STS_SetTargetStep(sts_servo_t *servo, uint32_t steps, sts_direction_t dir) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (steps > STS_MAX_STEP) {
        return STS_ERR_INVALID_PARAM;
    }

    uint16_t reg_val = (uint16_t)steps | ((uint16_t)dir * STS_SPEED_DIRECTION_BIT);
    return STS_Write16(servo, STS_REG_GOAL_POSITION, reg_val);
}

sts_result_t STS_SetTorqueLimit(sts_servo_t *servo, uint16_t limit) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (limit > STS_MAX_TORQUE) {
        return STS_ERR_INVALID_PARAM;
    }
    return STS_Write16(servo, STS_REG_TORQUE_LIMIT, limit);
}

sts_result_t STS_GetPresentLoad(sts_servo_t *servo, int16_t *load_out) {
    if (servo == NULL || load_out == NULL) {
        return STS_ERR_NULL_PTR;
    }

    uint16_t raw_load = 0U;
    sts_result_t res = STS_Read16(servo, STS_REG_PRESENT_LOAD, &raw_load);

    if (res == STS_OK) {
        int16_t magnitude = (int16_t)(raw_load & STS_LOAD_MAGNITUDE_MASK);

        if (raw_load & STS_LOAD_DIRECTION_BIT) {
            *load_out = magnitude;
        } else {
            *load_out = -magnitude;
        }
    }
    return res;
}

sts_result_t STS_GetPresentVoltage(sts_servo_t *servo, uint8_t *voltage_out) {
    if (servo == NULL || voltage_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    return STS_Read8(servo, STS_REG_PRESENT_VOLTAGE, voltage_out);
}

sts_result_t STS_GetPresentTemperature(sts_servo_t *servo, uint8_t *temp_out) {
    if (servo == NULL || temp_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    return STS_Read8(servo, STS_REG_PRESENT_TEMP, temp_out);
}

sts_result_t STS_GetMovingStatus(sts_servo_t *servo, uint8_t *status_out) {
    if (servo == NULL || status_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    return STS_Read8(servo, STS_REG_MOVING_FLAG, status_out);
}

sts_result_t STS_SetEEPROMLock(sts_servo_t *servo, uint8_t lock) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    uint8_t value = (lock != EEPROM_UNLOCK) ? EEPROM_LOCK : EEPROM_UNLOCK;
    return STS_Write8(servo, STS_REG_LOCK, value);
}

sts_result_t STS_SetID(sts_servo_t *servo, uint8_t new_id) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (new_id > STS_ID_BROADCAST_SYNC) {
        return STS_ERR_INVALID_PARAM;
    }

    /* Caller must unlock EEPROM before this and lock it immediately after.
     * Not bundled here to prevent accidental flash wear on misuse. */
    return STS_Write8(servo, STS_REG_ID, new_id);
}

/* --- Verified writes --- */

/* Confirmed (with or without a fault flag) or a caller error: nothing to gain from a re-send. */
static uint8_t verify_is_final(sts_result_t res) {
    return (res == STS_OK || res == STS_ERR_HARDWARE || res == STS_ERR_NULL_PTR || res == STS_ERR_INVALID_PARAM) ? 1U : 0U;
}

/* Absolute register: a second identical write gives the same final state, so one re-send is safe. */
static sts_result_t verify8_resend_once(sts_servo_t *servo, uint8_t reg, uint8_t value) {
    sts_result_t res = STS_Write8Verified(servo, reg, value);
    if (verify_is_final(res) == 0U) {
        res = STS_Write8Verified(servo, reg, value);
    }
    return res;
}

static sts_result_t verify16_resend_once(sts_servo_t *servo, uint8_t reg, uint16_t value) {
    sts_result_t res = STS_Write16Verified(servo, reg, value);
    if (verify_is_final(res) == 0U) {
        res = STS_Write16Verified(servo, reg, value);
    }
    return res;
}

sts_result_t STS_SetTorqueEnableVerified(sts_servo_t *servo, uint8_t enable) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    uint8_t value = (enable != 0U) ? STS_TORQUE_ENABLE : STS_TORQUE_DISABLE;
    return verify8_resend_once(servo, STS_REG_TORQUE_ENABLE, value);
}

sts_result_t STS_SetOperatingModeVerified(sts_servo_t *servo, sts_operating_mode_t mode) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (mode > STS_MODE_STEP) {
        return STS_ERR_INVALID_PARAM;
    }
    return verify8_resend_once(servo, STS_REG_OPERATION_MODE, (uint8_t)mode);
}

sts_result_t STS_SetTorqueLimitVerified(sts_servo_t *servo, uint16_t limit) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (limit > STS_MAX_TORQUE) {
        return STS_ERR_INVALID_PARAM;
    }
    return verify16_resend_once(servo, STS_REG_TORQUE_LIMIT, limit);
}

sts_result_t STS_SetIDVerified(sts_servo_t *servo, uint8_t new_id) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (new_id >= STS_ID_BROADCAST_SYNC || servo->id >= STS_ID_BROADCAST_SYNC) {
        return STS_ERR_INVALID_PARAM;
    }

    /* Never re-sent: a repeat would go to the old ID after a change that happened. */
    sts_result_t res = STS_SetID(servo, new_id);
    if (res == STS_ERR_NULL_PTR || res == STS_ERR_INVALID_PARAM) {
        return res;
    }

    sts_servo_t probe = *servo;
    probe.id = new_id;
    res = STS_servo_ping(&probe);
    if (res != STS_OK && res != STS_ERR_HARDWARE) {
        return STS_ERR_VERIFY;
    }
    servo->id = new_id;
    return STS_OK;
}
