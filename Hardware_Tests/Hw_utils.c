#include "Hw_utils.h"
#include "sts_servo_cmd.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>

sts_test_report_t test_report = {0};

#define TEARDOWN_TARGET_POS 2048
#define TEARDOWN_TOLERANCE  10
#define TEARDOWN_TIMEOUT_MS 2000 
#define POLL_INTERVAL_MS    20

sts_result_t STS_Setup(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Homing Servo (Setup) ---\n");
    
    uint16_t current_pos;

    if (STS_GetPresentPosition(servo, &current_pos) == STS_OK) {
        if (abs(current_pos - TEARDOWN_TARGET_POS) <= 10) {
            SEGGER_RTT_WriteString(0, "Servo already at home position. Skipping move.\n");
            return STS_OK; // Bail out early, a perfect success!
        }
    }

    sts_result_t res = STS_SetTargetPosition(servo, TEARDOWN_TARGET_POS);
    if (res != STS_OK) return res;
    uint32_t start_time = HAL_GetTick();
    uint8_t target_reached = 0;

    while ((HAL_GetTick() - start_time) < 2000) { 
        if (STS_GetPresentPosition(servo, &current_pos) == STS_OK) {
            if (abs(current_pos - TEARDOWN_TARGET_POS) <= 10) { 
                target_reached = 1;
                break;
            }
        }
        HAL_Delay(10); 
    }

    if (target_reached) {
        SEGGER_RTT_WriteString(0, "Setup homing successful.\n");
        return STS_OK;
    } else {
        SEGGER_RTT_WriteString(0, "CRITICAL: Setup homing TIMEOUT! Servo jammed?\n");
        return STS_ERR_TIMEOUT;
    }
}

sts_result_t STS_Teardown(sts_servo_t *servo) {
    if (servo->is_online != STS_ONLINE) return STS_ERR_HARDWARE;

    SEGGER_RTT_WriteString(0, "--- Hardware Teardown ---\n");
    sts_result_t res = STS_SetTargetPosition(servo, TEARDOWN_TARGET_POS);
    
    if (res != STS_OK) {
        SEGGER_RTT_printf(0, "Teardown  failed (Err: %d)\n", res);
    }

    uint32_t start_time = HAL_GetTick();
    uint8_t target_reached = 0;

    while ((HAL_GetTick() - start_time) < TEARDOWN_TIMEOUT_MS) {
        uint16_t current_pos;
        if (STS_GetPresentPosition(servo, &current_pos) == STS_OK) {
            test_report.pos = current_pos;

            if (abs(current_pos - TEARDOWN_TARGET_POS) <= TEARDOWN_TOLERANCE) {
                target_reached = 1;
                break;
            }
        }
        HAL_Delay(POLL_INTERVAL_MS);
    }

    if (target_reached) {
        SEGGER_RTT_WriteString(0, "Teardown homing successful.\n");
        return STS_OK;
    } else {
        SEGGER_RTT_WriteString(0, "CRITICAL: Teardown homing TIMEOUT or STUCK!\n");
        STS_SetTorqueEnable(servo, 0); 
        return STS_ERR_TIMEOUT;
    }
}