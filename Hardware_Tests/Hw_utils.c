#include "Hw_utils.h"
#include "sts_servo_cmd.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <math.h>

sts_test_report_t test_report  = {0};
telem_sample_t    telem_buf[TELEM_MAX_SAMPLES];
uint16_t          telem_count  = 0U;

#define TEARDOWN_TARGET_POS  2048
#define TEARDOWN_TOLERANCE   10
#define TEARDOWN_TIMEOUT_MS  5000
#define POLL_INTERVAL_MS     20

void Telem_Reset(void) {
    telem_count = 0U;
}

void Telem_Record(int16_t load, uint16_t pos) {
    if (telem_count < TELEM_MAX_SAMPLES) {
        telem_buf[telem_count].load = load;
        telem_buf[telem_count].pos  = pos;
        telem_count++;
    }
}

load_stats_t Telem_ComputeStats(void) {
    load_stats_t s = {0};
    if (telem_count == 0U) return s;

    s.n = telem_count;

    int32_t sum  = 0;
    int16_t peak = 0;
    for (uint16_t i = 0U; i < telem_count; i++) {
        int16_t v = telem_buf[i].load;
        sum += v;
        if (abs(v) > abs(peak)) peak = v;
    }
    s.mean = (int16_t)(sum / (int32_t)telem_count);
    s.peak = peak;

    float var = 0.0f;
    for (uint16_t i = 0U; i < telem_count; i++) {
        float diff = (float)telem_buf[i].load - (float)s.mean;
        var += diff * diff;
    }
    s.stddev = (uint16_t)sqrtf(var / (float)telem_count);

    return s;
}

sts_result_t STS_Setup(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Homing Servo (Setup) ---\n");

    uint16_t current_pos;
    if (STS_GetPresentPosition(servo, &current_pos) == STS_OK) {
        if (abs(current_pos - TEARDOWN_TARGET_POS) <= 10) {
            SEGGER_RTT_WriteString(0, "Servo already at home position. Skipping move.\n");
            return STS_OK;
        }
    }

    sts_result_t res = STS_SetTargetPosition(servo, TEARDOWN_TARGET_POS);
    if (res != STS_OK) return res;

    uint32_t start_time    = HAL_GetTick();
    uint8_t  target_reached = 0;
    while ((HAL_GetTick() - start_time) < 2000U) {
        if (STS_GetPresentPosition(servo, &current_pos) == STS_OK) {
            if (abs(current_pos - TEARDOWN_TARGET_POS) <= 10) {
                target_reached = 1;
                break;
            }
        }
        HAL_Delay(10U);
    }

    if (target_reached) {
        SEGGER_RTT_WriteString(0, "Setup homing successful.\n");
        return STS_OK;
    }
    SEGGER_RTT_WriteString(0, "CRITICAL: Setup homing TIMEOUT! Servo jammed?\n");
    return STS_ERR_TIMEOUT;
}

sts_result_t STS_Teardown(sts_servo_t *servo) {
    if (servo->is_online != STS_ONLINE) return STS_ERR_HARDWARE;

    SEGGER_RTT_WriteString(0, "--- Hardware Teardown ---\n");

    /* The servo's RAM registers survive STM32 resets as long as servo power is on.
     * If a bare write fails silently (UART glitch), the old value stays and poisons
     * the next test run. Write then read back; retry up to 3x until confirmed 0. */
    for (uint8_t i = 0U; i < 3U; i++) {
        uint16_t rb = 0xFFFFU;
        STS_Write16(servo, STS_REG_GOAL_TIME, 0U);
        STS_Read16(servo, STS_REG_GOAL_TIME, &rb);
        if (rb == 0U) break;
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        uint16_t rb = 0xFFFFU;
        STS_Write16(servo, STS_REG_GOAL_SPEED, 0U);
        STS_Read16(servo, STS_REG_GOAL_SPEED, &rb);
        if (rb == 0U) break;
    }
    STS_SetTargetAcceleration(servo, ACCEL_DEFAULT);

    sts_result_t res = STS_SetTargetPosition(servo, TEARDOWN_TARGET_POS);
    if (res != STS_OK) {
        SEGGER_RTT_printf(0, "Teardown move failed (Err: %d)\n", res);
    }

    uint32_t start_time    = HAL_GetTick();
    uint8_t  target_reached = 0;
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
    }

    SEGGER_RTT_printf(0, "CRITICAL: Teardown homing TIMEOUT or STUCK! (pos=%d, target=%d)\n",
                      (int)test_report.pos, TEARDOWN_TARGET_POS);
    STS_SetTorqueEnable(servo, 0);
    return STS_ERR_TIMEOUT;
}
