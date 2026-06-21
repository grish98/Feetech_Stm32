#include "Hw_Tests.h"
#include "Hw_utils.h"
#include "stm32f1xx_hal.h"
#include "sts_protocol.h"
#include "sts_servo.h"
#include "SEGGER_RTT.h"
#include <stdint.h>
#include <string.h>
#include "sts_servo_cmd.h"
#include "sts_registers.h"

#define TARGET_POS_TEST        4000U
#define START_POS_OFFSET       0U
#define POS_TOLERANCE          15U

#define VOLT_MIN               90U
#define VOLT_MAX               130U
#define TEMP_MIN               0U
#define TEMP_MAX               80U

#define DYNAMIC_LOAD_MAX       1000
#define HOLDING_LOAD_MAX       300
#define HOLDING_LOAD_MIN      -300

#define DELAY_POST_MOVE        50U
#define DELAY_UART             2U
#define DELAY_POLL_INTERVAL    10U
#define TEST_MOVE_TIMEOUT      4000U
#define TEST_SPEED_TIMEOUT     7000U  // generous: covers capped (4000ms) and uncapped graceful-degradation (~1.3s) moves

#define TORQUE_OFF 0U
#define TORQUE_ON  1U

#define TARGET_TEST_SPEED      1000U  // steps/s
#define MIN_SPEED              750U   // steps/s; below target to tolerate ramp-up averaging
#define MAX_SPEED_MARGIN       500U   // upper bound = target + margin; catches speed cap failures
#define TARGET_SLOW_ACCEL      10U    // counter-intuitive: lower register value = slower ramp
#define MIN_SLOW_MOVE_TIME_MS  2000U  // accel=10 produces ~3840ms; baseline (no ramp) is ~1300ms


static sts_result_t Test_Ping(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Ping Test ---\n");

    sts_result_t res = STS_servo_ping(servo);

    TEST_ASSERT(res == STS_OK, 1, res, "Ping failed!");
    TEST_ASSERT(servo->is_online == STS_ONLINE, 2, STS_ERR_HARDWARE, "Servo not online!");
    return res;
}

static sts_result_t Test_Sensors(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Sensor Protocol Validation ---\n");
    sts_result_t res;

    res = STS_GetPresentVoltage(servo, &test_report.volt);
    TEST_ASSERT(res == STS_OK, 3, res, "UART Rx Failed: Could not read voltage");

    res = STS_GetPresentTemperature(servo, &test_report.temp);
    TEST_ASSERT(res == STS_OK, 4, res, "UART Rx Failed: Could not read temperature");

    SEGGER_RTT_printf(0, ">> Telemetry - Voltage: %d (0.1V), Temp: %d C\n", test_report.volt, test_report.temp);

    TEST_ASSERT(test_report.volt >= VOLT_MIN && test_report.volt <= VOLT_MAX, 5, STS_ERR_HARDWARE, "Voltage out of safe operating range");
    TEST_ASSERT(test_report.temp >= TEMP_MIN && test_report.temp <= TEMP_MAX, 6, STS_ERR_HARDWARE, "Temperature out of safe operating range");

    return STS_OK;
}

static sts_result_t Test_Pos(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Position Control Validation ---\n");

    STS_Write16(servo, STS_REG_GOAL_TIME,  0U);
    STS_Write16(servo, STS_REG_GOAL_SPEED, 0U);
    STS_SetTargetAcceleration(servo, ACCEL_DEFAULT);
    HAL_Delay(50U);

    Telem_Reset();

    STS_SetTargetPosition(servo, START_POS_OFFSET);
    uint32_t settle_elapsed = 0U;
    {
        uint32_t settle_start = HAL_GetTick();
        uint16_t settle_pos   = 0xFFFFU;
        do {
            HAL_Delay(20U);
            STS_GetPresentPosition(servo, &settle_pos);
        } while (settle_pos > (START_POS_OFFSET + POS_TOLERANCE) &&
                 (HAL_GetTick() - settle_start) < 5000U);
        settle_elapsed = HAL_GetTick() - settle_start;
    }
    uint16_t pos_after_settle = 0U;
    STS_GetPresentPosition(servo, &pos_after_settle);
    SEGGER_RTT_printf(0, ">> Settle: final=%d  elapsed=%dms  (target ~%d)\n",
                      pos_after_settle, settle_elapsed, START_POS_OFFSET);
    HAL_Delay(50U);

    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 7, res, "Tx Failed: Move Command");
    HAL_Delay(100U);

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)servo->bus->port_handle;
    HAL_Delay(DELAY_UART);

    uint32_t start_time         = HAL_GetTick();
    uint8_t  target_reached     = 0;
    uint8_t  consecutive_errors = 0;
    int16_t  current_load       = 0;
    sts_result_t loop_res       = STS_OK;

    while ((HAL_GetTick() - start_time) < TEST_MOVE_TIMEOUT) {
        loop_res = STS_GetPresentPosition(servo, &test_report.pos);

        if (loop_res != STS_OK) {
            consecutive_errors++;
            if (consecutive_errors == 1U) {
                uint32_t err = HAL_UART_GetError(huart);
                SEGGER_RTT_printf(0, ">> Test 8 Loop Error. HAL Error Code: 0x%08X\n", err);
            }
            HAL_Delay(50U);
            continue;
        }
        consecutive_errors = 0U;

        HAL_Delay(DELAY_UART);

        if (STS_GetPresentLoad(servo, &current_load) == STS_OK) {
            Telem_Record(current_load, test_report.pos);
        }

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
            test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
            target_reached = 1;
            break;
        }

        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    TEST_ASSERT(loop_res == STS_OK, 8, loop_res, "UART Rx Failed: Bus died during polling");
    TEST_ASSERT(target_reached, 9, STS_ERR_TIMEOUT, "Servo failed to reach commanded state");

    HAL_Delay(DELAY_POST_MOVE);
    STS_GetPresentLoad(servo, &test_report.holding_load);

    load_stats_t ls = Telem_ComputeStats();
    SEGGER_RTT_printf(0, ">> Load Profile:  n=%-3d  mean=%-5d  peak=%-5d  stddev=%d\n",
                      ls.n, ls.mean, ls.peak, ls.stddev);
    SEGGER_RTT_printf(0, ">> Holding Load:  %d\n", test_report.holding_load);

    TEST_ASSERT(abs(ls.peak) <= DYNAMIC_LOAD_MAX, 10, STS_ERR_HARDWARE,
                "Peak load exceeded limit — possible overload or spring misconfiguration");
    TEST_ASSERT(test_report.holding_load >= HOLDING_LOAD_MIN &&
                test_report.holding_load <= HOLDING_LOAD_MAX, 11, STS_ERR_HARDWARE,
                "Holding load out of expected range — servo straining at target");

    return STS_OK;
}

static sts_result_t Test_Speed(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Speed Profiling Validation ---\n");

    STS_Write16(servo, STS_REG_GOAL_TIME,  0U);
    STS_Write16(servo, STS_REG_GOAL_SPEED, 0U);
    STS_SetTargetAcceleration(servo, ACCEL_DEFAULT);

    STS_SetTargetPosition(servo, START_POS_OFFSET);
    {
        uint32_t settle_start = HAL_GetTick();
        uint16_t settle_pos   = TARGET_POS_TEST;
        do {
            HAL_Delay(20U);
            STS_GetPresentPosition(servo, &settle_pos);
        } while (settle_pos > (START_POS_OFFSET + POS_TOLERANCE) &&
                 (HAL_GetTick() - settle_start) < 3000U);
    }

    uint16_t start_pos = 0;
    STS_GetPresentPosition(servo, &start_pos);
    SEGGER_RTT_printf(0, ">> Start Pos: %d (should be ~%d)\n", start_pos, START_POS_OFFSET);

    /* 50ms idle gap before writing Goal Speed: without this the servo intermittently
     * ignores the speed cap when commanded immediately after a settle move. */
    HAL_Delay(50U);

    sts_result_t speed_res = STS_SetTargetSpeed(servo, TARGET_TEST_SPEED, STS_DIR_CCW);
    if (speed_res != STS_OK) {
        SEGGER_RTT_printf(0, "WARN: Goal Speed write failed (err: %d)\n", speed_res);
    }

    uint16_t goal_speed_rb = 0U;
    STS_Read16(servo, STS_REG_GOAL_SPEED, &goal_speed_rb);
    SEGGER_RTT_printf(0, ">> Goal Speed: commanded=%d, readback=%d (steps/s)\n",
                      TARGET_TEST_SPEED, goal_speed_rb);

    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 12, res, "UART Tx Failed: Speed Move Command");

    uint32_t start_time         = HAL_GetTick();
    uint8_t  target_reached     = 0;
    uint8_t  consecutive_errors = 0;
    uint8_t  total_errors       = 0;
    sts_result_t loop_res       = STS_OK;

    while ((HAL_GetTick() - start_time) < TEST_SPEED_TIMEOUT) {
        loop_res = STS_GetPresentPosition(servo, &test_report.pos);
        if (loop_res != STS_OK) {
            consecutive_errors++;
            total_errors++;
            if (consecutive_errors >= 5U) break;
            HAL_Delay(50U);
            continue;
        }
        consecutive_errors = 0U;

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
            test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
            target_reached = 1;
            break;
        }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    uint32_t end_time = HAL_GetTick();

    STS_SetTargetSpeed(servo, 0, STS_DIR_CCW);

    TEST_ASSERT(loop_res == STS_OK, 13, loop_res, "UART Rx Failed: Bus dead (5 consecutive failures)");
    TEST_ASSERT(target_reached, 14, STS_ERR_TIMEOUT, "Servo failed to reach commanded state");

    uint32_t total_time_ms = end_time - start_time;
    if (total_time_ms == 0) total_time_ms = 1;

    int32_t distance    = abs((int32_t)test_report.pos - (int32_t)start_pos);
    float actual_speed  = ((float)distance / (float)total_time_ms) * 1000.0f;

    SEGGER_RTT_printf(0, ">> Avg Speed:    %d steps/s  (limit: %d, max: %d, uart_errors: %d)\n",
                      (int)actual_speed, TARGET_TEST_SPEED, STS_MAX_SPEED, total_errors);

    TEST_ASSERT((int)actual_speed >= (int)MIN_SPEED &&
                (int)actual_speed <= (int)(TARGET_TEST_SPEED + MAX_SPEED_MARGIN),
                15, STS_ERR_HARDWARE,
                "Avg speed outside expected range (Goal Speed cap inactive or timing measurement corrupted)");

    return STS_OK;
}

static sts_result_t Test_Accel(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Acceleration Command Validation ---\n");

    uint8_t saved_accel = 0U;
    STS_Read8(servo, STS_REG_ACCELERATION, &saved_accel);

    STS_SetTargetPosition(servo, START_POS_OFFSET);
    {
        uint32_t settle_start = HAL_GetTick();
        uint16_t settle_pos   = TARGET_POS_TEST;
        do {
            HAL_Delay(20U);
            STS_GetPresentPosition(servo, &settle_pos);
        } while (settle_pos > (START_POS_OFFSET + POS_TOLERANCE) &&
                 (HAL_GetTick() - settle_start) < 5000U);
    }
    HAL_Delay(50U);

    sts_result_t res = STS_SetTargetAcceleration(servo, TARGET_SLOW_ACCEL);
    TEST_ASSERT(res == STS_OK, 16, res, "UART Tx Failed: Accel Command");

    uint8_t hardware_accel_state = 0;
    res = STS_Read8(servo, STS_REG_ACCELERATION, &hardware_accel_state);
    TEST_ASSERT(res == STS_OK, 17, res, "UART Rx Failed: Accel Readback");

    SEGGER_RTT_printf(0, ">> Hardware Accel Register: %d\n", hardware_accel_state);
    TEST_ASSERT(hardware_accel_state == TARGET_SLOW_ACCEL, 18, STS_ERR_HARDWARE, "Hardware rejected Acceleration write!");

    uint32_t start_time = HAL_GetTick();
    STS_SetTargetPosition(servo, TARGET_POS_TEST);

    while (1) {
        res = STS_GetPresentPosition(servo, &test_report.pos);
        if (res != STS_OK) break;

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
            test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
            break;
        }

        if ((HAL_GetTick() - start_time) > TEST_MOVE_TIMEOUT) break;
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    uint32_t total_move_time = HAL_GetTick() - start_time;

    SEGGER_RTT_printf(0, ">> Telemetry - Accel Move Time: %d ms\n", total_move_time);
    TEST_ASSERT(total_move_time >= MIN_SLOW_MOVE_TIME_MS, 19, STS_ERR_HARDWARE, "Slow accel move completed too quickly — accel setting may have been rejected");

    STS_SetTargetAcceleration(servo, saved_accel);
    return STS_OK;
}

static sts_result_t Test_Torque(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Torque State Machine Validation ---\n");

    sts_result_t res = STS_SetTorqueEnable(servo, TORQUE_OFF);
    TEST_ASSERT(res == STS_OK, 20, res, "UART Tx Failed: Disable Torque");

    HAL_Delay(50);

    int16_t disabled_load = 0;
    res = STS_GetPresentLoad(servo, &disabled_load);
    TEST_ASSERT(res == STS_OK, 21, res, "UART Rx Failed: Load Read");

    TEST_ASSERT(disabled_load == 0, 22, STS_ERR_HARDWARE, "Protocol fault: Load not 0 when disabled");

    res = STS_SetTargetPosition(servo, START_POS_OFFSET);
    TEST_ASSERT(res == STS_OK, 23, res, "UART Tx Failed: Move Command");
    HAL_Delay(500);

    res = STS_GetPresentPosition(servo, &test_report.pos);
    TEST_ASSERT(res == STS_OK, 24, res, "UART Rx Failed: Position Read");
    TEST_ASSERT(test_report.pos != START_POS_OFFSET, 25, STS_ERR_HARDWARE, "State fault: Motor accepted move command while disabled!");

    res = STS_SetTorqueEnable(servo, TORQUE_ON);
    TEST_ASSERT(res == STS_OK, 26, res, "UART Tx Failed: Re-enable Torque");

    return STS_OK;
}

static sts_result_t Test_MovingStatus(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Moving Status Validation ---\n");

    sts_result_t res = STS_SetTargetPosition(servo, START_POS_OFFSET);
    TEST_ASSERT(res == STS_OK, 27, res, "UART Tx Failed: MovingStatus move command");

    HAL_Delay(100);

    uint8_t moving = 0;
    res = STS_GetMovingStatus(servo, &moving);
    TEST_ASSERT(res == STS_OK, 28, res, "UART Rx Failed: Moving status read (moving)");
    TEST_ASSERT(moving == 1, 29, STS_ERR_HARDWARE, "Moving flag not set during active move");

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < TEST_MOVE_TIMEOUT) {
        uint16_t pos = 0;
        uint8_t  mv  = 1;
        if (STS_GetPresentPosition(servo, &pos) == STS_OK &&
            STS_GetMovingStatus(servo, &mv)     == STS_OK) {
            if (pos <= (START_POS_OFFSET + POS_TOLERANCE) && mv == 0) {
                break;
            }
        }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    res = STS_GetMovingStatus(servo, &moving);
    TEST_ASSERT(res == STS_OK, 30, res, "UART Rx Failed: Moving status read (stopped)");
    TEST_ASSERT(moving == 0, 31, STS_ERR_HARDWARE, "Moving flag not cleared after reaching target");

    return STS_OK;
}

uint8_t STS_RunIntegrationTests(sts_servo_t *servo) {
    memset(&test_report, 0, sizeof(sts_test_report_t));
    SEGGER_RTT_WriteString(0, "\n\n=== STARTING INTEGRATION TESTS ===\n");

    if (Test_Ping(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Ping failed. Aborting further tests.\n");
        goto test_end;
    }

    if (Test_Sensors(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Sensor Tests Failed. Aborting further tests.\n");
        goto test_end;
    }

    if (STS_Setup(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Setup homing failed. Aborting further tests.\n");
        goto test_end;
    }

    if (Test_Pos(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Position tests failed.\n");
        goto test_end;
    }

    if (Test_Speed(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Speed tests failed.\n");
        goto test_end;
    }

    if (Test_Accel(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Acceleration tests failed.\n");
        goto test_end;
    }

    if (Test_Torque(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Torque tests failed.\n");
        goto test_end;
    }

    if (Test_MovingStatus(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Moving status tests failed.\n");
        goto test_end;
    }

test_end:
    if (STS_Teardown(servo) != STS_OK) {
        test_report.tests_failed++;
        SEGGER_RTT_WriteString(0, "\n>> NOTE: Hardware left in an UNSAFE state.\n");
    }

    SEGGER_RTT_printf(0, "\n=== TEST SUITE COMPLETE ===\n");
    SEGGER_RTT_printf(0, "Passed: %d / %d\n", test_report.tests_passed, test_report.total_test_run);

    if (test_report.tests_failed > 0) {
        SEGGER_RTT_printf(0, ">> STATUS: FAILED (Last Error ID: %d)\n\n", test_report.last_failed_test_id);
    } else {
        SEGGER_RTT_printf(0, ">> STATUS: ALL TESTS PASSED!\n\n");
    }

    return test_report.tests_failed;
}
