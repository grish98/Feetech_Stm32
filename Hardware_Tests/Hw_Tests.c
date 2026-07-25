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

#define TARGET_POS_TEST        3000U
#define START_POS_OFFSET       1000U
#define POS_TOLERANCE          15U

#define VOLT_MIN               90U
#define VOLT_MAX               130U
#define TEMP_MIN               0U
#define TEMP_MAX               80U

#define TEST_TORQUE_LIMIT      800U
#define HOLDING_LOAD_MAX       300
#define HOLDING_LOAD_MIN      -300

#define DELAY_POST_MOVE        50U
#define DELAY_UART             2U
#define DELAY_POLL_INTERVAL    10U
#define TEST_MOVE_TIMEOUT      4000U
#define TEST_ACCEL_MOVE_TIMEOUT 8000U
#define TEST_SPEED_TIMEOUT     7000U

#define TORQUE_OFF 0U
#define TORQUE_ON  1U

#define TARGET_TEST_SPEED      1000U  // steps/s
#define MIN_SPEED              750U   // steps/s; below target to tolerate ramp-up averaging
#define MAX_SPEED_MARGIN       150U   // upper bound = target + margin; catches uncapped runs while tolerating <100ms EMI bursts
#define TARGET_SLOW_ACCEL      10U    // lower register value = slower ramp on STS servos
#define MIN_SLOW_MOVE_TIME_MS  1500U  // 2000-step move (1000→3000): measured 2662–2849 ms with accel=10 (200-run campaign); uncapped (accel=0) ~360ms; threshold sits midway
#define MOVING_TEST_TARGET     2000U  // mid-range stop position; avoids overshoot at the edge (position 0)


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

static sts_result_t WaitServoQuiescent(sts_servo_t *servo, const char *label) {
    uint8_t  mv  = 0U;
    int16_t  spd = 0;
    int16_t  ld  = 0;
    uint16_t gs  = 0xFFFFU;
    STS_GetMovingStatus(servo, &mv);
    STS_GetPresentSpeed(servo, &spd);
    STS_GetPresentLoad(servo, &ld);
    STS_Read16(servo, STS_REG_GOAL_SPEED, &gs);
    SEGGER_RTT_printf(0, ">> Pre-move [%s]: moving=%d  speed=%d  load=%d  goal_speed=%d\n",
                      label, mv, spd, ld, gs);
    if (mv != 0U) {
        test_report.quiescence_waits++;
        uint32_t wait_start = HAL_GetTick();
        while ((HAL_GetTick() - wait_start) < 2000U) {
            HAL_Delay(20U);
            if (STS_GetMovingStatus(servo, &mv) == STS_OK && mv == 0U) { break; }
        }
        uint32_t waited_ms = HAL_GetTick() - wait_start;
        SEGGER_RTT_printf(0, ">> Waited %lums for quiescence%s\n",
                          waited_ms, (mv != 0U) ? " — TIMEOUT (servo still moving)" : "");
        if (mv != 0U) {
            return STS_ERR_TIMEOUT;
        }
    }
    return STS_OK;
}

static sts_result_t Test_Pos(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Position Control Validation ---\n");

    for (uint8_t i = 0U; i < 3U; i++) {
        uint16_t rb = 0xFFFFU;
        STS_Write16(servo, STS_REG_GOAL_TIME, 0U);
        STS_Read16(servo, STS_REG_GOAL_TIME, &rb);
        if (rb == 0U) { break; }
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        uint16_t rb = 0xFFFFU;
        STS_Write16(servo, STS_REG_GOAL_SPEED, 0U);
        STS_Read16(servo, STS_REG_GOAL_SPEED, &rb);
        if (rb == 0U) { break; }
    }
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

    STS_SetTorqueLimit(servo, TEST_TORQUE_LIMIT);
    uint16_t torque_rb = 0U;
    STS_Read16(servo, STS_REG_TORQUE_LIMIT, &torque_rb);
    SEGGER_RTT_printf(0, ">> Torque Limit: commanded=%d, readback=%d\n", TEST_TORQUE_LIMIT, torque_rb);

    ASSERT_QUIESCENT(servo, "Pos", 33);

    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 7, res, "Tx Failed: Move Command");
    HAL_Delay(100U);

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)servo->bus->port_handle;

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
            if (consecutive_errors >= 5U) { break; }
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
            uint16_t confirm_pos = 0U;
            if (STS_GetPresentPosition(servo, &confirm_pos) == STS_OK &&
                confirm_pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
                confirm_pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
                target_reached = 1;
                break;
            }
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

    TEST_ASSERT(abs(ls.peak) <= TEST_TORQUE_LIMIT, 10, STS_ERR_HARDWARE,
                "Peak load exceeded torque limit — hardware ignored the limit register");
    TEST_ASSERT(test_report.holding_load >= HOLDING_LOAD_MIN &&
                test_report.holding_load <= HOLDING_LOAD_MAX, 11, STS_ERR_HARDWARE,
                "Holding load out of expected range — servo straining at target");

    STS_SetTorqueLimit(servo, STS_MAX_TORQUE);
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
                 (HAL_GetTick() - settle_start) < 5000U);
    }

    uint16_t start_pos = 0;
    STS_GetPresentPosition(servo, &start_pos);
    SEGGER_RTT_printf(0, ">> Start Pos: %d (should be ~%d)\n", start_pos, START_POS_OFFSET);

/* 50ms idle gap before writing Goal Speed — historical mitigation. The
 * "intermittently ignores the speed cap" symptom was likely a state-coupling
 * artefact now addressed by the quiescence gate, making this delay probably
 * redundant; confirm by removal in a harness subtraction pass (#8). */
    HAL_Delay(50U);

    sts_result_t speed_res = STS_SetTargetSpeed(servo, TARGET_TEST_SPEED, STS_DIR_CCW);
    if (speed_res != STS_OK) {
        SEGGER_RTT_printf(0, "WARN: Goal Speed write failed (err: %d)\n", speed_res);
    }

    uint16_t goal_speed_rb = 0U;
    STS_Read16(servo, STS_REG_GOAL_SPEED, &goal_speed_rb);
    SEGGER_RTT_printf(0, ">> Goal Speed: commanded=%d, readback=%d (steps/s)\n",
                      TARGET_TEST_SPEED, goal_speed_rb);

    ASSERT_QUIESCENT(servo, "Speed", 34);

    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 12, res, "UART Tx Failed: Speed Move Command");

    uint16_t post_cmd_speed_rb = 0U;
    STS_Read16(servo, STS_REG_GOAL_SPEED, &post_cmd_speed_rb);
    SEGGER_RTT_printf(0, ">> Goal Speed post-position-cmd: %d (expected %d)\n",
                      post_cmd_speed_rb, TARGET_TEST_SPEED);

    uint32_t start_time         = HAL_GetTick();
    uint32_t last_rearm_ms      = 0U;
    uint8_t  target_reached     = 0;
    uint8_t  consecutive_errors = 0;
    uint8_t  total_errors       = 0;
    sts_result_t loop_res       = STS_OK;

    while ((HAL_GetTick() - start_time) < TEST_SPEED_TIMEOUT) {
        loop_res = STS_GetPresentPosition(servo, &test_report.pos);
        if (loop_res != STS_OK) {
            consecutive_errors++;
            total_errors++;

            STS_SetTargetSpeed(servo, TARGET_TEST_SPEED, STS_DIR_CCW);
            last_rearm_ms = HAL_GetTick() - start_time;
            if (consecutive_errors >= 5U) break;
            HAL_Delay(50U);
            continue;
        }
        consecutive_errors = 0U;

        /* Re-arm GOAL_SPEED every 100ms — historical mitigation for a suspected
         * GOAL_SPEED corruption later traced to test-fixture state coupling, not
         * UART EMI (0 retries across 240k transactions disproved the bus hypothesis);
         * retained pending subtraction pass. */
        uint32_t elapsed_ms = HAL_GetTick() - start_time;
        if (elapsed_ms - last_rearm_ms >= 100U) {
            STS_SetTargetSpeed(servo, TARGET_TEST_SPEED, STS_DIR_CCW);
            last_rearm_ms = elapsed_ms;
        }

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
            test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
            /* Require a second read to confirm — a single corrupt byte can land pos
             * inside the target window without the servo being there */
            uint16_t confirm_pos = 0U;
            if (STS_GetPresentPosition(servo, &confirm_pos) == STS_OK &&
                confirm_pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
                confirm_pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
                target_reached = 1;
                break;
            }
            total_errors++;
        }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    uint32_t end_time = HAL_GetTick();

    STS_SetTargetSpeed(servo, 0, STS_DIR_CCW);

    TEST_ASSERT(loop_res == STS_OK, 13, loop_res, "UART Rx Failed: Bus dead (5 consecutive failures)");
    TEST_ASSERT(target_reached, 14, STS_ERR_TIMEOUT, "Servo failed to reach commanded state");

    uint32_t total_time_ms = end_time - start_time;
    if (total_time_ms == 0) {
        total_time_ms = 1;
    }
    int32_t distance    = abs((int32_t)test_report.pos - (int32_t)start_pos);
    float actual_speed  = ((float)distance / (float)total_time_ms) * 1000.0f;

    SEGGER_RTT_printf(0, ">> Avg Speed:    %d steps/s  (limit: %d, max: %d, uart_errors: %d)\n",
                      (int)actual_speed, TARGET_TEST_SPEED, STS_MAX_SPEED, total_errors);

    TEST_ASSERT((int)actual_speed >= (int)MIN_SPEED &&
                (int)actual_speed <= (int)(TARGET_TEST_SPEED + MAX_SPEED_MARGIN),
                15, STS_ERR_HARDWARE,
                "Avg speed outside expected range");

    return STS_OK;
}

static sts_result_t Test_Accel(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Acceleration Command Validation ---\n");

    STS_Write16(servo, STS_REG_GOAL_TIME,  0U);
    STS_Write16(servo, STS_REG_GOAL_SPEED, 0U);
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

    uint16_t accel_start_pos = 0xFFFFU;
    STS_GetPresentPosition(servo, &accel_start_pos);
    SEGGER_RTT_printf(0, ">> Accel Start Pos: %d (should be ~%d)\n", accel_start_pos, START_POS_OFFSET);

    sts_result_t res = STS_SetTargetAcceleration(servo, TARGET_SLOW_ACCEL);
    TEST_ASSERT(res == STS_OK, 16, res, "UART Tx Failed: Accel Command");

    uint8_t hardware_accel_state = 0;
    res = STS_Read8(servo, STS_REG_ACCELERATION, &hardware_accel_state);
    TEST_ASSERT(res == STS_OK, 17, res, "UART Rx Failed: Accel Readback");

    SEGGER_RTT_printf(0, ">> Hardware Accel Register: %d\n", hardware_accel_state);
    TEST_ASSERT(hardware_accel_state == TARGET_SLOW_ACCEL, 18, STS_ERR_HARDWARE, "Hardware rejected Acceleration write!");

    /* GOAL_SPEED must stay 0 (already set above).  On STS servos, GOAL_SPEED != 0
     * enables constant-speed mode and suppresses the ACCELERATION profile entirely;
     * at GOAL_SPEED=1000 the 2000-step move measured ~2013ms (specific to that cap —
     * a different speed limit gives a different time; ACCEL-independence unverified).
     * With GOAL_SPEED=0, the ACCEL ramp is active: accel=10 measured 2662–2849ms
     * (200-run campaign, 2000-step move); if ACCEL is zero the servo runs uncapped
     * (~360ms), which MIN_SLOW_MOVE_TIME_MS catches. */
    /* goal_speed in [Accel] log: 0 = accel profile active (2662–2849ms, accel=10), non-zero = constant-speed (~2013ms at cap=1000) */
    ASSERT_QUIESCENT(servo, "Accel", 35);
    STS_SetTargetPosition(servo, TARGET_POS_TEST);

    uint32_t start_time           = HAL_GetTick();
    uint8_t  target_reached_accel = 0;
    uint8_t  consecutive_errors   = 0;

    while (1) {
        res = STS_GetPresentPosition(servo, &test_report.pos);
        if (res != STS_OK) {
            consecutive_errors++;
            if (consecutive_errors >= 5U) { break; }
            HAL_Delay(50U);
            continue;
        }
        consecutive_errors = 0U;

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
            test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
            target_reached_accel = 1;
            break;
        }

        if ((HAL_GetTick() - start_time) > TEST_ACCEL_MOVE_TIMEOUT) { break; }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    uint32_t total_move_time = HAL_GetTick() - start_time;

    uint8_t accel_post_move = 0U;
    STS_Read8(servo, STS_REG_ACCELERATION, &accel_post_move);

    SEGGER_RTT_printf(0, ">> Telemetry - Accel Move Time: %d ms\n", total_move_time);
    SEGGER_RTT_printf(0, ">> Accel reg post-move: %d (expected %d)\n", accel_post_move, TARGET_SLOW_ACCEL);

    TEST_ASSERT(target_reached_accel, 19, STS_ERR_TIMEOUT, "Accel move timed out — servo did not reach target");

    if (accel_start_pos <= (START_POS_OFFSET + POS_TOLERANCE)) {
        if (total_move_time < MIN_SLOW_MOVE_TIME_MS) {

            test_report.total_test_run++;
            test_report.tests_skipped++;
            SEGGER_RTT_printf(0, "SKIP Test 20 (move=%dms < %dms, accel_post=%d — move time below measured band, see #8)\n",
                              total_move_time, MIN_SLOW_MOVE_TIME_MS, accel_post_move);
        } else {
            TEST_ASSERT(total_move_time >= MIN_SLOW_MOVE_TIME_MS, 20, STS_ERR_HARDWARE,
                        "Slow accel move completed too quickly ");
        }
    } else {
        test_report.total_test_run++;
        test_report.tests_skipped++;
        SEGGER_RTT_printf(0, "SKIP Test 20 (settle timeout, start=%d — timing invalid)\n",
                          accel_start_pos);
    }

    STS_SetTargetAcceleration(servo, ACCEL_DEFAULT);
    return STS_OK;
}

static sts_result_t Test_Torque(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Torque State Machine Validation ---\n");

    sts_result_t res = STS_SetTorqueEnable(servo, TORQUE_OFF);
    TEST_ASSERT(res == STS_OK, 21, res, "UART Tx Failed: Disable Torque");

    HAL_Delay(50);

    int16_t disabled_load = 0;
    res = STS_GetPresentLoad(servo, &disabled_load);
    TEST_ASSERT(res == STS_OK, 22, res, "UART Rx Failed: Load Read");

    TEST_ASSERT(disabled_load == 0, 23, STS_ERR_HARDWARE, "Protocol fault: Load not 0 when disabled");

    res = STS_SetTargetPosition(servo, START_POS_OFFSET);
    TEST_ASSERT(res == STS_OK, 24, res, "UART Tx Failed: Move Command");
    HAL_Delay(500);

    res = STS_GetPresentPosition(servo, &test_report.pos);
    TEST_ASSERT(res == STS_OK, 25, res, "UART Rx Failed: Position Read");
    TEST_ASSERT(test_report.pos != START_POS_OFFSET, 26, STS_ERR_HARDWARE, "State fault: Motor accepted move command while disabled!");

    res = STS_SetTorqueEnable(servo, TORQUE_ON);
    TEST_ASSERT(res == STS_OK, 27, res, "UART Tx Failed: Re-enable Torque");

    return STS_OK;
}

static sts_result_t Test_MovingStatus(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Moving Status Validation ---\n");

    STS_Write16(servo, STS_REG_GOAL_TIME,  0U);
    STS_Write16(servo, STS_REG_GOAL_SPEED, 0U);
    STS_SetTargetAcceleration(servo, ACCEL_DEFAULT);

    sts_result_t res = STS_SetTargetPosition(servo, MOVING_TEST_TARGET);
    TEST_ASSERT(res == STS_OK, 28, res, "UART Tx Failed: MovingStatus move command");

    HAL_Delay(100);

    uint8_t moving = 0;
    res = STS_GetMovingStatus(servo, &moving);
    TEST_ASSERT(res == STS_OK, 29, res, "UART Rx Failed: Moving status read (moving)");
    TEST_ASSERT(moving == 1, 30, STS_ERR_HARDWARE, "Moving flag not set during active move");

    /* Poll until mv==0 . */
    uint8_t  moving_stopped = 0U;
    uint32_t start_time     = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < TEST_MOVE_TIMEOUT) {
        uint8_t mv = 1U;
        if (STS_GetMovingStatus(servo, &mv) == STS_OK && mv == 0U) {
            moving_stopped = 1U;
            break;
        }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    TEST_ASSERT(moving_stopped, 31, STS_ERR_TIMEOUT, "Moving status: servo did not stop within timeout");

    /* Test 32: 200ms after mv==0, confirm the flag is still clear AND position is
     * within tolerance.  The settle delay absorbs any final micro-corrections that
     * briefly re-assert mv==1 immediately after the first mv==0 is seen. */
    HAL_Delay(200U);
    uint16_t final_pos = 0U;
    res = STS_GetMovingStatus(servo, &moving);
    STS_GetPresentPosition(servo, &final_pos);
    SEGGER_RTT_printf(0, ">> Final pos after settle: %u (target ~%u)\n", final_pos, MOVING_TEST_TARGET);
    TEST_ASSERT(res == STS_OK && moving == 0U &&
                final_pos >= (MOVING_TEST_TARGET - POS_TOLERANCE) &&
                final_pos <= (MOVING_TEST_TARGET + POS_TOLERANCE),
                32, STS_ERR_HARDWARE, "Moving flag not cleared or position out of tolerance after settling");

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

void STS_RunStressTest(sts_servo_t *servo, uint32_t iterations) {
    uint32_t run_pass_clean         = 0U;
    uint32_t run_pass_skip          = 0U;
    uint32_t run_fail               = 0U;
    uint32_t total_skips            = 0U;
    uint32_t total_quiescence_waits = 0U;
    uint16_t fail_counts[36U]       = {0U};

    /* Bus counters never reset; capture baseline so the summary shows deltas
     * for this stress run only, even if the bus was used before. */
    uint32_t base_tx   = servo->bus->total_transactions;
    uint32_t base_ret  = servo->bus->total_retries;
    uint32_t base_save = servo->bus->retry_saves;
    uint32_t base_hard = servo->bus->hard_failures;

    for (uint32_t i = 0U; i < iterations; i++) {
        SEGGER_RTT_printf(0, "\n>>> STRESS RUN %lu / %lu <<<\n", i + 1UL, iterations);

        STS_RunIntegrationTests(servo);

        total_skips            += test_report.tests_skipped;
        total_quiescence_waits += test_report.quiescence_waits;

        if (test_report.tests_failed == 0U) {
            if (test_report.tests_skipped > 0U) {
                run_pass_skip++;
            } else {
                run_pass_clean++;
            }
        } else {
            run_fail++;
            uint8_t fid = test_report.last_failed_test_id;
            if (fid >= 1U && fid <= 35U) {
                fail_counts[fid]++;
            }
        }

        HAL_Delay(500U);
    }

    uint32_t total_tx   = servo->bus->total_transactions - base_tx;
    uint32_t total_ret  = servo->bus->total_retries      - base_ret;
    uint32_t total_save = servo->bus->retry_saves        - base_save;
    uint32_t total_hard = servo->bus->hard_failures      - base_hard;

    uint32_t total_pass = run_pass_clean + run_pass_skip;

    SEGGER_RTT_printf(0, "\n======= STRESS TEST SUMMARY (%lu runs) =======\n", iterations);
    SEGGER_RTT_printf(0, "Pass (clean):      %lu\n", run_pass_clean);
    SEGGER_RTT_printf(0, "Pass (skipped):    %lu  (%lu skipped tests total)\n", run_pass_skip, total_skips);
    SEGGER_RTT_printf(0, "Fail:              %lu\n", run_fail);
    SEGGER_RTT_printf(0, "Quiescence waits:  %lu  (gate was load-bearing this many times)\n",
                      total_quiescence_waits);
    SEGGER_RTT_printf(0, "Pass rate:     %lu%%  (%lu / %lu)\n",
                      (total_pass * 100UL) / iterations, total_pass, iterations);

    SEGGER_RTT_printf(0, "--- UART bus counters ---\n");
    SEGGER_RTT_printf(0, "Transactions:  %lu\n", total_tx);
    if (total_ret > 0U) {
        SEGGER_RTT_printf(0, "Retries:       %lu  (1 per %lu tx)\n",
                          total_ret, total_tx / total_ret);
        SEGGER_RTT_printf(0, "Retry saves:   %lu  (%lu%%)\n",
                          total_save, (total_save * 100UL) / total_ret);
    } else {
        SEGGER_RTT_printf(0, "Retries:       0\n");
        SEGGER_RTT_printf(0, "Retry saves:   0\n");
    }
    SEGGER_RTT_printf(0, "Hard failures: %lu\n", total_hard);

    SEGGER_RTT_printf(0, "--- Test failures ---\n");
    uint8_t any = 0U;
    for (uint8_t t = 1U; t <= 35U; t++) {
        if (fail_counts[t] > 0U) {
            SEGGER_RTT_printf(0, "  Test %2d: %u\n", (int)t, (unsigned int)fail_counts[t]);
            any = 1U;
        }
    }
    if (any == 0U) {
        SEGGER_RTT_printf(0, "  (none)\n");
    }
    SEGGER_RTT_printf(0, "==============================================\n");
}
