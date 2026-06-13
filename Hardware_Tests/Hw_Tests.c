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


// --- Position Constants ---
#define TARGET_POS_TEST        4000U
#define START_POS_OFFSET       0U  
#define POS_TOLERANCE          15U

// --- sensor Thresholds ---
#define VOLT_MIN            90U    // 9.0V
#define VOLT_MAX            130U   // 13.0V
#define TEMP_MIN           0U  //In celsius
#define TEMP_MAX           80U

// --- Load/Torque Thresholds ---
#define DYNAMIC_LOAD_MAX      1000
#define DYNAMIC_LOAD_MIN     -1000
#define HOLDING_LOAD_MAX      300
#define HOLDING_LOAD_MIN     -300

// --- Timing & Delays (ms) ---
#define DELAY_PRE_TEST_MOVE     1000U
#define DELAY_POST_MOVE          50U
#define DELAY_UART              2U
#define DELAY_POLL_INTERVAL    10U
#define TEST_MOVE_TIMEOUT      4000U
#define TEST_SPEED_TIMEOUT     7000U  // Speed test moves at 1000 steps/s over 4000 steps (~4s) + polling overhead

#define TORQUE_OFF 0U
#define TORQUE_ON  1U

// --- Kinematic Thresholds ---
#define TARGET_TEST_SPEED       1000U  // Steps per second
#define MIN_SPEED       1200U  // Minimum acceptable speed under load
#define TARGET_SLOW_ACCEL       100U   
#define MIN_SLOW_MOVE_TIME_MS   1000U  // A slow accel move should take AT LEAST this long


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
    
    STS_SetTargetPosition(servo, START_POS_OFFSET);
    HAL_Delay(DELAY_PRE_TEST_MOVE);

    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 7, res, "Tx Failed: Move Command");
    HAL_Delay(100);

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)servo->bus->port_handle;
    HAL_Delay(DELAY_UART); 

    uint32_t start_time = HAL_GetTick();
    uint8_t target_reached = 0;
    int16_t current_load = 0;
    test_report.peak_load = 0; 

    sts_result_t loop_res = STS_OK;
    uint8_t consecutive_errors = 0;

    while ((HAL_GetTick() - start_time) < TEST_MOVE_TIMEOUT) { 
        loop_res = STS_GetPresentPosition(servo, &test_report.pos);
    
        if (loop_res != STS_OK) {
            consecutive_errors++;
            if (consecutive_errors == 1) {
                uint32_t err = HAL_UART_GetError(huart);
                SEGGER_RTT_printf(0, ">> Test 8 Loop Error. HAL Error Code: 0x%08X\n", err);
            }

            HAL_Delay(50); 
            continue; 
        }
        consecutive_errors = 0; 
    
        HAL_Delay(DELAY_UART); 
        
        if (STS_GetPresentLoad(servo, &current_load) == STS_OK) {

            SEGGER_RTT_printf(0, "Live Load: %d\n", current_load);

            if (abs(current_load) > abs(test_report.peak_load)) {
                test_report.peak_load = current_load;
            }
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

    SEGGER_RTT_printf(0, ">> Telemetry - Peak Load: %d, Holding Load: %d\n", test_report.peak_load, test_report.holding_load);

    TEST_ASSERT(test_report.peak_load >= DYNAMIC_LOAD_MIN && test_report.peak_load <= DYNAMIC_LOAD_MAX, 10, STS_ERR_HARDWARE, "Peak load exceeded dynamic range — possible overload");
    TEST_ASSERT(test_report.holding_load >= HOLDING_LOAD_MIN && test_report.holding_load <= HOLDING_LOAD_MAX, 11, STS_ERR_HARDWARE, "Holding load out of expected range — servo straining at target");

    return STS_OK;
}

static sts_result_t Test_Speed(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Speed Profiling Validation ---\n");
    
    STS_SetTargetPosition(servo, START_POS_OFFSET);
    HAL_Delay(DELAY_PRE_TEST_MOVE);

    uint16_t start_pos = 0;
    STS_GetPresentPosition(servo, &start_pos);

    STS_SetTargetSpeed(servo, TARGET_TEST_SPEED, STS_DIR_CCW);
    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 12, res, "UART Tx Failed: Speed Move Command");

    uint32_t start_time = HAL_GetTick();
    int16_t current_speed = 0;
    uint8_t target_reached = 0;
    uint8_t consecutive_errors = 0;
    test_report.peak_speed = 0;

    sts_result_t loop_res = STS_OK; 

    while ((HAL_GetTick() - start_time) < TEST_SPEED_TIMEOUT) {

        loop_res = STS_GetPresentSpeed(servo, &current_speed);

        if (loop_res == STS_OK) {
            consecutive_errors = 0;

            if (abs(current_speed) > abs(test_report.peak_speed)) {
                test_report.peak_speed = current_speed;
            }
        } else {
            consecutive_errors++;
            if (consecutive_errors >= 5) break;
        }

        loop_res = STS_GetPresentPosition(servo, &test_report.pos);
        if (loop_res != STS_OK) break;

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&
            test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) {
            target_reached = 1;
            break;
        }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    uint32_t end_time = HAL_GetTick(); 

    STS_SetTargetSpeed(servo, 0, STS_DIR_CCW); 

    TEST_ASSERT(loop_res == STS_OK, 13, loop_res, "UART Rx Failed: Bus crashed during polling");
    TEST_ASSERT(target_reached, 14, STS_ERR_TIMEOUT, "Servo failed to reach commanded state");

    uint32_t total_time_ms = end_time - start_time;
    if (total_time_ms == 0) total_time_ms = 1; 
    
    int32_t distance = abs((int32_t)test_report.pos - (int32_t)start_pos);
    float actual_speed = ((float)distance / (float)total_time_ms) * 1000.0f;

    SEGGER_RTT_printf(0, ">> Telemetry - Commanded: %d, Peak Decoded: %d\n", TARGET_TEST_SPEED, test_report.peak_speed);
    SEGGER_RTT_printf(0, ">> DIAGNOSTIC - Reality Check: %d steps/s\n", (int)actual_speed);

    TEST_ASSERT(abs(test_report.peak_speed) >= (int16_t)MIN_SPEED, 15, STS_ERR_HARDWARE, "Peak speed below minimum threshold under load");

    return STS_OK;
}

static sts_result_t Test_Accel(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Acceleration Command Validation ---\n");
    
    STS_SetTargetPosition(servo, START_POS_OFFSET);
    HAL_Delay(DELAY_PRE_TEST_MOVE);

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

    STS_SetTargetAcceleration(servo, 0);
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

    // Servo arrives here near TARGET_POS_TEST; command a full return move to START_POS_OFFSET
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
        if (STS_GetPresentPosition(servo, &pos) == STS_OK) {
            if (pos <= (START_POS_OFFSET + POS_TOLERANCE)) {
                break;
            }
        }
        HAL_Delay(DELAY_POLL_INTERVAL);
    }

    HAL_Delay(DELAY_POST_MOVE);

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

    if (Test_Speed(servo) != STS_OK){
        SEGGER_RTT_WriteString(0, "CRITICAL: Speed tests failed.\n");
        goto test_end;

    }
    
    if (Test_Accel( servo) != STS_OK){
        SEGGER_RTT_WriteString(0, "CRITICAL: Acelleration tests failed.\n");
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