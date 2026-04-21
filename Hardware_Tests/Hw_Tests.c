#include "Hw_Tests.h"
#include "Hw_utils.h"
#include "stm32f1xx_hal.h"
#include "sts_protocol.h"
#include "sts_servo.h"
#include "SEGGER_RTT.h"
#include <stdint.h>
#include <string.h>
#include "sts_servo_cmd.h"


// --- Position Constants ---
#define TARGET_POS_TEST        1024U
#define START_POS_OFFSET       3000U  
#define POS_TOLERANCE          10U

// --- sensor Thresholds ---
#define VOLT_MIN            90U    // 9.0V
#define VOLT_MAX            130U   // 13.0V
#define TEMP_MIN           0U 
#define TEMP_MAX           80U

// --- Load/Torque Thresholds ---
#define DYNAMIC_LOAD_MAX      1000
#define DYNAMIC_LOAD_MIN     -1000
#define HOLDING_LOAD_MAX      300
#define HOLDING_LOAD_MIN     -300

// --- Timing & Delays (ms) ---
#define DELAY_PRE_TEST_MOVE     1000U
#define DELAY_POST_MOVE_SETTLE  50U
#define DELAY_UART_BREATH      2U
#define DELAY_POLL_INTERVAL    10U
#define TEST_MOVE_TIMEOUT      2000U

static sts_result_t Test_Ping(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Ping Test ---\n");
    
    sts_result_t res = STS_servo_ping(servo);
    
    TEST_ASSERT(res == STS_OK, 1, res, "Ping failed!");
    TEST_ASSERT(servo->is_online == STS_ONLINE, 2, STS_ERR_HARDWARE, "Servo not online!");
    return res;
}

static sts_result_t Test_Sensors(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Sensor Health Test ---\n");
    sts_result_t res;

    res = STS_GetPresentVoltage(servo, &test_report.volt);
    TEST_ASSERT(res == STS_OK, 3, res, "Failed to read voltage");
    TEST_ASSERT(test_report.volt > VOLT_MIN && test_report.volt < VOLT_MAX, 4, STS_ERR_HARDWARE, "Warning: Voltage too low (Less than 9.0V or Higher than 13V)"); 

    res = STS_GetPresentTemperature(servo, &test_report.temp);
    TEST_ASSERT(res == STS_OK, 5, res, "Failed to read temperature");
    TEST_ASSERT(test_report.temp > TEMP_MIN && test_report.temp < TEMP_MAX, 6, STS_ERR_HARDWARE, "Warning: Unsafe Temperature");

    return STS_OK;
}


static sts_result_t Test_Pos(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Position & Dynamic Load Test ---\n");
    
    STS_SetTargetPosition(servo, START_POS_OFFSET);
    HAL_Delay(DELAY_PRE_TEST_MOVE);

    sts_result_t res = STS_SetTargetPosition(servo, TARGET_POS_TEST);
    TEST_ASSERT(res == STS_OK, 7, res, "Move Command Failed");

    uint32_t start_time = HAL_GetTick();
    uint8_t target_reached = 0;
    int16_t current_load = 0;
    
    test_report.peak_load = 0; 

    while ((HAL_GetTick() - start_time) < TEST_MOVE_TIMEOUT) { 
        
        STS_GetPresentPosition(servo, &test_report.pos);
        HAL_Delay(DELAY_UART_BREATH); 
        
        if (STS_GetPresentLoad(servo, &current_load) == STS_OK) {

            if (abs(current_load) > abs(test_report.peak_load)) {
                test_report.peak_load = current_load;
            }
        }

        if (test_report.pos >= (TARGET_POS_TEST - POS_TOLERANCE) &&  test_report.pos <= (TARGET_POS_TEST + POS_TOLERANCE)) 
        {
            target_reached = 1;
            break; 
        }
        
        HAL_Delay(DELAY_POLL_INTERVAL); 
    }

    TEST_ASSERT(target_reached, 8, STS_ERR_HARDWARE, "Servo timed out reaching target");
    TEST_ASSERT(test_report.peak_load > DYNAMIC_LOAD_MIN && test_report.peak_load < DYNAMIC_LOAD_MAX, 9, STS_ERR_HARDWARE, "High Resistance detected (Stalling)!");
    HAL_Delay(DELAY_POST_MOVE_SETTLE);

    res = STS_GetPresentLoad(servo, &test_report.holding_load);
    TEST_ASSERT(res == STS_OK, 10, res, "Failed to read holding load");
    TEST_ASSERT(test_report.holding_load > HOLDING_LOAD_MIN && test_report.holding_load < HOLDING_LOAD_MAX, 11, STS_ERR_HARDWARE, "Servo is straining at rest!");

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
        SEGGER_RTT_WriteString(0, "CRITICAL: Position test failed.\n");
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