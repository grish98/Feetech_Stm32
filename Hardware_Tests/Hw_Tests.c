#include "Hw_Tests.h"
#include "stm32f1xx_hal.h"
#include "sts_protocol.h"
#include "sts_servo.h"
#include "SEGGER_RTT.h"
#include <stdint.h>
#include <string.h>
#include "sts_servo_cmd.h"

sts_test_report_t global_test_report = {0}; 

#define TEST_ASSERT(condition, test_id, error_code, error_msg) \
    do { \
        global_test_report.total_test_run++; \
        if (condition) { \
            global_test_report.tests_passed++; \
            SEGGER_RTT_printf(0, "Pass Test %d\n", test_id); \
        } else { \
            global_test_report.tests_failed++; \
            global_test_report.last_failed_test_id = test_id; \
            global_test_report.last_error_code = error_code; \
            SEGGER_RTT_printf(0, "FAIL Test %d: %s (Error: %d)\n", test_id, error_msg, error_code); \
            return error_code; \
        } \
    } while(0)

static sts_result_t Test_Ping(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Ping Test ---\n");
    
    sts_result_t res = STS_servo_ping(servo);
    
    TEST_ASSERT(res == STS_OK, 1, res, "Ping failed!");
    TEST_ASSERT(servo->is_online == STS_ONLINE, 2, STS_ERR_HARDWARE, "Servo not online!");
    return res;
}

static sts_result_t Test_Pos(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "---  Pos Test ---\n");
    
    sts_result_t res = STS_SetTargetPosition(servo, 1024);
    TEST_ASSERT(res == STS_OK, 3, res, "Failed to send move command");
    HAL_Delay(500); 
    
    res = STS_GetPresentPosition(servo, &global_test_report.pos);
    TEST_ASSERT(res == STS_OK, 4, res, "Failed to read pos after move");
    
    uint8_t reached_target = (global_test_report.pos >= 1014 && global_test_report.pos <= 1034);
    TEST_ASSERT(reached_target, 5, STS_ERR_HARDWARE, "Servo did not reach target pos");
    
    return STS_OK;
}

uint8_t STS_RunIntegrationTests(sts_servo_t *servo) {
    memset(&global_test_report, 0, sizeof(sts_test_report_t));
    SEGGER_RTT_WriteString(0, "\n\n=== STARTING INTERGRATION TESTS ===\n");

    if (Test_Ping(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, " Ping failed. Aborting further tests.\n");
        goto test_end; 
    }

    SEGGER_RTT_WriteString(0, "--- Homing servo Position ---\n");
    STS_SetTargetPosition(servo, 2048);
    HAL_Delay(1000); 


    if (Test_Pos(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "Position test failed.\n");
        goto test_end;
    }
    
    // TODO: The rest of the tests

    test_end:
   
    if (servo->is_online == STS_ONLINE) {
    SEGGER_RTT_WriteString(0, "--- Hardware Teardown ---\n");
    STS_SetTargetPosition(servo, 2048); 
    HAL_Delay(1000); 
    
    sts_result_t res = STS_GetPresentPosition(servo, &global_test_report.pos);
    
    if (res != STS_OK) {
        SEGGER_RTT_printf(0, "WARNING: Teardown read failed (Error: %d)\n", res);
    } 
    else if (global_test_report.pos < 2038 || global_test_report.pos > 2058) {
        SEGGER_RTT_printf(0, "WARNING: Teardown homing failed! Stuck at pos: %d\n", global_test_report.pos);
    } 
    else {
        SEGGER_RTT_WriteString(0, "Teardown homing successful.\n");
    }
}

    SEGGER_RTT_printf(0, "\n=== TEST SUITE COMPLETE ===\n");
    SEGGER_RTT_printf(0, "Passed: %d / %d\n", global_test_report.tests_passed, global_test_report.total_test_run);
    
    if (global_test_report.tests_failed > 0) {
        SEGGER_RTT_printf(0, ">> STATUS: FAILED (Last Error ID: %d)\n\n", global_test_report.last_failed_test_id);
    } else {
        SEGGER_RTT_printf(0, ">> STATUS: ALL TESTS PASSED!\n\n");
    }

    return global_test_report.tests_failed;
}