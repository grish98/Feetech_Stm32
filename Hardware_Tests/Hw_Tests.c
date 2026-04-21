#include "Hw_Tests.h"
#include "Hw_utils.h"
#include "stm32f1xx_hal.h"
#include "sts_protocol.h"
#include "sts_servo.h"
#include "SEGGER_RTT.h"
#include <stdint.h>
#include <string.h>
#include "sts_servo_cmd.h"

#define Target_Pos 1024U

static sts_result_t Test_Ping(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "--- Ping Test ---\n");
    
    sts_result_t res = STS_servo_ping(servo);
    
    TEST_ASSERT(res == STS_OK, 1, res, "Ping failed!");
    TEST_ASSERT(servo->is_online == STS_ONLINE, 2, STS_ERR_HARDWARE, "Servo not online!");
    return res;
}

static sts_result_t Test_Pos(sts_servo_t *servo) {
    SEGGER_RTT_WriteString(0, "---  Pos Test ---\n");
    
    sts_result_t res = STS_SetTargetPosition(servo, Target_Pos);
    TEST_ASSERT(res == STS_OK, 3, res, "Failed to send move command");
    HAL_Delay(500); 
    
    res = STS_GetPresentPosition(servo, &test_report.pos);
    TEST_ASSERT(res == STS_OK, 4, res, "Failed to read pos after move");
    
    uint8_t reached_target = (test_report.pos >= (Target_Pos - 10U) && test_report.pos <= (Target_Pos + 10U));
    TEST_ASSERT(reached_target, 5, STS_ERR_HARDWARE, "Servo did not reach target pos");
    
    return STS_OK;
}

uint8_t STS_RunIntegrationTests(sts_servo_t *servo) {
    memset(&test_report, 0, sizeof(sts_test_report_t));
    SEGGER_RTT_WriteString(0, "\n\n=== STARTING INTEGRATION TESTS ===\n");

    if (Test_Ping(servo) != STS_OK) {
        SEGGER_RTT_WriteString(0, "CRITICAL: Ping failed. Aborting further tests.\n");
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
    
    // TODO: The rest of the tests

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