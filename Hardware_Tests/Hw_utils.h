#pragma once

#include <stdint.h>
#include "sts_protocol.h"
#include "sts_servo.h"
#include "SEGGER_RTT.h"
typedef struct {
    uint8_t total_test_run;
    uint8_t tests_passed;
    uint8_t tests_failed;

    uint8_t last_failed_test_id;
    sts_result_t last_error_code;
    
    uint16_t pos;
    int16_t  peak_load;   
    int16_t  holding_load; //Load at target Pos
    uint8_t temp;
    uint8_t volt;
    

} sts_test_report_t;

extern sts_test_report_t test_report;

#define TEST_ASSERT(condition, test_id, error_code, error_msg) \
    do { \
       test_report.total_test_run++; \
        if (condition) { \
           test_report.tests_passed++; \
            SEGGER_RTT_printf(0, "Pass Test %d\n", test_id); \
        } else { \
            test_report.tests_failed++; \
            test_report.last_failed_test_id = test_id; \
            test_report.last_error_code = error_code; \
            SEGGER_RTT_printf(0, "FAIL Test %d: %s (Error: %d)\n", test_id, error_msg, error_code); \
            return error_code; \
        } \
    } while(0)

    sts_result_t STS_Setup(sts_servo_t *servo);
    sts_result_t STS_Teardown(sts_servo_t *servo);