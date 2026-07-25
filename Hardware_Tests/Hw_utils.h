#pragma once

#include <stdint.h>
#include "sts_protocol.h"
#include "sts_servo.h"
#include "SEGGER_RTT.h"

#define TELEM_MAX_SAMPLES  512U
#define ACCEL_DEFAULT      0U

typedef struct {
    int16_t  load;
    uint16_t pos;
} telem_sample_t;

typedef struct {
    int16_t  mean;
    int16_t  peak;
    uint16_t stddev;
    uint16_t n;
} load_stats_t;

typedef struct {
    uint8_t total_test_run;
    uint8_t tests_passed;
    uint8_t tests_failed;
    uint8_t tests_skipped;        /**< EMI/settle events; not passes, not failures */
    uint8_t quiescence_waits;     /**< Times WaitServoQuiescent blocked (gate was load-bearing) */

    uint8_t last_failed_test_id;
    sts_result_t last_error_code;

    uint16_t pos;
    int16_t  holding_load;
    uint8_t  temp;
    uint8_t  volt;
} sts_test_report_t;

extern sts_test_report_t test_report;

extern telem_sample_t telem_buf[TELEM_MAX_SAMPLES];
extern uint16_t       telem_count;

void         Telem_Reset(void);
void         Telem_Record(int16_t load, uint16_t pos);
load_stats_t Telem_ComputeStats(void);

#define ASSERT_QUIESCENT(servo, label, test_id) \
    do { \
        if (WaitServoQuiescent((servo), (label)) != STS_OK) { \
            test_report.total_test_run++; \
            test_report.tests_failed++; \
            test_report.last_failed_test_id = (test_id); \
            test_report.last_error_code = STS_ERR_TIMEOUT; \
            SEGGER_RTT_printf(0, "FAIL Test %d: servo stuck before move\n", (int)(test_id)); \
            return STS_ERR_TIMEOUT; \
        } \
    } while(0)

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
