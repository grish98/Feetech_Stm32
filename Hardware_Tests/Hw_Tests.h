#pragma once
#include <stdint.h>
#include "sts_servo.h"

typedef struct {
    uint8_t total_test_run;
    uint8_t tests_passed;
    uint8_t tests_failed;

    uint8_t last_failed_test_id;
    sts_result_t last_error_code;
    
    uint16_t pos;
    uint8_t temp;
    uint8_t volt;

} sts_test_report_t;

extern sts_test_report_t test_report;

uint8_t STS_RunIntegrationTests(sts_servo_t *servo);

