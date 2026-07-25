#pragma once

#include "sts_servo.h"

uint8_t STS_RunIntegrationTests(sts_servo_t *servo);
void    STS_RunStressTest(sts_servo_t *servo, uint32_t iterations);

