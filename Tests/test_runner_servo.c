/**
 ******************************************************************************
 * @file           : test_runner_servo.c
 * @brief          : Unit test runner for STS Servo Service Layer
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 0.1.0
 ******************************************************************************
 * @details
 * Entry point for STS Service layer testing using the Unity framework.
 * Orchestrates execution of Bus HAL and Servo handle validation tests.
 ******************************************************************************
 */
 
#include "unity.h"
#include <stdio.h>
#include "test_sts_servo.h"
#include "unity_internals.h"



int main(void) {
    UNITY_BEGIN();

    printf("========== STS Service Layer Unit Tests ==========\n");

    printf("\n--- STS Bus Initialization Tests ---\n");
    RUN_TEST(test_STS_Bus_Init_Success);
    RUN_TEST(test_STS_Bus_Init_Null_Bus);
    RUN_TEST(test_STS_Bus_Init_Null_TX);
    RUN_TEST(test_STS_Bus_Init_Null_RX);
    RUN_TEST(test_STS_Bus_Init_Null_Port_Handle_Succeeds);
    RUN_TEST(test_STS_Bus_Init_Overwrites_Garbage);

    return UNITY_END();
}