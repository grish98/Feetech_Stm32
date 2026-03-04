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

    printf("\n--- STS Servo Initialization Tests ---\n");
    RUN_TEST(test_STS_Servo_Init_Success);
    RUN_TEST(test_STS_Servo_Init_Null_Servo);
    RUN_TEST(test_STS_Servo_Init_Null_Bus);
    RUN_TEST(test_STS_Servo_Init_Invalid_ID);
    RUN_TEST(test_STS_Servo_Init_ID_Boundaries);
    RUN_TEST(test_STS_Servo_Init_Clears_Handle_State);
    RUN_TEST(test_STS_Servo_Init_Atomic_Failure);
    RUN_TEST(test_STS_Servo_Init_Reset_Online_Status);
    RUN_TEST(test_STS_Servo_Init_Multi_Bus_Link);

    printf("\n--- STS Servo Read/Write Tests ---\n");
    RUN_TEST(test_STS_Write8_Success);
    RUN_TEST(test_STS_Read8_Success);
    RUN_TEST(test_STS_Write16_Success);
    RUN_TEST(test_STS_Read16_Success);
    RUN_TEST(test_STS_Primitives_All_Null_Guards);
    RUN_TEST(test_STS_Primitives_All_Timeout);
    RUN_TEST(test_STS_Primitives_All_Data_Integrity);
    RUN_TEST(test_STS_Primitives_All_Hardware_Errors);
    RUN_TEST(test_STS_ExecuteCommand_Header_Corruption);
    RUN_TEST(test_STS_ExecuteCommand_Length_Mismatch);
    RUN_TEST(test_STS_ExecuteCommand_Buffer_Overflow);
    RUN_TEST(test_STS_ExecuteCommand_Truncated_Packet);
    RUN_TEST(test_STS_Primitives_Read_Broadcast_Forbidden);
    

    return UNITY_END();
}