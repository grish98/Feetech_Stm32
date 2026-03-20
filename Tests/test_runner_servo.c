/**
 ******************************************************************************
 * @file           : test_runner_servo.c
 * @brief          : Unit test runner for STS Servo Service Layer
 * @author         : Grisham Balloo
 * @date           : 2026-03-19
 * @version        : 0.2.0
 ******************************************************************************
 * @details
 * Entry point for STS Service Layer testing using the Unity framework.
 * Orchestrates execution of all bus HAL, servo handle, read/write primitive,
 * command engine, and ping service test cases.
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
    RUN_TEST(test_STS_Bus_Init_Reinitialization);
    RUN_TEST(test_STS_Bus_Interface_Execution);

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

    printf("\n--- STS Command Engine Tests ---\n");
    RUN_TEST(test_STS_Primitives_All_Null_Guards);
    RUN_TEST(test_STS_Primitives_All_Timeout);
    RUN_TEST(test_STS_Primitives_All_Data_Integrity);
    RUN_TEST(test_STS_Primitives_All_Hardware_Errors);
    RUN_TEST(test_STS_ExecuteCommand_Header_Corruption);
    RUN_TEST(test_STS_ExecuteCommand_Length_Mismatch);
    RUN_TEST(test_STS_ExecuteCommand_Buffer_Overflow);
    RUN_TEST(test_STS_ExecuteCommand_Truncated_Packet);
    RUN_TEST(test_STS_Primitives_Read_Broadcast_Forbidden);
    RUN_TEST(test_STS_ExecuteCommand_TrashBin_Redirect);
    RUN_TEST(test_STS_ExecuteCommand_RX_Buffer_Overflow);
    RUN_TEST(test_STS_ExecuteCommand_TX_Buffer_Overflow);
    RUN_TEST(test_STS_ExecuteCommand_Zero_Expected_RX);
    RUN_TEST(test_STS_ExecuteCommand_Broadcast_Forces_Early_Exit);
    RUN_TEST(test_STS_ExecuteCommand_Null_Cmd_Guard);

    printf("\n--- STS Servo Ping Tests ---\n");
    RUN_TEST(test_STS_Ping_Success);    
    RUN_TEST(test_STS_Ping_Timeout_Sets_Offline);
    RUN_TEST(test_STS_Ping_ID_Mismatch_Sets_Offline);
    RUN_TEST(test_STS_Ping_Checksum_Error_Sets_Offline);
    RUN_TEST(test_STS_Ping_Bus_Busy_Sets_Offline);
    RUN_TEST(test_STS_Ping_Null_Servo_Guard);
    RUN_TEST(test_STS_Ping_Broadcast_Forbidden);
    RUN_TEST(test_STS_Ping_Null_Bus_Pointer);
    RUN_TEST(test_STS_Ping_Hardware_Error_Still_Online);
    RUN_TEST(test_STS_Ping_Max_Valid_ID);
    RUN_TEST(test_STS_Ping_Recovery_Offline_To_Online);

    printf("\n--- STS Servo Public API  ---\n");
    RUN_TEST(test_STS_SetTorqueEnable_Success);
    RUN_TEST(test_STS_SetTorqueEnable_Disable);
    RUN_TEST(test_STS_SetTorqueEnable_NonStandard_True);
    RUN_TEST(test_STS_SetTorqueEnable_Null_Pointer);
    RUN_TEST(test_STS_SetTorqueEnable_Error_Propagation);
    RUN_TEST(test_STS_SetTorqueEnable_Null_Pointer);
    RUN_TEST(test_STS_GetPresentPosition_Success);
    RUN_TEST(test_STS_Position_API_Null_Guards);
    RUN_TEST(test_STS_GetPresentPosition_Endianness);
    RUN_TEST(test_STS_SetTargetPosition_Hardware_Fault);
    RUN_TEST(test_STS_SetTargetPosition_Out_Of_Range);
    RUN_TEST(test_STS_SetTargetPosition_Broadcast_No_Wait);
    RUN_TEST(test_STS_GetPresentPosition_Fragmented_Packet);
    RUN_TEST(test_STS_GetPresentPosition_Bad_Checksum);
    RUN_TEST(test_STS_GetPresentPosition_Zero_Length_Fault);
    RUN_TEST(test_STS_GetPresentPosition_Buffer_Overflow_Guard);
    RUN_TEST(test_STS_GetPresentPosition_Wrong_ID_Response);
    RUN_TEST(test_STS_SetTargetPosition_Min_Boundary);
    RUN_TEST(test_STS_SetTargetPosition_Max_Boundary);
    RUN_TEST(test_STS_SetTargetPosition_Bus_Busy);
    RUN_TEST(test_STS_GetPresentPosition_Broadcast_Forbidden);
    RUN_TEST(test_STS_GetPresentPosition_Payload_Length_Mismatch);
    RUN_TEST(test_STS_GetPresentPosition_Stage2_Timeout);
    RUN_TEST(test_STS_SetTargetPosition_Just_Out_Of_Range);

    RUN_TEST(test_STS_Speed_Accel_Null_Guards);
    RUN_TEST(test_STS_SetTargetSpeed_Success);
    RUN_TEST(test_STS_SetTargetSpeed_Out_Of_Range);
    RUN_TEST(test_STS_GetPresentSpeed_Success);
    RUN_TEST(test_STS_SetTargetAcceleration_Success);
    RUN_TEST(test_STS_SetTargetAcceleration_Out_Of_Range);
    RUN_TEST(test_STS_SetTargetAcceleration_Max_Boundary);
    RUN_TEST(test_STS_SetTargetSpeed_Max_Boundary);
    RUN_TEST(test_STS_SetTargetSpeed_Reverse_Direction_Allowed);
    
    RUN_TEST(test_STS_SetOperatingMode_Null_Guard);
    RUN_TEST(test_STS_SetOperatingMode_Success);
    RUN_TEST(test_STS_SetOperatingMode_Invalid_Mode);
  

   
    return UNITY_END();
}