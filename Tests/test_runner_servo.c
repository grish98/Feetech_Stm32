/**
 ******************************************************************************
 * @file           : test_runner_servo.c
 * @brief          : Unit test runner for STS Servo Service Layer
 * @author         : Grisham Balloo
 * @date           : 2026-03-21
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
#include "test_sts_servo.h" 



int main(void) {
    UNITY_BEGIN();

 printf("========== STS Service Layer Unit Tests ==========\n");

    printf("\n--- STS Bus Tests ---\n");
    RUN_TEST(test_STS_Bus_Init_Success);
    RUN_TEST(test_init_fails_with_null_bus);
    RUN_TEST(test_STS_Bus_Init_Null_TX);
    RUN_TEST(test_STS_Bus_Init_Null_RX);
    RUN_TEST(test_STS_Bus_Null_TX);
    RUN_TEST(test_STS_Bus_NULL_RX);
    RUN_TEST(test_STS_Bus_TX_Null_Bus);
    RUN_TEST(test_STS_Bus_RX_Null_Bus);
    RUN_TEST(test_STS_Bus_Init_Null_Port);
    RUN_TEST(test_STS_Bus_Transmit_Null_Port);
    RUN_TEST(test_STS_Bus_Init_Overwrites_Invalid);
    RUN_TEST(test_STS_Bus_Reinitialisation);
    RUN_TEST(test_STS_Bus_Transmit_Routes_To_HAL);
    RUN_TEST(test_STS_Bus_Receive_Pulls_From_HAL);
    RUN_TEST(test_STS_Bus_Tx_Valid_Payload);
    RUN_TEST(test_STS_Bus_Rx_Valid_Payload);
    RUN_TEST(test_Bus_Tx_Fail);
    RUN_TEST(test_STS_Bus_Rx_Hal_Timeout);
    RUN_TEST(test_STS_Bus_Rx_Partial);
    RUN_TEST(test_STS_Bus_Transmit_Zero_Length_NoOp);
    RUN_TEST(test_STS_Bus_Receive_Zero_Length_NoOp);
    RUN_TEST(test_STS_Bus_Transmit_Null_Function_Pointer_Guarded);


    printf("\n--- STS Servo Initialisation Tests ---\n");
    RUN_TEST(test_STS_Servo_Init_Success);
    RUN_TEST(test_STS_Servo_Init_Null_Servo);
    RUN_TEST(test_STS_Servo_Init_Null_Bus);
    RUN_TEST(test_STS_Servo_Init_BroadcastSync_ID_Rejected);
    RUN_TEST(test_STS_Servo_Init_BroadcastAsync_ID_Rejected);
    RUN_TEST(test_STS_Servo_Init_MinID_Accepted);
    RUN_TEST(test_STS_Servo_Init_MaxID_Accepted);
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
    RUN_TEST(test_STS_Write8_Rejects_Null_Servo);
    RUN_TEST(test_STS_Write16_Rejects_Null_Servo);
    RUN_TEST(test_STS_Read8_Rejects_Null_Servo);
    RUN_TEST(test_STS_Read16_Rejects_Null_Servo);
    RUN_TEST(test_STS_Read8_Rejects_Null_Out_Pointer);
    RUN_TEST(test_STS_Read16_Rejects_Null_Out_Pointer);
    RUN_TEST(test_STS_Write8_Yields_Timeout);
    RUN_TEST(test_STS_Write16_Yields_Timeout);
    RUN_TEST(test_STS_Read8_Yields_Timeout);
    RUN_TEST(test_STS_Read16_Yields_Timeout);
    RUN_TEST(test_STS_Write16_Yields_ID_Mismatch);
    RUN_TEST(test_STS_Read16_Yields_Checksum_Error);
    RUN_TEST(test_STS_Write8_Yields_Bus_Busy);
    RUN_TEST(test_STS_Read16_Yields_TX_Fail);
    RUN_TEST(test_STS_Read8_Yields_Bus_Busy);
    RUN_TEST(test_STS_Write16_Yields_TX_Fail);
    RUN_TEST(test_STS_ExecuteCommand_Header_Corruption);
    RUN_TEST(test_STS_ExecuteCommand_Length_Mismatch);
    RUN_TEST(test_STS_ExecuteCommand_Truncated_Packet);
    RUN_TEST(test_STS_Primitives_Read_Broadcast_Forbidden);
    RUN_TEST(test_STS_ExecuteCommand_Null_Cmd_Guard);
    RUN_TEST(test_STS_ExecuteCommand_Null_Servo_Guard);
    RUN_TEST(test_STS_ExecuteCommand_Null_RxOut_Discards_Payload);
    RUN_TEST(test_STS_ExecuteCommand_RX_Buffer_Overflow);
    RUN_TEST(test_STS_ExecuteCommand_TX_Buffer_Overflow);
    RUN_TEST(test_STS_ExecuteCommand_Zero_Expected_RX);
    RUN_TEST(test_STS_ExecuteCommand_Broadcast_Forces_Early_Exit);
    RUN_TEST(test_STS_ExecuteCommand_Rejects_Uninitialised_Bus_Pointers);
    RUN_TEST(test_STS_ExecuteCommand_Standard_Write_Safety);


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

    printf("\n--- STS Torque Enable Tests ---\n");    
    RUN_TEST(test_STS_SetTorqueEnable_Success);
    RUN_TEST(test_STS_SetTorqueEnable_Disable);
    RUN_TEST(test_STS_SetTorqueEnable_NonStandard_True);
    RUN_TEST(test_STS_SetTorqueEnable_Null_Pointer);
    RUN_TEST(test_STS_SetTorqueEnable_Error_Propagation);

    printf("\n--- STS Position Control Tests ---\n");
    RUN_TEST(test_STS_GetPresentPosition_Success);
    RUN_TEST(test_STS_SetTargetPosition_Success);
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

    printf("\n--- STS Speed & Acceleration Tests ---\n");
    RUN_TEST(test_STS_Speed_Accel_Null_Guards);
    RUN_TEST(test_STS_SetTargetSpeed_Success);
    RUN_TEST(test_STS_SetTargetSpeed_Out_Of_Range);
    RUN_TEST(test_STS_GetPresentSpeed_Success);
    RUN_TEST(test_STS_SetTargetAcceleration_Success);
    RUN_TEST(test_STS_SetTargetAcceleration_Out_Of_Range);
    RUN_TEST(test_STS_SetTargetAcceleration_Max_Boundary);
    RUN_TEST(test_STS_SetTargetAcceleration_Min_Boundary);
    RUN_TEST(test_STS_SetTargetSpeed_Max_Boundary);
    RUN_TEST(test_STS_SetTargetSpeed_Min_Boundary);
    RUN_TEST(test_STS_SetTargetSpeed_Reverse_Direction_Allowed);
    RUN_TEST(test_STS_SetTargetSpeed_Max_Reverse_Allowed);

    printf("\n--- STS Operating Mode Tests ---\n");
    RUN_TEST(test_STS_SetOperatingMode_Null_Guard);
    RUN_TEST(test_STS_SetOperatingMode_Success);
    RUN_TEST(test_STS_SetOperatingMode_Invalid_Mode);

    printf("\n--- STS PWM & Step Control Tests ---\n");
    RUN_TEST(test_STS_PWM_Step_Null_Guards);
    RUN_TEST(test_STS_SetTargetPWM_Out_Of_Range);
    RUN_TEST(test_STS_SetTargetStep_Out_Of_Range);
    RUN_TEST(test_STS_SetTargetPWM_Success);
    RUN_TEST(test_STS_SetTargetStep_Success);
    RUN_TEST(test_STS_SetTargetPWM_Max_Boundary);
    RUN_TEST(test_STS_SetTargetStep_Max_Boundary);
    RUN_TEST(test_STS_SetTarget_Zero_Boundary);
    RUN_TEST(test_STS_SetTargetStep_Zero_Boundary);
    RUN_TEST(test_STS_SetTargetPWM_Zero_Boundary);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Min_Boundary);
    RUN_TEST(test_STS_SetTarget_PWMMode_Min_Boundary);
    RUN_TEST(test_STS_SetTarget_StepMode_Min_Boundary);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Negative_Out_Of_Range);
    RUN_TEST(test_STS_SetTarget_PWMMode_Negative_Out_Of_Range);
    RUN_TEST(test_STS_SetTarget_StepMode_Positive_Out_Of_Range);

    printf("\n--- STS Universal Target Routing Tests ---\n");
    RUN_TEST(test_STS_SetTarget_PositionMode_Normal);
    RUN_TEST(test_STS_SetTarget_PositionMode_Negative_Rejected);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Positive_CCW);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Negative_CW);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Out_Of_Range);
    RUN_TEST(test_STS_SetTarget_PWMMode_Negative_CW);
    RUN_TEST(test_STS_SetTarget_StepMode_Positive_CCW);
    RUN_TEST(test_STS_SetTarget_InvalidMode);
    RUN_TEST(test_STS_SetTarget_PWMMode_Out_Of_Range);
    RUN_TEST(test_STS_SetTarget_StepMode_Out_Of_Range);
    RUN_TEST(test_STS_SetTarget_PWMMode_Positive_CCW);
    RUN_TEST(test_STS_SetTarget_StepMode_Negative_CW);

    printf("\n--- STS Universal Target Range & Overflow Tests ---\n");
    RUN_TEST(test_STS_SetTarget_PositionMode_Zero);
    RUN_TEST(test_STS_SetTarget_PositionMode_Max_Boundary);
    RUN_TEST(test_STS_SetTarget_PositionMode_Above_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_PositionMode_Uint16_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_PositionMode_No_Wrap_To_Zero);
    RUN_TEST(test_STS_SetTarget_PositionMode_No_Wrap_To_Valid);
    RUN_TEST(test_STS_SetTarget_PositionMode_No_Wrap_To_Max);
    RUN_TEST(test_STS_SetTarget_PositionMode_Int32_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_PositionMode_Minus_One_Rejected);
    RUN_TEST(test_STS_SetTarget_PositionMode_Int32_Min_Rejected);
    RUN_TEST(test_STS_SetTarget_PositionMode_Int32_Min_Plus_One_Rejected);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Max_Boundary);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Above_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Below_Min_Rejected);
    RUN_TEST(test_STS_SetTarget_SpeedMode_No_Wrap_To_Zero);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Negative_No_Wrap);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Int32_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Int32_Min_Plus_One_Rejected);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Int32_Min_Rejected);
    RUN_TEST(test_STS_SetTarget_PWMMode_Max_Boundary);
    RUN_TEST(test_STS_SetTarget_PWMMode_Above_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_PWMMode_Speed_Range_Rejected);
    RUN_TEST(test_STS_SetTarget_PWMMode_No_Wrap_To_Valid);
    RUN_TEST(test_STS_SetTarget_PWMMode_Int32_Min_Rejected);
    RUN_TEST(test_STS_SetTarget_PWMMode_Zero);
    RUN_TEST(test_STS_SetTarget_PWMMode_Int32_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_PWMMode_Int32_Min_Plus_One_Rejected);
    RUN_TEST(test_STS_SetTarget_StepMode_Zero);
    RUN_TEST(test_STS_SetTarget_StepMode_Max_Boundary);
    RUN_TEST(test_STS_SetTarget_StepMode_Above_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_StepMode_Below_Min_Rejected);
    RUN_TEST(test_STS_SetTarget_StepMode_No_Wrap_To_Valid);
    RUN_TEST(test_STS_SetTarget_StepMode_Int32_Min_Rejected);
    RUN_TEST(test_STS_SetTarget_StepMode_Int32_Max_Rejected);
    RUN_TEST(test_STS_SetTarget_StepMode_Int32_Min_Plus_One_Rejected);
    RUN_TEST(test_STS_SetTarget_InvalidMode_Int32_Min);
    RUN_TEST(test_STS_SetTarget_Null_Servo_Int32_Min);
    RUN_TEST(test_STS_SetTarget_Propagates_Timeout);
    RUN_TEST(test_STS_SetTarget_Propagates_Hardware_Error);
    RUN_TEST(test_STS_SetTarget_SpeedMode_Propagates_Timeout);
    RUN_TEST(test_STS_SetTarget_PWMMode_Propagates_Timeout);
    RUN_TEST(test_STS_SetTarget_StepMode_Propagates_Timeout);
    RUN_TEST(test_STS_SetTarget_Null_Bus);
    RUN_TEST(test_STS_SetTarget_Null_Bus_Range_Checked_First);

    printf("\n--- STS Torque Limit Tests ---\n");
    RUN_TEST(test_STS_SetTorqueLimit_Null_Guard);
    RUN_TEST(test_STS_SetTorqueLimit_Out_Of_Range);
    RUN_TEST(test_STS_SetTorqueLimit_Success);
    RUN_TEST(test_STS_SetTorqueLimit_Max_Boundary);
    RUN_TEST(test_STS_SetTorqueLimit_Zero_Boundary);


    printf("\n--- STS Telemetry Tests ---\n");
    RUN_TEST(test_STS_Telemetry_Null_Guards);
    RUN_TEST(test_STS_GetPresentLoad_Success);
    RUN_TEST(test_STS_GetPresentVoltage_Success);
    RUN_TEST(test_STS_GetPresentTemperature_Success);
    RUN_TEST(test_STS_GetMovingStatus_Success);
    RUN_TEST(test_STS_Telemetry_Bubbles_Hardware_Error);
    RUN_TEST(test_STS_Telemetry_Preserves_State_On_Timeout);


    printf("\n--- STS EEPROM & ID Config Tests ---\n");
    RUN_TEST(test_STS_SetEEPROMLock_Null_Guard);
    RUN_TEST(test_STS_SetEEPROMLock_States);
    RUN_TEST(test_STS_SetID_Null_Guard);
    RUN_TEST(test_STS_SetID_Out_Of_Range);
    RUN_TEST(test_STS_SetID_Success);

    printf("\n--- Scripted Transport Self-Tests ---\n");
    RUN_TEST(test_Mock_Script_Reply_Arrives_At_Its_Transmit);
    RUN_TEST(test_Mock_Script_Reply_Order_Reversed);
    RUN_TEST(test_Mock_Script_Consumes_Exactly_What_It_Returns);
    RUN_TEST(test_Mock_Script_Uneven_Reads_Return_Every_Byte);
    RUN_TEST(test_Mock_Script_No_Reply_Past_End);
    RUN_TEST(test_Mock_Script_Lost_Reply_Ignores_Length);
    RUN_TEST(test_Mock_Script_Transmit_Injection_Hits_Chosen_Transmit);
    RUN_TEST(test_Mock_Script_Transmit_Injection_Returns_Chosen_Code);
    RUN_TEST(test_Mock_Script_Receive_Injection_Hits_Chosen_Call);
    RUN_TEST(test_Mock_Script_Receive_Injection_On_Empty_Queue);
    RUN_TEST(test_Mock_Script_Receive_Injection_Keeps_Queued_Bytes);
    RUN_TEST(test_Mock_Script_Failed_Transmit_Is_Logged);
    RUN_TEST(test_Mock_Script_Failed_Transmit_Discards_When_Set);
    RUN_TEST(test_Mock_Script_Failed_Transmit_Keeps_Bytes_Without_Discard);
    RUN_TEST(test_Mock_Script_Flush_Empties_Queue_And_Counts);
    RUN_TEST(test_Mock_Script_Repeated_Flushes_Keep_Counters_And_Schedule);
    RUN_TEST(test_Mock_Script_Timeout_Leaves_Queue);
    RUN_TEST(test_Mock_Script_Unread_Bytes_Persist_Without_Discard);
    RUN_TEST(test_Mock_Script_Discard_On_Transmit_Clears_Unread_Bytes);
    RUN_TEST(test_Mock_Script_Reset_Installs_Default_Fail);
    RUN_TEST(test_Mock_Script_Cap_Hook_Fires_After_Cap);
    RUN_TEST(test_Mock_Script_Cap_Below_Limit_Is_Honoured);
    RUN_TEST(test_Mock_Script_Default_Cap_Is_Limit);
    RUN_TEST(test_Mock_Script_Cap_Above_Limit_Is_Clamped);
    RUN_TEST(test_Mock_Script_Logs_Longest_Packet);
    RUN_TEST(test_Mock_Script_Overlong_Packet_Fails);
    RUN_TEST(test_Mock_Script_Queue_Fills_To_Capacity_Then_Fails);
    RUN_TEST(test_Mock_Script_Logs_Every_Packet_In_Order);
    RUN_TEST(test_Mock_Script_Index_Past_256_Keeps_Log_And_Steps);
    RUN_TEST(test_Mock_Script_Discard_Applies_Past_Script_End);
    RUN_TEST(test_Mock_Script_Flush_Keeps_Discard_Setting);
    RUN_TEST(test_Mock_Script_Receive_Injection_Past_255_Calls);
    RUN_TEST(test_Mock_Script_Receive_Injection_Does_Not_Refire_After_Wrap);
    RUN_TEST(test_Mock_Script_Flush_Count_Past_255);
    RUN_TEST(test_Mock_Script_Timeouts_And_Discards_Are_Not_Flushes);
    RUN_TEST(test_Mock_Script_Flush_Keeps_Cap_And_Hook);
    RUN_TEST(test_Mock_Script_Cap_Just_Above_Limit_Is_Clamped);
    RUN_TEST(test_Mock_Script_Transmit_Null_Arguments);
    RUN_TEST(test_Mock_Script_Receive_Null_Arguments);
    RUN_TEST(test_Mock_Script_Flush_Null_Arguments);

    printf("\n--- STS Bus Scan Tests ---\n");
    RUN_TEST(test_STS_Bus_Scan_Finds_Responding_Servo);
    RUN_TEST(test_STS_Bus_Scan_Hardware_Error_Counts_As_Found);
    RUN_TEST(test_STS_Bus_Scan_All_Silent_Returns_Timeout);
    RUN_TEST(test_STS_Bus_Scan_Skips_Transmit_Failure);
    RUN_TEST(test_STS_Bus_Scan_Skips_Busy_Transport);
    RUN_TEST(test_STS_Bus_Scan_Skips_Transport_Null_Pointer);
    RUN_TEST(test_STS_Bus_Scan_Repings_After_Receive_Failure);
    RUN_TEST(test_STS_Bus_Scan_Reping_Recovers_Noisy_Reply);
    RUN_TEST(test_STS_Bus_Scan_Repings_After_Header_Error);
    RUN_TEST(test_STS_Bus_Scan_Repings_After_Id_Mismatch);
    RUN_TEST(test_STS_Bus_Scan_Repings_After_Checksum_Error);
    RUN_TEST(test_STS_Bus_Scan_Repings_After_Malformed_Reply);
    RUN_TEST(test_STS_Bus_Scan_Repings_After_Oversize_Reply);
    RUN_TEST(test_STS_Bus_Scan_Stops_At_First_Responder);
    RUN_TEST(test_STS_Bus_Scan_Reping_Recovers_Corrupted_Reply);
    RUN_TEST(test_STS_Bus_Scan_Moves_On_After_Two_Corrupted_Replies);
    RUN_TEST(test_STS_Bus_Scan_Moves_On_When_Reping_Is_Lost);
    RUN_TEST(test_STS_Bus_Scan_Miss_Reports_Local_Fault);
    RUN_TEST(test_STS_Bus_Scan_Miss_Reports_Corrupted_First_Ping);
    RUN_TEST(test_STS_Bus_Scan_Miss_Reports_Last_Error);
    RUN_TEST(test_STS_Bus_Scan_Without_Callback);
    RUN_TEST(test_STS_Bus_Scan_Null_Bus);
    RUN_TEST(test_STS_Bus_Scan_Null_Found_Id);
    RUN_TEST(test_STS_Bus_Scan_Bus_Without_Transmit);
    RUN_TEST(test_STS_Bus_Scan_Bus_Without_Receive);
    RUN_TEST(test_STS_Bus_Scan_Reversed_Range);
    RUN_TEST(test_STS_Bus_Scan_Broadcast_Id_Rejected);
    RUN_TEST(test_STS_Bus_Scan_Last_Id_255_Rejected);
    RUN_TEST(test_STS_Bus_Scan_Single_Id_Range);
    RUN_TEST(test_STS_Bus_Scan_Includes_Id_Zero);
    RUN_TEST(test_STS_Bus_Scan_Reaches_Id_253);
    RUN_TEST(test_STS_Bus_Scan_Uses_Configured_Retries);
    RUN_TEST(test_STS_Max_Retries_255_Rejected_Before_Sending);
    RUN_TEST(test_STS_Max_Retries_254_Makes_255_Attempts);
    RUN_TEST(test_STS_Bus_Scan_Max_Retries_255_Finds_Nothing);

    return UNITY_END();
}
