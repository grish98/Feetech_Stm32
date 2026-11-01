/**
 ******************************************************************************
 * @file           : test_sts_servo.h
 * @brief          : Unit test declarations for the STS Service Layer
 * @author         : Grisham Balloo
 ******************************************************************************
 * @details
 * Declares all unit test cases for the STS Service Layer, covering bus and
 * servo initialisation, read/write primitives, the command engine, and ping.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#pragma once
#include "sts_servo.h"

/* --- Bus Initialisation, Trasmit and Receive Tests --- */
void test_STS_Bus_Init_Success(void);
void test_init_fails_with_null_bus(void);
void test_STS_Bus_Init_Null_TX(void);
void test_STS_Bus_Init_Null_RX(void);
void test_STS_Bus_Null_TX(void);
void test_STS_Bus_NULL_RX(void);
void test_STS_Bus_TX_Null_Bus(void);
void test_STS_Bus_RX_Null_Bus(void);
void test_STS_Bus_Init_Null_Port(void);
void test_STS_Bus_Transmit_Null_Port(void);
void test_STS_Bus_Init_Overwrites_Invalid(void);
void test_STS_Bus_Reinitialisation(void);
void test_STS_Bus_Transmit_Routes_To_HAL(void);
void test_STS_Bus_Receive_Pulls_From_HAL(void);
void test_STS_Bus_Tx_Valid_Payload(void);
void test_STS_Bus_Rx_Valid_Payload(void);
void test_Bus_Tx_Fail(void);
void test_STS_Bus_Rx_Hal_Timeout(void);
void test_STS_Bus_Rx_Partial(void);
void test_STS_Bus_Transmit_Zero_Length_NoOp(void);
void test_STS_Bus_Receive_Zero_Length_NoOp(void);
void test_STS_Bus_Transmit_Null_Function_Pointer_Guarded(void);

/* --- Servo Initialisation Tests --- */
void test_STS_Servo_Init_Success(void);
void test_STS_Servo_Init_Null_Servo(void);
void test_STS_Servo_Init_Null_Bus(void);
void test_STS_Servo_Init_BroadcastSync_ID_Rejected(void);
void test_STS_Servo_Init_BroadcastAsync_ID_Rejected(void);
void test_STS_Servo_Init_MinID_Accepted(void);
void test_STS_Servo_Init_MaxID_Accepted(void);
void test_STS_Servo_Init_Clears_Handle_State(void);
void test_STS_Servo_Init_Atomic_Failure(void);
void test_STS_Servo_Init_Reset_Online_Status(void);
void test_STS_Servo_Init_Multi_Bus_Link(void);

/* --- Servo Read/Write Tests --- */
void test_STS_Write8_Success(void);
void test_STS_Read8_Success(void);
void test_STS_Write16_Success(void);
void test_STS_Read16_Success(void);

/*--- STS Command Engine Tests ---*/
void test_STS_Write8_Rejects_Null_Servo(void);
void test_STS_Write16_Rejects_Null_Servo(void);
void test_STS_Read8_Rejects_Null_Servo(void);
void test_STS_Read16_Rejects_Null_Servo(void);
void test_STS_Read8_Rejects_Null_Out_Pointer(void);
void test_STS_Read16_Rejects_Null_Out_Pointer(void);
void test_STS_Write8_Yields_Timeout(void);
void test_STS_Write16_Yields_Timeout(void);
void test_STS_Read8_Yields_Timeout(void);
void test_STS_Read16_Yields_Timeout(void);
void test_STS_Write16_Yields_ID_Mismatch(void);
void test_STS_Read16_Yields_Checksum_Error(void);
void test_STS_Write8_Yields_Bus_Busy(void);
void test_STS_Read16_Yields_TX_Fail(void);
void test_STS_Read8_Yields_Bus_Busy(void);
void test_STS_Write16_Yields_TX_Fail(void);
void test_STS_ExecuteCommand_Header_Corruption(void);
void test_STS_ExecuteCommand_Length_Mismatch(void);
void test_STS_ExecuteCommand_Truncated_Packet(void);
void test_STS_Primitives_Read_Broadcast_Forbidden(void);
void test_STS_ExecuteCommand_Null_Cmd_Guard(void);
void test_STS_ExecuteCommand_Null_Servo_Guard(void);
void test_STS_ExecuteCommand_Null_RxOut_Discards_Payload(void);
void test_STS_ExecuteCommand_RX_Buffer_Overflow(void);
void test_STS_ExecuteCommand_TX_Buffer_Overflow(void);
void test_STS_ExecuteCommand_Zero_Expected_RX(void);
void test_STS_ExecuteCommand_Broadcast_Forces_Early_Exit(void);
void test_STS_ExecuteCommand_Rejects_Uninitialised_Bus_Pointers(void);
void test_STS_ExecuteCommand_Standard_Write_Safety(void);

/* --- Servo Ping Tests --- */
void test_STS_Ping_Success(void);
void test_STS_Ping_Timeout_Sets_Offline(void);
void test_STS_Ping_ID_Mismatch_Sets_Offline(void);
void test_STS_Ping_Checksum_Error_Sets_Offline(void);
void test_STS_Ping_Bus_Busy_Sets_Offline(void);
void test_STS_Ping_Null_Servo_Guard(void);    
void test_STS_Ping_Broadcast_Forbidden(void);
void test_STS_Ping_Null_Bus_Pointer(void);
void test_STS_Ping_Hardware_Error_Still_Online(void);
void test_STS_Ping_Max_Valid_ID(void);
void test_STS_Ping_Recovery_Offline_To_Online(void);

/* --- Servo Torque Tests --- */
void test_STS_SetTorqueEnable_Success(void);
void test_STS_SetTorqueEnable_Disable(void);
void test_STS_SetTorqueEnable_NonStandard_True(void);
void test_STS_SetTorqueEnable_Null_Pointer(void);
void test_STS_SetTorqueEnable_Error_Propagation(void);

/* --- Servo Position Tests --- */
void test_STS_SetTargetPosition_Success(void);
void test_STS_GetPresentPosition_Success(void);
void test_STS_Position_API_Null_Guards(void);
void test_STS_GetPresentPosition_Endianness(void);
void test_STS_SetTargetPosition_Hardware_Fault(void);
void test_STS_SetTargetPosition_Out_Of_Range(void);
void test_STS_SetTargetPosition_Just_Out_Of_Range(void);
void test_STS_SetTargetPosition_Broadcast_No_Wait(void);
void test_STS_GetPresentPosition_Fragmented_Packet(void);
void test_STS_GetPresentPosition_Bad_Checksum(void);
void test_STS_GetPresentPosition_Zero_Length_Fault(void);
void test_STS_GetPresentPosition_Buffer_Overflow_Guard(void);
void test_STS_GetPresentPosition_Wrong_ID_Response(void);
void test_STS_SetTargetPosition_Min_Boundary(void);
void test_STS_SetTargetPosition_Max_Boundary(void);
void test_STS_SetTargetPosition_Bus_Busy(void);
void test_STS_GetPresentPosition_Broadcast_Forbidden(void);
void test_STS_GetPresentPosition_Payload_Length_Mismatch(void);
void test_STS_GetPresentPosition_Stage2_Timeout(void);

/* --- STS Speed & Acceleration Tests --- */
void test_STS_Speed_Accel_Null_Guards(void);
void test_STS_SetTargetSpeed_Success(void);
void test_STS_SetTargetSpeed_Out_Of_Range(void);
void test_STS_GetPresentSpeed_Success(void);
void test_STS_SetTargetAcceleration_Success(void);
void test_STS_SetTargetAcceleration_Out_Of_Range(void);
void test_STS_SetTargetAcceleration_Max_Boundary(void);
void test_STS_SetTargetSpeed_Max_Boundary(void);
void test_STS_SetTargetSpeed_Min_Boundary(void);
void test_STS_SetTargetAcceleration_Min_Boundary(void);
void test_STS_SetTargetSpeed_Reverse_Direction_Allowed(void);
void test_STS_SetTargetSpeed_Max_Reverse_Allowed(void);

/* --- STS Operating Mode Tests --- */
void test_STS_SetOperatingMode_Null_Guard(void);
void test_STS_SetOperatingMode_Success(void);
void test_STS_SetOperatingMode_Invalid_Mode(void);

/* --- STS PWM & Step Control Tests --- */
void test_STS_PWM_Step_Null_Guards(void);
void test_STS_SetTargetPWM_Out_Of_Range(void);
void test_STS_SetTargetStep_Out_Of_Range(void);
void test_STS_SetTargetPWM_Success(void);
void test_STS_SetTargetStep_Success(void);
void test_STS_SetTargetPWM_Max_Boundary(void);
void test_STS_SetTargetStep_Max_Boundary(void);
void test_STS_SetTarget_Zero_Boundary(void);
void test_STS_SetTargetPWM_Zero_Boundary(void);
void test_STS_SetTargetStep_Zero_Boundary(void);
void test_STS_SetTarget_SpeedMode_Min_Boundary(void);
void test_STS_SetTarget_PWMMode_Min_Boundary(void);
void test_STS_SetTarget_StepMode_Min_Boundary(void);
void test_STS_SetTarget_SpeedMode_Negative_Out_Of_Range(void);
void test_STS_SetTarget_PWMMode_Negative_Out_Of_Range(void);
void test_STS_SetTarget_StepMode_Positive_Out_Of_Range(void);

/* --- STS Universal Target Routing Tests --- */
void test_STS_SetTarget_PositionMode_Normal(void);
void test_STS_SetTarget_PositionMode_Negative_Rejected(void);
void test_STS_SetTarget_SpeedMode_Positive_CCW(void);
void test_STS_SetTarget_SpeedMode_Negative_CW(void);
void test_STS_SetTarget_SpeedMode_Out_Of_Range(void);
void test_STS_SetTarget_PWMMode_Negative_CW(void);
void test_STS_SetTarget_StepMode_Positive_CCW(void);
void test_STS_SetTarget_InvalidMode(void);
void test_STS_SetTarget_PWMMode_Positive_CCW(void);
void test_STS_SetTarget_StepMode_Negative_CW(void);
void test_STS_SetTarget_PWMMode_Out_Of_Range(void);
void test_STS_SetTarget_StepMode_Out_Of_Range(void);

/* --- STS Universal Target Range & Overflow Tests --- */
void test_STS_SetTarget_PositionMode_Zero(void);
void test_STS_SetTarget_PositionMode_Max_Boundary(void);
void test_STS_SetTarget_PositionMode_Above_Max_Rejected(void);
void test_STS_SetTarget_PositionMode_Uint16_Max_Rejected(void);
void test_STS_SetTarget_PositionMode_No_Wrap_To_Zero(void);
void test_STS_SetTarget_PositionMode_No_Wrap_To_Valid(void);
void test_STS_SetTarget_PositionMode_No_Wrap_To_Max(void);
void test_STS_SetTarget_PositionMode_Int32_Max_Rejected(void);
void test_STS_SetTarget_PositionMode_Minus_One_Rejected(void);
void test_STS_SetTarget_PositionMode_Int32_Min_Rejected(void);
void test_STS_SetTarget_PositionMode_Int32_Min_Plus_One_Rejected(void);
void test_STS_SetTarget_SpeedMode_Max_Boundary(void);
void test_STS_SetTarget_SpeedMode_Above_Max_Rejected(void);
void test_STS_SetTarget_SpeedMode_Below_Min_Rejected(void);
void test_STS_SetTarget_SpeedMode_No_Wrap_To_Zero(void);
void test_STS_SetTarget_SpeedMode_Negative_No_Wrap(void);
void test_STS_SetTarget_SpeedMode_Int32_Max_Rejected(void);
void test_STS_SetTarget_SpeedMode_Int32_Min_Plus_One_Rejected(void);
void test_STS_SetTarget_SpeedMode_Int32_Min_Rejected(void);
void test_STS_SetTarget_PWMMode_Max_Boundary(void);
void test_STS_SetTarget_PWMMode_Above_Max_Rejected(void);
void test_STS_SetTarget_PWMMode_Speed_Range_Rejected(void);
void test_STS_SetTarget_PWMMode_No_Wrap_To_Valid(void);
void test_STS_SetTarget_PWMMode_Int32_Min_Rejected(void);
void test_STS_SetTarget_PWMMode_Zero(void);
void test_STS_SetTarget_PWMMode_Int32_Max_Rejected(void);
void test_STS_SetTarget_PWMMode_Int32_Min_Plus_One_Rejected(void);
void test_STS_SetTarget_StepMode_Zero(void);
void test_STS_SetTarget_StepMode_Max_Boundary(void);
void test_STS_SetTarget_StepMode_Above_Max_Rejected(void);
void test_STS_SetTarget_StepMode_Below_Min_Rejected(void);
void test_STS_SetTarget_StepMode_No_Wrap_To_Valid(void);
void test_STS_SetTarget_StepMode_Int32_Min_Rejected(void);
void test_STS_SetTarget_StepMode_Int32_Max_Rejected(void);
void test_STS_SetTarget_StepMode_Int32_Min_Plus_One_Rejected(void);
void test_STS_SetTarget_InvalidMode_Int32_Min(void);
void test_STS_SetTarget_Null_Servo_Int32_Min(void);
void test_STS_SetTarget_Propagates_Timeout(void);
void test_STS_SetTarget_Propagates_Hardware_Error(void);
void test_STS_SetTarget_SpeedMode_Propagates_Timeout(void);
void test_STS_SetTarget_PWMMode_Propagates_Timeout(void);
void test_STS_SetTarget_StepMode_Propagates_Timeout(void);
void test_STS_SetTarget_Null_Bus(void);
void test_STS_SetTarget_Null_Bus_Range_Checked_First(void);

/* --- STS Torque Limit Tests --- */
void test_STS_SetTorqueLimit_Null_Guard(void);
void test_STS_SetTorqueLimit_Out_Of_Range(void);
void test_STS_SetTorqueLimit_Success(void);
void test_STS_SetTorqueLimit_Max_Boundary(void);
void test_STS_SetTorqueLimit_Zero_Boundary(void);

/* --- STS Telemetry Tests --- */
void test_STS_Telemetry_Null_Guards(void);
void test_STS_GetPresentLoad_Success(void);
void test_STS_GetPresentVoltage_Success(void);
void test_STS_GetPresentTemperature_Success(void);
void test_STS_GetMovingStatus_Success(void);
void test_STS_Telemetry_Bubbles_Hardware_Error(void); 
void test_STS_Telemetry_Preserves_State_On_Timeout(void);

/* --- STS EEPROM & ID Config Tests --- */
void test_STS_SetEEPROMLock_Null_Guard(void);
void test_STS_SetEEPROMLock_States(void);
void test_STS_SetID_Null_Guard(void);
void test_STS_SetID_Out_Of_Range(void);
void test_STS_SetID_Success(void);

/* --- Scripted Transport Self-Tests --- */
void test_Mock_Script_Reply_Arrives_At_Its_Transmit(void);
void test_Mock_Script_Reply_Order_Reversed(void);
void test_Mock_Script_Consumes_Exactly_What_It_Returns(void);
void test_Mock_Script_Uneven_Reads_Return_Every_Byte(void);
void test_Mock_Script_No_Reply_Past_End(void);
void test_Mock_Script_Lost_Reply_Ignores_Length(void);
void test_Mock_Script_Transmit_Injection_Hits_Chosen_Transmit(void);
void test_Mock_Script_Transmit_Injection_Returns_Chosen_Code(void);
void test_Mock_Script_Receive_Injection_Hits_Chosen_Call(void);
void test_Mock_Script_Receive_Injection_On_Empty_Queue(void);
void test_Mock_Script_Receive_Injection_Keeps_Queued_Bytes(void);
void test_Mock_Script_Failed_Transmit_Is_Logged(void);
void test_Mock_Script_Failed_Transmit_Discards_When_Set(void);
void test_Mock_Script_Failed_Transmit_Keeps_Bytes_Without_Discard(void);
void test_Mock_Script_Flush_Empties_Queue_And_Counts(void);
void test_Mock_Script_Repeated_Flushes_Keep_Counters_And_Schedule(void);
void test_Mock_Script_Timeout_Leaves_Queue(void);
void test_Mock_Script_Unread_Bytes_Persist_Without_Discard(void);
void test_Mock_Script_Discard_On_Transmit_Clears_Unread_Bytes(void);
void test_Mock_Script_Reset_Installs_Default_Fail(void);
void test_Mock_Script_Cap_Hook_Fires_After_Cap(void);
void test_Mock_Script_Cap_Below_Limit_Is_Honoured(void);
void test_Mock_Script_Default_Cap_Is_Limit(void);
void test_Mock_Script_Cap_Above_Limit_Is_Clamped(void);
void test_Mock_Script_Logs_Longest_Packet(void);
void test_Mock_Script_Overlong_Packet_Fails(void);
void test_Mock_Script_Queue_Fills_To_Capacity_Then_Fails(void);
void test_Mock_Script_Logs_Every_Packet_In_Order(void);
void test_Mock_Script_Index_Past_256_Keeps_Log_And_Steps(void);
void test_Mock_Script_Discard_Applies_Past_Script_End(void);
void test_Mock_Script_Flush_Keeps_Discard_Setting(void);
void test_Mock_Script_Receive_Injection_Past_255_Calls(void);
void test_Mock_Script_Receive_Injection_Does_Not_Refire_After_Wrap(void);
void test_Mock_Script_Flush_Count_Past_255(void);
void test_Mock_Script_Timeouts_And_Discards_Are_Not_Flushes(void);
void test_Mock_Script_Flush_Keeps_Cap_And_Hook(void);
void test_Mock_Script_Cap_Just_Above_Limit_Is_Clamped(void);
void test_Mock_Script_Transmit_Null_Arguments(void);
void test_Mock_Script_Receive_Null_Arguments(void);
void test_Mock_Script_Flush_Null_Arguments(void);

/* --- STS Bus Scan Tests --- */
void test_STS_Bus_Scan_Finds_Responding_Servo(void);
void test_STS_Bus_Scan_Hardware_Error_Counts_As_Found(void);
void test_STS_Bus_Scan_All_Silent_Returns_Timeout(void);
void test_STS_Bus_Scan_Skips_Transmit_Failure(void);
void test_STS_Bus_Scan_Skips_Busy_Transport(void);
void test_STS_Bus_Scan_Skips_Transport_Null_Pointer(void);
void test_STS_Bus_Scan_Repings_After_Receive_Failure(void);
void test_STS_Bus_Scan_Reping_Recovers_Noisy_Reply(void);
void test_STS_Bus_Scan_Repings_After_Header_Error(void);
void test_STS_Bus_Scan_Repings_After_Id_Mismatch(void);
void test_STS_Bus_Scan_Repings_After_Checksum_Error(void);
void test_STS_Bus_Scan_Repings_After_Malformed_Reply(void);
void test_STS_Bus_Scan_Repings_After_Oversize_Reply(void);
void test_STS_Bus_Scan_Stops_At_First_Responder(void);
void test_STS_Bus_Scan_Reping_Recovers_Corrupted_Reply(void);
void test_STS_Bus_Scan_Moves_On_After_Two_Corrupted_Replies(void);
void test_STS_Bus_Scan_Moves_On_When_Reping_Is_Lost(void);
void test_STS_Bus_Scan_Miss_Reports_Local_Fault(void);
void test_STS_Bus_Scan_Miss_Reports_Corrupted_First_Ping(void);
void test_STS_Bus_Scan_Miss_Reports_Last_Error(void);
void test_STS_Bus_Scan_Without_Callback(void);
void test_STS_Bus_Scan_Null_Bus(void);
void test_STS_Bus_Scan_Null_Found_Id(void);
void test_STS_Bus_Scan_Bus_Without_Transmit(void);
void test_STS_Bus_Scan_Bus_Without_Receive(void);
void test_STS_Bus_Scan_Reversed_Range(void);
void test_STS_Bus_Scan_Broadcast_Id_Rejected(void);
void test_STS_Bus_Scan_Last_Id_255_Rejected(void);
void test_STS_Bus_Scan_Single_Id_Range(void);
void test_STS_Bus_Scan_Includes_Id_Zero(void);
void test_STS_Bus_Scan_Reaches_Id_253(void);
void test_STS_Bus_Scan_Uses_Configured_Retries(void);
void test_STS_Max_Retries_255_Rejected_Before_Sending(void);
void test_STS_Max_Retries_254_Makes_255_Attempts(void);
void test_STS_Bus_Scan_Max_Retries_255_Finds_Nothing(void);

