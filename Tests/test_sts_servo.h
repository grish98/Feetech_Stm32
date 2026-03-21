/**
 ******************************************************************************
 * @file           : test_sts_servo.h
 * @brief          : Unit test declarations for the STS Service Layer
 * @author         : Grisham Balloo
 * @date           : 2026-03-19
 * @version        : 0.2.0
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

/* --- Bus Initialization Tests --- */
extern void test_STS_Bus_Init_Success(void);
extern void test_STS_Bus_Init_Null_Bus(void);
extern void test_STS_Bus_Init_Null_TX(void);
extern void test_STS_Bus_Init_Null_RX(void);
extern void test_STS_Bus_Init_Null_Port_Handle_Succeeds(void);
extern void test_STS_Bus_Init_Overwrites_Garbage(void);
extern void test_STS_Bus_Init_Reinitialization(void);
extern void test_STS_Bus_Interface_Execution(void);

/* --- Servo Initialization Tests --- */
extern void test_STS_Servo_Init_Success(void);
extern void test_STS_Servo_Init_Null_Servo(void);
extern void test_STS_Servo_Init_Null_Bus(void);
extern void test_STS_Servo_Init_Invalid_ID(void);
extern void test_STS_Servo_Init_ID_Boundaries(void);
extern void test_STS_Servo_Init_Clears_Handle_State(void);
extern void test_STS_Servo_Init_Atomic_Failure(void);
extern void test_STS_Servo_Init_Reset_Online_Status(void);
extern void test_STS_Servo_Init_Multi_Bus_Link(void);

/* --- Servo Read/Write Tests --- */
extern void test_STS_Write8_Success(void);
extern void test_STS_Read8_Success(void);
extern void test_STS_Write16_Success(void);
extern void test_STS_Read16_Success(void);

/*--- STS Command  Engine Tests ---*/
extern void test_STS_Primitives_All_Null_Guards(void);
extern void test_STS_Primitives_All_Timeout(void);
extern void test_STS_Primitives_All_Data_Integrity(void);
extern void test_STS_Primitives_All_Hardware_Errors(void);
extern void test_STS_ExecuteCommand_Header_Corruption(void);
extern void test_STS_ExecuteCommand_Length_Mismatch(void);
extern void test_STS_ExecuteCommand_Buffer_Overflow(void);
extern void test_STS_ExecuteCommand_Truncated_Packet(void);
extern void test_STS_Primitives_Read_Broadcast_Forbidden(void);
extern void test_STS_ExecuteCommand_TrashBin_Redirect(void);
extern void test_STS_ExecuteCommand_RX_Buffer_Overflow(void);
extern void test_STS_ExecuteCommand_TX_Buffer_Overflow(void);
extern void test_STS_ExecuteCommand_Zero_Expected_RX(void);
extern void test_STS_ExecuteCommand_Broadcast_Forces_Early_Exit(void);
extern void test_STS_ExecuteCommand_Null_Cmd_Guard(void);

/* --- Servo Ping Tests --- */
extern void test_STS_Ping_Success(void);
extern void test_STS_Ping_Timeout_Sets_Offline(void);
extern void test_STS_Ping_ID_Mismatch_Sets_Offline(void);
extern void test_STS_Ping_Checksum_Error_Sets_Offline(void);
extern void test_STS_Ping_Bus_Busy_Sets_Offline(void);
extern void test_STS_Ping_Null_Servo_Guard(void);    
extern void test_STS_Ping_Broadcast_Forbidden(void);
extern void test_STS_Ping_Null_Bus_Pointer(void);
extern void test_STS_Ping_Hardware_Error_Still_Online(void);
extern void test_STS_Ping_Max_Valid_ID(void);
extern void test_STS_Ping_Recovery_Offline_To_Online(void);

/* --- Servo Torque Tests --- */
extern void test_STS_SetTorqueEnable_Success(void);
extern void test_STS_SetTorqueEnable_Disable(void);
extern void test_STS_SetTorqueEnable_NonStandard_True(void);
extern void test_STS_SetTorqueEnable_Null_Pointer(void);
extern void test_STS_SetTorqueEnable_Error_Propagation(void);

/* --- Servo Position Tests --- */
extern void test_STS_SetTargetPosition_Success(void);
extern void test_STS_GetPresentPosition_Success(void);
extern void test_STS_Position_API_Null_Guards(void);
extern void test_STS_GetPresentPosition_Endianness(void);
extern void test_STS_SetTargetPosition_Hardware_Fault(void);
extern void test_STS_SetTargetPosition_Out_Of_Range(void);
extern void test_STS_SetTargetPosition_Just_Out_Of_Range(void);
extern void test_STS_SetTargetPosition_Broadcast_No_Wait(void);
extern void test_STS_GetPresentPosition_Fragmented_Packet(void);
extern void test_STS_GetPresentPosition_Bad_Checksum(void);
extern void test_STS_GetPresentPosition_Zero_Length_Fault(void);
extern void test_STS_GetPresentPosition_Buffer_Overflow_Guard(void);
extern void test_STS_GetPresentPosition_Wrong_ID_Response(void);
extern void test_STS_SetTargetPosition_Min_Boundary(void);
extern void test_STS_SetTargetPosition_Max_Boundary(void);
extern void test_STS_SetTargetPosition_Bus_Busy(void);
extern void test_STS_GetPresentPosition_Broadcast_Forbidden(void);
extern void test_STS_GetPresentPosition_Payload_Length_Mismatch(void);
extern void test_STS_GetPresentPosition_Stage2_Timeout(void);

extern void test_STS_Speed_Accel_Null_Guards(void);
extern void test_STS_SetTargetSpeed_Success(void);
extern void test_STS_SetTargetSpeed_Out_Of_Range(void);
extern void test_STS_GetPresentSpeed_Success(void);
extern void test_STS_SetTargetAcceleration_Success(void);
extern void test_STS_SetTargetAcceleration_Out_Of_Range(void);
extern void test_STS_SetTargetAcceleration_Max_Boundary(void);
extern void test_STS_SetTargetSpeed_Max_Boundary(void);
extern void test_STS_SetTargetSpeed_Min_Boundary(void);
extern void test_STS_SetTargetAcceleration_Min_Boundary(void);
extern void test_STS_SetTargetSpeed_Reverse_Direction_Allowed(void);
extern void test_STS_SetTargetSpeed_Max_Reverse_Allowed(void);

extern void test_STS_SetOperatingMode_Null_Guard(void);
extern void test_STS_SetOperatingMode_Success(void);
extern void test_STS_SetOperatingMode_Invalid_Mode(void);

extern void test_STS_PWM_Step_Null_Guards(void);
extern void test_STS_SetTargetPWM_Out_Of_Range(void);
extern void test_STS_SetTargetStep_Out_Of_Range(void);
extern void test_STS_SetTargetPWM_Success(void);
extern void test_STS_SetTargetStep_Success(void);
extern void test_STS_SetTargetPWM_Max_Boundary(void);
extern void test_STS_SetTargetStep_Max_Boundary(void);
extern void test_STS_SetTarget_Zero_Boundary(void);
extern void test_STS_SetTargetPWM_Zero_Boundary(void);
extern void test_STS_SetTargetStep_Zero_Boundary(void);
extern void test_STS_SetTarget_SpeedMode_Min_Boundary(void);
extern void test_STS_SetTarget_PWMMode_Min_Boundary(void);
extern void test_STS_SetTarget_StepMode_Min_Boundary(void);
extern void test_STS_SetTarget_SpeedMode_Negative_Out_Of_Range(void);
extern void test_STS_SetTarget_PWMMode_Negative_Out_Of_Range(void);
extern void test_STS_SetTarget_StepMode_Positive_Out_Of_Range(void);

extern void test_STS_SetTarget_PositionMode_Normal(void);
extern void test_STS_SetTarget_PositionMode_ClampsNegativeToZero(void);
extern void test_STS_SetTarget_SpeedMode_Positive_CCW(void);
extern void test_STS_SetTarget_SpeedMode_Negative_CW(void);
extern void test_STS_SetTarget_SpeedMode_Out_Of_Range(void);
extern void test_STS_SetTarget_PWMMode_Negative_CW(void);
extern void test_STS_SetTarget_StepMode_Positive_CCW(void);
extern void test_STS_SetTarget_InvalidMode(void);
extern void test_STS_SetTarget_PWMMode_Positive_CCW(void);
extern void test_STS_SetTarget_StepMode_Negative_CW(void);
extern void test_STS_SetTarget_PWMMode_Out_Of_Range(void);
extern void test_STS_SetTarget_StepMode_Out_Of_Range(void);

extern void test_STS_SetTorqueLimit_Null_Guard(void);
extern void test_STS_SetTorqueLimit_Out_Of_Range(void);
extern void test_STS_SetTorqueLimit_Success(void);
extern void test_STS_SetTorqueLimit_Max_Boundary(void);
extern void test_STS_SetTorqueLimit_Zero_Boundary(void);
