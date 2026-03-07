/**
 ******************************************************************************
 * @file           : test_sts_servo.h
 * @brief          : Unit test declarations for the STS Service Layer
 * @author         : Grisham Balloo
 * @date           : 2026-03-05
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

extern void test_STS_SetTorqueEnable_Success(void);
extern void test_STS_SetTorqueEnable_Disable(void);
extern void test_STS_SetTorqueEnable_NonStandard_True(void);
extern void test_STS_SetTorqueEnable_Null_Pointer(void);
extern void test_STS_SetTorqueEnable_Error_Propagation(void);