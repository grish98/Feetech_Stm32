/**
 ******************************************************************************
 * @file           : test_sts_servo.h
 * @brief          : Unit test declarations for the STS Service Layer
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 0.1.0
 ******************************************************************************
 * @details
 * This header declares the test cases for the STS Bus Hardware Abstraction Layer.
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