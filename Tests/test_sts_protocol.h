/**
 ******************************************************************************
 * @file           : test_sts_protocol.h
 * @brief          : Prototypes for STS Protocol Verification Suite
 * @author         : Grisham Balloo
 * @date           : 2026-02-22
 * @version        : 1.0.0
 ******************************************************************************
 * @details
 * This header centralizes all test function prototypes for the STS Protocol 
 * verification layer. 
 *
 * Test Groups:
 * 1. Checksum Verification (7 tests): Algorithm accuracy and boundary safety.
 * 2. Serialization Verification (11 tests): Packet encoding and buffer safety.
 * 3. Protocol Validation (8 tests): Rejection of malformed or invalid packets.
 * 4. Frame Synchronization (4 tests): Recovery from noise and false headers.
 *
 * @note All functions conform to the Unity Test Framework signature: void func(void).
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#pragma once

/* =========================================================================
   CHECKSUM TESTS
   ========================================================================= */
extern void test_Checksum_Ping(void);
extern void test_Checksum_Maximum_Sum(void);
extern void test_Checksum_Min_Length_Alternative_ID(void);
extern void test_Checksum_Null_Pointer(void);
extern void test_Checksum_Too_Short(void);
extern void test_Checksum_Ignores_Headers_And_Final_Byte(void);
extern void test_Checksum_SumExactly256(void);

/* =========================================================================
   PACKET CREATION TESTS
   ========================================================================= */
extern void test_CreatePacket_Ping_Success(void);
extern void test_CreatePacket_BufferTooSmall(void);
extern void test_CreatePacket_NullBuffer(void);
extern void test_CreatePacket_WritePosition_Success(void);
extern void test_CreatePacket_MaxPayload(void);
extern void test_CreatePacket_BroadcastID(void);
extern void test_CreatePacket_EmptyParamsIgnorePointer(void);
extern void test_CreatePacket_BufferExactlyOneByteTooSmall(void);
extern void test_CreatePacket_NullParamsWithNonZeroLength(void);
extern void test_CreatePacket_ParamLimitOverflow(void);
extern void test_CreatePacket_InvalidIDRange(void);

/* =========================================================================
   RESPONSE PARSING TESTS
   ========================================================================= */
extern void test_ParseResponse_ValidPosition(void);
extern void test_ParseResponse_HardwareError(void);
extern void test_ParseResponse_IDMismatch(void);
extern void test_ParseResponse_ChecksumError(void);
extern void test_ParseResponse_MinimumLength(void);
extern void test_ParseResponse_LengthMismatch(void);
extern void test_ParseResponse_NullGuards(void);
extern void test_ParseResponse_OverloadError(void);

/* =========================================================================
   PROTOCOL INTEGRATION TESTS
   ========================================================================= */
extern void test_Protocol_Integration_Loopback(void);
extern void test_Protocol_Integration_MultiByteRead(void);
extern void test_Protocol_Integration_Robustness_Seeker(void);
extern void test_Protocol_Integration_FalseHeaderRecovery(void);

/* =========================================================================
   UNITY FRAMEWORK FIXTURES
   ========================================================================= */
extern void setUp(void);
extern void tearDown(void);


