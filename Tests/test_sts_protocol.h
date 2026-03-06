/**
 ******************************************************************************
 * @file           : test_sts_protocol.h
 * @brief          : Prototypes for STS Protocol Verification Suite
 * @author         : Grisham Balloo
 * @date           : 2026-03-6
 * @version        : 1.1.0
 ******************************************************************************
 * @details
 * This header centralizes all test function prototypes for the STS Protocol 
 * verification layer. 
 *
 * Test Groups:
 * 1. Checksum Verification (7 tests): Algorithm accuracy and boundary safety.
 * 2. Packet Creation (11 tests): Packet encoding and buffer safety.
 * 3. Packet Parsing (15 tests): Rejection of malformed or invalid packets.
 * 4. Protocol Intergration (5 tests): Recovery from noise and false headers.
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
extern void test_ParseResponse_HardwareError_With_Noise_Prefix(void);
extern void test_ParseResponse_HardwareError_ParamLen_Zeroed(void);
extern void test_ParseResponse_IDMismatch(void);
extern void test_ParseResponse_ChecksumError(void);
extern void test_ParseResponse_MinimumLength(void);
extern void test_ParseResponse_LengthMismatch(void);
extern void test_ParseResponse_NullGuards(void);
extern void test_ParseResponse_OverloadError(void);
extern void test_ParseResponse_OutputBufferSafety(void);
extern void test_ParseResponse_HeaderError_FirstByte(void);
extern void test_ParseResponse_HeaderError_SecondByte(void);
extern void test_ParseResponse_IncompletePacket(void);
extern void test_ParseResponse_MaxPayload(void);
extern void test_ParseResponse_BoundaryIDs(void);
extern void test_ParseResponse_JunkData(void);

/* =========================================================================
   PROTOCOL INTEGRATION TESTS
   ========================================================================= */
extern void test_Protocol_Integration_Loopback(void);
extern void test_Protocol_Integration_MultiByteRead(void);
extern void test_Protocol_Integration_Robustness_Seeker(void);
extern void test_Protocol_Integration_FalseHeaderRecovery(void);
extern void test_Protocol_Integration_Filters_Wrong_ID(void);
/* =========================================================================
   UNITY FRAMEWORK FIXTURES
   ========================================================================= */
extern void setUp(void);
extern void tearDown(void);


