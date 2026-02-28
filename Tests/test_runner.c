/**
 ******************************************************************************
 * @file           : test_runner.c
 * @brief          : STS Protocol Layer Unit Test Runner
 * @author         : Grisham Balloo
 * @date           : 2026-02-28
 * @version        : 1.1.0
 ******************************************************************************
 * * @details 
 * This file serves as the main entry point for the STS Protocol Layer unit 
 * tests. It utilizes the Unity Test Framework to orchestrate the execution 
 * of test cases defined in 'test_sts_protocol.c'.
 * * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "unity.h"
#include "test_sts_protocol.h"
#include <stdio.h>

int main(void) {
    UNITY_BEGIN();

    printf("==========` STS Protocol Layer Unit Tests ==========\n");

    printf("\n--- STS Checksum Tests ---\n");
    RUN_TEST(test_Checksum_Ping);
    RUN_TEST(test_Checksum_Maximum_Sum);
    RUN_TEST(test_Checksum_Min_Length_Alternative_ID);
    RUN_TEST(test_Checksum_Null_Pointer);
    RUN_TEST(test_Checksum_Too_Short);
    RUN_TEST(test_Checksum_Ignores_Headers_And_Final_Byte);
    RUN_TEST(test_Checksum_SumExactly256);

    printf("\n--- STS Creation Tests ---\n");
    RUN_TEST(test_CreatePacket_Ping_Success);
    RUN_TEST(test_CreatePacket_BufferTooSmall);
    RUN_TEST(test_CreatePacket_NullBuffer);
    RUN_TEST(test_CreatePacket_WritePosition_Success);
    RUN_TEST(test_CreatePacket_MaxPayload);
    RUN_TEST(test_CreatePacket_BroadcastID);
    RUN_TEST(test_CreatePacket_EmptyParamsIgnorePointer);
    RUN_TEST(test_CreatePacket_BufferExactlyOneByteTooSmall);
    RUN_TEST(test_CreatePacket_NullParamsWithNonZeroLength);
    RUN_TEST(test_CreatePacket_ParamLimitOverflow);
    RUN_TEST(test_CreatePacket_InvalidIDRange);

    printf("\n--- STS Parser Tests ---\n");
    RUN_TEST(test_ParseResponse_ValidPosition);
    RUN_TEST(test_ParseResponse_HardwareError);
    RUN_TEST(test_ParseResponse_IDMismatch);
    RUN_TEST(test_ParseResponse_ChecksumError);
    RUN_TEST(test_ParseResponse_MinimumLength);
    RUN_TEST(test_ParseResponse_LengthMismatch);
    RUN_TEST(test_ParseResponse_NullGuards);
    RUN_TEST(test_ParseResponse_OverloadError);
    RUN_TEST(test_ParseResponse_OutputBufferSafety);
    RUN_TEST(test_ParseResponse_HeaderError_FirstByte);
    RUN_TEST(test_ParseResponse_HeaderError_SecondByte);
    RUN_TEST(test_ParseResponse_IncompletePacket);
    RUN_TEST(test_ParseResponse_MaxPayload);
    RUN_TEST(test_ParseResponse_BoundaryIDs);
    RUN_TEST(test_ParseResponse_JunkData);

    printf("\n--- STS Protocol Integration Tests ---\n");
    RUN_TEST(test_Protocol_Integration_Loopback);
    RUN_TEST(test_Protocol_Integration_MultiByteRead);
    RUN_TEST(test_Protocol_Integration_Robustness_Seeker);
    RUN_TEST(test_Protocol_Integration_FalseHeaderRecovery);
    RUN_TEST(test_Protocol_Integration_Filters_Wrong_ID);
    
    return UNITY_END();
}
