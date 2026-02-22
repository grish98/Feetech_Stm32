#include "unity.h"

extern void test_Checksum_Ping(void);
extern void test_Checksum_Maximum_Sum(void);
extern void test_Checksum_Min_Length_Alternative_ID(void);
extern void test_Checksum_Null_Pointer(void);
extern void test_Checksum_Too_Short(void);
extern void test_Checksum_Ignores_Headers_And_Final_Byte(void);

extern void test_CreatePacket_Ping_Success(void);
extern void test_CreatePacket_BufferTooSmall(void);
extern void test_CreatePacket_NullBuffer(void);
extern void test_CreatePacket_WritePosition_Success(void) ;
extern void test_CreatePacket_MaxPayload(void);
extern void test_CreatePacket_BroadcastID(void);
extern void test_CreatePacket_EmptyParamsIgnorePointer(void);
extern void test_CreatePacket_BufferExactlyOneByteTooSmall(void);
extern void test_CreatePacket_NullParamsWithNonZeroLength(void);
extern void test_CreatePacket_ParamLimitOverflow(void);
extern void test_CreatePacket_InvalidIDRange(void);
extern void test_Checksum_SumExactly256(void);

extern void test_ParseResponse_HardwareError(void);
extern void test_ParseResponse_IDMismatch(void);
extern void test_ParseResponse_ChecksumError(void);
extern void test_ParseResponse_ValidPosition(void);
extern void test_ParseResponse_MinimumLength(void);
extern void test_ParseResponse_LengthMismatch(void);
extern void test_ParseResponse_NullGuards(void);
extern void test_ParseResponse_OverloadError(void);

extern void test_Protocol_Integration_Loopback(void);
extern void test_Protocol_Integration_MultiByteRead(void);
extern void test_Protocol_Integration_Robustness_Seeker(void);
extern void test_Protocol_Integration_FalseHeaderRecovery(void);

extern void setUp(void);
extern void tearDown(void);

int main(void) {
    UNITY_BEGIN();

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

    printf("\n--- STS Protocol Integration Tests ---\n");
    RUN_TEST(test_Protocol_Integration_Loopback);
    RUN_TEST(test_Protocol_Integration_MultiByteRead);
    RUN_TEST(test_Protocol_Integration_Robustness_Seeker);
    RUN_TEST(test_Protocol_Integration_FalseHeaderRecovery);
    
    return UNITY_END();
}
