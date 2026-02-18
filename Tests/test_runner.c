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

extern void setUp(void);
extern void tearDown(void);

int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_Checksum_Ping);
    RUN_TEST(test_Checksum_Maximum_Sum);
    RUN_TEST(test_Checksum_Min_Length_Alternative_ID);
    RUN_TEST(test_Checksum_Null_Pointer);
    RUN_TEST(test_Checksum_Too_Short);
    RUN_TEST(test_Checksum_Ignores_Headers_And_Final_Byte);
    RUN_TEST(test_Checksum_SumExactly256);
    
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
    
    return UNITY_END();
}
