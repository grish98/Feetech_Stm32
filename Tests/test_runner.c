#include "unity.h"

extern void test_Checksum_Ping(void);
extern void test_Checksum_Maximum_Sum(void);
extern void test_Checksum_Min_Length_Alternative_ID(void);
extern void test_Checksum_Null_Pointer(void);
extern void test_Checksum_Too_Short(void);
extern void test_Checksum_Ignores_Headers_And_Final_Byte(void);


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
 
    
    return UNITY_END();
}
