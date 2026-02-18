#include "unity.h"
#include "sts_protocol.h"
#include <stdint.h>


void setUp(void) {
    
}

void tearDown(void) {
    
}

void test_Checksum_Ping(void) {
    // 6 bytes total: {FF, FF, ID, LEN, INST, CHK_SLOT}
    uint8_t packet[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0x00}; 
    uint8_t calculated = sts_calculate_checksum(packet, sizeof(packet));

    TEST_ASSERT_EQUAL_HEX8(0xFB, calculated);
}

void test_Checksum_Maximum_Sum(void) {
    // ID:254, Len:254, Instr:254, Params:[254, 254]
    // Sum = 254 * 5 = 1270 (0x4F6)
    // Truncate to 8-bit = 0xF6
    // ~0xF6 = 0x09
    uint8_t packet[] = {0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x00};
    TEST_ASSERT_EQUAL_HEX8(0x09, sts_calculate_checksum(packet, sizeof(packet)));
}

void test_Checksum_Min_Length_Alternative_ID(void) {
    // ID: 0x0A (10), Len: 2, Instr: 1 (Ping)
    // Sum = 10 + 2 + 1 = 13 (0x0D)
    // ~0x0D = 0xF2
    uint8_t packet[] = {0xFF, 0xFF, 0x0A, 0x02, 0x01, 0x00};
    TEST_ASSERT_EQUAL_HEX8(0xF2, sts_calculate_checksum(packet, sizeof(packet)));
}

void test_Checksum_Null_Pointer(void) {
    // Should return 0 and NOT crash (no HardFault)
    TEST_ASSERT_EQUAL_HEX8(0, sts_calculate_checksum(NULL, 10));
}

void test_Checksum_Too_Short(void) {
    // Any packet under 6 bytes is invalid protocol-wise
    uint8_t packet[] = {0xFF, 0xFF, 0x01};
    TEST_ASSERT_EQUAL_HEX8(0, sts_calculate_checksum(packet, sizeof(packet)));
}

void test_Checksum_Ignores_Headers_And_Final_Byte(void) {
    // Sum should only care about: ID(1) + Len(2) + Instr(1) = 4
    uint8_t packet[] = {0xAA, 0xAA, 0x01, 0x02, 0x01, 0x55};
    TEST_ASSERT_EQUAL_HEX8(0xFB, sts_calculate_checksum(packet, sizeof(packet)));
}