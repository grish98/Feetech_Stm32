#include "unity.h"
#include "sts_protocol.h"
#include <stdint.h>
#include<string.h>


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

void test_Checksum_SumExactly256(void) {
    // ID: 0x80 (128), Len: 0x7E (126), Inst: 0x02
    // Sum = 128 + 126 + 2 = 256 (0x100)
    // Truncate to 8-bit = 0x00
    // ~0x00 = 0xFF
    uint8_t packet[] = {0xFF, 0xFF, 0x80, 0x7E, 0x02, 0x00};
    TEST_ASSERT_EQUAL_HEX8(0xFF, sts_calculate_checksum(packet, sizeof(packet)));
}




void test_CreatePacket_Ping_Success(void) {
    uint8_t buffer[STS_MIN_PACKET_SIZE]; 
    uint8_t id = 0x01;
    uint8_t instruction = 0x01; // ping command

    sts_result_t result = sts_create_packet(id, instruction, NULL, 0, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    // Packet should be: FF FF 01 02 01 FB
    TEST_ASSERT_EQUAL_HEX8(0xFF, buffer[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buffer[1]);
    TEST_ASSERT_EQUAL_HEX8(id,   buffer[2]);
    TEST_ASSERT_EQUAL_HEX8(0x02, buffer[3]); 
    TEST_ASSERT_EQUAL_HEX8(instruction, buffer[4]);
    TEST_ASSERT_EQUAL_HEX8(0xFB, buffer[5]); 
}

void test_CreatePacket_BufferTooSmall(void) {
    uint8_t buffer[5]; 
    sts_result_t result = sts_create_packet(0x01, 0x01, NULL, 0, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN, result);
}

void test_CreatePacket_NullBuffer(void) {
    sts_result_t result = sts_create_packet(0x01, 0x01, NULL, 0, NULL, 10);
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, result);
}

void test_CreatePacket_WritePosition_Success(void) {
    uint8_t buffer[15];
    //  ID 1, Write (0x03), Params: {0x1E, 0x00, 0x01} 
    uint8_t params[] = {0x1E, 0x00, 0x01}; 
    uint8_t p_len = sizeof(params);
    
    sts_result_t result = sts_create_packet(0x01, 0x03, params, p_len, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(0x05, buffer[3]); // Length should be 3 (params) + 2 = 5
    TEST_ASSERT_EQUAL_HEX8(0x1E, buffer[5]); // First parameter 
    TEST_ASSERT_EQUAL_HEX8(0x01, buffer[7]); // Last parameter 

    // Checksum for: ID(01) + Len(05) + Instr(03) + P1(1E) + P2(00) + P3(01) = 0x27
    // ~0x27 = 0xD7
    TEST_ASSERT_EQUAL_HEX8(0xD7, buffer[5 + p_len]); 
}

void test_CreatePacket_MaxPayload(void) {
uint8_t buffer[260]; 
    uint8_t params[253]; 
    memset(params, 0x00, 253); 
    // total_packet_size = 6 + 253 = 259 bytes
    sts_result_t result = sts_create_packet(0x01, 0x03, params, 253, buffer, 260);
    TEST_ASSERT_EQUAL(STS_OK, result);
    // Max possible length byte: 253 + 2 = 255 (0xFF)
    TEST_ASSERT_EQUAL_HEX8(0xFF, buffer[3]);
    // ~(0x01 + 0xFF + 0x03 + 0x00) = ~0x103 -> ~0x03 = 0xFC
    TEST_ASSERT_EQUAL_HEX8(0xFC, buffer[258]);
}

void test_CreatePacket_BroadcastID(void) {
    uint8_t buffer[STS_MIN_PACKET_SIZE];
    // 0xFE is the Broadcast ID 
    sts_result_t result = sts_create_packet(0xFE, 0x01, NULL, 0, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(0xFE, buffer[2]);
}

void test_CreatePacket_InvalidIDRange(void) {
    uint8_t buffer[10];
    // ID 0xFF is  reserved 
    sts_result_t result = sts_create_packet(0xFF, 0x01, NULL, 0, buffer, 10);
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN, result);
}

void test_CreatePacket_EmptyParamsIgnorePointer(void) {
    uint8_t buffer[STS_MIN_PACKET_SIZE];
    uint8_t dummy_val = 0xAA;

    sts_result_t result = sts_create_packet(0x01, 0x01, &dummy_val, 0, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(0x02, buffer[3]); 
}
void test_CreatePacket_BufferExactlyOneByteTooSmall(void) {
    uint8_t buffer[10];
    uint8_t params[5]; // Total size needed: 6 + 5 = 11
    sts_result_t result = sts_create_packet(0x01, 0x03, params, 5, buffer, 10);
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN, result);
}

void test_CreatePacket_NullParamsWithNonZeroLength(void) {
    uint8_t buffer[20];
    // p_len is 5, but pointer is NULL. 
    sts_result_t result = sts_create_packet(0x01, 0x03, NULL, 5, buffer, 20);
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, result); 
}

void test_CreatePacket_ParamLimitOverflow(void) {
    uint8_t buffer[300];
    uint8_t params[254]; // 254 + 2 = 256, which rolls over to 0 in uint8_t
    sts_result_t result = sts_create_packet(0x01, 0x03, params, 254, buffer, 300);
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN, result);
}