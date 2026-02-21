#include "unity.h"
#include "sts_protocol.h"
#include <stdint.h>
#include<string.h>


void setUp(void) {
    
}

void tearDown(void) {
    
}

/* =========================================================================
    CHECKSUM LOGIC TESTS
   ========================================================================= */

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
    // Should return 0 and NOT crash 
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

/* =========================================================================
    PACKET CREATION TESTS
   ========================================================================= */


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

/* =========================================================================
    RESPONSE PARSING TESTS
   ========================================================================= */

void test_ParseResponse_ValidPosition(void) {
    // Packet: FF FF | ID:01 | Len:04 | Status:00 | Data:0A 05 | CS:EA
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x0A, 0x05, 0xEB};
    uint8_t out_data[10];
    uint16_t out_len;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL(2, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x0A, out_data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x05, out_data[1]);
}

void test_ParseResponse_HardwareError(void) {
    // Status byte (index 4) = 0x04 (Overheat)
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x02, 0x04, 0xF8}; 
    uint8_t out_data[10];
    uint16_t out_len;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_HARDWARE, result);
}

void test_ParseResponse_IDMismatch(void) {
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    uint8_t out_data[10];
    uint16_t out_len;

    // We expect ID 0x02, but packet has ID 0x01
    sts_result_t result = sts_parse_response(0x02, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);
    TEST_ASSERT_EQUAL(STS_ERR_ID_MISMATCH, result);
}

void test_ParseResponse_ChecksumError(void) {
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0x00}; // Forced bad CS
    uint8_t out_data[10];
    uint16_t out_len;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);
    TEST_ASSERT_EQUAL(STS_ERR_CHECKSUM, result);
}

void test_ParseResponse_MinimumLength(void) {
    // ID:01, Len:02, Status:00, CS:FC (~0x03)
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    uint8_t out_data[10];
    uint16_t out_len;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL(0, out_len); // Should be 0 parameters
}

void test_ParseResponse_LengthMismatch(void) {
    // Claimed Len: 04, but only 7 bytes total (should be 8)
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x0A, 0x05}; 
    uint8_t out_data[10];
    uint16_t out_len;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);
    TEST_ASSERT_EQUAL(STS_ERR_MALFORMED, result);
}

void test_ParseResponse_NullGuards(void) {
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFD};
    uint16_t out_len;
    
    // Test null output data buffer
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, sts_parse_response(0x01, rx, 6, NULL, sizeof(rx), &out_len));
    // Test null rx buffer
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, sts_parse_response(0x01, NULL, 6, rx, sizeof(rx), &out_len));
}

void test_ParseResponse_OverloadError(void) {
    // Status Byte = 0x08 (Overload Error bit set)
    // CS: ~ (01 + 02 + 08) = ~0x0B = 0xF4
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x02, 0x08, 0xF4};
    uint8_t out_data[10];
    uint16_t out_len;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), out_data,sizeof(out_data), &out_len);
    TEST_ASSERT_EQUAL(STS_ERR_HARDWARE, result);
}

void test_ParseResponse_HeaderError_FirstByte(void) {
    uint8_t rx[] = {0x00, 0xFF, 0x01, 0x02, 0x00, 0xFC}; // 0x00 instead of 0xFF
    uint8_t out_data[10];
    uint16_t out_len;

    TEST_ASSERT_EQUAL(STS_ERR_HEADER, sts_parse_response(0x01, rx, 6, out_data,sizeof(out_data), &out_len));
}

void test_ParseResponse_HeaderError_SecondByte(void) {
    uint8_t rx[] = {0xFF, 0x00, 0x01, 0x02, 0x00, 0xFC}; // 0x00 instead of 0xFF
    uint8_t out_data[10];
    uint16_t out_len;

    TEST_ASSERT_EQUAL(STS_ERR_HEADER, sts_parse_response(0x01, rx, 6, out_data,sizeof(out_data), &out_len));
}

void test_ParseResponse_IncompletePacket(void) {
    // Claims Len 04 (8 bytes total), but we only provide 7
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x0A, 0x05, 0xEB}; 
    uint8_t out_data[10];
    uint16_t out_len;

    // Passing sizeof(rx) - 1 simulates a truncated UART buffer
    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx) - 1, out_data,sizeof(out_data), &out_len);
    TEST_ASSERT_EQUAL(STS_ERR_MALFORMED, result);
}

void test_ParseResponse_MaxPayload(void) {
    uint8_t rx[259]; // Max size
    rx[0] = 0xFF; rx[1] = 0xFF; rx[2] = 0x01;
    rx[3] = 255;  // protocol_len
    rx[4] = 0x00;  // Status
    memset(&rx[5], 0xAA, 253); // Fill 253 bytes of data
    rx[258] = sts_calculate_checksum(rx, 259);

    uint8_t param_buf[255];
    uint16_t param_len;
    sts_result_t result = sts_parse_response(0x01, rx, 259, param_buf,sizeof(param_buf), &param_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL(253, param_len);
    TEST_ASSERT_EQUAL_HEX8(0xAA, param_buf[252]); // Check the last byte
}

void test_ParseResponse_BoundaryIDs(void) {
    uint8_t param_buf[10];
    uint16_t param_len;
    
    // Test ID 0x00 (Valid)
    uint8_t rx_min[] = {0xFF, 0xFF, 0x00, 0x02, 0x00, 0xFD};
    TEST_ASSERT_EQUAL(STS_OK, sts_parse_response(0x00, rx_min, 6, param_buf,sizeof(param_buf), &param_len));

    // Test ID 0xFE (Broadcast/Valid)
    uint8_t rx_max[] = {0xFF, 0xFF, 0xFE, 0x02, 0x00, 0xFF};
    TEST_ASSERT_EQUAL(STS_OK, sts_parse_response(0xFE, rx_max, 6, param_buf,sizeof(param_buf), &param_len));
}

void test_ParseResponse_OutputBufferSafety(void) {
    // Packet: Headers(2), ID(1), Len(12), Status(1), Data(10), Checksum(1) = 16 bytes total
    // protocol_len (rx[3]) is 12 (Status + 10 Data + Checksum)
    uint8_t rx[] = {0xFF, 0xFF, 0x01, 0x0C, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0x00};
    rx[15] = sts_calculate_checksum(rx, sizeof(rx));
    
    uint8_t small_param_buf[2]; // Target buffer is only 2 bytes
    uint16_t param_len = 0;

    sts_result_t result = sts_parse_response(0x01, rx, sizeof(rx), small_param_buf, sizeof(small_param_buf), &param_len);

    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN, result);
    // Ensure param_len wasn't modified or at least didn't indicate success
    TEST_ASSERT_EQUAL(0, param_len); 
}

void test_ParseResponse_JunkData(void) {
    uint8_t all_zeros[10] = {0};
    uint8_t all_ones[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t param_buf[10];
    uint16_t param_len;

    TEST_ASSERT_NOT_EQUAL(STS_OK, sts_parse_response(0x01, all_zeros, 10, param_buf,sizeof(param_buf), &param_len));
    TEST_ASSERT_NOT_EQUAL(STS_OK, sts_parse_response(0x01, all_ones, 10, param_buf,sizeof(param_buf), &param_len));
}