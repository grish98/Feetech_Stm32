/**
 ******************************************************************************
 * @file           : test_sts_protocol.c
 * @brief          : Unit Tests for STS Protocol Layer
 * @author         : Grisham Balloo
 * @date           : 2026-02-28
 * @version        : 1.2.0
 ******************************************************************************
 * @details
 * This test suite provides verification for the Feetech STS
 * Servo Protocol implementation. It utilises the Unity Test Framework to
 * validate the following functional areas:
 *
 * 1. Checksum Logic (7 tests): Mathematical verification of the 8-bit NOT-sum
 *    algorithm, handling of maximum sums, and null-pointer safety.
 * 2. Packet Serialisation (11 tests): Validation of command construction,
 *    broadcast ID handling, and buffer boundary constraints.
 * 3. Parser Robustness (15 tests): Verification of error rejection for
 *    ID mismatches, hardware faults, and malformed length fields.
 * 4. Synchronization and Integration (5 tests): Stress-testing the
 *    sliding-window seeker against leading noise and false-header triggers
 *    to ensure recovery in real-world serial bus environments.
 *
 * @see Lib/STS_Servo/Inc/sts_protocol.h for the target API documentation.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "unity.h"
#include "sts_protocol.h"
#include <stdint.h>
#include <string.h>
#include "test_sts_utils.h"
#include "sts_registers.h"

// Standard servo ID used across the majority of tests */
#define TEST_ID_DEFAULT     0x01U

// Minimum valid STS response fields for ID 0x01 with no parameters
//  CS: ~(01 + 02 + 00) = ~0x03 = 0xFC */
#define TEST_MIN_RESP_LEN  0x02U
#define TEST_MIN_RESP_CS   0xFCU

void setUp(void) {

}

void tearDown(void) {

}

/* =========================================================================
    CHECKSUM LOGIC TESTS
   ========================================================================= */

void test_Checksum_Ping(void) {
    // 6 bytes total: {FF, FF, ID, LEN, INST, CHK_SLOT}
    // Sum = 01 + 02 + 01 = 0x04 -> ~0x04 = 0xFB
    const uint8_t packet[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, STS_INST_PING, 0x00U};
    uint8_t calculated = 0U;

    sts_result_t res = sts_calculate_checksum(packet, (uint16_t)sizeof(packet), &calculated);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_HEX8(0xFB, calculated);
}

void test_Checksum_Maximum_Sum(void) {
    // ID:254, Len:254, Instr:254, Params:[254, 254]
    // Sum = 254 * 5 = 1270 (0x4F6)
    // Truncate to 8-bit = 0xF6
    // ~0xF6 = 0x09
    const uint8_t packet[] = {STS_HEADER, STS_HEADER, 0xFEU, 0xFEU, 0xFEU, 0xFEU, 0xFEU, 0x00U};
    uint8_t calculated = 0U;

    sts_result_t res = sts_calculate_checksum(packet, (uint16_t)sizeof(packet), &calculated);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_HEX8(0x09U, calculated);
}

void test_Checksum_Min_Length_Alternative_ID(void) {
    // ID: 0x0A (10), Len: 2, Instr: 1 (Ping)
    // Sum = 10 + 2 + 1 = 13 (0x0D)
    // ~0x0D = 0xF2
    const uint8_t packet[] = {STS_HEADER, STS_HEADER, 0x0AU, 0x02U, STS_INST_PING, 0x00U};
    uint8_t calculated = 0U;

    sts_result_t res = sts_calculate_checksum(packet, (uint16_t)sizeof(packet), &calculated);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_HEX8(0xF2U, calculated);
}

void test_Checksum_Null_Pointer(void) {
    uint8_t calculated = 0U;

    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, sts_calculate_checksum(NULL, 10U, &calculated));
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, sts_calculate_checksum(NULL, 10U, NULL));
}

void test_Checksum_Too_Short(void) {
    // Any packet under STS_MIN_PACKET_SIZE bytes is invalid protocol-wise
    uint8_t packet[STS_MIN_PACKET_SIZE - 1U] = {0U};
    packet[0] = STS_HEADER;
    packet[1] = STS_HEADER;
    packet[2] = TEST_ID_DEFAULT;

    uint8_t calculated = 0U;

    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN,
        sts_calculate_checksum(packet, (uint16_t)sizeof(packet), &calculated));
}

void test_Checksum_Ignores_Headers_And_Final_Byte(void) {
    // Sum should only cover: ID(1) + Len(2) + Instr(1) = 4
    // Corrupt headers (0xAA) and checksum slot (0x55) must not affect the result
    const uint8_t packet[] = {0xAAU, 0xAAU, TEST_ID_DEFAULT, 0x02U, STS_INST_PING, 0x55U};
    uint8_t calculated = 0U;

    sts_calculate_checksum(packet, (uint16_t)sizeof(packet), &calculated);
    TEST_ASSERT_EQUAL_HEX8(0xFBU, calculated);
}

void test_Checksum_SumExactly256(void) {
    // ID: 0x80 (128), Len: 0x7E (126), Inst: 0x02
    // Sum = 128 + 126 + 2 = 256 (0x100)
    // Truncate to 8-bit = 0x00
    // ~0x00 = 0xFF
    const uint8_t packet[] = {STS_HEADER, STS_HEADER, 0x80U, 0x7EU, 0x02U, 0x00U};
    uint8_t calculated = 0U;

    sts_calculate_checksum(packet, (uint16_t)sizeof(packet), &calculated);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, calculated);
}

/* =========================================================================
    PACKET CREATION TESTS
   ========================================================================= */

void test_CreatePacket_Ping_Success(void) {
    uint8_t buffer[STS_MIN_PACKET_SIZE] = {0U};

    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_PING, NULL, 0U, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    // Expected packet: FF FF 01 02 01 FB
    TEST_ASSERT_EQUAL_HEX8(STS_HEADER,     buffer[STS_IDX_HEADER_1]);
    TEST_ASSERT_EQUAL_HEX8(STS_HEADER,     buffer[STS_IDX_HEADER_2]);
    TEST_ASSERT_EQUAL_HEX8(TEST_ID_DEFAULT, buffer[STS_IDX_ID]);
    TEST_ASSERT_EQUAL_HEX8(0x02U,            buffer[STS_IDX_LENGTH]);
    TEST_ASSERT_EQUAL_HEX8(STS_INST_PING,     buffer[STS_IDX_INSTRUCTION]);
    TEST_ASSERT_EQUAL_HEX8(0xFBU,            buffer[STS_MIN_PACKET_SIZE - 1U]);
}

void test_CreatePacket_BufferTooSmall(void) {
    uint8_t buffer[STS_MIN_PACKET_SIZE - 1U] = {0U};

    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_PING, NULL, 0U, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_ERR_BUF_TOO_SMALL, result);
}

void test_CreatePacket_NullBuffer(void) {
    sts_result_t result = sts_create_packet(TEST_ID_DEFAULT, STS_INST_PING, NULL, 0U, NULL, 10U);

    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, result);
}

void test_CreatePacket_WritePosition_Success(void) {
    uint8_t buffer[15] = {0U};
    // ID 1, Write (0x03), Params: {0x1E, 0x00, 0x01}
    const uint8_t params[] = {0x1EU, 0x00U, 0x01U};
    uint16_t p_len = (uint16_t)sizeof(params);

    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_WRITE, params, p_len, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(0x05U, buffer[STS_IDX_LENGTH]);      // 3 params + 2 overhead = 5
    TEST_ASSERT_EQUAL_HEX8(0x1EU, buffer[STS_IDX_PARAM_START]); // First parameter

    // Checksum: ID(01) + Len(05) + Instr(03) + P1(1E) + P2(00) + P3(01) = 0x27
    // ~0x27 = 0xD7
    TEST_ASSERT_EQUAL_HEX8(0xD7U, buffer[STS_IDX_PARAM_START + p_len]);
}

void test_CreatePacket_MaxPayload(void) {
    // total_packet_size = STS_MIN_PACKET_SIZE + STS_MAX_PARAM_LEN
    uint8_t buffer[STS_MIN_PACKET_SIZE + STS_MAX_PARAM_LEN] = {0U};
    uint8_t params[STS_MAX_PARAM_LEN] = {0U};
    memset(params, 0x00U, STS_MAX_PARAM_LEN);

    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_WRITE, params, (uint16_t)STS_MAX_PARAM_LEN,
        buffer, (uint16_t)sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    // Max length byte: STS_MAX_PARAM_LEN + STS_LENGTH_FIXED_OVERHEAD = 0xFF
    TEST_ASSERT_EQUAL_HEX8(0xFFU, buffer[STS_IDX_LENGTH]);
    // ~(0x01 + 0xFF + 0x03 + 0x00 * 253) = ~0x03 = 0xFC
    TEST_ASSERT_EQUAL_HEX8(0xFCU, buffer[sizeof(buffer) - 1U]);
}

void test_CreatePacket_BroadcastID(void) {
    uint8_t buffer[STS_MIN_PACKET_SIZE] = {0U};
    // 0xFE is the Broadcast ID — must be accepted and written correctly
    sts_result_t result = sts_create_packet(0xFEU, STS_INST_PING, NULL, 0U, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(0xFEU, buffer[STS_IDX_ID]);
}

void test_CreatePacket_InvalidIDRange(void) {
    uint8_t buffer[10]= {0U};
    // ID 0xFF is reserved — must be rejected
    sts_result_t result = sts_create_packet(0xFFU, STS_INST_PING, NULL, 0U, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, result);
}

void test_CreatePacket_EmptyParamsIgnorePointer(void) {
    /* Buffer is written completely by the API; zeroing keeps static analyzers happy */
    uint8_t buffer[STS_MIN_PACKET_SIZE] = {0U};
    uint8_t dummy_val = 0xAAU;

    // Non-null pointer with zero length — pointer must be ignored
    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_PING, &dummy_val, 0U, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(0x02U, buffer[STS_IDX_LENGTH]); // Length reflects zero parameters
}

void test_CreatePacket_BufferExactlyOneByteTooSmall(void) {
    // Total needed: STS_MIN_PACKET_SIZE + 5 = 11, buffer is 10
    uint8_t buffer[STS_MIN_PACKET_SIZE + 4U] = {0U};
    uint8_t params[5]= {0U};

    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_WRITE, params, 5U, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_ERR_BUF_TOO_SMALL, result);
}

void test_CreatePacket_NullParamsWithNonZeroLength(void) {
    uint8_t buffer[20] = {0U};
    // NULL pointer with non-zero length — must be rejected
    sts_result_t result = sts_create_packet(TEST_ID_DEFAULT, STS_INST_WRITE, NULL, 5U, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, result);
}

void test_CreatePacket_ParamLimitOverflow(void) {
    uint8_t buffer[300] =  {0U};
    // STS_MAX_PARAM_LEN + 1 overflows the protocol length byte
    uint8_t params[STS_MAX_PARAM_LEN + 1U] = {0U};

    sts_result_t result = sts_create_packet(
        TEST_ID_DEFAULT, STS_INST_WRITE, params, (uint16_t)(STS_MAX_PARAM_LEN + 1U),
        buffer, (uint16_t)sizeof(buffer));

    TEST_ASSERT_EQUAL(STS_ERR_INVALID_LEN, result);
}

/* =========================================================================
    RESPONSE PARSING TESTS
   ========================================================================= */

void test_ParseResponse_ValidPosition(void) {
    /* RESPONSE STRUCTURE (Servo -> STM32):
     * [ 0  1 ]  [ 2 ]  [ 3 ]  [ 4 ]               [ 5  6 ]  [ 7 ]
     * [ H1 H2]  [ID ]  [LEN]  [STA]                [ P1 P2]  [CS ]
     * [ FF FF ] [ 01 ] [ 04 ] [ 00 ]               [ 0A 05]  [ EB ]
     *                          ^ Status 0x00 = OK | Params: 0x0A, 0x05
     */
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x04U, 0x00U, 0x0AU, 0x05U, 0xEBU};
    uint8_t out_data[10] =  {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16(2U, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, out_data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x05U, out_data[1]);
}

void test_ParseResponse_HardwareError(void) {
    // Status byte = 0x04 (Overheat bit set)
    // CS: ~(01 + 02 + 04) = ~0x07 = 0xF8
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x02U, 0x04U, 0xF8U};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_HARDWARE, result);
}

void test_ParseResponse_IDMismatch(void) {
    const uint8_t INCORRECT_ID = TEST_ID_DEFAULT;
    const uint8_t TARGET_ID    = 0x02U;
    // Packet belongs to ID 0x01, but we request ID 0x02
    uint8_t rx[] = {STS_HEADER, STS_HEADER, INCORRECT_ID, 0x02U, 0x00U, 0xFCU};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TARGET_ID, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_ID_MISMATCH, result);
}

void test_ParseResponse_ChecksumError(void) {
    // Valid structure but checksum byte forced to 0x00 — must be rejected
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x02U, 0x00U, 0x00U};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_CHECKSUM, result);
}

void test_ParseResponse_MinimumLength(void) {
    // Minimum valid response — status byte only, no parameters
    // CS: ~(01 + 02 + 00) = ~0x03 = 0xFC
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, 0x00U, TEST_MIN_RESP_CS};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16(0U, out_len); // No parameters in minimum packet
}

void test_ParseResponse_LengthMismatch(void) {
    // Len field claims 4 (8 bytes total), but only 7 bytes provided
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x04U, 0x00U, 0x0AU, 0x05U};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_MALFORMED, result);
}

void test_ParseResponse_NullGuards(void) {
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, 0x00U, TEST_MIN_RESP_CS};
    uint16_t out_len;

    // Null output data buffer
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR,
        sts_parse_response(TEST_ID_DEFAULT, rx, (uint16_t)STS_MIN_PACKET_SIZE,
                           NULL, (uint16_t)sizeof(rx), &out_len));
    // Null rx buffer
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR,
        sts_parse_response(TEST_ID_DEFAULT, NULL, (uint16_t)STS_MIN_PACKET_SIZE,
                           rx, (uint16_t)sizeof(rx), &out_len));
}

void test_ParseResponse_OverloadError(void) {
    // Status byte = 0x08 (Overload error bit set)
    // CS: ~(01 + 02 + 08) = ~0x0B = 0xF4
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x02U, 0x08U, 0xF4U};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_HARDWARE, result);
}

void test_ParseResponse_HeaderError_FirstByte(void) {
    // First header byte corrupt (0x00 instead of 0xFF)
    uint8_t rx[] = {0x00U, 0xFFU, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, 0x00U, TEST_MIN_RESP_CS};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    TEST_ASSERT_EQUAL(STS_ERR_HEADER,
        sts_parse_response(TEST_ID_DEFAULT, rx, (uint16_t)STS_MIN_PACKET_SIZE,
                           out_data, (uint16_t)sizeof(out_data), &out_len));
}

void test_ParseResponse_HeaderError_SecondByte(void) {
    // Second header byte corrupt (0x00 instead of 0xFF)
    uint8_t rx[] = {0xFFU, 0x00U, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, 0x00U, TEST_MIN_RESP_CS};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    TEST_ASSERT_EQUAL(STS_ERR_HEADER,
        sts_parse_response(TEST_ID_DEFAULT, rx, (uint16_t)STS_MIN_PACKET_SIZE,
                           out_data, (uint16_t)sizeof(out_data), &out_len));
}

void test_ParseResponse_IncompletePacket(void) {
    // Complete packet is 8 bytes — passing 7 simulates a truncated UART buffer
    uint8_t rx[] = {STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x04U, 0x00U, 0x0AU, 0x05U, 0xEBU};
    uint8_t out_data[10] = {0U};
    uint16_t out_len;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)(sizeof(rx) - 1U), out_data, (uint16_t)sizeof(out_data), &out_len);

    TEST_ASSERT_EQUAL(STS_ERR_MALFORMED, result);
}

void test_ParseResponse_MaxPayload(void) {
    // Maximum response: STS_MIN_PACKET_SIZE + STS_MAX_PARAM_LEN bytes total
    const uint16_t max_pkt_len = (uint16_t)(STS_MIN_PACKET_SIZE + STS_MAX_PARAM_LEN);
    uint8_t rx[STS_MIN_PACKET_SIZE + STS_MAX_PARAM_LEN] = {0U};
    
    rx[STS_IDX_HEADER_1]    = STS_HEADER;
    rx[STS_IDX_HEADER_2]    = STS_HEADER;
    rx[STS_IDX_ID]          = TEST_ID_DEFAULT;
    rx[STS_IDX_LENGTH]      = (uint8_t)(STS_MAX_PARAM_LEN + STS_LENGTH_FIXED_OVERHEAD); // 0xFF
    rx[STS_IDX_STATUS ]     = STS_HARDWARE_OK;
    memset(&rx[STS_IDX_PARAM_START], 0xAAU, STS_MAX_PARAM_LEN);

    uint8_t checksum = 0U;
    sts_result_t crc_res = sts_calculate_checksum(rx, max_pkt_len, &checksum);
    TEST_ASSERT_EQUAL(STS_OK, crc_res);
    rx[max_pkt_len - 1U] = checksum;

    /* output buffer is zeroed before use for clarity */
    uint8_t param_buf[STS_MAX_PARAM_LEN] = {0U};
    uint16_t param_len = 0U;
    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, max_pkt_len, param_buf, (uint16_t)sizeof(param_buf), &param_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)STS_MAX_PARAM_LEN, param_len);
    TEST_ASSERT_EQUAL_HEX8(0xAAU, param_buf[STS_MAX_PARAM_LEN - 1U]); // Verify last byte
}

void test_ParseResponse_BoundaryIDs(void) {
    uint8_t param_buf[10] = {0U};
    uint16_t param_len = 0U;

    // ID 0x00 — lowest valid ID
    // CS: ~(00 + 02 + 00) = ~0x02 = 0xFD
    const uint8_t rx_min[] = {STS_HEADER, STS_HEADER, 0x00U, TEST_MIN_RESP_LEN, 0x00U, 0xFDU};
    TEST_ASSERT_EQUAL(STS_OK,
        sts_parse_response(0x00U, rx_min, (uint16_t)STS_MIN_PACKET_SIZE,
                           param_buf, (uint16_t)sizeof(param_buf), &param_len));

    // ID 0xFE — broadcast ID, highest valid
    // CS: ~(FE + 02 + 00) = ~0x100 -> ~0x00 = 0xFF
    const uint8_t rx_max[] = {STS_HEADER, STS_HEADER, 0xFEU, TEST_MIN_RESP_LEN, 0x00U, 0xFFU};
    TEST_ASSERT_EQUAL(STS_OK,
        sts_parse_response(0xFEU, rx_max, (uint16_t)STS_MIN_PACKET_SIZE,
                           param_buf, (uint16_t)sizeof(param_buf), &param_len));
}

void test_ParseResponse_OutputBufferSafety(void) {
    // Packet with 10 parameter bytes — output buffer only 2 bytes, must reject safely
    // protocol_len = Status(1) + Data(10) + Checksum(1) = 12 (0x0C)
    uint8_t rx[] = {
        STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, 0x0CU, 0x00U,
        0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, 0x09U, 0x0AU,
        0x00U
    };

    uint8_t checksum = 0U;
    sts_result_t crc_res = sts_calculate_checksum(rx, (uint16_t)sizeof(rx), &checksum);
    TEST_ASSERT_EQUAL(STS_OK, crc_res);
    rx[(uint16_t)sizeof(rx) - 1U] = checksum;

    uint8_t small_param_buf[2] = {0xEEU, 0xEEU};
    uint16_t param_len = 0U;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, rx, (uint16_t)sizeof(rx), small_param_buf, (uint16_t)sizeof(small_param_buf), &param_len);

    TEST_ASSERT_EQUAL(STS_ERR_BUF_TOO_SMALL, result);
    TEST_ASSERT_EQUAL_UINT16(0U, param_len); // Must not be modified on failure

    TEST_ASSERT_EQUAL_HEX8(0xEEU, small_param_buf[0]); 
    TEST_ASSERT_EQUAL_HEX8(0xEEU, small_param_buf[1]);
}

void test_ParseResponse_JunkData(void) {
    // All-zero and all-0xFF buffers must never parse as valid packets
    const uint8_t all_zeros[10] = {0U};
    const uint8_t all_ones[10]  = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    uint8_t param_buf[10] = {0U};
    uint16_t param_len = 0U;

    TEST_ASSERT_NOT_EQUAL(STS_OK,
        sts_parse_response(TEST_ID_DEFAULT, all_zeros, (uint16_t)sizeof(all_zeros),
                           param_buf, (uint16_t)sizeof(param_buf), &param_len));
    TEST_ASSERT_NOT_EQUAL(STS_OK,
        sts_parse_response(TEST_ID_DEFAULT, all_ones, (uint16_t)sizeof(all_ones),
                           param_buf, (uint16_t)sizeof(param_buf), &param_len));
}

/* =========================================================================
    PROTOCOL SYNCHRONIZATION & INTEGRATION TESTS
   ========================================================================= */

void test_Protocol_Integration_Loopback(void) {
    const uint8_t original_params[] = {0xAAU, 0xBBU};
    uint16_t original_len = (uint16_t)sizeof(original_params);

    /* EXPECTED PACKET STRUCTURE:
     * [ 0  1 ]  [ 2 ]  [ 3 ]  [ 4 ]  [ 5  6 ]  [ 7 ]
     * [ H1 H2]  [ID ]  [LEN]  [STA]  [ P1 P2]  [CS ]
     * [ FF FF ] [ 01 ] [ 04 ] [ 00 ] [ AA BB]  [ EB ]
     */
    /* smoke–test buffer for the loopback helper – zeroing is cheap */
    uint8_t simulated_stream[STS_MIN_PACKET_SIZE + 2U] = {0U};
    simulate_servo_response(TEST_ID_DEFAULT, STS_HARDWARE_OK,
                            original_params, original_len, simulated_stream);

    uint8_t parsed_payload[8] = {0U};
    uint16_t parsed_len = 0U;

    sts_result_t result = sts_parse_response(
        TEST_ID_DEFAULT, simulated_stream, (uint16_t)sizeof(simulated_stream),
        parsed_payload, (uint16_t)sizeof(parsed_payload), &parsed_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16(original_len, parsed_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(original_params, parsed_payload, original_len);
}

void test_Protocol_Integration_MultiByteRead(void) {
    const uint8_t SENSOR_ID = 10U;

    /* Simulate reading 5 bytes of data (e.g., Position + Speed + Temperature)
     *
     * EXPECTED PACKET STRUCTURE:
     * [ 0  1 ]  [ 2 ]  [ 3 ]  [ 4 ]  [ 5  6  7  8  9 ]  [ 10 ]
     * [ H1 H2]  [ID ]  [LEN]  [STA]  [ P1 P2 P3 P4 P5]  [CS  ]
     * [ FF FF ] [ 0A ] [ 07 ] [ 00 ] [ 01 02 03 04 05 ]  [ CS ]
     * ^ LEN = Status(1) + Params(5) + CS(1) = 7
     */
    const uint8_t raw_sensor_values[] = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U};
    uint16_t sensor_data_len = (uint16_t)sizeof(raw_sensor_values);

    uint8_t simulated_stream[STS_MIN_PACKET_SIZE + 5U] = {0U};
    simulate_servo_response(SENSOR_ID, STS_HARDWARE_OK,
                            raw_sensor_values, sensor_data_len, simulated_stream);

    uint8_t parsed_payload[16] = {0U};
    uint16_t parsed_len = 0U;

    sts_result_t result = sts_parse_response(
        SENSOR_ID, simulated_stream, (uint16_t)sizeof(simulated_stream),
        parsed_payload, (uint16_t)sizeof(parsed_payload), &parsed_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16(sensor_data_len, parsed_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(raw_sensor_values, parsed_payload, sensor_data_len);
}

void test_Protocol_Integration_Robustness_Seeker(void) {
    /* Buffer Layout:
     * [ 00 AA ] [ FF FF 01 02 00 FC ]
     * | Noise | |--- Valid Packet --|
     *             ^ Starts at index 2
     */
    const uint8_t noisy_bus_stream[] = {
        0x00U, 0xAAU,
        STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, 0x00U, TEST_MIN_RESP_CS
    };

    uint8_t extracted_payload[2] = {0U};
    uint16_t extracted_len=0;

    sts_result_t status = sts_parse_response(
        TEST_ID_DEFAULT, noisy_bus_stream, (uint16_t)sizeof(noisy_bus_stream),
        extracted_payload, (uint16_t)sizeof(extracted_payload), &extracted_len);

    TEST_ASSERT_EQUAL(STS_OK, status);
    TEST_ASSERT_EQUAL_UINT16(0U, extracted_len); // Ping response carries no parameters
}

void test_Protocol_Integration_FalseHeaderRecovery(void) {
    /* Buffer Layout:
     * [ 00 FF FF 01 FF ] [ FF FF 01 02 00 FC ]
     * |--- Junk Data ---| |--- Valid Packet --|
     *  ^ Fake Header        ^ Real Header
     */
    const uint8_t noisy_bus_stream[] = {
        0x00U, 0xFFU, 0xFFU, 0x01U, 0xFFU,
        STS_HEADER, STS_HEADER, TEST_ID_DEFAULT, TEST_MIN_RESP_LEN, 0x00U, TEST_MIN_RESP_CS
    };

    uint8_t extracted_payload[2] = {0U};
    uint16_t extracted_len = 0U;

    sts_result_t status = sts_parse_response(
        TEST_ID_DEFAULT, noisy_bus_stream, (uint16_t)sizeof(noisy_bus_stream),
        extracted_payload, (uint16_t)sizeof(extracted_payload), &extracted_len);

    TEST_ASSERT_EQUAL(STS_OK, status);
    TEST_ASSERT_EQUAL_UINT16(0U, extracted_len);
}

void test_Protocol_Integration_Filters_Wrong_ID(void) {
     const uint8_t SEARCH_TARGET_ID = TEST_ID_DEFAULT;
     const uint8_t IGNORED_DECOY_ID = 0x05U;

    /* Buffer Layout:
     * [ 0  1  2  3  4  5 ] [ 6  7  8  9  10 11 12 13 ]
     * |--- Decoy Pkt ----| |-----  Target Pkt   -----|
     * [ FF FF 05 02 00 FD] [ FF FF 01 04 00 DE AD EB ]
     *         ^ ID 5               ^ ID 1
     */
    uint8_t raw_uart_stream[2U * STS_MIN_PACKET_SIZE + 2U] = {0U};
    uint8_t extracted_payload[8] = {0U};
    uint16_t extracted_len = 0U;

    // Decoy response — parser must skip this
    simulate_servo_response(IGNORED_DECOY_ID, STS_HARDWARE_OK, NULL, 0U, &raw_uart_stream[0]);

    // Target response — parser must return this
    const uint8_t expected_payload[] = {0xDEU, 0xADU};
    simulate_servo_response(SEARCH_TARGET_ID, STS_HARDWARE_OK, expected_payload, 2U,
                            &raw_uart_stream[STS_MIN_PACKET_SIZE]);

    sts_result_t result = sts_parse_response(
        SEARCH_TARGET_ID, raw_uart_stream, (uint16_t)sizeof(raw_uart_stream),
        extracted_payload, (uint16_t)sizeof(extracted_payload), &extracted_len);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16(2U, extracted_len);
    TEST_ASSERT_EQUAL_HEX8(0xDEU, extracted_payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0xADU, extracted_payload[1]);
}
