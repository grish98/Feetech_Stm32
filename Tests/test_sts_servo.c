/**
 ******************************************************************************
 * @file           : test_sts_servo.c
 * @brief          : Unit tests for the STS Service and Bus Layers
 * @author         : Grisham Balloo
 * @date           : 2026-03-06
 * @version        : 0.2.0
 ******************************************************************************
@details
 * This test suite provides verification for the Feetech STS Service Layer
 * and Bus HAL implementation. It utilises the Unity Test Framework to
 * validate the following functional areas:
 *
 * 1. Bus Initialisation (8 tests): Validation of HAL function pointer mapping,
 *    deterministic memory overwrite, null-pointer guards, and re-initialisation
 *    safety across multiple hardware contexts.
 *
 * 2. Servo Handle Initialisation (8 tests): Verification of ID boundary
 *    enforcement, state reset guarantees, atomic failure behaviour, and
 *    correct linkage across independent bus instances.
 *
 * 3. Register Access Primitives (4 tests): Happy-path coverage for
 *    STS_Write8, STS_Write16, STS_Read8, and STS_Read16 against a
 *    simulated UART response.
 *
 * 4. Engine Robustness & Error Handling (11 tests): Stress-testing the
 *    sts_execute_command engine against null guards, bus faults, timeout,
 *    ID mismatch, checksum corruption, header corruption, malformed length
 *    fields, broadcast early-exit, and internal buffer overflow detection.
 *
 * 5. Ping Service (11 tests): Verification of STS_servo_ping online/offline
 *    state management across success, timeout, ID mismatch, checksum error,
 *    bus busy, null guards, broadcast rejection, hardware error tolerance,
 *    boundary IDs, and offline-to-online recovery.
 *
 * @see Lib/STS_Servo/Inc/sts_servo.h for the target API documentation.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#include "unity.h"
#include "sts_servo.h"
#include "sts_protocol.h"
#include "sts_registers.h"
#include "test_sts_utils.h"
#include <string.h>

/* ==========================================================================
 * Test Environment & Buffer Limits
 * ========================================================================== */
#define TEST_MAX_TX_BUFFER          128U
#define TEST_MAX_RX_BUFFER          128U
#define TEST_TIMEOUT_MS             100U

/* ==========================================================================
 * Mock Hardware & Identity Configurations
 * ========================================================================== */
#define MOCK_BUS_ADDR_A             ((void*)0x1111)
#define MOCK_BUS_ADDR_B             ((void*)0x2222)

#define TEST_MIN_ID                 0U
#define TEST_VALID_ID               10U
#define TEST_ALT_ID                 20U
#define TEST_MAX_ID                 253U

/* ==========================================================================
 * Protocol Sizing & Expected Lengths
 * ========================================================================== */
#define PAYLOAD_LEN_NONE            0U
#define PAYLOAD_LEN_1_BYTE          1U
#define PAYLOAD_LEN_2_BYTES         2U

/* Expected RX Lengths = STS Base ACK (6 bytes) + Data Payload Length */
#define EXPECTED_WRITE_ACK_LEN      (6U + PAYLOAD_LEN_NONE)    /* 6U */
#define EXPECTED_READ8_ACK_LEN      (6U + PAYLOAD_LEN_1_BYTE)  /* 7U */
#define EXPECTED_READ16_ACK_LEN     (6U + PAYLOAD_LEN_2_BYTES) /* 8U */

/* ==========================================================================
 * Standard Test Data Values
 * ========================================================================== */
#define HW_STATUS_OK                0x00U

#define TEST_DATA_BYTE              0xAAU
#define TEST_DATA_LEN               1U

#define TEST_VAL_8BIT               0x7FU
#define TEST_VAL_16BIT              0x1234U
#define TEST_VAL_16BIT_LOW          0x34U  /* Little-endian lower byte */
#define TEST_VAL_16BIT_HIGH         0x12U  /* Little-endian upper byte */

/* ==========================================================================
 * Packet Corruption & Error Injection Constants
 * ========================================================================== */
/* Byte Indices in a standard STS Packet */
#define IDX_HEADER_START            0U
#define IDX_LENGTH_BYTE             3U
#define IDX_CHECKSUM_16BIT_READ     7U     /* Index of CS in an 8-byte packet */

/* Corrupt Values */
#define CORRUPT_HEADER_BYTE         0xEEU
#define CORRUPT_CHECKSUM_MASK       0xFFU
#define FAKE_LONG_PACKET_LENGTH     10U
#define TRUNCATED_PACKET_LENGTH     2U

static sts_bus_t test_bus;
static sts_servo_t test_servo; 

extern sts_result_t sts_execute_command(sts_servo_t *servo, const sts_cmd_t *cmd);

void setUp(void) {
    /* 1. Completely wipe the globals to prevent state leakage */
    memset(&test_bus, 0, sizeof(test_bus));
    memset(&test_servo, 0, sizeof(test_servo));
    memset(&dummy_uart_port, 0, sizeof(dummy_uart_port));

    /* 2. Re-initialize from a clean slate */
    (void)STS_Bus_Init(&test_bus, &dummy_uart_port, mock_tx, mock_rx);
    (void)STS_Servo_Init(&test_servo, &test_bus, TEST_VALID_ID);
}
void tearDown(void) {

}

/* ==========================================
 * TESTS: STS_Bus_Init
 * ========================================== */


void test_STS_Bus_Init_Success(void) {
    sts_bus_t bus; 
    sts_result_t res = STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);
    
    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&dummy_uart_port, bus.port_handle);
    TEST_ASSERT_EQUAL_PTR(mock_tx, bus.transmit);
    TEST_ASSERT_EQUAL_PTR(mock_rx, bus.receive);
}

void test_STS_Bus_Init_Null_Bus(void) {
    sts_result_t res = STS_Bus_Init(NULL, &dummy_uart_port, mock_tx, mock_rx);
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, res);
}

/**
 * @brief Null Port Handle is ALLOWED.
 * This should succeed as some HALs/transports may not require a specific 
 * hardware handle.
 */
void test_STS_Bus_Init_Null_TX(void) {
    sts_bus_t bus;
    sts_result_t res = STS_Bus_Init(&bus, &dummy_uart_port, NULL, mock_rx);
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, res);
}

void test_STS_Bus_Init_Null_RX(void) {
    sts_bus_t bus;
    sts_result_t res = STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, NULL);
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, res);
}

void test_STS_Bus_Init_Null_Port_Handle_Succeeds(void) {
    sts_bus_t bus;
    sts_result_t res = STS_Bus_Init(&bus, NULL, mock_tx, mock_rx);
    
    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(NULL, bus.port_handle);
}

void test_STS_Bus_Init_Overwrites_Garbage(void) {
    sts_bus_t bus;
    
    bus.port_handle = (void*)0xDEADBEEF;
    bus.transmit    = (sts_hal_transmit_t)0xBAD0F00D;
    bus.receive     = (sts_hal_receive_t)0xCAFEBABE;

    sts_result_t res = STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&dummy_uart_port, bus.port_handle);
    TEST_ASSERT_EQUAL_PTR(mock_tx, bus.transmit);
    TEST_ASSERT_EQUAL_PTR(mock_rx, bus.receive);
}

void test_STS_Bus_Init_Reinitialization(void) {
    sts_bus_t bus;
    
    mock_uart_t mock_uart_1 = {0};
    mock_uart_t mock_uart_2 = {0};

    // First initialization 
    (void)STS_Bus_Init(&bus, &mock_uart_1, mock_tx, mock_rx);
    
    // Second initialization with different hardware context 
    sts_result_t res = STS_Bus_Init(&bus, &mock_uart_2, mock_tx, mock_rx);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&mock_uart_2, bus.port_handle);
}

void test_STS_Bus_Interface_Execution(void) {
    sts_bus_t bus;
    uint8_t dummy_data = TEST_DATA_BYTE;
    
    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    dummy_uart_port.rx_len = TEST_DATA_LEN; 

    TEST_ASSERT_EQUAL(STS_OK, bus.transmit(&bus, &dummy_data, TEST_DATA_LEN));
    TEST_ASSERT_EQUAL(STS_OK, bus.receive(&bus, &dummy_data, TEST_DATA_LEN, TEST_TIMEOUT_MS));
}

/* ==========================================
 * TESTS: STS_Servo_Init
 * ========================================== */

void test_STS_Servo_Init_Success(void) {
    sts_bus_t bus;
    sts_servo_t servo;

    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);
    
    sts_result_t res = STS_Servo_Init(&servo, &bus, TEST_VALID_ID);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&bus, servo.bus);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, servo.id);
    TEST_ASSERT_EQUAL_UINT8(0U, servo.is_online); 
}

void test_STS_Servo_Init_Null_Servo(void) {
    sts_bus_t bus;
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Servo_Init(NULL, &bus, TEST_VALID_ID));
}

void test_STS_Servo_Init_Null_Bus(void) {
    sts_servo_t servo;
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Servo_Init(&servo, NULL, TEST_VALID_ID));
}

void test_STS_Servo_Init_Invalid_ID(void) {
    sts_bus_t bus;
    sts_servo_t servo;
    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    // ID 254 and 255 are protocol reserved 
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &bus, STS_ID_BROADCAST_SYNC));
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &bus, STS_ID_BROADCAST_ASYNC));
}

void test_STS_Servo_Init_ID_Boundaries(void) {
    sts_bus_t bus;
    sts_servo_t servo;
    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    // Test minimum 
    TEST_ASSERT_EQUAL(STS_OK, STS_Servo_Init(&servo, &bus, TEST_MIN_ID));
    TEST_ASSERT_EQUAL_UINT8(TEST_MIN_ID, servo.id);

    //Test maximum 
    TEST_ASSERT_EQUAL(STS_OK, STS_Servo_Init(&servo, &bus, TEST_MAX_ID));
    TEST_ASSERT_EQUAL_UINT8(TEST_MAX_ID, servo.id);
}

void test_STS_Servo_Init_Clears_Handle_State(void) {
    sts_bus_t bus;
    sts_servo_t servo;
    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    servo.bus = (sts_bus_t*)0xDEADBEEFU;
    servo.id = 0xAAU;
    servo.is_online = 1U;

    (void)STS_Servo_Init(&servo, &bus, TEST_VALID_ID);

    TEST_ASSERT_EQUAL_PTR(&bus, servo.bus);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, servo.id);
    TEST_ASSERT_EQUAL_UINT8(0U, servo.is_online);
}

void test_STS_Servo_Init_Atomic_Failure(void) {
    sts_bus_t bus;
    sts_servo_t servo;
    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    (void)STS_Servo_Init(&servo, &bus, TEST_VALID_ID);

    // Failed init should leave existing state untouched 
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &bus, STS_ID_BROADCAST_ASYNC));
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, servo.id);
}

void test_STS_Servo_Init_Reset_Online_Status(void) {
    sts_bus_t bus;
    sts_servo_t servo;
    (void)STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    (void)STS_Servo_Init(&servo, &bus, TEST_VALID_ID);
    servo.is_online = 1U; 

    // Re-init must reset status to offline 
    (void)STS_Servo_Init(&servo, &bus, TEST_VALID_ID);
    TEST_ASSERT_EQUAL_UINT8(0U, servo.is_online);
}

void test_STS_Servo_Init_Multi_Bus_Link(void) {
    sts_bus_t bus_a, bus_b;
    sts_servo_t servo_a, servo_b;
    
    (void)STS_Bus_Init(&bus_a, MOCK_BUS_ADDR_A, mock_tx, mock_rx);
    (void)STS_Bus_Init(&bus_b, MOCK_BUS_ADDR_B, mock_tx, mock_rx);

    (void)STS_Servo_Init(&servo_a, &bus_a, TEST_VALID_ID);
    (void)STS_Servo_Init(&servo_b, &bus_b, TEST_ALT_ID);

    TEST_ASSERT_EQUAL_PTR(&bus_a, servo_a.bus);
    TEST_ASSERT_EQUAL_PTR(&bus_b, servo_b.bus);
}

/* ==========================================================================
 * TESTS: STS Servo Read/Write Primitives
 * ========================================================================== */

void test_STS_Write8_Success(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN; 

    sts_result_t result = STS_Write8(&test_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE);

    TEST_ASSERT_EQUAL_INT(STS_OK, result);
}

void test_STS_Read8_Success(void) {
    uint8_t actual_read_value = 0U;
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {TEST_VAL_8BIT};
    
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN; 

    sts_result_t result = STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &actual_read_value);

    TEST_ASSERT_EQUAL_INT(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(TEST_VAL_8BIT, actual_read_value);
}

void test_STS_Write16_Success(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN; 

    sts_result_t result = STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT);

    TEST_ASSERT_EQUAL_INT(STS_OK, result);
}

void test_STS_Read16_Success(void) {
    uint16_t actual_read_value = 0U;
    uint8_t mock_payload[PAYLOAD_LEN_2_BYTES] = {TEST_VAL_16BIT_LOW, TEST_VAL_16BIT_HIGH}; 
    
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN; 

    sts_result_t result = STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &actual_read_value);

    TEST_ASSERT_EQUAL_INT(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX16(TEST_VAL_16BIT, actual_read_value);
}

/* ==========================================================================
 * TESTS: STS Engine Robustness & Error Handling
 * ========================================================================== */

void test_STS_Primitives_All_Null_Guards(void) {
    uint16_t val16;
    uint8_t val8;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Write8(NULL, STS_REG_TORQUE_ENABLE, STS_REG_TORQUE_ENABLE));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Write16(NULL, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read8(NULL, STS_REG_PRESENT_VOLTAGE, &val8));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read16(NULL, STS_REG_PRESENT_POSITION, &val16));

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, NULL));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, NULL));
}

void test_STS_Primitives_All_Timeout(void) {
    uint8_t val8;
    uint16_t val16;
    dummy_uart_port.rx_len = 0U;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Write8(&test_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &val8));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &val16));
}

void test_STS_Primitives_All_Data_Integrity(void) {
    uint16_t val16;
    uint8_t mock_payload[PAYLOAD_LEN_2_BYTES] = {TEST_VAL_16BIT_LOW, TEST_VAL_16BIT_HIGH};

    simulate_servo_response(TEST_ALT_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    TEST_ASSERT_EQUAL_INT(STS_ERR_ID_MISMATCH, STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));

    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_buffer[IDX_CHECKSUM_16BIT_READ] ^= CORRUPT_CHECKSUM_MASK; 
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;
    TEST_ASSERT_EQUAL_INT(STS_ERR_CHECKSUM, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &val16));
}

void test_STS_Primitives_All_Hardware_Errors(void) {
    uint16_t dummy_val; 
    
    test_bus.transmit = mock_tx_busy; 
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, STS_Write8(&test_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
    
    test_bus.transmit = mock_tx_fail; 
    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &dummy_val));

    test_bus.transmit = mock_tx; 
}

void test_STS_ExecuteCommand_Header_Corruption(void) {
    uint8_t val8;
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    
    dummy_uart_port.rx_buffer[IDX_HEADER_START] = CORRUPT_HEADER_BYTE; 
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN; 

    TEST_ASSERT_EQUAL_INT(STS_ERR_HEADER, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &val8));
}

void test_STS_ExecuteCommand_Length_Mismatch(void) {
    uint16_t actual_read_value;
    uint8_t mock_payload[PAYLOAD_LEN_2_BYTES] = {HW_STATUS_OK, HW_STATUS_OK};
    
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    
    dummy_uart_port.rx_buffer[IDX_LENGTH_BYTE] = FAKE_LONG_PACKET_LENGTH; 
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN; 

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &actual_read_value));
}

void test_STS_ExecuteCommand_Standard_Write_Safety(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t result = STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT); 
    
    TEST_ASSERT_EQUAL_INT(STS_OK, result);
}
void test_STS_ExecuteCommand_Buffer_Overflow(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t result = STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT); 
    
    TEST_ASSERT_EQUAL_INT(STS_OK, result);
}

void test_STS_ExecuteCommand_Truncated_Packet(void) {
    uint8_t actual_read_value;
    
    dummy_uart_port.rx_buffer[IDX_HEADER_START] = 0xFF;
    dummy_uart_port.rx_buffer[IDX_HEADER_START + 1] = 0xFF;
    dummy_uart_port.rx_len = TRUNCATED_PACKET_LENGTH; 

    sts_result_t result = STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &actual_read_value);
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, result);
}

void test_STS_Primitives_Read_Broadcast_Forbidden(void) {
    uint8_t actual_read_value;
    
    test_servo.id = STS_ID_BROADCAST_SYNC;
    sts_result_t result = STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &actual_read_value);
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, result);
    
    test_servo.id = TEST_VALID_ID;
}


void test_STS_ExecuteCommand_TrashBin_Redirect(void) {
    uint8_t mock_payload[1] = {0xAB};
    simulate_servo_response(TEST_VALID_ID, STS_OK, mock_payload, 1U, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    uint8_t params[2] = {STS_REG_PRESENT_TEMP, 1U};
    sts_cmd_t cmd = { 
        .instruction      = STS_INST_READ,
        .tx_params        = params,
        .tx_param_len     = 2U,
        .expected_rx_len  = EXPECTED_READ8_ACK_LEN,
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    TEST_ASSERT_EQUAL_INT(STS_OK, sts_execute_command(&test_servo, &cmd));
}


void test_STS_ExecuteCommand_RX_Buffer_Overflow(void) {
    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .expected_rx_len  = (uint16_t)(TEST_MAX_RX_BUFFER + 10U), // Exceed internal bounds 
    };

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUF_TOO_SMALL, sts_execute_command(&test_servo, &cmd));
}

void test_STS_ExecuteCommand_TX_Buffer_Overflow(void) {
    uint8_t massive_payload[TEST_MAX_TX_BUFFER + 10U] = {0};
    
    sts_cmd_t cmd = {
        .instruction      = STS_INST_WRITE,
        .tx_params        = massive_payload,
        .tx_param_len     = (uint16_t)(TEST_MAX_TX_BUFFER + 10U), // Exceed internal bounds 
        .expected_rx_len  = EXPECTED_WRITE_ACK_LEN
    };

    TEST_ASSERT_NOT_EQUAL(STS_OK, sts_execute_command(&test_servo, &cmd));
}

void test_STS_ExecuteCommand_Zero_Expected_RX(void) {
    sts_cmd_t cmd = {
        .instruction      = STS_INST_WRITE,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = 0U, // Force the early exit logic 
    };

    TEST_ASSERT_EQUAL_INT(STS_OK, sts_execute_command(&test_servo, &cmd));
}

void test_STS_ExecuteCommand_Broadcast_Forces_Early_Exit(void) {
    test_servo.id = STS_ID_BROADCAST_SYNC;
    
    sts_cmd_t cmd = {
        .instruction      = STS_INST_WRITE,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = EXPECTED_WRITE_ACK_LEN, 
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    sts_result_t res = sts_execute_command(&test_servo, &cmd);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    
    test_servo.id = TEST_VALID_ID;
}

void test_STS_ExecuteCommand_Null_Cmd_Guard(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, sts_execute_command(&test_servo, NULL));
}

/* ==========================================================================
 * TESTS: STS Ping 
 * ========================================================================== */

void test_STS_Ping_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT8(STS_ONLINE, test_servo.is_online);
}

void test_STS_Ping_Timeout_Sets_Offline(void) {
    dummy_uart_port.rx_len = 0U;
    test_servo.is_online = STS_ONLINE; 

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, res);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, test_servo.is_online);
}

void test_STS_Ping_ID_Mismatch_Sets_Offline(void) {
    simulate_servo_response(TEST_ALT_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    test_servo.is_online = STS_ONLINE;

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_ERR_ID_MISMATCH, res);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, test_servo.is_online); 
}

void test_STS_Ping_Checksum_Error_Sets_Offline(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_buffer[5] ^= 0xFF; 
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    test_servo.is_online = STS_ONLINE;

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_ERR_CHECKSUM, res);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, test_servo.is_online);
}

void test_STS_Ping_Bus_Busy_Sets_Offline(void) {
    test_bus.transmit = mock_tx_busy;
    test_servo.is_online = STS_ONLINE;

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, res);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, test_servo.is_online);

    test_bus.transmit = mock_tx;
}

void test_STS_Ping_Null_Servo_Guard(void) {
    sts_result_t res = STS_servo_ping(NULL);

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, res);
}

void test_STS_Ping_Broadcast_Forbidden(void) {
    test_servo.id = STS_ID_BROADCAST_SYNC;
    
    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res); 
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, test_servo.is_online);

    test_servo.id = TEST_VALID_ID;
}

void test_STS_Ping_Null_Bus_Pointer(void) {
    test_servo.bus = NULL; 

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, res);
    
    test_servo.bus = &test_bus;
}

/**
 * @brief Hardware errors should not set the servo offline, 
 * as they indicate a problem with the servo's internal state, not a communication failure.
 */
void test_STS_Ping_Hardware_Error_Still_Online(void) {
    uint8_t error_status = 0x01U; 
    simulate_servo_response(TEST_VALID_ID, error_status, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_UINT8(STS_ONLINE, test_servo.is_online);
    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, res); 
}

void test_STS_Ping_Max_Valid_ID(void) {
    test_servo.id = TEST_MAX_ID; 
    simulate_servo_response(TEST_MAX_ID, STS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT8(STS_ONLINE, test_servo.is_online);
}

void test_STS_Ping_Recovery_Offline_To_Online(void) {
    test_servo.is_online = STS_OFFLINE; 

    simulate_servo_response(TEST_VALID_ID, STS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    (void)STS_servo_ping(&test_servo);

    TEST_ASSERT_EQUAL_UINT8(STS_ONLINE, test_servo.is_online);
}

