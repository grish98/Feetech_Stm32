/**
 ******************************************************************************
 * @file           : test_sts_servo.c
 * @brief          : Unit tests for the STS Service and Bus Layers
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 0.1.0
 ******************************************************************************
 * @details
 * This module implements the test cases for the Feetech STS Service layer.
 * Initial focus is placed on the Bus Hardware Abstraction Layer (HAL),
 * ensuring that function pointers are correctly mapped, memory is 
 * initialized deterministically, and NULL pointer guards are enforced.
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

#define TEST_DATA_BYTE    0xAAU
#define TEST_DATA_LEN     1U
#define TEST_TIMEOUT_MS   100U

#define TEST_VALID_ID          10U
#define TEST_ALT_ID            20U
#define TEST_MIN_ID             0U
#define TEST_MAX_ID             253U


#define MOCK_BUS_ADDR_A         ((void*)0x1111)
#define MOCK_BUS_ADDR_B         ((void*)0x2222)

#define HW_STATUS_OK                 0x00U

#define EXPECTED_WRITE_ACK_LEN       6U
#define EXPECTED_READ8_ACK_LEN       7U
#define EXPECTED_READ16_ACK_LEN      8U

#define PAYLOAD_LEN_NONE             0U
#define PAYLOAD_LEN_1_BYTE           1U
#define PAYLOAD_LEN_2_BYTES          2U

#define TEST_VAL_8BIT                0x7FU
#define TEST_VAL_16BIT               0x1234U
#define TEST_VAL_16BIT_LOW           0x34U
#define TEST_VAL_16BIT_HIGH          0x12U

#define IDX_HEADER_START             0U
#define IDX_LENGTH_BYTE              3U
#define IDX_CHECKSUM_16BIT_READ      7U

#define CORRUPT_HEADER_BYTE          0xEEU
#define CORRUPT_CHECKSUM_MASK        0xFFU
#define FAKE_LONG_PACKET_LENGTH      10U
#define TRUNCATED_PACKET_LENGTH      2U

static sts_bus_t test_bus;
static sts_servo_t test_servo; 

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

    // Verify that the pointers in the bus are functional and correctly routed 
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