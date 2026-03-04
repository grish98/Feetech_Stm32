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
#include "test_sts_utils.h"

#define TEST_DATA_BYTE    0xAAU
#define TEST_DATA_LEN     1U
#define TEST_TIMEOUT_MS   100U

#define TEST_VALID_ID           10U
#define TEST_ALT_ID            20U
#define TEST_MIN_ID             0U
#define TEST_MAX_ID             253U
#define TEST_ID_BROADCAST       255U
#define TEST_ID_RESERVED        254U

#define MOCK_BUS_ADDR_A         ((void*)0x1111)
#define MOCK_BUS_ADDR_B         ((void*)0x2222)

void setUp(void) {

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
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &bus, TEST_ID_RESERVED));
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &bus, TEST_ID_BROADCAST));
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
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &bus, TEST_ID_BROADCAST));
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