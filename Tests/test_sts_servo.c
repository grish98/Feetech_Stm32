/**
 ******************************************************************************
 * @file           : test_sts_servo.c
 * @brief          : Unit tests for the STS Service and Bus Layers
 * @author         : Grisham Balloo
 ******************************************************************************
 * @details
 * This test suite provides verification for the Feetech STS Service Layer,
 * Command API, and Bus HAL. It utilises the Unity Test Framework to
 * validate the following functional areas:
 *
 * 1. Bus Initialisation: Validation of HAL function pointer mapping,
 * deterministic memory overwrite, and null-pointer guards.
 *
 * 2. Servo Handle Initialisation: Verification of ID boundary enforcement,
 * state reset guarantees, and multi-bus linkage.
 *
 * 3. Read/Write Primitives: Happy-path coverage for base registers.
 *
 * 4. Engine Robustness: Stress-testing sts_execute_command against null guards,
 * timeout, ID mismatch, checksum corruption, and buffer overflow.
 *
 * 5. Ping Service: Online/offline state management and hardware error tolerance.
 *
 * 6. Kinematics & Motion Control: Exhaustive boundary testing for Position,
 * Speed, Acceleration, PWM, and Step commands.
 *
 * 7. Telemetry & Feedback: Validation of load, voltage, temperature, and
 * moving status polling, including timeout state preservation.
 *
 * 8. Configuration & EEPROM: Strict verification of EEPROM lock states,
 * ternary boundary sanitisation, and valid ID assignments.
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
#include "sts_servo_cmd.h"
#include "sts_servo_internal.h"
#include <string.h>
#include <stddef.h>
#include <stdio.h>

/* ==========================================================================
 * TEST ENVIRONMENT & BUFFER LIMITS
 * ========================================================================== */
#define TEST_MAX_TX_BUFFER          128U
#define TEST_MAX_RX_BUFFER          128U
#define TEST_TIMEOUT_MS             10U
#define TEST_TIMEOUT_LONG_MS        100U

#define TEST_BUFFER_SIZE_SMALL      2U
#define TEST_BUFFER_SIZE_MED        4U
#define TEST_BUFFER_SIZE_LARGE      5U

/* ==========================================================================
 * MOCK HARDWARE & IDENTITY CONFIGURATIONS
 * ========================================================================== */
#define MOCK_PORT_HANDLE_A          ((void*)0x1111U)
#define MOCK_PORT_HANDLE_B          ((void*)0x2222U)
#define MOCK_GARBAGE_PTR_A          ((void*)0xDEADBEEFU)
#define MOCK_GARBAGE_PTR_B          (sts_hal_transmit_t)0xBAD0F00DU
#define MOCK_GARBAGE_PTR_C          (sts_hal_receive_t)0xCAFEBABEU

#define TEST_MIN_ID                 0U
#define TEST_VALID_ID               10U
#define TEST_ALT_ID                 20U
#define TEST_MAX_ID                 253U

/* ==========================================================================
 * PROTOCOL SIZING & EXPECTED LENGTHS
 * ========================================================================== */
#define PAYLOAD_LEN_NONE            0U
#define PAYLOAD_LEN_1_BYTE          1U
#define PAYLOAD_LEN_2_BYTES         2U

/* Expected RX Lengths = STS Base ACK (6 bytes) + Data Payload Length */
#define EXPECTED_WRITE_ACK_LEN      (6U + PAYLOAD_LEN_NONE)    /* 6U */
#define EXPECTED_READ8_ACK_LEN      (6U + PAYLOAD_LEN_1_BYTE)  /* 7U */
#define EXPECTED_READ16_ACK_LEN     (6U + PAYLOAD_LEN_2_BYTES) /* 8U */

/* ==========================================================================
 * RAW BYTE & REGISTER TEST VALUES
 * ========================================================================== */
#define HW_STATUS_OK                0x00U

#define TEST_DATA_BYTE              0xAAU
#define TEST_ALT_DATA_BYTE          0x55U
#define TEST_DUMMY_BYTE_FF          0xFFU
#define TEST_DATA_LEN               1U

#define TEST_VAL_8BIT               0x7FU
#define TEST_VAL_16BIT              0x1234U
#define TEST_VAL_16BIT_LOW          0x34U  /* Little-endian lower byte */
#define TEST_VAL_16BIT_HIGH         0x12U  /* Little-endian upper byte */

#define EXACTLY_ONE_CALL            1U
#define RESET_CALL_COUNT            0U

/* ==========================================================================
 * KINEMATIC TARGET BOUNDARIES & INTENTS
 * ========================================================================== */
#define TARGET_ZERO                 0

/* --- PWM Mode --- */
#define TARGET_VALID_PWM_CCW        500

/* --- Step Mode --- */
#define TARGET_VALID_STEP_CCW       25000

/* --- Expected WRITE16 Packet: FF FF id 05 03 addr lo hi cs --- */
#define EXPECTED_HEADER_BYTE        0xFFU
#define EXPECTED_WRITE16_LEN_BYTE   0x05U     /* Instruction + 3 params + checksum */
#define EXPECTED_INST_WRITE         0x03U
#define EXPECTED_WRITE16_PACKET_LEN 9U
#define EXPECTED_REG_GOAL_POSITION  0x2AU
#define EXPECTED_REG_GOAL_SPEED     0x2EU

/* --- Expected WRITE8 Packet: FF FF id 04 03 addr value cs --- */
#define EXPECTED_WRITE8_LEN_BYTE    0x04U     /* Instruction + 2 params + checksum */
#define EXPECTED_WRITE8_PACKET_LEN  8U
#define EXPECTED_REG_OPERATION_MODE 0x21U

/* --- Expected PING Packet: FF FF id 02 01 cs --- */
#define EXPECTED_PING_LEN_BYTE      0x02U
#define EXPECTED_INST_PING          0x01U

/* --- Nonzero Seeds: a write of zero must show up in the no-side-effect snapshot --- */
#define TEST_SEED_TRANSACTIONS      11U
#define TEST_SEED_RETRIES           22U
#define TEST_SEED_RETRY_SAVES       33U
#define TEST_SEED_HARD_FAILURES     44U

#define TARGET_VALID_TORQUE_LIMIT   500U

/* Mock Telemetry Expected Data */
#define TEST_MOCK_LOAD_VAL          1000   /* Raw expected integer */
#define TEST_MOCK_LOAD_LOW_BYTE     0xE8U  /* 1000 in Little Endian */
#define TEST_MOCK_LOAD_HIGH_BYTE    0x03U

#define TEST_MOCK_VOLTAGE_VAL       125U   /* Represents 12.5V */
#define TEST_MOCK_TEMP_VAL          50U    /* Represents 50°C */
#define TEST_MOCK_TEMP_OVERHEAT_VAL 85U    /* Represents 85°C */
#define TEST_MOCK_STATUS_STOPPED    0U     /* Motor not moving */

/* Sentinel Values for State Preservation Checks */
#define TEST_SENTINEL_MAX_U8        255U   /* Used to verify variable was overwritten */
#define TEST_SENTINEL_GARBAGE_U8    99U    /* Used to verify variable was NOT overwritten */
#define TEST_ZERO_RX_LEN            0U     /* Simulates disconnected line */

/* EEPROM & ID Test Values */
#define TEST_MOCK_NEW_ID            10U
#define TEST_MOCK_GARBAGE_LOCK      255U   

/* ==========================================================================
 * PACKET CORRUPTION & ERROR INJECTION CONSTANTS
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

void setUp(void) {
    memset(&test_bus, 0, sizeof(test_bus));
    memset(&test_servo, 0, sizeof(test_servo));
    memset(&dummy_uart_port, 0, sizeof(dummy_uart_port));

    (void)STS_Bus_Init(&test_bus, &dummy_uart_port, mock_tx, mock_rx);
    (void)STS_Servo_Init(&test_servo, &test_bus, TEST_VALID_ID);
}
void tearDown(void) {

}

void test_STS_Bus_Init_Success(void) {
    sts_bus_t bus = {0};
    sts_result_t res = STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);
    
    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&dummy_uart_port, bus.port_handle);
    TEST_ASSERT_EQUAL_PTR(mock_tx, bus.transmit);
    TEST_ASSERT_EQUAL_PTR(mock_rx, bus.receive);
    TEST_ASSERT_NOT_NULL(bus.transmit);
    TEST_ASSERT_NOT_NULL(bus.receive);
}

void test_init_fails_with_null_bus(void) {
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Bus_Init(NULL, &dummy_uart_port, mock_tx, mock_rx));
}

void test_STS_Bus_Init_Null_TX(void) {
    sts_bus_t bus = {0};
    bus.port_handle = MOCK_GARBAGE_PTR_A;
    
    sts_bus_t snapshot;
    memcpy(&snapshot, &bus, sizeof(sts_bus_t));

    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Bus_Init(&bus, &dummy_uart_port, NULL, mock_rx));
    TEST_ASSERT_STRUCT_UNCHANGED(&snapshot, &bus, sts_bus_t);
}

void test_STS_Bus_Init_Null_RX(void) {
    sts_bus_t bus = {0};
    bus.port_handle = MOCK_GARBAGE_PTR_A;
    
    sts_bus_t snapshot;
    memcpy(&snapshot, &bus, sizeof(sts_bus_t));

    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, NULL));
    TEST_ASSERT_STRUCT_UNCHANGED(&snapshot, &bus, sts_bus_t);
}

void test_STS_Bus_Null_TX(void) {
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Bus_Transmit(&test_bus, NULL, TEST_DATA_LEN));
}

void test_STS_Bus_NULL_RX(void) {
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Bus_Receive(&test_bus, NULL, TEST_DATA_LEN, TEST_TIMEOUT_MS));
}

void test_STS_Bus_TX_Null_Bus(void) {
    const uint8_t data[] = { TEST_DUMMY_BYTE_FF };
    sts_result_t result = STS_Bus_Transmit(NULL, data, sizeof(data));
    TEST_ASSERT_EQUAL_MESSAGE(STS_ERR_NULL_PTR, result, "Failed to reject NULL bus handle (TX)");
}

void test_STS_Bus_RX_Null_Bus(void) {
    uint8_t rx_buf[1];
    sts_result_t result = STS_Bus_Receive(NULL, rx_buf, 1, TEST_TIMEOUT_MS);
    TEST_ASSERT_EQUAL_MESSAGE(STS_ERR_NULL_PTR, result, " Failed to reject NULL bus handle (RX)");
}

void test_STS_Bus_Init_Null_Port(void) {
    sts_bus_t bus = {0};
    sts_result_t res = STS_Bus_Init(&bus, NULL, mock_tx, mock_rx);
    /*Allows Null Port Handle*/
    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(NULL, bus.port_handle);
}

void test_STS_Bus_Transmit_Null_Port(void) {
    sts_bus_t bus = {0};
    (void)STS_Bus_Init(&bus, NULL, mock_tx_bare_metal, mock_rx); 

    const uint8_t tx_data[] = { TEST_DATA_BYTE };
    dummy_uart_port.tx_call_count = RESET_CALL_COUNT;

    sts_result_t result = STS_Bus_Transmit(&bus, tx_data, TEST_DATA_LEN);
    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, dummy_uart_port.tx_call_count);
}

void test_STS_Bus_Init_Overwrites_Invalid(void) {
    sts_bus_t bus = {0};
    
    bus.port_handle = MOCK_GARBAGE_PTR_A;
    bus.transmit    = MOCK_GARBAGE_PTR_B;
    bus.receive     = MOCK_GARBAGE_PTR_C;

    sts_result_t res = STS_Bus_Init(&bus, &dummy_uart_port, mock_tx, mock_rx);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&dummy_uart_port, bus.port_handle);
    TEST_ASSERT_EQUAL_PTR(mock_tx, bus.transmit);
    TEST_ASSERT_EQUAL_PTR(mock_rx, bus.receive);
}

void test_STS_Bus_Reinitialisation(void) {
    sts_bus_t bus = {0};
    
    mock_uart_t mock_uart_1 = {0};
    mock_uart_t mock_uart_2 = {0};

    (void)STS_Bus_Init(&bus, &mock_uart_1, mock_tx, mock_rx);
    sts_result_t res = STS_Bus_Init(&bus, &mock_uart_2, mock_tx, mock_rx);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&mock_uart_2, bus.port_handle);
}

void test_STS_Bus_Transmit_Routes_To_HAL(void) {

    uint8_t tx_data = STS_HEADER;
    sts_result_t result = STS_Bus_Transmit(&test_bus, &tx_data, TEST_DATA_LEN);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT16(TEST_DATA_LEN, dummy_uart_port.tx_len);
    TEST_ASSERT_EQUAL_HEX8(tx_data, dummy_uart_port.tx_buffer[0]);
}

void test_STS_Bus_Receive_Pulls_From_HAL(void) {
    uint8_t rx_data = TEST_ALT_DATA_BYTE;
    dummy_uart_port.rx_buffer[0] = rx_data;
    dummy_uart_port.rx_len = TEST_DATA_LEN;

    uint8_t actual_received_byte = 0x00U;
    sts_result_t result = STS_Bus_Receive(&test_bus, &actual_received_byte, TEST_DATA_LEN, TEST_TIMEOUT_MS);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_HEX8(rx_data, actual_received_byte);
}

void test_STS_Bus_Tx_Valid_Payload(void) {
    const uint8_t data[] = { STS_HEADER, STS_HEADER, 0x01U, 0x02U, 0x01U, 0xFBU };
    const uint16_t payload_length = (uint16_t)sizeof(data);

    sts_result_t result = STS_Bus_Transmit(&test_bus, data, payload_length);

    TEST_ASSERT_EQUAL(STS_OK, result);
    
    TEST_ASSERT_EQUAL_UINT16(payload_length, dummy_uart_port.tx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, dummy_uart_port.tx_buffer, payload_length);
}

void test_STS_Bus_Rx_Valid_Payload(void) {
    dummy_uart_port.last_timeout_ms = 0U;
    const uint8_t sts_response_ok[] = { STS_HEADER, STS_HEADER, 0x01, 0x02, 0x00, 0xFC };
    const uint16_t response_length = (uint16_t)sizeof(sts_response_ok);
    
    memcpy(dummy_uart_port.rx_buffer, sts_response_ok, response_length);
    dummy_uart_port.rx_len = response_length;
    uint8_t actual_received_data[sizeof(sts_response_ok)];
    POISON_BUFFER(actual_received_data, response_length, TEST_DUMMY_BYTE_EE);

    sts_result_t result = STS_Bus_Receive(&test_bus, actual_received_data, response_length, TEST_TIMEOUT_LONG_MS);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT32(TEST_TIMEOUT_LONG_MS, dummy_uart_port.last_timeout_ms);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(sts_response_ok, actual_received_data, response_length);
}

void test_Bus_Tx_Fail(void) {
    const uint8_t tx_data[] = { 0x11U, 0x22U };
    const uint16_t payload_length = (uint16_t)sizeof(tx_data);

    test_bus.transmit = mock_tx_fail;
    sts_result_t result = STS_Bus_Transmit(&test_bus, tx_data, payload_length);
    TEST_ASSERT_EQUAL(STS_ERR_TX_FAIL, result);
}

void test_STS_Bus_Rx_Hal_Timeout(void) {
    uint8_t actual_received_data[TEST_BUFFER_SIZE_SMALL]; 
    POISON_BUFFER(actual_received_data, TEST_BUFFER_SIZE_SMALL, TEST_DUMMY_BYTE_EE);
    const uint16_t requested_length = (uint16_t)sizeof(actual_received_data);
    
    sts_result_t result = STS_Bus_Receive(&test_bus, actual_received_data, requested_length, TEST_TIMEOUT_MS);

    TEST_ASSERT_EQUAL(STS_ERR_TIMEOUT, result);
    TEST_ASSERT_BUFFER_UNCHANGED(actual_received_data, TEST_BUFFER_SIZE_SMALL, TEST_DUMMY_BYTE_EE);
}

void test_STS_Bus_Rx_Partial(void) {
    const uint8_t partial[] = { 0xAAU, 0xBBU };
    memcpy(dummy_uart_port.rx_buffer, partial, sizeof(partial));
    dummy_uart_port.rx_len = sizeof(partial);
    uint8_t rx_buffer[4]; 
    POISON_BUFFER(rx_buffer, 4, TEST_DUMMY_BYTE_EE);

    sts_result_t res = STS_Bus_Receive(&test_bus, rx_buffer, 4, TEST_TIMEOUT_MS);

    TEST_ASSERT_EQUAL(STS_ERR_TIMEOUT, res);
    TEST_ASSERT_BUFFER_UNCHANGED(rx_buffer, 4, TEST_DUMMY_BYTE_EE);
}


void test_STS_Bus_Transmit_Zero_Length_NoOp(void) {
    const uint8_t data[] = { TEST_DUMMY_BYTE_FF };
    dummy_uart_port.tx_call_count = RESET_CALL_COUNT;

    sts_result_t result = STS_Bus_Transmit(&test_bus, data, PAYLOAD_LEN_NONE);

    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT32(RESET_CALL_COUNT, dummy_uart_port.tx_call_count);
}

void test_STS_Bus_Receive_Zero_Length_NoOp(void) {
    uint8_t buffer[TEST_DATA_LEN] = { TEST_DATA_BYTE }; 
    dummy_uart_port.rx_call_count = RESET_CALL_COUNT;
    
    sts_result_t result = STS_Bus_Receive(&test_bus, buffer, PAYLOAD_LEN_NONE, TEST_TIMEOUT_LONG_MS);
    TEST_ASSERT_EQUAL(STS_OK, result);
    TEST_ASSERT_EQUAL_UINT32(RESET_CALL_COUNT, dummy_uart_port.rx_call_count);
    TEST_ASSERT_EQUAL_HEX8(TEST_DATA_BYTE, buffer[0]); 
}

void test_STS_Bus_Transmit_Null_Function_Pointer_Guarded(void) {
    sts_bus_t uninit_bus = {0}; 
    const uint8_t data[] = { TEST_DUMMY_BYTE_FF };

    sts_result_t result = STS_Bus_Transmit(&uninit_bus, data, TEST_DATA_LEN);
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, result);
}

/*==========================================
 * TESTS:STS_Servo_Init
 * ========================================== */

void test_STS_Servo_Init_Success(void) {
    sts_servo_t servo = {0};

    (void)STS_Bus_Init(&test_bus, &dummy_uart_port, mock_tx, mock_rx);
    sts_result_t res = STS_Servo_Init(&servo, &test_bus, TEST_VALID_ID);

    TEST_ASSERT_EQUAL(STS_OK, res);
    TEST_ASSERT_EQUAL_PTR(&test_bus, servo.bus);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, servo.id);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, servo.is_online); 
}

void test_STS_Servo_Init_Null_Servo(void) {
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Servo_Init(NULL, &test_bus, TEST_VALID_ID));
}

void test_STS_Servo_Init_Null_Bus(void) {
    sts_servo_t servo = {0};
    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, STS_Servo_Init(&servo, NULL, TEST_VALID_ID));
}

void test_STS_Servo_Init_BroadcastSync_ID_Rejected(void) {
    sts_servo_t servo = {0};
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &test_bus, STS_ID_BROADCAST_SYNC));
}

void test_STS_Servo_Init_BroadcastAsync_ID_Rejected(void) {
    sts_servo_t servo = {0};
    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, STS_Servo_Init(&servo, &test_bus, STS_ID_BROADCAST_ASYNC));
}

void test_STS_Servo_Init_MinID_Accepted(void) {
    sts_servo_t servo = {0};
    TEST_ASSERT_EQUAL(STS_OK, STS_Servo_Init(&servo, &test_bus, TEST_MIN_ID));
    TEST_ASSERT_EQUAL_UINT8(TEST_MIN_ID, servo.id);
}

void test_STS_Servo_Init_MaxID_Accepted(void) {
    sts_servo_t servo = {0};
    TEST_ASSERT_EQUAL(STS_OK, STS_Servo_Init(&servo, &test_bus, TEST_MAX_ID));
    TEST_ASSERT_EQUAL_UINT8(TEST_MAX_ID, servo.id);
}

void test_STS_Servo_Init_Clears_Handle_State(void) {
    sts_servo_t servo = {0};
    (void)STS_Bus_Init(&test_bus, &dummy_uart_port, mock_tx, mock_rx);

    servo.bus = MOCK_GARBAGE_PTR_A;
    servo.id = TEST_DATA_BYTE;
    servo.is_online = STS_ONLINE;

    (void)STS_Servo_Init(&servo, &test_bus, TEST_VALID_ID);

    TEST_ASSERT_EQUAL_PTR(&test_bus, servo.bus);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, servo.id);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, servo.is_online);
}

void test_STS_Servo_Init_Atomic_Failure(void) {
    sts_servo_t servo = {0};
    sts_servo_t servo_snapshot = {0}; 

    (void)STS_Bus_Init(&test_bus, &dummy_uart_port, mock_tx, mock_rx);
    (void)STS_Servo_Init(&servo, &test_bus, TEST_VALID_ID);
    
    servo.is_online = STS_ONLINE; 

    memcpy(&servo_snapshot, &servo, sizeof(sts_servo_t));

    sts_result_t res = STS_Servo_Init(&servo, &test_bus, STS_ID_BROADCAST_ASYNC);

    TEST_ASSERT_EQUAL(STS_ERR_INVALID_PARAM, res);
    TEST_ASSERT_STRUCT_UNCHANGED(&servo_snapshot, &servo, sts_servo_t);
}

void test_STS_Servo_Init_Reset_Online_Status(void) {
    sts_servo_t servo = {0};
    (void)STS_Bus_Init(&test_bus, &dummy_uart_port, mock_tx, mock_rx);

    (void)STS_Servo_Init(&servo, &test_bus, TEST_VALID_ID);
    servo.is_online = STS_ONLINE; 

    // Re-init must reset status to offline 
    (void)STS_Servo_Init(&servo, &test_bus, TEST_VALID_ID);
    TEST_ASSERT_EQUAL_UINT8(STS_OFFLINE, servo.is_online);
}

void test_STS_Servo_Init_Multi_Bus_Link(void) {
    sts_bus_t bus_a = {0}, bus_b = {0};
    sts_servo_t servo_a = {0}, servo_b = {0};
    
    (void)STS_Bus_Init(&bus_a, MOCK_PORT_HANDLE_A, mock_tx, mock_rx);
    (void)STS_Bus_Init(&bus_b, MOCK_PORT_HANDLE_B, mock_tx, mock_rx);

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

void test_STS_Write8_Rejects_Null_Servo(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Write8(NULL, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
}

void test_STS_Write16_Rejects_Null_Servo(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Write16(NULL, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));
}

void test_STS_Read8_Rejects_Null_Servo(void) {
    uint8_t val8;
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read8(NULL, STS_REG_PRESENT_VOLTAGE, &val8));
}

void test_STS_Read16_Rejects_Null_Servo(void) {
    uint16_t val16;
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read16(NULL, STS_REG_PRESENT_POSITION, &val16));
}

void test_STS_Read8_Rejects_Null_Out_Pointer(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, NULL));
}

void test_STS_Read16_Rejects_Null_Out_Pointer(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, NULL));
}
void test_STS_Write8_Yields_Timeout(void) {
    dummy_uart_port.rx_len = 0U;
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Write8(&test_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
}

void test_STS_Write16_Yields_Timeout(void) {
    dummy_uart_port.rx_len = 0U;
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));
}

void test_STS_Read8_Yields_Timeout(void) {
    uint8_t val8;
    dummy_uart_port.rx_len = 0U;
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &val8));
}

void test_STS_Read16_Yields_Timeout(void) {
    uint16_t val16;
    dummy_uart_port.rx_len = 0U;
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &val16));
}

/* ==========================================================================
 * TESTS: STS Primitives - Data Integrity & Hardware Faults
 * ========================================================================== */

void test_STS_Write16_Yields_ID_Mismatch(void) {
    simulate_servo_response(TEST_ALT_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_ID_MISMATCH, STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));
}

void test_STS_Read16_Yields_Checksum_Error(void) {
    uint16_t val16;
    uint8_t mock_payload[PAYLOAD_LEN_2_BYTES] = {TEST_VAL_16BIT_LOW, TEST_VAL_16BIT_HIGH};

    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_buffer[IDX_CHECKSUM_16BIT_READ] ^= CORRUPT_CHECKSUM_MASK; 
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_CHECKSUM, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &val16));
}

void test_STS_Write8_Yields_Bus_Busy(void) {
    test_bus.transmit = mock_tx_busy; 
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, STS_Write8(&test_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));

}

void test_STS_Read16_Yields_TX_Fail(void) {
    uint16_t dummy_val; 
    test_bus.transmit = mock_tx_fail; 
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &dummy_val));
}

void test_STS_Read8_Yields_Bus_Busy(void) {
    uint8_t dummy_val;
    test_bus.transmit = mock_tx_busy; 
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &dummy_val));
}

void test_STS_Write16_Yields_TX_Fail(void) {
    test_bus.transmit = mock_tx_fail; 
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT));
}

void test_STS_ExecuteCommand_Null_Servo_Guard(void) {
    sts_cmd_t cmd = { .instruction = STS_INST_PING };
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, sts_execute_command(NULL, &cmd));
}

void test_STS_ExecuteCommand_Header_Corruption(void) {
    uint8_t val8;
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {HW_STATUS_OK};

    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_buffer[IDX_HEADER_START] = CORRUPT_HEADER_BYTE;
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_ERR_HEADER, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &val8));
}

void test_STS_ExecuteCommand_Rejects_Uninitialised_Bus_Pointers(void) {
    sts_bus_t corrupted_bus = {0};
    corrupted_bus.port_handle = &dummy_uart_port; 
    
    test_servo.bus = &corrupted_bus;
    
    sts_cmd_t cmd = { .instruction = STS_INST_PING };
   
    sts_result_t result = sts_execute_command(&test_servo, &cmd);

    TEST_ASSERT_EQUAL(STS_ERR_NULL_PTR, result);
}
void test_STS_ExecuteCommand_Length_Mismatch(void) {
    uint8_t dummy_payload[STS_DATA_LEN_16BIT] = {0};
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, dummy_payload, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    dummy_uart_port.rx_buffer[STS_IDX_LENGTH] = 0x00U;// Below minimum valid length for a 16-bit read ACK

    sts_cmd_t cmd = {
        .instruction     = STS_INST_READ,
        .expected_rx_len = EXPECTED_READ16_ACK_LEN
    };

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, sts_execute_command(&test_servo, &cmd));
}

void test_STS_ExecuteCommand_Truncated_Packet(void) {
    uint8_t actual_read_value;
    
    dummy_uart_port.rx_buffer[IDX_HEADER_START] = STS_HEADER;
    dummy_uart_port.rx_buffer[IDX_HEADER_START + 1] = STS_HEADER;
    dummy_uart_port.rx_len = TRUNCATED_PACKET_LENGTH; 

    sts_result_t result = STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &actual_read_value);
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, result);
}

void test_STS_Primitives_Read_Broadcast_Forbidden(void) {
    sts_servo_t local_servo = test_servo; 
    local_servo.id = STS_ID_BROADCAST_SYNC; 
    
    uint8_t actual_read_value;
    sts_result_t result = STS_Read8(&local_servo, STS_REG_PRESENT_VOLTAGE, &actual_read_value);
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, result);
   
}

void test_STS_ExecuteCommand_Null_RxOut_Discards_Payload(void) {
    uint8_t mock_payload[1] = {0xAB};
    
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, 1U, dummy_uart_port.rx_buffer);
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

void test_STS_ExecuteCommand_TX_Buffer_Overflow(void) {
    uint8_t massive_payload[STS_MAX_TX_BUFFER + 1U] = {0}; 
    sts_cmd_t cmd = {
        .instruction     = STS_INST_WRITE,
        .tx_params       = massive_payload,
        .tx_param_len    = (uint16_t)(STS_MAX_TX_BUFFER + 1U),
        .expected_rx_len = EXPECTED_WRITE_ACK_LEN
    };
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUF_TOO_SMALL, sts_execute_command(&test_servo, &cmd));
}


void test_STS_ExecuteCommand_RX_Buffer_Overflow(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH] = 0xFEU; // Set length to exceed STS_MAX_RX_BUFFER

    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = (uint16_t)(STS_MAX_RX_BUFFER + 10U),
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };
    test_bus.total_transactions = TEST_SEED_TRANSACTIONS;
    test_bus.total_retries      = TEST_SEED_RETRIES;
    test_bus.retry_saves        = TEST_SEED_RETRY_SAVES;
    test_bus.hard_failures      = TEST_SEED_HARD_FAILURES;

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, sts_execute_command(&test_servo, &cmd));
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, dummy_uart_port.tx_call_count);
    TEST_ASSERT_EQUAL_UINT32(TEST_SEED_TRANSACTIONS + 1U, test_bus.total_transactions);
    TEST_ASSERT_EQUAL_UINT32(TEST_SEED_RETRIES, test_bus.total_retries);
    TEST_ASSERT_EQUAL_UINT32(TEST_SEED_RETRY_SAVES, test_bus.retry_saves);
    TEST_ASSERT_EQUAL_UINT32(TEST_SEED_HARD_FAILURES + 1U, test_bus.hard_failures);
}


/**
 * @brief Tests the engine's early-exit invariant for asynchronous commands.
 *
 * @details In the STS protocol, certain operations (like SYNC_WRITE, REG_WRITE,
 * or ACTION) do not return a status packet to prevent bus collisions. 
 * The engine contract states that if `expected_rx_len` is 0, the engine MUST 
 * transmit the packet but MUST completely bypass the Stage 1 and Stage 2 
 * receive logic, returning STS_OK immediately to free the bus thread.
 */
void test_STS_ExecuteCommand_Zero_Expected_RX(void) {
    sts_cmd_t cmd = {
        .instruction      = STS_INST_WRITE,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = 0U, /* The invariant trigger */
    };

    dummy_uart_port.rx_call_count = RESET_CALL_COUNT;

    TEST_ASSERT_EQUAL_INT(STS_OK, sts_execute_command(&test_servo, &cmd));
    
    TEST_ASSERT_EQUAL_UINT32(RESET_CALL_COUNT, dummy_uart_port.rx_call_count);
}

void test_STS_ExecuteCommand_Standard_Write_Safety(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t result = STS_Write16(&test_servo, STS_REG_GOAL_POSITION, TEST_VAL_16BIT); 
    
    TEST_ASSERT_EQUAL_INT(STS_OK, result);

    TEST_ASSERT_EQUAL_HEX8(STS_HEADER, dummy_uart_port.tx_buffer[STS_IDX_HEADER_1]);
    TEST_ASSERT_EQUAL_HEX8(STS_HEADER, dummy_uart_port.tx_buffer[STS_IDX_HEADER_2]);
    TEST_ASSERT_EQUAL_HEX8(TEST_VALID_ID, dummy_uart_port.tx_buffer[STS_IDX_ID]);
    TEST_ASSERT_EQUAL_HEX8(STS_INST_WRITE, dummy_uart_port.tx_buffer[STS_IDX_INSTRUCTION]);
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

    dummy_uart_port.rx_call_count = RESET_CALL_COUNT;

    sts_result_t res = sts_execute_command(&test_servo, &cmd);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT32(RESET_CALL_COUNT, dummy_uart_port.rx_call_count);
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

/* ==========================================================================
 * TESTS: STS Puplic API 
 * ========================================================================== */

 void test_STS_SetTorqueEnable_Success(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN; 

    sts_result_t res = STS_SetTorqueEnable(&test_servo, 1U); 

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTorqueEnable_Disable(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN; 

    sts_result_t res = STS_SetTorqueEnable(&test_servo, 0U); 

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTorqueEnable_NonStandard_True(void) {
    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN; 

    sts_result_t res = STS_SetTorqueEnable(&test_servo, 255U); 

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTorqueEnable_Null_Pointer(void) {
    sts_result_t res = STS_SetTorqueEnable(NULL, 1U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, res);
}

void test_STS_SetTorqueEnable_Error_Propagation(void) {
    uint8_t mock_hardware_error = 0x20; 
    simulate_servo_response(TEST_VALID_ID, mock_hardware_error, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN; 

    sts_result_t res = STS_SetTorqueEnable(&test_servo, 1U); 

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, res);
}

/* ==========================================================================
 * TARGET COMMAND ASSERTION HELPERS
 * ========================================================================== */

static void queue_write_ack(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
}

/* Exactly one WRITE16 of reg_value to reg was sent; pins address, value, length and checksum. */
static void assert_single_write16(uint8_t reg, uint16_t reg_value) {
    const uint8_t lo = (uint8_t)(reg_value & 0xFFU);
    const uint8_t hi = (uint8_t)(reg_value >> 8);
    const uint8_t expected[EXPECTED_WRITE16_PACKET_LEN] = {
        EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, TEST_VALID_ID, EXPECTED_WRITE16_LEN_BYTE,
        EXPECTED_INST_WRITE, reg, lo, hi,
        (uint8_t)~(TEST_VALID_ID + EXPECTED_WRITE16_LEN_BYTE + EXPECTED_INST_WRITE + reg + lo + hi)
    };

    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, dummy_uart_port.tx_call_count);
    TEST_ASSERT_EQUAL_UINT16(EXPECTED_WRITE16_PACKET_LEN, dummy_uart_port.tx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dummy_uart_port.tx_buffer, EXPECTED_WRITE16_PACKET_LEN);
}

static sts_bus_t   no_effect_bus;
static sts_servo_t no_effect_servo;

/* Before a call that must be rejected: seed nonzero counters and poisoned buffers, then snapshot bus and handle. */
static void arm_no_effect_check(void) {
    test_bus.total_transactions = TEST_SEED_TRANSACTIONS;
    test_bus.total_retries      = TEST_SEED_RETRIES;
    test_bus.retry_saves        = TEST_SEED_RETRY_SAVES;
    test_bus.hard_failures      = TEST_SEED_HARD_FAILURES;
    POISON_BUFFER(test_bus.tx_buf, sizeof(test_bus.tx_buf), TEST_DATA_BYTE);
    POISON_BUFFER(test_bus.rx_buf, sizeof(test_bus.rx_buf), TEST_ALT_DATA_BYTE);
    test_servo.is_online = STS_ONLINE;

    memcpy(&no_effect_bus, &test_bus, sizeof(sts_bus_t));
    memcpy(&no_effect_servo, &test_servo, sizeof(sts_servo_t));
}

/* After the rejected call: no bus traffic; bus and handle byte-identical to the snapshot. */
static void assert_no_effect(void) {
    TEST_ASSERT_EQUAL_UINT32(RESET_CALL_COUNT, dummy_uart_port.tx_call_count);
    TEST_ASSERT_EQUAL_UINT32(RESET_CALL_COUNT, dummy_uart_port.rx_call_count);
    TEST_ASSERT_STRUCT_UNCHANGED(&no_effect_bus, &test_bus, sts_bus_t);
    TEST_ASSERT_STRUCT_UNCHANGED(&no_effect_servo, &test_servo, sts_servo_t);
}

/* ==========================================================================
 *  Possition COMMANDS
 * ========================================================================== */

void test_STS_Position_API_Null_Guards(void) {
    uint16_t pos = 0U;
    const uint16_t dummy_pos = 100U;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetPosition(NULL, dummy_pos));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentPosition(NULL, &pos));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentPosition(&test_servo, NULL));
}

void test_STS_SetTargetPosition_Success(void) {
    const uint16_t target_pos = 2048U; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPosition(&test_servo, target_pos);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0x0800U);
}

void test_STS_SetTargetPosition_Hardware_Fault(void) {
    const uint16_t target_pos = 2048U; 
    const uint8_t hw_fault_code = STS_BIT_ERR_LOAD; 
    
    simulate_servo_response(TEST_VALID_ID, hw_fault_code, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPosition(&test_servo, target_pos);
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, res);
}

void test_STS_SetTargetPosition_Out_Of_Range(void) {
    const uint16_t out_of_range_pos = 5000U;

    sts_result_t res = STS_SetTargetPosition(&test_servo, out_of_range_pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTargetPosition_Broadcast_No_Wait(void) {
    const uint16_t target_pos = 2048U;
    
    test_servo.id = STS_ID_BROADCAST_SYNC;
    dummy_uart_port.rx_call_count = 0U; // Reset counter to ensure no reads occur 

    sts_result_t res = STS_SetTargetPosition(&test_servo, target_pos);

    // Broadcast commands must return immediately without waiting for an ACK 
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT32(0U, dummy_uart_port.rx_call_count); 
    
    test_servo.id = TEST_VALID_ID;
}

void test_STS_GetPresentPosition_Success(void) {
    const uint16_t expected_pos = 1024U; 
  
    uint8_t pos_data[STS_DATA_LEN_16BIT] = { 
        (uint8_t)(expected_pos & 0xFFU), 
        (uint8_t)((expected_pos >> 8U) & 0xFFU) 
    };
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    uint16_t actual_pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &actual_pos);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT16(expected_pos, actual_pos);
}

void test_STS_GetPresentPosition_Endianness(void) {
    const uint16_t endian_test_val = 511U; // 0x01FF in Hex 
    
    // map Low Byte (0xFF) then High Byte (0x01) 
    uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0xFFU, 0x01U };
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    uint16_t actual_pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &actual_pos);

    //If bytes were read Big-Endian, this would equal 65281 (0xFF01) instead of 511 
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT16(endian_test_val, actual_pos);
}

void test_STS_GetPresentPosition_Fragmented_Packet(void) {
    uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL - 1U; // ! less than the full packet length to trigger timeout in Stage 1

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    // Stage 1 (Header Read) should time out immediately.
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, res);
}

void test_STS_GetPresentPosition_Bad_Checksum(void) {
    uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U }; 
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    
   //Intentionally flip the bits of the final Checksum byte 
    const uint16_t checksum_idx = EXPECTED_READ16_ACK_LEN - 1U;
    dummy_uart_port.rx_buffer[checksum_idx] ^= CORRUPT_CHECKSUM_MASK; 
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_CHECKSUM, res);
}

void test_STS_GetPresentPosition_Zero_Length_Fault(void) {
    uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH] = 0x00U; 

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, res);
}

void test_STS_GetPresentPosition_Buffer_Overflow_Guard(void) {
uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH] = 0xFEU; // Set length to exceed STS_MAX_RX_BUFFER

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, res);
    TEST_ASSERT_EQUAL_UINT16(0U, pos);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.hard_failures);
}

void test_STS_GetPresentPosition_Wrong_ID_Response(void) {
    const uint8_t wrong_id = TEST_ALT_ID; 
    uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    
    simulate_servo_response(wrong_id, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_ID_MISMATCH, res);
}

void test_STS_SetTargetPosition_Min_Boundary(void) {
    const uint16_t min_pos = 0U; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPosition(&test_servo, min_pos);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0x0000U);
}

void test_STS_SetTargetPosition_Max_Boundary(void) {
    const uint16_t max_pos = 4095U;
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPosition(&test_servo, max_pos);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0x0FFFU);
}

void test_STS_SetTargetPosition_Just_Out_Of_Range(void) {
    const uint16_t off_by_one_pos = 4096U;

    arm_no_effect_check();
    sts_result_t res = STS_SetTargetPosition(&test_servo, off_by_one_pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
    assert_no_effect();
}

void test_STS_SetTargetPosition_Rejects_Value_Wider_Than_16_Bits(void) {
    const uint32_t wide_pos     = 0x00010064U; /* 65636: the low 16 bits are 100, a valid position */
    const int32_t  negative_pos = -130972;     /* 0xFFFE0064 once unsigned: the low 16 bits are 100 */

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetPosition(&test_servo, wide_pos));
    assert_no_effect();

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetPosition(&test_servo, (uint32_t)negative_pos));
    assert_no_effect();
}

void test_STS_SetTargetPosition_Bus_Busy(void) {
    const uint16_t target_pos = 2048U;
    
    test_bus.transmit = mock_tx_busy; 

    sts_result_t res = STS_SetTargetPosition(&test_servo, target_pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, res);

    test_bus.transmit = mock_tx; 
}

void test_STS_GetPresentPosition_Broadcast_Forbidden(void) {
    uint16_t pos = 0U;

    test_servo.id = STS_ID_BROADCAST_SYNC;

    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);

    test_servo.id = TEST_VALID_ID;
}

void test_STS_GetPresentPosition_Payload_Length_Mismatch(void) {
    const uint8_t short_payload_len = STS_DATA_LEN_8BIT; 
    uint8_t pos_data[1] = { 0xFFU }; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, short_payload_len, dummy_uart_port.rx_buffer);
    
    dummy_uart_port.rx_len = STS_ACK_BASE_LEN + short_payload_len;

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, res);
}

void test_STS_GetPresentPosition_Stage2_Timeout(void) {
uint8_t pos_data[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL; 

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    // Stage 1 (Header Read) should succeed, but Stage 2 (Payload Read) should time out.
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, res);
}

/* ==========================================================================
 *  Speed & Acceleratiion COMMANDS
 * ========================================================================== */
void test_STS_Speed_Accel_Null_Guards(void) {
    uint16_t speed = 0U;
    const uint16_t dummy_speed = 1000U;
    const uint8_t dummy_accel = 50U;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetSpeed(NULL, dummy_speed, STS_DIR_CCW));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentSpeed(NULL, &speed));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentSpeed(&test_servo, NULL));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetAcceleration(NULL, dummy_accel));
}

void test_STS_SetTargetSpeed_Success(void) {
    const uint16_t target_speed = 2000U; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, target_speed, STS_DIR_CCW);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x07D0U);
}

void test_STS_SetTargetSpeed_Out_Of_Range(void) {
    const uint16_t out_of_range_speed = STS_MAX_SPEED + 1U;

    arm_no_effect_check();
    sts_result_t res = STS_SetTargetSpeed(&test_servo, out_of_range_speed, STS_DIR_CCW);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
    assert_no_effect();
}

void test_STS_SetTargetSpeed_Rejects_Value_Wider_Than_16_Bits(void) {
    const uint32_t wide_speed     = 0x000103E8U; /* 66536: the low 16 bits are 1000, a valid speed */
    const int32_t  negative_speed = -130072;     /* 0xFFFE03E8 once unsigned: the low 16 bits are 1000 */

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetSpeed(&test_servo, wide_speed, STS_DIR_CCW));
    assert_no_effect();

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM,
                          STS_SetTargetSpeed(&test_servo, (uint32_t)negative_speed, STS_DIR_CCW));
    assert_no_effect();
}

void test_STS_GetPresentSpeed_Success(void) {
    int16_t actual_speed = 0;
    uint8_t mock_payload[STS_DATA_LEN_16BIT];

    /* Test Positive Speed (CCW / Bit 15 = 0)
     * Raw encoder register: magnitude 1500 ticks/s (0x05DC), bit 15 = 0 (CCW).
     * Decoded = 1500 / 2 = 750 position steps/s (encoder runs at 2x position resolution). */
    const uint16_t raw_bus_speed_ccw = 0x05DCU;

    mock_payload[0] = (uint8_t)(raw_bus_speed_ccw & 0xFFU);
    mock_payload[1] = (uint8_t)((raw_bus_speed_ccw >> 8U) & 0xFFU);

    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentSpeed(&test_servo, &actual_speed));
    TEST_ASSERT_EQUAL_INT16(750, actual_speed);

    /* Test Negative Speed (CW / Bit 15 = 1)
     * Raw encoder register: magnitude 1500 ticks/s (0x05DC), bit 15 = 1 (CW) = 0x85DC.
     * Decoded = -(1500 / 2) = -750 position steps/s. */
    const uint16_t raw_bus_speed_cw = 0x85DCU;

    mock_payload[0] = (uint8_t)(raw_bus_speed_cw & 0xFFU);
    mock_payload[1] = (uint8_t)((raw_bus_speed_cw >> 8U) & 0xFFU);

    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentSpeed(&test_servo, &actual_speed));
    TEST_ASSERT_EQUAL_INT16(-750, actual_speed);
}

void test_STS_SetTargetAcceleration_Success(void) {
    const uint8_t target_accel = 50U; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetAcceleration(&test_servo, target_accel);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetAcceleration_Out_Of_Range(void) {
    const uint16_t out_of_range_accel = STS_MAX_ACCELERATION + 1U;

    sts_result_t res = STS_SetTargetAcceleration(&test_servo, (uint8_t)out_of_range_accel);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTargetSpeed_Max_Boundary(void) {
    const uint16_t max_speed = STS_MAX_SPEED; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, max_speed, STS_DIR_CCW);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x0C01U);
}

void test_STS_SetTargetAcceleration_Max_Boundary(void) {
    const uint8_t max_accel = STS_MAX_ACCELERATION; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetAcceleration(&test_servo, max_accel);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetSpeed_Min_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, 0U, STS_DIR_CCW);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x0000U);
}

void test_STS_SetTargetAcceleration_Min_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetAcceleration(&test_servo, 0U);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetSpeed_Reverse_Direction_Allowed(void) {
    const uint16_t valid_speed = 1000U; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, valid_speed, STS_DIR_CW);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x83E8U);
}

void test_STS_SetTargetSpeed_Max_Reverse_Allowed(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, STS_MAX_SPEED, STS_DIR_CW);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x8C01U);
}

void test_STS_SetOperatingMode_Null_Guard(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetOperatingMode(NULL, STS_MODE_POSITION));
}

void test_STS_SetOperatingMode_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetOperatingMode(&test_servo, STS_MODE_SPEED);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetOperatingMode_Invalid_Mode(void) {
    sts_operating_mode_t invalid_mode = (sts_operating_mode_t)4U;

    sts_result_t res = STS_SetOperatingMode(&test_servo, invalid_mode);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

/* Each mode reaches the servo as its own value: one WRITE8 to 0x21, whole packet pinned. */
void test_STS_SetOperatingMode_Writes_Each_Mode(void) {
    static const struct { const char *name; sts_operating_mode_t mode; uint8_t wire_value; } cases[] = {
        {"position", STS_MODE_POSITION, 0x00U}, {"speed", STS_MODE_SPEED, 0x01U},
        {"pwm", STS_MODE_PWM, 0x02U}, {"step", STS_MODE_STEP, 0x03U},
    };

    for (uint32_t i = 0U; i < (uint32_t)(sizeof(cases) / sizeof(cases[0])); i++) {
        const uint8_t value = cases[i].wire_value;
        const uint8_t expected[EXPECTED_WRITE8_PACKET_LEN] = {
            EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, TEST_VALID_ID, EXPECTED_WRITE8_LEN_BYTE,
            EXPECTED_INST_WRITE, EXPECTED_REG_OPERATION_MODE, value,
            (uint8_t)~(TEST_VALID_ID + EXPECTED_WRITE8_LEN_BYTE + EXPECTED_INST_WRITE + EXPECTED_REG_OPERATION_MODE + value)
        };

        memset(&dummy_uart_port, 0, sizeof(dummy_uart_port));
        queue_write_ack();

        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_OK, STS_SetOperatingMode(&test_servo, cases[i].mode), cases[i].name);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(EXACTLY_ONE_CALL, dummy_uart_port.tx_call_count, cases[i].name);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(EXPECTED_WRITE8_PACKET_LEN, dummy_uart_port.tx_len, cases[i].name);
        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(expected, dummy_uart_port.tx_buffer, EXPECTED_WRITE8_PACKET_LEN, cases[i].name);
    }
}

/* ==========================================================================
 * PWM & STEP COMMANDS
 * ========================================================================== */

void test_STS_PWM_Step_Null_Guards(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetPWM(NULL, TARGET_VALID_PWM_CCW, STS_DIR_CCW));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetStep(NULL, TARGET_VALID_STEP_CCW, STS_DIR_CW));
}

void test_STS_SetTargetPWM_Out_Of_Range(void) {
    const uint16_t out_of_range_pwm = STS_MAX_PWM + 1U;
    arm_no_effect_check();
    sts_result_t res = STS_SetTargetPWM(&test_servo, out_of_range_pwm, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
    assert_no_effect();
}

void test_STS_SetTargetStep_Out_Of_Range(void) {
    const uint16_t out_of_range_step = STS_MAX_STEP + 1U;
    arm_no_effect_check();
    sts_result_t res = STS_SetTargetStep(&test_servo, out_of_range_step, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
    assert_no_effect();
}

void test_STS_SetTargetPWM_Rejects_Value_Wider_Than_16_Bits(void) {
    const uint32_t wide_pwm     = 0x000101F4U; /* 66036: the low 16 bits are 500, a valid PWM value */
    const int32_t  negative_pwm = -130572;     /* 0xFFFE01F4 once unsigned: the low 16 bits are 500 */

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetPWM(&test_servo, wide_pwm, STS_DIR_CCW));
    assert_no_effect();

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetPWM(&test_servo, (uint32_t)negative_pwm, STS_DIR_CCW));
    assert_no_effect();
}

void test_STS_SetTargetStep_Rejects_Value_Wider_Than_16_Bits(void) {
    const uint32_t wide_step     = 0x00010064U; /* 65636: the low 16 bits are 100, a valid step count */
    const int32_t  negative_step = -130972;     /* 0xFFFE0064 once unsigned: the low 16 bits are 100 */

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetStep(&test_servo, wide_step, STS_DIR_CCW));
    assert_no_effect();

    arm_no_effect_check();
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetStep(&test_servo, (uint32_t)negative_step, STS_DIR_CCW));
    assert_no_effect();
}

void test_STS_SetTargetPWM_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPWM(&test_servo, TARGET_VALID_PWM_CCW, STS_DIR_CW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x81F4U);
}

void test_STS_SetTargetStep_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetStep(&test_servo, TARGET_VALID_STEP_CCW, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0x61A8U);
}

void test_STS_SetTargetStep_Reverse_Direction(void) {
    queue_write_ack();

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTargetStep(&test_servo, TARGET_VALID_STEP_CCW, STS_DIR_CW));
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0xE1A8U);
}

void test_STS_SetTargetPWM_Max_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPWM(&test_servo, STS_MAX_PWM, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x03E8U);
}

void test_STS_SetTargetStep_Max_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetStep(&test_servo, STS_MAX_STEP, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0x7FFFU);
}

void test_STS_SetTargetPWM_Max_Reverse(void) {
    queue_write_ack();

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTargetPWM(&test_servo, STS_MAX_PWM, STS_DIR_CW));
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x83E8U);
}

void test_STS_SetTargetStep_Max_Reverse(void) {
    queue_write_ack();

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTargetStep(&test_servo, STS_MAX_STEP, STS_DIR_CW));
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0xFFFFU);
}

void test_STS_SetTargetPWM_Zero_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPWM(&test_servo, TARGET_ZERO, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_SPEED, 0x0000U);
}

void test_STS_SetTargetStep_Zero_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetStep(&test_servo, TARGET_ZERO, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    assert_single_write16(EXPECTED_REG_GOAL_POSITION, 0x0000U);
}

/* ==========================================================================
 * TORQUE LIMIT COMMAND TESTS
 * ========================================================================== */

void test_STS_SetTorqueLimit_Null_Guard(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTorqueLimit(NULL, TARGET_VALID_TORQUE_LIMIT));
}

void test_STS_SetTorqueLimit_Out_Of_Range(void) {
    const uint16_t out_of_range_torque = STS_MAX_TORQUE + 1U;
    sts_result_t res = STS_SetTorqueLimit(&test_servo, out_of_range_torque);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTorqueLimit_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTorqueLimit(&test_servo, TARGET_VALID_TORQUE_LIMIT);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTorqueLimit_Max_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTorqueLimit(&test_servo, STS_MAX_TORQUE);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTorqueLimit_Zero_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTorqueLimit(&test_servo, TARGET_ZERO);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

/* ==========================================================================
 * STS TELEMETRY TESTS
 * ========================================================================== */

void test_STS_Telemetry_Null_Guards(void) {
    uint8_t dummy_u8 = 0;
    int16_t dummy_s16 = 0;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentLoad(NULL, &dummy_s16));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentVoltage(NULL, &dummy_u8));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentTemperature(NULL, &dummy_u8));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetMovingStatus(NULL, &dummy_u8));

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentLoad(&test_servo, NULL));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentVoltage(&test_servo, NULL));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetPresentTemperature(&test_servo, NULL));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_GetMovingStatus(&test_servo, NULL));
}

void test_STS_GetPresentLoad_Success(void) {
    int16_t actual_load = 0;
    uint8_t mock_payload[PAYLOAD_LEN_2_BYTES];

    //  Test Positive Load (CCW / Bit 10 = 1)
    const uint16_t raw_bus_load_ccw = 0x07E8U; // Magnitude 1000 + Bit 10 Set
    
    mock_payload[0] = (uint8_t)(raw_bus_load_ccw & 0xFFU);
    mock_payload[1] = (uint8_t)((raw_bus_load_ccw >> 8U) & 0xFFU);
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentLoad(&test_servo, &actual_load));
    TEST_ASSERT_EQUAL_INT16(1000, actual_load);

    // Test Negative Load (CW / Bit 10 = 0)
    const uint16_t raw_bus_load_cw = 0x03E8U; // Magnitude 1000 + Bit 10 Cleared
    
    mock_payload[0] = (uint8_t)(raw_bus_load_cw & 0xFFU);
    mock_payload[1] = (uint8_t)((raw_bus_load_cw >> 8U) & 0xFFU);
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentLoad(&test_servo, &actual_load));
    TEST_ASSERT_EQUAL_INT16(-1000, actual_load);
}

void test_STS_GetPresentVoltage_Success(void) {
    uint8_t voltage_out = 0;
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {TEST_MOCK_VOLTAGE_VAL}; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentVoltage(&test_servo, &voltage_out));
    TEST_ASSERT_EQUAL_UINT8(TEST_MOCK_VOLTAGE_VAL, voltage_out);
}

void test_STS_GetPresentTemperature_Success(void) {
    uint8_t temp_out = 0;
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {TEST_MOCK_TEMP_VAL}; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentTemperature(&test_servo, &temp_out));
    TEST_ASSERT_EQUAL_UINT8(TEST_MOCK_TEMP_VAL, temp_out);
}

void test_STS_GetMovingStatus_Success(void) {
    uint8_t status_out = TEST_SENTINEL_MAX_U8;
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {TEST_MOCK_STATUS_STOPPED}; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetMovingStatus(&test_servo, &status_out));
    TEST_ASSERT_EQUAL_UINT8(TEST_MOCK_STATUS_STOPPED, status_out);
}

void test_STS_Telemetry_Bubbles_Hardware_Error(void) {
    uint8_t temp_out = 0;
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {TEST_MOCK_TEMP_OVERHEAT_VAL}; 
    
    simulate_servo_response(TEST_VALID_ID, STS_BIT_ERR_OVERHEAT, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    sts_result_t res = STS_GetPresentTemperature(&test_servo, &temp_out);
    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, res); 
}

void test_STS_Telemetry_Preserves_State_On_Timeout(void) {
    uint8_t voltage_out = TEST_SENTINEL_GARBAGE_U8; 
    
    dummy_uart_port.rx_len = TEST_ZERO_RX_LEN; 

    sts_result_t res = STS_GetPresentVoltage(&test_servo, &voltage_out);
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, res); 
    TEST_ASSERT_EQUAL_UINT8(TEST_SENTINEL_GARBAGE_U8, voltage_out); 
}

/* ==========================================================================
 * STS EEPROM & ID CONFIG TESTS
 * ========================================================================== */

void test_STS_SetEEPROMLock_Null_Guard(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetEEPROMLock(NULL, EEPROM_LOCK));
}

void test_STS_SetEEPROMLock_States(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetEEPROMLock(&test_servo, EEPROM_UNLOCK));
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetEEPROMLock(&test_servo, EEPROM_LOCK));

    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetEEPROMLock(&test_servo, TEST_MOCK_GARBAGE_LOCK));
}

void test_STS_SetID_Null_Guard(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetID(NULL, TEST_MOCK_NEW_ID));
}

void test_STS_SetID_Out_Of_Range(void) {
    const uint8_t invalid_id = STS_ID_BROADCAST_SYNC + 1U;
    
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetID(&test_servo, invalid_id));
}

void test_STS_SetID_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetID(&test_servo, TEST_MOCK_NEW_ID));
}

/* ==========================================================================
 * SCRIPTED TRANSPORT SELF-TESTS
 * ========================================================================== */

#define SCRIPT_CAP_SMALL            2U
#define SCRIPT_TX_LIMIT             300U      /* Required default cap, written independently of the harness */
#define SCRIPT_CAP_ABOVE_LIMIT      1000U
#define SCRIPT_QUEUE_CAPACITY       256U
#define SCRIPT_QUEUE_FIRST_FILL     200U
#define SCRIPT_WRAP_INDEX           256U      /* First transmit index a uint8_t cannot hold */
#define SCRIPT_LONG_STEPS           (SCRIPT_WRAP_INDEX + 2U)

static mock_script_t script;
static uint8_t script_ack[EXPECTED_WRITE_ACK_LEN];
static uint8_t script_bulk[SCRIPT_QUEUE_FIRST_FILL];
static mock_step_t script_long_steps[SCRIPT_LONG_STEPS];
static const uint8_t script_alt_reply[] = {0x55U, 0x66U, 0x77U, 0x88U};
static uint32_t script_fail_count;
static const char *script_fail_reason;

static void record_fail(const char *reason) {
    script_fail_count++;
    script_fail_reason = reason;
}

/* Rebinds the fixture bus to the scripted transport; the servo stays at TEST_VALID_ID. */
static void use_script(const mock_step_t *steps, uint16_t step_count) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, script_ack);
    mock_script_reset(&script, steps, step_count);
    (void)STS_Bus_Init(&test_bus, &script, mock_script_tx, mock_script_rx);
    test_bus.flush_rx = mock_script_flush;
    script_fail_count = 0U;
    script_fail_reason = NULL;
}

/* Sends n one-byte packets straight to the transport and checks every one is accepted. */
static void send_accepted_packets(uint32_t n) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    for (uint32_t i = 0U; i < n; i++) {
        TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    }
}

void test_Mock_Script_Reply_Arrives_At_Its_Transmit(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {script_ack, sizeof(script_ack), STS_OK} };
    use_script(steps, 2U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
}

void test_Mock_Script_Reply_Order_Reversed(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK}, {NULL, 0U, STS_OK} };
    use_script(steps, 2U);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_servo_ping(&test_servo));
}

void test_Mock_Script_Consumes_Exactly_What_It_Returns(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    use_script(steps, 2U);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
}

void test_Mock_Script_Uneven_Reads_Return_Every_Byte(void) {
    const uint8_t reply[] = {0x11U, 0x22U, 0x33U, 0x44U};
    const mock_step_t steps[] = { {reply, sizeof(reply), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t first[1];
    uint8_t rest[3];
    use_script(steps, 1U);

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, first, sizeof(first), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rest, sizeof(rest), TEST_TIMEOUT_MS));

    TEST_ASSERT_EQUAL_HEX8(reply[0], first[0]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&reply[1], rest, sizeof(rest));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
}

void test_Mock_Script_No_Reply_Past_End(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK} };
    use_script(steps, 1U);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT32(2U, script.tx_count);
}

void test_Mock_Script_Lost_Reply_Ignores_Length(void) {
    const mock_step_t steps[] = { {NULL, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 1U);

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
}

void test_Mock_Script_Transmit_Injection_Hits_Chosen_Transmit(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_ERR_TX_FAIL},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    use_script(steps, 2U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
    TEST_ASSERT_EQUAL_UINT32(0U, script.rx_count);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT32(2U, script.tx_count);
}

void test_Mock_Script_Transmit_Injection_Returns_Chosen_Code(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_ERR_BUSY},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    use_script(steps, 3U);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
}

void test_Mock_Script_Receive_Injection_Hits_Chosen_Call(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    use_script(steps, 2U);
    script.discard_on_tx  = 1U;
    script.rx_fail_call   = 2U;
    script.rx_fail_result = STS_ERR_RX_FAIL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_RX_FAIL, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_servo_ping(&test_servo));
    TEST_ASSERT_EQUAL_UINT32(4U, script.rx_count);
}

void test_Mock_Script_Receive_Injection_On_Empty_Queue(void) {
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(NULL, 0U);
    script.rx_fail_call   = 1U;
    script.rx_fail_result = STS_ERR_BUSY;

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_UINT32(2U, script.rx_count);
}

void test_Mock_Script_Receive_Injection_Keeps_Queued_Bytes(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(steps, 1U);
    script.rx_fail_call   = 1U;
    script.rx_fail_result = STS_ERR_RX_FAIL;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_RX_FAIL, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_UINT16(sizeof(script_ack), script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(script_ack, rx, sizeof(rx));
}

void test_Mock_Script_Failed_Transmit_Is_Logged(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_TX_FAIL} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 1U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, script.tx_count);
    TEST_ASSERT_EQUAL_UINT16(sizeof(packet), script.tx_log_len[0]);
    TEST_ASSERT_EQUAL_HEX8(TEST_DATA_BYTE, script.tx_log[0][0]);
}

void test_Mock_Script_Failed_Transmit_Discards_When_Set(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK}, {NULL, 0U, STS_ERR_TX_FAIL} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 2U);
    script.discard_on_tx = 1U;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
}

void test_Mock_Script_Failed_Transmit_Keeps_Bytes_Without_Discard(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK}, {NULL, 0U, STS_ERR_TX_FAIL} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 2U);

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(sizeof(script_ack), script.queue_len);
}

void test_Mock_Script_Flush_Empties_Queue_And_Counts(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(steps, 1U);

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(sizeof(script_ack), script.queue_len);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Bus_FlushRx(&test_bus));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, script.flush_count);
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
}

void test_Mock_Script_Repeated_Flushes_Keep_Counters_And_Schedule(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_alt_reply, sizeof(script_alt_reply), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(steps, 2U);
    script.rx_fail_call   = 3U;
    script.rx_fail_result = STS_ERR_BUSY;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Bus_FlushRx(&test_bus));
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Bus_FlushRx(&test_bus));
    TEST_ASSERT_EQUAL_UINT32(2U, script.flush_count);

    /* The next transmit gets the next step, and the receive schedule carries on across flushes. */
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(script_alt_reply, rx, sizeof(rx));
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));

    TEST_ASSERT_EQUAL_UINT32(2U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT32(3U, script.rx_count);
}

void test_Mock_Script_Timeout_Leaves_Queue(void) {
    const mock_step_t steps[] = { {script_ack, STS_PKT_FIXED_TOTAL, STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[EXPECTED_WRITE_ACK_LEN];
    use_script(steps, 1U);

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_UINT16(STS_PKT_FIXED_TOTAL, script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, STS_PKT_FIXED_TOTAL, TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(script_ack, rx, STS_PKT_FIXED_TOTAL);
}

void test_Mock_Script_Unread_Bytes_Persist_Without_Discard(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(steps, 2U);

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));

    /* The first reply's last two bytes come out ahead of the second reply. */
    const uint8_t expected[STS_PKT_FIXED_TOTAL] = {script_ack[4], script_ack[5], script_ack[0], script_ack[1]};
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, rx, sizeof(rx));
}

void test_Mock_Script_Discard_On_Transmit_Clears_Unread_Bytes(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(steps, 2U);
    script.discard_on_tx = 1U;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));

    TEST_ASSERT_EQUAL_UINT16(sizeof(script_ack), script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(script_ack, rx, sizeof(rx));
}

void test_Mock_Script_Reset_Installs_Default_Fail(void) {
    use_script(NULL, 0U);
    TEST_ASSERT_EQUAL_PTR(mock_script_default_fail, script.on_fail);
}

void test_Mock_Script_Cap_Hook_Fires_After_Cap(void) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(NULL, 0U);
    script.tx_cap  = SCRIPT_CAP_SMALL;
    script.on_fail = record_fail;

    send_accepted_packets(SCRIPT_CAP_SMALL);
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, script_fail_count);
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_CAP, script_fail_reason);
    TEST_ASSERT_EQUAL_UINT32(SCRIPT_CAP_SMALL, script.tx_count);
}

void test_Mock_Script_Cap_Below_Limit_Is_Honoured(void) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(NULL, 0U);
    script.tx_cap  = SCRIPT_TX_LIMIT - 1U;
    script.on_fail = record_fail;

    send_accepted_packets(SCRIPT_TX_LIMIT - 1U);
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_CAP, script_fail_reason);
}

void test_Mock_Script_Default_Cap_Is_Limit(void) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(NULL, 0U);
    script.on_fail = record_fail;

    send_accepted_packets(SCRIPT_TX_LIMIT);
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_CAP, script_fail_reason);
}

void test_Mock_Script_Cap_Above_Limit_Is_Clamped(void) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(NULL, 0U);
    script.tx_cap  = SCRIPT_CAP_ABOVE_LIMIT;
    script.on_fail = record_fail;

    send_accepted_packets(SCRIPT_TX_LIMIT);
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_CAP, script_fail_reason);
}

void test_Mock_Script_Logs_Longest_Packet(void) {
    uint8_t packet[STS_MAX_TX_BUFFER];
    for (uint16_t i = 0U; i < sizeof(packet); i++) {
        packet[i] = (uint8_t)i;
    }
    use_script(NULL, 0U);
    script.on_fail = record_fail;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);
    TEST_ASSERT_EQUAL_UINT16(sizeof(packet), script.tx_log_len[0]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(packet, script.tx_log[0], sizeof(packet));
}

void test_Mock_Script_Overlong_Packet_Fails(void) {
    uint8_t packet[STS_MAX_TX_BUFFER + 1U];
    memset(packet, TEST_DATA_BYTE, sizeof(packet));
    use_script(NULL, 0U);
    script.on_fail = record_fail;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, script_fail_count);
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_PACKET, script_fail_reason);
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

void test_Mock_Script_Queue_Fills_To_Capacity_Then_Fails(void) {
    const mock_step_t steps[] = { {script_bulk, SCRIPT_QUEUE_FIRST_FILL, STS_OK},
                                  {script_bulk, SCRIPT_QUEUE_CAPACITY - SCRIPT_QUEUE_FIRST_FILL, STS_OK},
                                  {script_bulk, 1U, STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 3U);
    script.on_fail = record_fail;

    send_accepted_packets(2U);
    TEST_ASSERT_EQUAL_UINT16(SCRIPT_QUEUE_CAPACITY, script.queue_len);
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_QUEUE, script_fail_reason);
    TEST_ASSERT_EQUAL_UINT16(SCRIPT_QUEUE_CAPACITY, script.queue_len);
}

void test_Mock_Script_Logs_Every_Packet_In_Order(void) {
    const uint8_t expected_first[]  = {EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, TEST_VALID_ID,
                                       EXPECTED_PING_LEN_BYTE, EXPECTED_INST_PING, 0xF2U};
    const uint8_t expected_second[] = {EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, TEST_ALT_ID,
                                       EXPECTED_PING_LEN_BYTE, EXPECTED_INST_PING, 0xE8U};
    use_script(NULL, 0U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_servo_ping(&test_servo));
    (void)STS_Servo_Init(&test_servo, &test_bus, TEST_ALT_ID);
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_servo_ping(&test_servo));

    TEST_ASSERT_EQUAL_UINT32(2U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected_first), script.tx_log_len[0]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_first, script.tx_log[0], sizeof(expected_first));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected_second), script.tx_log_len[1]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_second, script.tx_log[1], sizeof(expected_second));
}

void test_Mock_Script_Index_Past_256_Keeps_Log_And_Steps(void) {
    static const uint8_t first_reply[] = {0xA0U};
    static const uint8_t wrap_reply[]  = {0xB6U};
    uint8_t rx[1];
    memset(script_long_steps, 0, sizeof(script_long_steps));
    script_long_steps[0]                 = (mock_step_t){first_reply, sizeof(first_reply), STS_OK};
    script_long_steps[SCRIPT_WRAP_INDEX] = (mock_step_t){wrap_reply, sizeof(wrap_reply), STS_OK};
    use_script(script_long_steps, SCRIPT_LONG_STEPS);

    for (uint32_t i = 0U; i <= SCRIPT_WRAP_INDEX; i++) {
        const uint8_t packet[] = { (uint8_t)(i & 0xFFU), (uint8_t)(i >> 8) };
        TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
        if (i == 0U) {
            TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
            TEST_ASSERT_EQUAL_HEX8(first_reply[0], rx[0]);
        }
    }

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_HEX8(wrap_reply[0], rx[0]);
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);

    const uint8_t expected_0[]   = {0x00U, 0x00U};
    const uint8_t expected_255[] = {0xFFU, 0x00U};
    const uint8_t expected_256[] = {0x00U, 0x01U};
    TEST_ASSERT_EQUAL_UINT32(SCRIPT_WRAP_INDEX + 1U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected_0), script.tx_log_len[0]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_0, script.tx_log[0], sizeof(expected_0));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected_255), script.tx_log_len[SCRIPT_WRAP_INDEX - 1U]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_255, script.tx_log[SCRIPT_WRAP_INDEX - 1U], sizeof(expected_255));
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected_256), script.tx_log_len[SCRIPT_WRAP_INDEX]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_256, script.tx_log[SCRIPT_WRAP_INDEX], sizeof(expected_256));
}

void test_Mock_Script_Discard_Applies_Past_Script_End(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 1U);
    script.discard_on_tx = 1U;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(sizeof(script_ack), script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
}

void test_Mock_Script_Flush_Keeps_Discard_Setting(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_OK},
                                  {NULL, 0U, STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(steps, 3U);
    script.discard_on_tx = 1U;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Bus_FlushRx(&test_bus));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(sizeof(script_ack), script.queue_len);
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT16(0U, script.queue_len);
}

void test_Mock_Script_Receive_Injection_Past_255_Calls(void) {
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(NULL, 0U);
    script.rx_fail_call   = SCRIPT_TX_LIMIT;
    script.rx_fail_result = STS_ERR_BUSY;

    for (uint32_t call = 1U; call < SCRIPT_TX_LIMIT; call++) {
        TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    }
    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_UINT32(SCRIPT_TX_LIMIT + 1U, script.rx_count);
}

void test_Mock_Script_Receive_Injection_Does_Not_Refire_After_Wrap(void) {
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    use_script(NULL, 0U);
    script.rx_fail_call   = 1U;
    script.rx_fail_result = STS_ERR_BUSY;

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUSY, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    for (uint32_t call = 2U; call <= 2U * SCRIPT_TX_LIMIT; call++) {
        TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    }
    TEST_ASSERT_EQUAL_UINT32(2U * SCRIPT_TX_LIMIT, script.rx_count);
}

void test_Mock_Script_Flush_Count_Past_255(void) {
    use_script(NULL, 0U);

    for (uint32_t i = 0U; i < SCRIPT_TX_LIMIT; i++) {
        TEST_ASSERT_EQUAL_INT(STS_OK, STS_Bus_FlushRx(&test_bus));
    }
    TEST_ASSERT_EQUAL_UINT32(SCRIPT_TX_LIMIT, script.flush_count);
}

void test_Mock_Script_Timeouts_And_Discards_Are_Not_Flushes(void) {
    const mock_step_t steps[] = { {script_ack, sizeof(script_ack), STS_OK},
                                  {script_ack, sizeof(script_ack), STS_OK} };
    const uint8_t packet[] = { TEST_DATA_BYTE };
    uint8_t rx[EXPECTED_WRITE_ACK_LEN + 2U];
    use_script(steps, 2U);
    script.discard_on_tx = 1U;

    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, mock_script_rx(&test_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_OK, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(0U, script.flush_count);
}

void test_Mock_Script_Flush_Keeps_Cap_And_Hook(void) {
    use_script(NULL, 0U);
    script.tx_cap  = SCRIPT_CAP_SMALL;
    script.on_fail = record_fail;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Bus_FlushRx(&test_bus));
    send_accepted_packets(SCRIPT_CAP_SMALL);

    const uint8_t packet[] = { TEST_DATA_BYTE };
    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(EXACTLY_ONE_CALL, script_fail_count);
    TEST_ASSERT_EQUAL_PTR(record_fail, script.on_fail);
}

void test_Mock_Script_Cap_Just_Above_Limit_Is_Clamped(void) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    use_script(NULL, 0U);
    script.tx_cap  = SCRIPT_TX_LIMIT + 1U;
    script.on_fail = record_fail;

    send_accepted_packets(SCRIPT_TX_LIMIT);
    TEST_ASSERT_EQUAL_UINT32(0U, script_fail_count);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, mock_script_tx(&test_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_STRING(MOCK_SCRIPT_FAIL_CAP, script_fail_reason);
    TEST_ASSERT_EQUAL_UINT32(SCRIPT_TX_LIMIT, script.tx_count);
}

void test_Mock_Script_Transmit_Null_Arguments(void) {
    const uint8_t packet[] = { TEST_DATA_BYTE };
    sts_bus_t no_port_bus;
    use_script(NULL, 0U);
    memcpy(&no_port_bus, &test_bus, sizeof(sts_bus_t));
    no_port_bus.port_handle = NULL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_tx(NULL, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_tx(&no_port_bus, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_tx(&test_bus, NULL, sizeof(packet)));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

void test_Mock_Script_Receive_Null_Arguments(void) {
    uint8_t rx[STS_PKT_FIXED_TOTAL];
    sts_bus_t no_port_bus;
    use_script(NULL, 0U);
    memcpy(&no_port_bus, &test_bus, sizeof(sts_bus_t));
    no_port_bus.port_handle = NULL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_rx(NULL, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_rx(&no_port_bus, rx, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_rx(&test_bus, NULL, sizeof(rx), TEST_TIMEOUT_MS));
    TEST_ASSERT_EQUAL_UINT32(0U, script.rx_count);
}

void test_Mock_Script_Flush_Null_Arguments(void) {
    sts_bus_t no_port_bus;
    use_script(NULL, 0U);
    memcpy(&no_port_bus, &test_bus, sizeof(sts_bus_t));
    no_port_bus.port_handle = NULL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_flush(NULL));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, mock_script_flush(&no_port_bus));
    TEST_ASSERT_EQUAL_UINT32(0U, script.flush_count);
}

/* ==========================================================================
 * BUS SCAN
 * ========================================================================== */

#define SCAN_FIRST_ID               1U
#define SCAN_LAST_ID                3U
#define SCAN_MAX_UNICAST_ID         253U
#define SCAN_FOUND_SENTINEL         0xAAU
#define SCAN_FOREIGN_ID             7U
#define SCAN_RETRIES                2U
#define SCAN_LOG_MAX                16U
#define SCAN_REPLY_SLOTS            4U
#define SCAN_OVERSIZE_LEN_BYTE      0xFEU
#define SCAN_LEN(a)                 ((uint32_t)(sizeof(a) / sizeof((a)[0])))

#define SCAN_LOST                   {NULL, 0U, STS_OK}
#define SCAN_REPLY(p, n)            {(p), (n), STS_OK}
#define SCAN_TX_FAILS(code)         {NULL, 0U, (code)}

typedef struct {
    uint8_t      id;
    sts_result_t result;
} scan_entry_t;

typedef struct {
    scan_entry_t entry[SCAN_LOG_MAX];
    uint32_t     count;
} scan_log_t;

static scan_log_t scan_log;
static uint8_t scan_found;
static uint8_t scan_replies[SCAN_REPLY_SLOTS][EXPECTED_READ8_ACK_LEN];

static void scan_record(uint8_t id, sts_result_t result, void *ctx) {
    scan_log_t *log = (scan_log_t *)ctx;
    TEST_ASSERT_TRUE_MESSAGE(log->count < SCAN_LOG_MAX, "Scan callback log full");
    log->entry[log->count].id      = id;
    log->entry[log->count].result  = result;
    log->count++;
}

static void use_scan(const mock_step_t *steps, uint16_t step_count) {
    use_script(steps, step_count);
    memset(&scan_log, 0, sizeof(scan_log));
    scan_found = SCAN_FOUND_SENTINEL;
}

static sts_result_t run_scan(uint8_t first_id, uint8_t last_id) {
    return STS_Bus_Scan(&test_bus, first_id, last_id, scan_record, &scan_log, &scan_found);
}

/* A 6-byte ping reply from id with the given status. */
static const uint8_t *scan_ack(uint8_t slot, uint8_t id, uint8_t status) {
    simulate_servo_response(id, status, NULL, PAYLOAD_LEN_NONE, scan_replies[slot]);
    return scan_replies[slot];
}

static const uint8_t *scan_bad_checksum(uint8_t slot, uint8_t id) {
    (void)scan_ack(slot, id, STS_STATUS_OK);
    scan_replies[slot][EXPECTED_WRITE_ACK_LEN - 1U] ^= CORRUPT_CHECKSUM_MASK;
    return scan_replies[slot];
}

static const uint8_t *scan_bad_header(uint8_t slot, uint8_t id) {
    (void)scan_ack(slot, id, STS_STATUS_OK);
    scan_replies[slot][0] = CORRUPT_HEADER_BYTE;
    scan_replies[slot][1] = CORRUPT_HEADER_BYTE;
    return scan_replies[slot];
}

/* Seven bytes with length byte 3: a ping expects 6, so the engine reports MALFORMED. */
static const uint8_t *scan_wrong_length(uint8_t slot, uint8_t id) {
    const uint8_t param = TEST_DATA_BYTE;
    simulate_servo_response(id, STS_STATUS_OK, &param, PAYLOAD_LEN_1_BYTE, scan_replies[slot]);
    return scan_replies[slot];
}

/* Six bytes with length byte 0xFE: malformed after four bytes; the engine flushes the other two. */
static const uint8_t *scan_oversize(uint8_t slot, uint8_t id) {
    (void)scan_ack(slot, id, STS_STATUS_OK);
    scan_replies[slot][IDX_LENGTH_BYTE] = SCAN_OVERSIZE_LEN_BYTE;
    return scan_replies[slot];
}

static void assert_scan(sts_result_t actual, sts_result_t expected, uint8_t expected_found,
                        const scan_entry_t *log, uint32_t log_count,
                        const uint8_t *pinged, uint32_t pinged_count, uint32_t flushes) {
    TEST_ASSERT_EQUAL_INT(expected, actual);
    TEST_ASSERT_EQUAL_UINT8(expected_found, scan_found);

    TEST_ASSERT_EQUAL_UINT32(log_count, scan_log.count);
    for (uint32_t i = 0U; i < log_count; i++) {
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(log[i].id, scan_log.entry[i].id, "callback id");
        TEST_ASSERT_EQUAL_INT_MESSAGE(log[i].result, scan_log.entry[i].result, "callback result");
    }

    TEST_ASSERT_EQUAL_UINT32(pinged_count, script.tx_count);
    for (uint32_t i = 0U; i < pinged_count; i++) {
        TEST_ASSERT_EQUAL_UINT16(EXPECTED_WRITE_ACK_LEN, script.tx_log_len[i]);
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(pinged[i], script.tx_log[i][STS_IDX_ID], "pinged id");
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(EXPECTED_INST_PING, script.tx_log[i][STS_IDX_INSTRUCTION], "instruction");
    }

    TEST_ASSERT_EQUAL_UINT32(flushes, script.flush_count);
}

/* Rejected before anything is sent: no traffic, no callback, found_id and the bus untouched. */
static void assert_scan_rejected(sts_bus_t *bus, uint8_t first_id, uint8_t last_id, uint8_t *found_id,
                                 sts_result_t expected) {
    test_bus.total_transactions = TEST_SEED_TRANSACTIONS;
    test_bus.hard_failures      = TEST_SEED_HARD_FAILURES;
    sts_bus_t bus_snapshot;
    memcpy(&bus_snapshot, &test_bus, sizeof(sts_bus_t));

    TEST_ASSERT_EQUAL_INT(expected, STS_Bus_Scan(bus, first_id, last_id, scan_record, &scan_log, found_id));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT32(0U, script.rx_count);
    TEST_ASSERT_EQUAL_UINT32(0U, scan_log.count);
    TEST_ASSERT_EQUAL_UINT8(SCAN_FOUND_SENTINEL, scan_found);
    TEST_ASSERT_STRUCT_UNCHANGED(&bus_snapshot, &test_bus, sts_bus_t);
}

void test_STS_Bus_Scan_Finds_Responding_Servo(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_ack(0U, 1U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_OK} };
    const uint8_t pinged[] = {1U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 1U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Hardware_Error_Counts_As_Found(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_ack(0U, 1U, STS_BIT_ERR_OVERHEAT), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_ERR_HARDWARE} };
    const uint8_t pinged[] = {1U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 1U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_All_Silent_Returns_Timeout(void) {
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT}, {2U, STS_ERR_TIMEOUT}, {3U, STS_ERR_TIMEOUT} };
    const uint8_t pinged[] = {1U, 2U, 3U};
    use_scan(NULL, 0U);

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_ERR_TIMEOUT, SCAN_FOUND_SENTINEL,
                log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

/* A transmit fault is final for that ID: the scan moves on. */
static void check_transmit_fault_skipped(sts_result_t fault) {
    const mock_step_t steps[] = { SCAN_TX_FAILS(fault), SCAN_REPLY(scan_ack(0U, 2U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, fault}, {2U, STS_OK} };
    const uint8_t pinged[] = {1U, 2U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 2U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Skips_Transmit_Failure(void) {
    check_transmit_fault_skipped(STS_ERR_TX_FAIL);
}

void test_STS_Bus_Scan_Skips_Busy_Transport(void) {
    check_transmit_fault_skipped(STS_ERR_BUSY);
}

void test_STS_Bus_Scan_Skips_Transport_Null_Pointer(void) {
    check_transmit_fault_skipped(STS_ERR_NULL_PTR);
}

void test_STS_Bus_Scan_Skips_Buffer_Size_Error(void) {
    check_transmit_fault_skipped(STS_ERR_BUF_TOO_SMALL);
}

/* A receive failure is final for that ID when no read retry is left. */
void test_STS_Bus_Scan_Receive_Failure_Moves_On(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_REPLY(scan_ack(0U, 2U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_ERR_RX_FAIL}, {2U, STS_OK} };
    const uint8_t pinged[] = {1U, 2U};
    use_scan(steps, SCAN_LEN(steps));
    script.rx_fail_call   = 1U;
    script.rx_fail_result = STS_ERR_RX_FAIL;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 2U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

/* A noise burst on the only servo's first reply: the read retry finds it. */
void test_STS_Bus_Scan_Read_Retry_Recovers_Noisy_Reply(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_REPLY(scan_ack(0U, 1U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_OK} };
    const uint8_t pinged[] = {1U, 1U};
    use_scan(steps, SCAN_LEN(steps));
    test_bus.max_read_retries = 1U;
    script.rx_fail_call   = 1U;
    script.rx_fail_result = STS_ERR_RX_FAIL;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 1U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 1U);
}

/* Without read retries a protocol error is final for that ID; the scan moves on. */
static void check_protocol_error_moves_on(const uint8_t *bad_reply, uint16_t bad_len, sts_result_t error,
                                          uint32_t flushes) {
    const mock_step_t steps[] = { SCAN_REPLY(bad_reply, bad_len),
                                  SCAN_REPLY(scan_ack(1U, 2U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, error}, {2U, STS_OK} };
    const uint8_t pinged[] = {1U, 2U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 2U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), flushes);
}

void test_STS_Bus_Scan_Header_Error_Moves_On(void) {
    check_protocol_error_moves_on(scan_bad_header(0U, 1U), EXPECTED_WRITE_ACK_LEN, STS_ERR_HEADER, 0U);
}

void test_STS_Bus_Scan_Id_Mismatch_Moves_On(void) {
    check_protocol_error_moves_on(scan_ack(0U, SCAN_FOREIGN_ID, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN,
                                  STS_ERR_ID_MISMATCH, 0U);
}

void test_STS_Bus_Scan_Checksum_Error_Moves_On(void) {
    check_protocol_error_moves_on(scan_bad_checksum(0U, 1U), EXPECTED_WRITE_ACK_LEN, STS_ERR_CHECKSUM, 0U);
}

void test_STS_Bus_Scan_Malformed_Reply_Moves_On(void) {
    /* The engine flushes on the length mismatch. */
    check_protocol_error_moves_on(scan_wrong_length(0U, 1U), EXPECTED_READ8_ACK_LEN, STS_ERR_MALFORMED, 1U);
}

void test_STS_Bus_Scan_Oversize_Reply_Moves_On(void) {
    /* The engine flushes the malformed reply. */
    check_protocol_error_moves_on(scan_oversize(0U, 1U), EXPECTED_WRITE_ACK_LEN, STS_ERR_MALFORMED, 1U);
}

void test_STS_Bus_Scan_Stops_At_First_Responder(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_LOST, SCAN_REPLY(scan_ack(0U, 3U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT}, {2U, STS_ERR_TIMEOUT}, {3U, STS_OK} };
    const uint8_t pinged[] = {1U, 2U, 3U};
    const uint8_t ping_1[] = {EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, 0x01U, EXPECTED_PING_LEN_BYTE, EXPECTED_INST_PING, 0xFBU};
    const uint8_t ping_2[] = {EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, 0x02U, EXPECTED_PING_LEN_BYTE, EXPECTED_INST_PING, 0xFAU};
    const uint8_t ping_3[] = {EXPECTED_HEADER_BYTE, EXPECTED_HEADER_BYTE, 0x03U, EXPECTED_PING_LEN_BYTE, EXPECTED_INST_PING, 0xF9U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 3U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(ping_1, script.tx_log[0], sizeof(ping_1));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(ping_2, script.tx_log[1], sizeof(ping_2));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(ping_3, script.tx_log[2], sizeof(ping_3));
}

void test_STS_Bus_Scan_Read_Retry_Recovers_Checksum_Error(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_bad_checksum(0U, 1U), EXPECTED_WRITE_ACK_LEN),
                                  SCAN_REPLY(scan_ack(1U, 1U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_OK} };
    const uint8_t pinged[] = {1U, 1U};
    use_scan(steps, SCAN_LEN(steps));
    test_bus.max_read_retries = 1U;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 1U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 1U);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.total_transactions);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.total_retries);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.retry_saves);
    TEST_ASSERT_EQUAL_UINT32(0U, test_bus.hard_failures);
}

void test_STS_Bus_Scan_Moves_On_After_Retry_Also_Corrupted(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_bad_checksum(0U, 1U), EXPECTED_WRITE_ACK_LEN),
                                  SCAN_REPLY(scan_bad_checksum(1U, 1U), EXPECTED_WRITE_ACK_LEN),
                                  SCAN_REPLY(scan_ack(2U, 2U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_ERR_CHECKSUM}, {2U, STS_OK} };
    const uint8_t pinged[] = {1U, 1U, 2U};
    use_scan(steps, SCAN_LEN(steps));
    test_bus.max_read_retries = 1U;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 2U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 1U);
}

/* The ping's final result is reported: a corrupted reply then a lost retry shows as a timeout. */
void test_STS_Bus_Scan_Lost_Retry_Reports_Timeout(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_bad_checksum(0U, 1U), EXPECTED_WRITE_ACK_LEN), SCAN_LOST,
                                  SCAN_REPLY(scan_ack(1U, 2U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT}, {2U, STS_OK} };
    const uint8_t pinged[] = {1U, 1U, 2U};
    use_scan(steps, SCAN_LEN(steps));
    test_bus.max_read_retries = 1U;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_OK, 2U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 1U);
}

void test_STS_Bus_Scan_Read_Retry_All_Silent(void) {
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT}, {2U, STS_ERR_TIMEOUT}, {3U, STS_ERR_TIMEOUT} };
    const uint8_t pinged[] = {1U, 1U, 2U, 2U, 3U, 3U};
    use_scan(NULL, 0U);
    test_bus.max_read_retries = 1U;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_ERR_TIMEOUT, SCAN_FOUND_SENTINEL,
                log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 3U);
}

void test_STS_Bus_Scan_Miss_Reports_Local_Fault(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_TX_FAILS(STS_ERR_TX_FAIL), SCAN_LOST };
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT}, {2U, STS_ERR_TX_FAIL}, {3U, STS_ERR_TIMEOUT} };
    const uint8_t pinged[] = {1U, 2U, 3U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_ERR_TX_FAIL, SCAN_FOUND_SENTINEL,
                log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Miss_Reports_Corrupted_Ping(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_REPLY(scan_bad_checksum(0U, 2U), EXPECTED_WRITE_ACK_LEN), SCAN_LOST };
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT}, {2U, STS_ERR_CHECKSUM}, {3U, STS_ERR_TIMEOUT} };
    const uint8_t pinged[] = {1U, 2U, 3U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_ERR_CHECKSUM, SCAN_FOUND_SENTINEL,
                log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Miss_Reports_Last_Error(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_bad_checksum(0U, 1U), EXPECTED_WRITE_ACK_LEN),
                                  SCAN_TX_FAILS(STS_ERR_TX_FAIL), SCAN_LOST };
    const scan_entry_t log[] = { {1U, STS_ERR_CHECKSUM}, {2U, STS_ERR_TX_FAIL}, {3U, STS_ERR_TIMEOUT} };
    const uint8_t pinged[] = {1U, 2U, 3U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_ERR_TX_FAIL, SCAN_FOUND_SENTINEL,
                log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Without_Callback(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_LOST, SCAN_REPLY(scan_ack(0U, 3U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const uint8_t pinged[] = {1U, 2U, 3U};
    use_scan(steps, SCAN_LEN(steps));

    sts_result_t res = STS_Bus_Scan(&test_bus, SCAN_FIRST_ID, SCAN_LAST_ID, NULL, NULL, &scan_found);
    assert_scan(res, STS_OK, 3U, NULL, 0U, pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Null_Bus(void) {
    use_scan(NULL, 0U);
    assert_scan_rejected(NULL, SCAN_FIRST_ID, SCAN_LAST_ID, &scan_found, STS_ERR_NULL_PTR);
}

void test_STS_Bus_Scan_Null_Found_Id(void) {
    use_scan(NULL, 0U);
    assert_scan_rejected(&test_bus, SCAN_FIRST_ID, SCAN_LAST_ID, NULL, STS_ERR_NULL_PTR);
}

void test_STS_Bus_Scan_Bus_Without_Transmit(void) {
    use_scan(NULL, 0U);
    test_bus.transmit = NULL;
    assert_scan_rejected(&test_bus, SCAN_FIRST_ID, SCAN_LAST_ID, &scan_found, STS_ERR_INVALID_PARAM);
}

void test_STS_Bus_Scan_Bus_Without_Receive(void) {
    use_scan(NULL, 0U);
    test_bus.receive = NULL;
    assert_scan_rejected(&test_bus, SCAN_FIRST_ID, SCAN_LAST_ID, &scan_found, STS_ERR_INVALID_PARAM);
}

void test_STS_Bus_Scan_Reversed_Range(void) {
    use_scan(NULL, 0U);
    assert_scan_rejected(&test_bus, SCAN_LAST_ID, SCAN_FIRST_ID, &scan_found, STS_ERR_INVALID_PARAM);
}

void test_STS_Bus_Scan_Broadcast_Id_Rejected(void) {
    use_scan(NULL, 0U);
    assert_scan_rejected(&test_bus, SCAN_FIRST_ID, SCAN_MAX_UNICAST_ID + 1U, &scan_found, STS_ERR_INVALID_PARAM);
}

void test_STS_Bus_Scan_Last_Id_255_Rejected(void) {
    use_scan(NULL, 0U);
    assert_scan_rejected(&test_bus, SCAN_FIRST_ID, UINT8_MAX, &scan_found, STS_ERR_INVALID_PARAM);
}

void test_STS_Bus_Scan_Single_Id_Range(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_ack(0U, 5U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {5U, STS_OK} };
    const uint8_t pinged[] = {5U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(5U, 5U), STS_OK, 5U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Includes_Id_Zero(void) {
    const mock_step_t steps[] = { SCAN_REPLY(scan_ack(0U, 0U, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {0U, STS_OK} };
    const uint8_t pinged[] = {0U};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(0U, 1U), STS_OK, 0U, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Reaches_Id_253(void) {
    const mock_step_t steps[] = { SCAN_LOST, SCAN_REPLY(scan_ack(0U, SCAN_MAX_UNICAST_ID, STS_STATUS_OK), EXPECTED_WRITE_ACK_LEN) };
    const scan_entry_t log[] = { {SCAN_MAX_UNICAST_ID - 1U, STS_ERR_TIMEOUT}, {SCAN_MAX_UNICAST_ID, STS_OK} };
    const uint8_t pinged[] = {SCAN_MAX_UNICAST_ID - 1U, SCAN_MAX_UNICAST_ID};
    use_scan(steps, SCAN_LEN(steps));

    assert_scan(run_scan(SCAN_MAX_UNICAST_ID - 1U, SCAN_MAX_UNICAST_ID), STS_OK, SCAN_MAX_UNICAST_ID,
                log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 0U);
}

void test_STS_Bus_Scan_Uses_Configured_Retries(void) {
    const scan_entry_t log[] = { {1U, STS_ERR_TIMEOUT} };
    const uint8_t pinged[] = {1U, 1U, 1U};
    use_scan(NULL, 0U);
    test_bus.max_read_retries = SCAN_RETRIES;

    /* Engine retries flush before each further attempt. */
    assert_scan(run_scan(1U, 1U), STS_ERR_TIMEOUT, SCAN_FOUND_SENTINEL, log, SCAN_LEN(log), pinged, SCAN_LEN(pinged), 2U);
    TEST_ASSERT_EQUAL_UINT8(SCAN_RETRIES, test_bus.max_read_retries);
}

/* --- Read retry limit: checked for every command --- */

#define RETRIES_LIMIT_LITERAL       254U
#define RETRIES_WRAPPING            255U
#define LIMIT_TEST_STEPS            1000U
#define LIMIT_TEST_SENTINEL         0xBEEFU

void test_STS_Max_Read_Retries_255_Rejects_Writes(void) {
    use_scan(NULL, 0U);
    test_bus.max_read_retries   = RETRIES_WRAPPING;
    test_bus.total_transactions = TEST_SEED_TRANSACTIONS;
    test_bus.total_retries      = TEST_SEED_RETRIES;
    test_bus.retry_saves        = TEST_SEED_RETRY_SAVES;
    test_bus.hard_failures      = TEST_SEED_HARD_FAILURES;
    sts_bus_t bus_snapshot;
    memcpy(&bus_snapshot, &test_bus, sizeof(sts_bus_t));

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTargetStep(&test_servo, LIMIT_TEST_STEPS, STS_DIR_CW));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_STRUCT_UNCHANGED(&bus_snapshot, &test_bus, sts_bus_t);
}

void test_STS_Max_Read_Retries_255_Rejects_Reads(void) {
    use_scan(NULL, 0U);
    test_bus.max_read_retries   = RETRIES_WRAPPING;
    test_bus.total_transactions = TEST_SEED_TRANSACTIONS;
    test_bus.total_retries      = TEST_SEED_RETRIES;
    test_bus.retry_saves        = TEST_SEED_RETRY_SAVES;
    test_bus.hard_failures      = TEST_SEED_HARD_FAILURES;
    sts_bus_t bus_snapshot;
    memcpy(&bus_snapshot, &test_bus, sizeof(sts_bus_t));
    uint16_t pos = LIMIT_TEST_SENTINEL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_GetPresentPosition(&test_servo, &pos));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_EQUAL_HEX16(LIMIT_TEST_SENTINEL, pos);
    TEST_ASSERT_STRUCT_UNCHANGED(&bus_snapshot, &test_bus, sts_bus_t);
}

void test_STS_Max_Read_Retries_254_Read_Makes_255_Attempts(void) {
    use_scan(NULL, 0U);
    test_bus.max_read_retries = RETRIES_LIMIT_LITERAL;
    uint16_t pos = 0U;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_GetPresentPosition(&test_servo, &pos));
    TEST_ASSERT_EQUAL_UINT32(RETRIES_LIMIT_LITERAL + 1U, script.tx_count);
    for (uint32_t i = 1U; i < script.tx_count; i++) {
        TEST_ASSERT_EQUAL_UINT16(script.tx_log_len[0], script.tx_log_len[i]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(script.tx_log[0], script.tx_log[i], script.tx_log_len[0]);
    }
    TEST_ASSERT_EQUAL_UINT32(RETRIES_LIMIT_LITERAL, script.flush_count);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.total_transactions);
    TEST_ASSERT_EQUAL_UINT32(RETRIES_LIMIT_LITERAL, test_bus.total_retries);
    TEST_ASSERT_EQUAL_UINT32(0U, test_bus.retry_saves);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.hard_failures);
}

void test_STS_Max_Read_Retries_254_Write_Makes_One_Attempt(void) {
    use_scan(NULL, 0U);
    test_bus.max_read_retries = RETRIES_LIMIT_LITERAL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_SetTargetStep(&test_servo, LIMIT_TEST_STEPS, STS_DIR_CW));
    TEST_ASSERT_EQUAL_UINT32(1U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT32(0U, script.flush_count);
    TEST_ASSERT_EQUAL_UINT32(0U, test_bus.total_retries);
    TEST_ASSERT_EQUAL_UINT32(1U, test_bus.hard_failures);
}

/* With 255 retries nothing is sent, so no ID may be reported found. */
void test_STS_Bus_Scan_Max_Read_Retries_255_Finds_Nothing(void) {
    const scan_entry_t log[] = { {1U, STS_ERR_INVALID_PARAM}, {2U, STS_ERR_INVALID_PARAM},
                                 {3U, STS_ERR_INVALID_PARAM} };
    use_scan(NULL, 0U);
    test_bus.max_read_retries = RETRIES_WRAPPING;

    assert_scan(run_scan(SCAN_FIRST_ID, SCAN_LAST_ID), STS_ERR_INVALID_PARAM, SCAN_FOUND_SENTINEL,
                log, SCAN_LEN(log), NULL, 0U, 0U);
}

/* ==========================================================================
 * REPLY LENGTH CHECK
 * ========================================================================== */

#define LEN_RETRIES                 2U
#define LEN_HEADER_SIZE             4U        /* FF FF, ID, length byte */
#define LEN_READ16_BYTE             4U        /* Length byte of a READ16 reply */
#define LEN_LARGEST_FITTING         124U      /* 4 + 124 fills the 128-byte rx_buf */
#define LEN_SMALLEST_OVERSIZE       125U
#define LEN_LARGEST_BYTE            255U
#define LEN_OVERSIZE_BYTE           0xFEU
#define LEN_MISMATCH_BYTE           5U
#define LEN_BODY_MAX                252U      /* Header plus body fill the 256-byte queue */
#define LEN_PATTERN_BYTE            0x5AU
#define LEN_GUARD_BYTE              0xC3U
#define LEN_GUARD_SIZE              16U
#define LEN_POSITION_VALUE          0x0800U
#define LEN_POSITION_SENTINEL       0xBEEFU
#define LEN_FULL_REPLY              128U      /* Longest reply rx_buf holds */
#define LEN_FULL_PARAMS             (LEN_FULL_REPLY - EXPECTED_WRITE_ACK_LEN)
#define LEN_EXPECTED_PAST_BUFFER    (LEN_FULL_REPLY + 1U)
#define LEN_EXPECTED_BELOW_REPLY    (EXPECTED_WRITE_ACK_LEN - 1U)

/* READ16 of present position (0x38) from ID 10. */
static const uint8_t len_read_position[] = {0xFFU, 0xFFU, 0x0AU, 0x04U, 0x02U, 0x38U, 0x02U, 0xB5U};
/* WRITE8 of 1 to torque enable (0x28), broadcast. */
static const uint8_t len_broadcast_torque_on[] = {0xFFU, 0xFFU, 0xFEU, 0x04U, 0x03U, 0x28U, 0x01U, 0xD1U};

typedef struct {
    sts_bus_t bus;
    uint8_t   guard[LEN_GUARD_SIZE];
} len_guarded_bus_t;

typedef struct {
    uint32_t transactions;
    uint32_t retries;
    uint32_t saves;
    uint32_t hard;
} len_counters_t;

static len_guarded_bus_t len_bus;
static sts_servo_t       len_servo;
static uint8_t           len_valid[EXPECTED_READ16_ACK_LEN];
static uint8_t           len_oversize[EXPECTED_READ16_ACK_LEN];
static uint8_t           len_mismatch[EXPECTED_READ16_ACK_LEN];
static uint8_t           len_reply[MOCK_SCRIPT_QUEUE_SIZE];

/* Guarded bus on the scripted transport, counters seeded, position replies rebuilt. */
static void use_len_bus(const mock_step_t *steps, uint16_t step_count, uint8_t read_retries) {
    const uint8_t pos[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, pos, STS_DATA_LEN_16BIT, len_valid);
    memcpy(len_oversize, len_valid, sizeof(len_valid));
    len_oversize[IDX_LENGTH_BYTE] = LEN_OVERSIZE_BYTE;
    memcpy(len_mismatch, len_valid, sizeof(len_valid));
    len_mismatch[IDX_LENGTH_BYTE] = LEN_MISMATCH_BYTE;

    mock_script_reset(&script, steps, step_count);
    (void)STS_Bus_Init(&len_bus.bus, &script, mock_script_tx, mock_script_rx);
    len_bus.bus.flush_rx           = mock_script_flush;
    len_bus.bus.max_read_retries   = read_retries;
    len_bus.bus.total_transactions = TEST_SEED_TRANSACTIONS;
    len_bus.bus.total_retries      = TEST_SEED_RETRIES;
    len_bus.bus.retry_saves        = TEST_SEED_RETRY_SAVES;
    len_bus.bus.hard_failures      = TEST_SEED_HARD_FAILURES;
    memset(len_bus.guard, LEN_GUARD_BYTE, sizeof(len_bus.guard));
    (void)STS_Servo_Init(&len_servo, &len_bus.bus, TEST_VALID_ID);
}

/* Header from ID 10 with length byte n, then min(n, LEN_BODY_MAX) pattern bytes; returns the size. */
static uint16_t len_build_raw(uint16_t n) {
    uint16_t body = (n < LEN_BODY_MAX) ? n : LEN_BODY_MAX;
    len_reply[0] = EXPECTED_HEADER_BYTE;
    len_reply[1] = EXPECTED_HEADER_BYTE;
    len_reply[2] = TEST_VALID_ID;
    len_reply[IDX_LENGTH_BYTE] = (uint8_t)n;
    memset(&len_reply[LEN_HEADER_SIZE], LEN_PATTERN_BYTE, body);
    return (uint16_t)(LEN_HEADER_SIZE + body);
}

/* Nothing written past rx_buf: tail padding still zero, guard still intact. */
static void assert_rx_buf_contained(const char *msg) {
    const uint8_t *raw = (const uint8_t *)&len_bus.bus;
    TEST_ASSERT_EQUAL_UINT32((uint32_t)sizeof(sts_bus_t), (uint32_t)offsetof(len_guarded_bus_t, guard));
    for (size_t i = offsetof(sts_bus_t, rx_buf) + STS_MAX_RX_BUFFER; i < sizeof(sts_bus_t); i++) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0U, raw[i], msg);
    }
    for (uint32_t i = 0U; i < LEN_GUARD_SIZE; i++) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(LEN_GUARD_BYTE, len_bus.guard[i], msg);
    }
}

/* Transmits (each equal to packet, unless NULL), flushes, counter deltas from the seeds, rx_buf contained. */
static void assert_len_outcome(const uint8_t *packet, uint16_t packet_len, uint32_t tx, uint32_t flushes,
                               len_counters_t delta, const char *msg) {
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(tx, script.tx_count, msg);
    for (uint32_t i = 0U; (packet != NULL) && (i < tx); i++) {
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(packet_len, script.tx_log_len[i], msg);
        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(packet, script.tx_log[i], packet_len, msg);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(flushes, script.flush_count, msg);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(TEST_SEED_TRANSACTIONS + delta.transactions, len_bus.bus.total_transactions, msg);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(TEST_SEED_RETRIES + delta.retries, len_bus.bus.total_retries, msg);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(TEST_SEED_RETRY_SAVES + delta.saves, len_bus.bus.retry_saves, msg);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(TEST_SEED_HARD_FAILURES + delta.hard, len_bus.bus.hard_failures, msg);
    assert_rx_buf_contained(msg);
}

/* One position read whose reply carries length byte n, with no retries. */
static void check_length_byte(uint16_t n) {
    char msg[24];
    (void)snprintf(msg, sizeof(msg), "length byte %u", (unsigned)n);
    const uint8_t *reply = len_valid;
    uint16_t reply_len   = EXPECTED_READ16_ACK_LEN;
    if (n != LEN_READ16_BYTE) {
        reply_len = len_build_raw(n);
        reply     = len_reply;
    }
    const mock_step_t steps[] = { {reply, reply_len, STS_OK} };
    use_len_bus(steps, 1U, 0U);
    uint16_t pos = LEN_POSITION_SENTINEL;

    sts_result_t res = STS_GetPresentPosition(&len_servo, &pos);

    if (n == LEN_READ16_BYTE) {
        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_OK, res, msg);
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(LEN_POSITION_VALUE, pos, msg);
        assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 0U}, msg);
    } else {
        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_ERR_MALFORMED, res, msg);
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(LEN_POSITION_SENTINEL, pos, msg);
        assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 1U, (len_counters_t){1U, 0U, 0U, 1U}, msg);
    }
}

void test_STS_Reply_Length_Within_Buffer_Only_Exact_Accepted(void) {
    for (uint16_t n = 0U; n <= LEN_LARGEST_FITTING; n++) {
        check_length_byte(n);
    }
}

void test_STS_Reply_Length_Past_Buffer_Is_Malformed(void) {
    for (uint16_t n = LEN_SMALLEST_OVERSIZE; n <= LEN_LARGEST_BYTE; n++) {
        check_length_byte(n);
    }
}

void test_STS_Oversize_Length_Retried_Then_Valid(void) {
    const mock_step_t steps[] = { {len_oversize, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_len_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos = LEN_POSITION_SENTINEL;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentPosition(&len_servo, &pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos);
    assert_len_outcome(len_read_position, sizeof(len_read_position), 2U, 2U, (len_counters_t){1U, 1U, 1U, 0U}, NULL);
}

void test_STS_Oversize_Length_Every_Attempt_Hard_Failure(void) {
    const mock_step_t steps[] = { {len_oversize, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_oversize, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_oversize, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_len_bus(steps, 3U, LEN_RETRIES);
    uint16_t pos = LEN_POSITION_SENTINEL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, STS_GetPresentPosition(&len_servo, &pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_SENTINEL, pos);
    /* One flush per malformed reply, one before each retry. */
    assert_len_outcome(len_read_position, sizeof(len_read_position), 3U, 5U, (len_counters_t){1U, 2U, 0U, 1U}, NULL);
}

void test_STS_Lost_Then_Oversize_Then_Valid(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK},
                                  {len_oversize, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_len_bus(steps, 3U, LEN_RETRIES);
    uint16_t pos = LEN_POSITION_SENTINEL;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentPosition(&len_servo, &pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos);
    assert_len_outcome(len_read_position, sizeof(len_read_position), 3U, 3U, (len_counters_t){1U, 2U, 1U, 0U}, NULL);
}

void test_STS_Length_Mismatch_Retried_Then_Valid(void) {
    const mock_step_t steps[] = { {len_mismatch, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_len_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos = LEN_POSITION_SENTINEL;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentPosition(&len_servo, &pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos);
    assert_len_outcome(len_read_position, sizeof(len_read_position), 2U, 2U, (len_counters_t){1U, 1U, 1U, 0U}, NULL);
}

/* Reply length agrees with the caller but exceeds rx_buf: malformed, stage 2 never read. */
void test_STS_ExecuteCommand_Expected_Length_Past_Buffer_Is_Malformed(void) {
    const mock_step_t steps[] = { {len_reply, len_build_raw(LEN_SMALLEST_OVERSIZE), STS_OK} };
    use_len_bus(steps, 1U, 0U);
    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = LEN_EXPECTED_PAST_BUFFER,
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, sts_execute_command(&len_servo, &cmd));
    TEST_ASSERT_EQUAL_UINT32(1U, script.rx_count);
    assert_len_outcome(NULL, 0U, 1U, 1U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_ExecuteCommand_Reply_Filling_Buffer_Accepted(void) {
    uint8_t params[LEN_FULL_PARAMS];
    for (uint16_t i = 0U; i < LEN_FULL_PARAMS; i++) {
        params[i] = (uint8_t)(i + 1U);
    }
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, params, LEN_FULL_PARAMS, len_reply);
    const mock_step_t steps[] = { {len_reply, LEN_FULL_REPLY, STS_OK} };
    use_len_bus(steps, 1U, 0U);
    uint8_t out[LEN_FULL_PARAMS] = {0};
    uint16_t out_len = 0U;
    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = LEN_FULL_REPLY,
        .rx_params_out    = out,
        .rx_params_size   = LEN_FULL_PARAMS,
        .rx_param_len_out = &out_len
    };

    TEST_ASSERT_EQUAL_INT(STS_OK, sts_execute_command(&len_servo, &cmd));
    TEST_ASSERT_EQUAL_UINT16(LEN_FULL_PARAMS, out_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(params, out, LEN_FULL_PARAMS);
    assert_len_outcome(NULL, 0U, 1U, 0U, (len_counters_t){1U, 0U, 0U, 0U}, NULL);
}

void test_STS_ExecuteCommand_Expected_Length_Below_Any_Reply(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, len_reply);
    const mock_step_t steps[] = { {len_reply, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_len_bus(steps, 1U, 0U);
    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = LEN_EXPECTED_BELOW_REPLY,
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, sts_execute_command(&len_servo, &cmd));
    assert_len_outcome(NULL, 0U, 1U, 1U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

/* A broadcast write is a write: a transmit failure is final, even with retries configured. */
void test_STS_Broadcast_Write_Transmit_Failure_Is_Final(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_TX_FAIL}, {NULL, 0U, STS_OK} };
    use_len_bus(steps, 2U, LEN_RETRIES);
    len_servo.id = STS_ID_BROADCAST_SYNC;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TX_FAIL, STS_Write8(&len_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
    TEST_ASSERT_EQUAL_UINT32(0U, script.rx_count);
    assert_len_outcome(len_broadcast_torque_on, sizeof(len_broadcast_torque_on), 1U, 0U,
                       (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Broadcast_Write_First_Attempt_No_Save(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK} };
    use_len_bus(steps, 1U, LEN_RETRIES);
    len_servo.id = STS_ID_BROADCAST_SYNC;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write8(&len_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
    TEST_ASSERT_EQUAL_UINT32(0U, script.rx_count);
    assert_len_outcome(len_broadcast_torque_on, sizeof(len_broadcast_torque_on), 1U, 0U,
                       (len_counters_t){1U, 0U, 0U, 0U}, NULL);
}

/* ==========================================================================
 * READ AND WRITE RETRIES
 * ========================================================================== */

#define RT_STEPS                    1000U
#define RT_NEW_ID                   11U
#define RT_FOREIGN_ID               20U
#define RT_ACK_LEN_BYTE             0x02U     /* Length byte of a write acknowledgement */
#define RT_LEN_BITS                 8U
#define RT_STAGE1_ONLY              4U        /* Header, ID and length byte: a reply cut before stage 2 */
#define RT_CORRUPT_BYTE             0xEEU

/* SetTargetStep(1000, CW) to ID 10: WRITE16 0x2A, 1000 with the direction bit. */
static const uint8_t rt_step_packet[] = {0xFFU, 0xFFU, 0x0AU, 0x05U, 0x03U, 0x2AU, 0xE8U, 0x83U, 0x58U};
/* SetID(11) to ID 10: WRITE8 0x05. */
static const uint8_t rt_set_id_packet[] = {0xFFU, 0xFFU, 0x0AU, 0x04U, 0x03U, 0x05U, 0x0BU, 0xDEU};

static uint8_t rt_ack[EXPECTED_WRITE_ACK_LEN];
static uint8_t rt_ack_hw[EXPECTED_WRITE_ACK_LEN];
static uint8_t rt_pos_hw[EXPECTED_READ16_ACK_LEN];
static uint8_t rt_bad[EXPECTED_READ16_ACK_LEN];

/* Guarded bus with retries, plus the acknowledgement and hardware-status replies. */
static void use_rt_bus(const mock_step_t *steps, uint16_t step_count, uint8_t read_retries) {
    const uint8_t pos[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, rt_ack);
    simulate_servo_response(TEST_VALID_ID, STS_BIT_ERR_OVERHEAT, NULL, PAYLOAD_LEN_NONE, rt_ack_hw);
    simulate_servo_response(TEST_VALID_ID, STS_BIT_ERR_OVERHEAT, pos, STS_DATA_LEN_16BIT, rt_pos_hw);
    use_len_bus(steps, step_count, read_retries);
}

static sts_result_t rt_read(uint16_t *pos) {
    *pos = LEN_POSITION_SENTINEL;
    return STS_GetPresentPosition(&len_servo, pos);
}

static sts_result_t rt_write(void) {
    return STS_SetTargetStep(&len_servo, RT_STEPS, STS_DIR_CW);
}

void test_STS_Read_Valid_Reply_One_Attempt(void) {
    const mock_step_t steps[] = { {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 1U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_OK, rt_read(&pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos);
    assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 0U}, NULL);
}

void test_STS_Write_Valid_Ack_One_Attempt(void) {
    const mock_step_t steps[] = { {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 1U, LEN_RETRIES);

    TEST_ASSERT_EQUAL_INT(STS_OK, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 0U}, NULL);
}

void test_STS_Read_Retried_After_Lost_Reply(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_OK, rt_read(&pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos);
    assert_len_outcome(len_read_position, sizeof(len_read_position), 2U, 1U, (len_counters_t){1U, 1U, 1U, 0U}, NULL);
}

/* The servo may have moved: a lost acknowledgement must not resend a relative step. */
void test_STS_Write_Not_Retried_After_Lost_Ack(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Read_Every_Attempt_Lost(void) {
    use_rt_bus(NULL, 0U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_read(&pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_SENTINEL, pos);
    assert_len_outcome(len_read_position, sizeof(len_read_position), 3U, 2U, (len_counters_t){1U, 2U, 0U, 1U}, NULL);
}

void test_STS_Write_Every_Ack_Lost(void) {
    use_rt_bus(NULL, 0U, LEN_RETRIES);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Read_Hardware_Status_Not_Retried(void) {
    const mock_step_t steps[] = { {rt_pos_hw, EXPECTED_READ16_ACK_LEN, STS_OK}, {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, rt_read(&pos));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 0U}, NULL);
}

void test_STS_Write_Hardware_Status_Not_Retried(void) {
    const mock_step_t steps[] = { {rt_ack_hw, EXPECTED_WRITE_ACK_LEN, STS_OK}, {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 0U}, NULL);
}

/* --- One failure class on the first attempt, then a valid reply --- */

typedef enum {
    RT_TX_FAIL, RT_TX_BUSY, RT_TX_RX_FAIL, RT_LOST, RT_STAGE1_RX_FAIL, RT_LEN_MISMATCH, RT_LEN_OVERSIZE,
    RT_STAGE2_TIMEOUT, RT_STAGE2_RX_FAIL, RT_CHECKSUM, RT_HEADER, RT_ID_MISMATCH, RT_CLASS_COUNT
} rt_class_t;

typedef struct {
    const char  *name;
    sts_result_t code;           /* What the failing attempt returns */
    uint32_t     engine_flushes; /* Flushes inside the failing attempt (length mismatch path) */
} rt_class_info_t;

static const rt_class_info_t rt_classes[RT_CLASS_COUNT] = {
    [RT_TX_FAIL]        = {"transmit TX_FAIL", STS_ERR_TX_FAIL, 0U},
    [RT_TX_BUSY]        = {"transmit BUSY", STS_ERR_BUSY, 0U},
    [RT_TX_RX_FAIL]     = {"transmit RX_FAIL", STS_ERR_RX_FAIL, 0U},
    [RT_LOST]           = {"reply lost", STS_ERR_TIMEOUT, 0U},
    [RT_STAGE1_RX_FAIL] = {"stage 1 RX_FAIL", STS_ERR_RX_FAIL, 0U},
    [RT_LEN_MISMATCH]   = {"length mismatch", STS_ERR_MALFORMED, 1U},
    [RT_LEN_OVERSIZE]   = {"oversize length", STS_ERR_MALFORMED, 1U},
    [RT_STAGE2_TIMEOUT] = {"stage 2 timeout", STS_ERR_TIMEOUT, 0U},
    [RT_STAGE2_RX_FAIL] = {"stage 2 RX_FAIL", STS_ERR_RX_FAIL, 0U},
    [RT_CHECKSUM]       = {"checksum", STS_ERR_CHECKSUM, 0U},
    [RT_HEADER]         = {"header", STS_ERR_HEADER, 0U},
    [RT_ID_MISMATCH]    = {"ID mismatch", STS_ERR_ID_MISMATCH, 0U},
};

/* First step for class c, corrupting a copy of the valid reply where the class needs it. */
static mock_step_t rt_failing_step(rt_class_t c, const uint8_t *valid, uint16_t len) {
    memcpy(rt_bad, valid, len);
    mock_step_t step = { rt_bad, len, STS_OK };
    switch (c) {
        case RT_TX_FAIL:        step = (mock_step_t){ NULL, 0U, STS_ERR_TX_FAIL }; break;
        case RT_TX_BUSY:        step = (mock_step_t){ NULL, 0U, STS_ERR_BUSY }; break;
        case RT_TX_RX_FAIL:     step = (mock_step_t){ NULL, 0U, STS_ERR_RX_FAIL }; break;
        case RT_LOST:           step = (mock_step_t){ NULL, 0U, STS_OK }; break;
        case RT_STAGE1_RX_FAIL: script.rx_fail_call = 1U; script.rx_fail_result = STS_ERR_RX_FAIL; break;
        case RT_LEN_MISMATCH:   rt_bad[IDX_LENGTH_BYTE] = LEN_MISMATCH_BYTE; break;
        case RT_LEN_OVERSIZE:   rt_bad[IDX_LENGTH_BYTE] = LEN_OVERSIZE_BYTE; break;
        case RT_STAGE2_TIMEOUT: step.reply_len = RT_STAGE1_ONLY; break;
        case RT_STAGE2_RX_FAIL: script.rx_fail_call = 2U; script.rx_fail_result = STS_ERR_RX_FAIL; break;
        case RT_CHECKSUM:       rt_bad[len - 1U] ^= CORRUPT_CHECKSUM_MASK; break;
        case RT_HEADER:         rt_bad[IDX_HEADER_START] = RT_CORRUPT_BYTE; break;
        case RT_ID_MISMATCH:    rt_bad[STS_IDX_ID] = RT_FOREIGN_ID; break;
        default: break;
    }
    return step;
}

void test_STS_Read_Retried_After_Every_Failure_Class(void) {
    for (uint32_t c = 0U; c < RT_CLASS_COUNT; c++) {
        mock_step_t steps[2] = { { NULL, 0U, STS_OK }, {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
        use_rt_bus(NULL, 0U, LEN_RETRIES);     /* builds len_valid before it is copied */
        steps[0] = rt_failing_step((rt_class_t)c, len_valid, EXPECTED_READ16_ACK_LEN);
        script.steps = steps;
        script.step_count = 2U;
        uint16_t pos;

        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_OK, rt_read(&pos), rt_classes[c].name);
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(LEN_POSITION_VALUE, pos, rt_classes[c].name);
        assert_len_outcome(len_read_position, sizeof(len_read_position), 2U, 1U + rt_classes[c].engine_flushes,
                           (len_counters_t){1U, 1U, 1U, 0U}, rt_classes[c].name);
    }
}

void test_STS_Write_Not_Retried_After_Any_Failure_Class(void) {
    for (uint32_t c = 0U; c < RT_CLASS_COUNT; c++) {
        mock_step_t steps[2] = { { NULL, 0U, STS_OK }, {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
        use_rt_bus(NULL, 0U, LEN_RETRIES);
        steps[0] = rt_failing_step((rt_class_t)c, rt_ack, EXPECTED_WRITE_ACK_LEN);
        script.steps = steps;
        script.step_count = 2U;

        TEST_ASSERT_EQUAL_INT_MESSAGE(rt_classes[c].code, rt_write(), rt_classes[c].name);
        assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, rt_classes[c].engine_flushes,
                           (len_counters_t){1U, 0U, 0U, 1U}, rt_classes[c].name);
    }
}

void test_STS_Read_Not_Retried_With_Zero_Read_Retries(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, 0U);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_read(&pos));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Write_Not_Retried_With_Zero_Read_Retries(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, 0U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

/* A corrupted ID byte (0xFF) in a reply from ID 4 parses as INVALID_LEN: a wire fault, so a read retries it. */
#define RT_INVALID_LEN_ID           4U
void test_STS_Read_Retried_After_Invalid_Length_Reply(void) {
    const uint8_t pos[STS_DATA_LEN_16BIT] = { 0x00U, 0x08U };
    uint8_t good[EXPECTED_READ16_ACK_LEN];
    simulate_servo_response(RT_INVALID_LEN_ID, STS_STATUS_OK, pos, STS_DATA_LEN_16BIT, good);
    memcpy(rt_bad, good, sizeof(good));
    rt_bad[STS_IDX_ID] = TEST_DUMMY_BYTE_FF;

    /* Without retries the corrupted reply alone gives INVALID_LEN. */
    const mock_step_t once[] = { {rt_bad, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(once, 1U, 0U);
    (void)STS_Servo_Init(&len_servo, &len_bus.bus, RT_INVALID_LEN_ID);
    uint16_t pos_out;
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_LEN, rt_read(&pos_out));

    const mock_step_t steps[] = { {rt_bad, EXPECTED_READ16_ACK_LEN, STS_OK}, {good, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);
    memcpy(rt_bad, good, sizeof(good));
    rt_bad[STS_IDX_ID] = TEST_DUMMY_BYTE_FF;
    (void)STS_Servo_Init(&len_servo, &len_bus.bus, RT_INVALID_LEN_ID);

    TEST_ASSERT_EQUAL_INT(STS_OK, rt_read(&pos_out));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos_out);
    assert_len_outcome(NULL, 0U, 2U, 1U, (len_counters_t){1U, 1U, 1U, 0U}, NULL);
}

/* --- Caller errors and reported codes --- */

void test_STS_Read_Stops_On_Transport_Null_Pointer(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_NULL_PTR}, {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, rt_read(&pos));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

/* Another port may reject an argument with INVALID_PARAM: it would fail the same way again. */
void test_STS_Read_Stops_On_Transport_Invalid_Param(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_INVALID_PARAM}, {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, rt_read(&pos));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Write_Returns_Transport_Null_Pointer(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_NULL_PTR}, {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

/* An output buffer too small for the reply fails the same way every time: one attempt. */
void test_STS_Read_Stops_On_Caller_Buffer_Too_Small(void) {
    const mock_step_t steps[] = { {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 3U, LEN_RETRIES);
    const uint8_t params[] = { 0x38U, STS_DATA_LEN_16BIT };
    uint8_t out[1] = {0};
    uint16_t out_len = 0U;
    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = params,
        .tx_param_len     = sizeof(params),
        .expected_rx_len  = EXPECTED_READ16_ACK_LEN,
        .rx_params_out    = out,
        .rx_params_size   = sizeof(out),
        .rx_param_len_out = &out_len
    };

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUF_TOO_SMALL, sts_execute_read(&len_servo, &cmd));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Read_Reports_Last_Attempt_Code(void) {
    const mock_step_t steps[] = { {rt_bad, EXPECTED_READ16_ACK_LEN, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK} };
    use_rt_bus(NULL, 0U, LEN_RETRIES);
    memcpy(rt_bad, len_valid, EXPECTED_READ16_ACK_LEN);
    rt_bad[EXPECTED_READ16_ACK_LEN - 1U] ^= CORRUPT_CHECKSUM_MASK;
    script.steps = steps;
    script.step_count = 3U;
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_read(&pos));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 3U, 2U, (len_counters_t){1U, 2U, 0U, 1U}, NULL);
}

void test_STS_Write_Reports_First_And_Only_Code(void) {
    const mock_step_t steps[] = { {rt_bad, EXPECTED_WRITE_ACK_LEN, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK} };
    use_rt_bus(NULL, 0U, LEN_RETRIES);
    memcpy(rt_bad, rt_ack, EXPECTED_WRITE_ACK_LEN);
    rt_bad[EXPECTED_WRITE_ACK_LEN - 1U] ^= CORRUPT_CHECKSUM_MASK;
    script.steps = steps;
    script.step_count = 3U;

    TEST_ASSERT_EQUAL_INT(STS_ERR_CHECKSUM, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

void test_STS_Read_Hardware_Status_After_Lost_Reply(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {rt_pos_hw, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);
    uint16_t pos;

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, rt_read(&pos));
    assert_len_outcome(len_read_position, sizeof(len_read_position), 2U, 1U, (len_counters_t){1U, 1U, 0U, 0U}, NULL);
}

void test_STS_Write_Lost_Ack_Hides_Later_Hardware_Status(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {rt_ack_hw, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 2U, LEN_RETRIES);

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, rt_write());
    assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

/* --- A corrupted acknowledgement must not resend the write --- */

void test_STS_Write_Not_Retried_After_Any_Ack_Length_Bit_Flip(void) {
    char msg[24];
    for (uint32_t bit = 0U; bit < RT_LEN_BITS; bit++) {
        mock_step_t steps[2] = { { rt_bad, EXPECTED_WRITE_ACK_LEN, STS_OK }, {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK} };
        use_rt_bus(steps, 2U, LEN_RETRIES);
        memcpy(rt_bad, rt_ack, EXPECTED_WRITE_ACK_LEN);
        rt_bad[IDX_LENGTH_BYTE] = (uint8_t)(RT_ACK_LEN_BYTE ^ (1U << bit));
        (void)snprintf(msg, sizeof(msg), "length byte 0x%02X", (unsigned)rt_bad[IDX_LENGTH_BYTE]);

        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_ERR_MALFORMED, rt_write(), msg);
        assert_len_outcome(rt_step_packet, sizeof(rt_step_packet), 1U, 1U, (len_counters_t){1U, 0U, 0U, 1U}, msg);
    }
}

void test_STS_SetID_Not_Resent_After_Corrupted_Ack(void) {
    const mock_step_t steps[] = { {rt_bad, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_rt_bus(steps, 1U, LEN_RETRIES);
    memcpy(rt_bad, rt_ack, EXPECTED_WRITE_ACK_LEN);
    rt_bad[IDX_LENGTH_BYTE] = (uint8_t)(RT_ACK_LEN_BYTE | 0x80U);

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, STS_SetID(&len_servo, RT_NEW_ID));
    assert_len_outcome(rt_set_id_packet, sizeof(rt_set_id_packet), 1U, 1U, (len_counters_t){1U, 0U, 0U, 1U}, NULL);
}

/* --- Every API function --- */

typedef sts_result_t (*rt_call_t)(sts_servo_t *servo);
typedef struct {
    const char *name;
    rt_call_t   call;
    uint16_t    reply_len;
} rt_api_t;

static sts_result_t rt_get_position(sts_servo_t *s) { uint16_t v; return STS_GetPresentPosition(s, &v); }
static sts_result_t rt_get_speed(sts_servo_t *s)    { int16_t v;  return STS_GetPresentSpeed(s, &v); }
static sts_result_t rt_get_load(sts_servo_t *s)     { int16_t v;  return STS_GetPresentLoad(s, &v); }
static sts_result_t rt_get_voltage(sts_servo_t *s)  { uint8_t v;  return STS_GetPresentVoltage(s, &v); }
static sts_result_t rt_get_temp(sts_servo_t *s)     { uint8_t v;  return STS_GetPresentTemperature(s, &v); }
static sts_result_t rt_get_moving(sts_servo_t *s)   { uint8_t v;  return STS_GetMovingStatus(s, &v); }
static sts_result_t rt_read8(sts_servo_t *s)        { uint8_t v;  return STS_Read8(s, STS_REG_PRESENT_TEMP, &v); }
static sts_result_t rt_read16(sts_servo_t *s)       { uint16_t v; return STS_Read16(s, STS_REG_PRESENT_POSITION, &v); }
static sts_result_t rt_ping(sts_servo_t *s)         { return STS_servo_ping(s); }

static const rt_api_t rt_reads[] = {
    {"GetPresentPosition", rt_get_position, EXPECTED_READ16_ACK_LEN},
    {"GetPresentSpeed", rt_get_speed, EXPECTED_READ16_ACK_LEN},
    {"GetPresentLoad", rt_get_load, EXPECTED_READ16_ACK_LEN},
    {"GetPresentVoltage", rt_get_voltage, EXPECTED_READ8_ACK_LEN},
    {"GetPresentTemperature", rt_get_temp, EXPECTED_READ8_ACK_LEN},
    {"GetMovingStatus", rt_get_moving, EXPECTED_READ8_ACK_LEN},
    {"Read8", rt_read8, EXPECTED_READ8_ACK_LEN},
    {"Read16", rt_read16, EXPECTED_READ16_ACK_LEN},
    {"servo_ping", rt_ping, EXPECTED_WRITE_ACK_LEN},
};

static sts_result_t rt_torque(sts_servo_t *s)  { return STS_SetTorqueEnable(s, STS_TORQUE_ENABLE); }
static sts_result_t rt_mode(sts_servo_t *s)    { return STS_SetOperatingMode(s, STS_MODE_SPEED); }
static sts_result_t rt_goal(sts_servo_t *s)    { return STS_SetTargetPosition(s, LEN_POSITION_VALUE); }
static sts_result_t rt_speed(sts_servo_t *s)   { return STS_SetTargetSpeed(s, RT_STEPS, STS_DIR_CW); }
static sts_result_t rt_accel(sts_servo_t *s)   { return STS_SetTargetAcceleration(s, TEST_VAL_8BIT); }
static sts_result_t rt_pwm(sts_servo_t *s)     { return STS_SetTargetPWM(s, RT_STEPS, STS_DIR_CW); }
static sts_result_t rt_step(sts_servo_t *s)    { return STS_SetTargetStep(s, RT_STEPS, STS_DIR_CW); }
static sts_result_t rt_limit(sts_servo_t *s)   { return STS_SetTorqueLimit(s, RT_STEPS); }
static sts_result_t rt_lock(sts_servo_t *s)    { return STS_SetEEPROMLock(s, EEPROM_LOCK); }
static sts_result_t rt_set_id(sts_servo_t *s)  { return STS_SetID(s, RT_NEW_ID); }
static sts_result_t rt_write8(sts_servo_t *s)  { return STS_Write8(s, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE); }
static sts_result_t rt_write16(sts_servo_t *s) { return STS_Write16(s, STS_REG_GOAL_POSITION, LEN_POSITION_VALUE); }

static const rt_api_t rt_writes[] = {
    {"SetTorqueEnable", rt_torque, 0U}, {"SetOperatingMode", rt_mode, 0U}, {"SetTargetPosition", rt_goal, 0U},
    {"SetTargetSpeed", rt_speed, 0U}, {"SetTargetAcceleration", rt_accel, 0U}, {"SetTargetPWM", rt_pwm, 0U},
    {"SetTargetStep", rt_step, 0U}, {"SetTorqueLimit", rt_limit, 0U}, {"SetEEPROMLock", rt_lock, 0U},
    {"SetID", rt_set_id, 0U}, {"Write8", rt_write8, 0U}, {"Write16", rt_write16, 0U},
};

void test_STS_Every_Read_Function_Retried(void) {
    for (uint32_t i = 0U; i < (uint32_t)(sizeof(rt_reads) / sizeof(rt_reads[0])); i++) {
        uint8_t reply[EXPECTED_READ16_ACK_LEN] = {0};
        const uint8_t zeros[STS_DATA_LEN_16BIT] = {0};
        simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, zeros, (uint16_t)(rt_reads[i].reply_len - EXPECTED_WRITE_ACK_LEN), reply);
        const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {reply, rt_reads[i].reply_len, STS_OK} };
        use_rt_bus(steps, 2U, LEN_RETRIES);

        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_OK, rt_reads[i].call(&len_servo), rt_reads[i].name);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(2U, script.tx_count, rt_reads[i].name);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(script.tx_log_len[0], script.tx_log_len[1], rt_reads[i].name);
        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(script.tx_log[0], script.tx_log[1], script.tx_log_len[0], rt_reads[i].name);
        assert_len_outcome(NULL, 0U, 2U, 1U, (len_counters_t){1U, 1U, 1U, 0U}, rt_reads[i].name);
    }
}

void test_STS_Every_Write_Function_Single_Attempt(void) {
    for (uint32_t i = 0U; i < (uint32_t)(sizeof(rt_writes) / sizeof(rt_writes[0])); i++) {
        use_rt_bus(NULL, 0U, LEN_RETRIES);

        TEST_ASSERT_EQUAL_INT_MESSAGE(STS_ERR_TIMEOUT, rt_writes[i].call(&len_servo), rt_writes[i].name);
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(EXPECTED_INST_WRITE, script.tx_log[0][STS_IDX_INSTRUCTION], rt_writes[i].name);
        assert_len_outcome(NULL, 0U, 1U, 0U, (len_counters_t){1U, 0U, 0U, 1U}, rt_writes[i].name);
    }
}


/* --- Engine state between attempts --- */

static uint16_t short_rx_cut;   /* 0 = normal; otherwise a stage-2 receive gets this many bytes but reports OK */

/* Like the STM32 port on a short frame: returns OK although fewer bytes arrived than were requested. */
static sts_result_t short_ok_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    if (short_rx_cut != 0U && data != bus->rx_buf && len > short_rx_cut) {   /* stage 2 only */
        (void)mock_script_rx(bus, data, short_rx_cut, timeout_ms);
        return STS_OK;
    }
    return mock_script_rx(bus, data, len, timeout_ms);
}

/* A repeated read cut short must not be completed by the previous reply's bytes left in rx_buf. */
void test_STS_Read_Short_Reply_Not_Completed_By_Stale_Buffer(void) {
    const mock_step_t steps[] = { {len_valid, EXPECTED_READ16_ACK_LEN, STS_OK},
                                  {len_valid, EXPECTED_READ16_ACK_LEN - 2U, STS_OK} };
    use_rt_bus(steps, 2U, 0U);
    len_bus.bus.receive = short_ok_rx;
    uint16_t pos;

    short_rx_cut = 0U;
    TEST_ASSERT_EQUAL_INT(STS_OK, rt_read(&pos));
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_VALUE, pos);

    short_rx_cut = STS_DATA_LEN_16BIT;   /* stage 2 wants 4 bytes, gets 2 */
    sts_result_t res = rt_read(&pos);
    short_rx_cut = 0U;
    TEST_ASSERT_NOT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_HEX16(LEN_POSITION_SENTINEL, pos);
}

/* ID 0xFF cannot be framed: the write is rejected and nothing is sent or counted. */
void test_STS_Write_Not_Sent_When_Packet_Cannot_Be_Built(void) {
    use_rt_bus(NULL, 0U, LEN_RETRIES);
    len_servo.id = STS_ID_BROADCAST_ASYNC;
    sts_bus_t snapshot;
    memcpy(&snapshot, &len_bus.bus, sizeof(snapshot));

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_Write8(&len_servo, STS_REG_TORQUE_ENABLE, STS_TORQUE_ENABLE));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT32(snapshot.total_transactions, len_bus.bus.total_transactions);
    TEST_ASSERT_EQUAL_UINT32(snapshot.hard_failures, len_bus.bus.hard_failures);
}

/* ==========================================================================
 * VERIFIED WRITES
 * ========================================================================== */

#define VW_INST_READ                0x02U
#define VW_IDX_ADDRESS              5U
#define VW_IDX_VALUE                6U
#define VW_TORQUE_ON                1U
#define VW_TORQUE_OFF               0U
#define VW_ENABLE_NONZERO           7U
#define VW_LIMIT                    500U
#define VW_LIMIT_OTHER              400U
#define VW_LIMIT_TOO_HIGH           1001U
#define VW_MODE_TOO_HIGH            9U
#define VW_NEW_ID                   11U
#define VW_REPLY_SLOTS              4U

static uint8_t vw_rb8[VW_REPLY_SLOTS][EXPECTED_READ8_ACK_LEN];
static uint8_t vw_rb16[VW_REPLY_SLOTS][EXPECTED_READ16_ACK_LEN];
static uint8_t vw_ping_new[EXPECTED_WRITE_ACK_LEN];
static uint8_t vw_ping_new_hw[EXPECTED_WRITE_ACK_LEN];

/* 8-bit read reply from ID 10 carrying v, in a slot. */
static const uint8_t *vw_read8(uint8_t slot, uint8_t v) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, &v, PAYLOAD_LEN_1_BYTE, vw_rb8[slot]);
    return vw_rb8[slot];
}

/* 16-bit read reply from ID 10 carrying v (little-endian), in a slot. */
static const uint8_t *vw_read16(uint8_t slot, uint16_t v) {
    const uint8_t p[STS_DATA_LEN_16BIT] = { (uint8_t)(v & 0xFFU), (uint8_t)(v >> 8U) };
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, p, STS_DATA_LEN_16BIT, vw_rb16[slot]);
    return vw_rb16[slot];
}

static void use_vw_bus(const mock_step_t *steps, uint16_t step_count) {
    simulate_servo_response(VW_NEW_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, vw_ping_new);
    simulate_servo_response(VW_NEW_ID, STS_BIT_ERR_OVERHEAT, NULL, PAYLOAD_LEN_NONE, vw_ping_new_hw);
    use_rt_bus(steps, step_count, LEN_RETRIES);
}

/* Packets sent, in order: 'W' write, 'R' read, 'P' ping; each addressed to `ids[i]`, writes and reads at `reg`. */
static void assert_vw_traffic(const char *kinds, const uint8_t *ids, uint8_t reg) {
    uint32_t n = (uint32_t)strlen(kinds);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, script.tx_count, "packet count");
    for (uint32_t i = 0U; i < n; i++) {
        uint8_t inst = (kinds[i] == 'W') ? EXPECTED_INST_WRITE : (kinds[i] == 'R') ? VW_INST_READ : EXPECTED_INST_PING;
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(inst, script.tx_log[i][STS_IDX_INSTRUCTION], "instruction");
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(ids[i], script.tx_log[i][STS_IDX_ID], "packet id");
        if (kinds[i] != 'P') {
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(reg, script.tx_log[i][VW_IDX_ADDRESS], "register");
        }
    }
}

static const uint8_t vw_id10[] = {10U, 10U, 10U, 10U, 10U, 10U, 10U, 10U};

#define VW_ACK                      {rt_ack, EXPECTED_WRITE_ACK_LEN, STS_OK}
#define VW_LOST                     {NULL, 0U, STS_OK}
#define VW_RB8(slot, v)             {vw_read8((slot), (v)), EXPECTED_READ8_ACK_LEN, STS_OK}
#define VW_RB16(slot, v)            {vw_read16((slot), (v)), EXPECTED_READ16_ACK_LEN, STS_OK}
#define VW_COUNT(a)                 ((uint16_t)(sizeof(a) / sizeof((a)[0])))

/* --- Generic primitives: one write, one read-back, never a re-send --- */

void test_STS_Write8Verified_Confirmed(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
    TEST_ASSERT_EQUAL_HEX8(VW_TORQUE_ON, script.tx_log[0][VW_IDX_VALUE]);
}

void test_STS_Write8Verified_Mismatch_Not_Resent(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, VW_TORQUE_OFF), VW_ACK, VW_RB8(1U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

/* The acknowledgement was lost but the write took effect: the read-back proves it. */
void test_STS_Write8Verified_Lost_Ack_Confirmed_By_Read_Back(void) {
    const mock_step_t steps[] = { VW_LOST, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_Write8Verified_Lost_Ack_Not_Applied(void) {
    const mock_step_t steps[] = { VW_LOST, VW_RB8(0U, VW_TORQUE_OFF), VW_ACK, VW_RB8(1U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_Write8Verified_Read_Back_Failure_Reported(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WRRR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_Write16Verified_Confirmed(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
    TEST_ASSERT_EQUAL_UINT16(EXPECTED_WRITE16_PACKET_LEN, script.tx_log_len[0]);
    TEST_ASSERT_EQUAL_HEX8(STS_DATA_LEN_16BIT, script.tx_log[1][VW_IDX_VALUE]);
}

void test_STS_Write16Verified_Mismatch_Not_Resent(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16(0U, VW_LIMIT_OTHER), VW_ACK, VW_RB16(1U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

/* Nothing can be read back from a broadcast handle, so nothing is sent. */
void test_STS_WriteVerified_Rejects_Null_And_Broadcast(void) {
    use_vw_bus(NULL, 0U);
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Write8Verified(NULL, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_Write16Verified(NULL, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    len_servo.id = STS_ID_BROADCAST_SYNC;
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

/* --- Absolute registers: re-sent once when the read-back differs or fails --- */

void test_STS_SetTorqueEnableVerified_Confirmed(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Resent_Once_After_Mismatch(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, VW_TORQUE_OFF), VW_ACK, VW_RB8(1U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WRWR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Second_Mismatch_Final(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, VW_TORQUE_OFF), VW_ACK, VW_RB8(1U, VW_TORQUE_OFF),
                                  VW_ACK, VW_RB8(2U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WRWR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Lost_Ack_Not_Resent_When_Applied(void) {
    const mock_step_t steps[] = { VW_LOST, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Resent_After_Failed_Read_Back(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WRRRWR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Second_Read_Back_Failure_Final(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK, VW_LOST, VW_LOST, VW_LOST,
                                  VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WRRRWRRR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Normalises_Enable(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueEnableVerified(&len_servo, VW_ENABLE_NONZERO));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
    TEST_ASSERT_EQUAL_HEX8(VW_TORQUE_ON, script.tx_log[0][VW_IDX_VALUE]);
}

void test_STS_SetOperatingModeVerified_Resends_Once(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, STS_MODE_POSITION), VW_ACK, VW_RB8(1U, STS_MODE_SPEED) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetOperatingModeVerified(&len_servo, STS_MODE_SPEED));
    assert_vw_traffic("WRWR", vw_id10, STS_REG_OPERATION_MODE);
    TEST_ASSERT_EQUAL_HEX8(STS_MODE_SPEED, script.tx_log[2][VW_IDX_VALUE]);
}

void test_STS_SetOperatingModeVerified_Rejects_Invalid_Mode(void) {
    use_vw_bus(NULL, 0U);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetOperatingModeVerified(&len_servo, (sts_operating_mode_t)VW_MODE_TOO_HIGH));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetOperatingModeVerified(NULL, STS_MODE_SPEED));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

void test_STS_SetTorqueLimitVerified_Resent_Once_After_Mismatch(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16(0U, VW_LIMIT_OTHER), VW_ACK, VW_RB16(1U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WRWR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetTorqueLimitVerified_Rejects_Out_Of_Range(void) {
    use_vw_bus(NULL, 0U);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT_TOO_HIGH));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTorqueLimitVerified(NULL, VW_LIMIT));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

/* The maximum itself is a valid limit. */
void test_STS_SetTorqueLimitVerified_Accepts_Maximum(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16(0U, STS_MAX_TORQUE) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueLimitVerified(&len_servo, STS_MAX_TORQUE));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetTorqueEnableVerified_Rejects_Null(void) {
    use_vw_bus(NULL, 0U);
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTorqueEnableVerified(NULL, VW_TORQUE_ON));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

/* --- ID change: confirmed by a ping at the new ID, never re-sent --- */

static const uint8_t vw_id_write_then_ping[] = {10U, VW_NEW_ID, VW_NEW_ID, VW_NEW_ID};

void test_STS_SetIDVerified_Confirmed_Updates_Handle(void) {
    const mock_step_t steps[] = { VW_ACK, {vw_ping_new, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    assert_vw_traffic("WP", vw_id_write_then_ping, STS_REG_ID);
    TEST_ASSERT_EQUAL_HEX8(VW_NEW_ID, script.tx_log[0][VW_IDX_VALUE]);
    TEST_ASSERT_EQUAL_UINT8(VW_NEW_ID, len_servo.id);
}

void test_STS_SetIDVerified_Unconfirmed_Keeps_Handle(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    assert_vw_traffic("WPPP", vw_id_write_then_ping, STS_REG_ID);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, len_servo.id);
}

void test_STS_SetIDVerified_Lost_Ack_Confirmed_By_Ping(void) {
    const mock_step_t steps[] = { VW_LOST, {vw_ping_new, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    assert_vw_traffic("WP", vw_id_write_then_ping, STS_REG_ID);
    TEST_ASSERT_EQUAL_UINT8(VW_NEW_ID, len_servo.id);
}

/* A fault flag in the ping reply still proves the servo answers at the new ID. */
void test_STS_SetIDVerified_Hardware_Status_Confirms(void) {
    const mock_step_t steps[] = { VW_ACK, {vw_ping_new_hw, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    TEST_ASSERT_EQUAL_UINT8(VW_NEW_ID, len_servo.id);
}

/* A misconfigured bus sends nothing, so the argument error is reported, not VERIFY. */
void test_STS_SetIDVerified_Reports_Rejected_Write(void) {
    use_vw_bus(NULL, 0U);
    len_bus.bus.max_read_retries = RETRIES_WRAPPING;

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, len_servo.id);
}

void test_STS_SetIDVerified_Rejects_Broadcast_And_Null(void) {
    use_vw_bus(NULL, 0U);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetIDVerified(&len_servo, STS_ID_BROADCAST_SYNC));
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetIDVerified(&len_servo, STS_ID_BROADCAST_ASYNC));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetIDVerified(NULL, VW_NEW_ID));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, len_servo.id);
}

/* --- 16-bit fault paths --- */

void test_STS_Write16Verified_Lost_Ack_Confirmed_By_Read_Back(void) {
    const mock_step_t steps[] = { VW_LOST, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_Write16Verified_Lost_Ack_Not_Applied(void) {
    const mock_step_t steps[] = { VW_LOST, VW_RB16(0U, VW_LIMIT_OTHER), VW_ACK, VW_RB16(1U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_Write16Verified_Read_Back_Failure_Reported(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WRRR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetTorqueLimitVerified_Second_Mismatch_Final(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16(0U, VW_LIMIT_OTHER), VW_ACK, VW_RB16(1U, VW_LIMIT_OTHER),
                                  VW_ACK, VW_RB16(2U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WRWR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetTorqueLimitVerified_Lost_Ack_Not_Resent_When_Applied(void) {
    const mock_step_t steps[] = { VW_LOST, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetTorqueLimitVerified_Resent_After_Failed_Read_Back(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WRRRWR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetTorqueLimitVerified_Second_Read_Back_Failure_Final(void) {
    const mock_step_t steps[] = { VW_ACK, VW_LOST, VW_LOST, VW_LOST, VW_ACK, VW_LOST, VW_LOST, VW_LOST,
                                  VW_ACK, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_TIMEOUT, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WRRRWRRR", vw_id10, STS_REG_TORQUE_LIMIT);
}

/* --- Boundaries and handle checks --- */

#define VW_LAST_UNICAST_ID          253U
static const uint8_t vw_id253[] = {VW_LAST_UNICAST_ID, VW_LAST_UNICAST_ID};

static void vw_from_id(uint8_t id) {
    (void)STS_Servo_Init(&len_servo, &len_bus.bus, id);
}

void test_STS_SetOperatingModeVerified_Accepts_Step_Mode(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8(0U, STS_MODE_STEP) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetOperatingModeVerified(&len_servo, STS_MODE_STEP));
    assert_vw_traffic("WR", vw_id10, STS_REG_OPERATION_MODE);
}

void test_STS_SetOperatingModeVerified_Rejects_First_Invalid_Mode(void) {
    use_vw_bus(NULL, 0U);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetOperatingModeVerified(&len_servo, (sts_operating_mode_t)(STS_MODE_STEP + 1U)));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

/* ID 253 is the last unicast ID: a valid handle and a valid target. */
void test_STS_Write8Verified_Works_From_Id_253(void) {
    uint8_t one = VW_TORQUE_ON;
    uint8_t rb[EXPECTED_READ8_ACK_LEN];
    uint8_t ack[EXPECTED_WRITE_ACK_LEN];
    simulate_servo_response(VW_LAST_UNICAST_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, ack);
    simulate_servo_response(VW_LAST_UNICAST_ID, STS_STATUS_OK, &one, PAYLOAD_LEN_1_BYTE, rb);
    const mock_step_t steps[] = { {ack, EXPECTED_WRITE_ACK_LEN, STS_OK}, {rb, EXPECTED_READ8_ACK_LEN, STS_OK} };
    use_vw_bus(steps, VW_COUNT(steps));
    vw_from_id(VW_LAST_UNICAST_ID);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id253, STS_REG_TORQUE_ENABLE);
}

void test_STS_Write16Verified_Works_From_Id_253(void) {
    const uint8_t p[STS_DATA_LEN_16BIT] = { (uint8_t)(VW_LIMIT & 0xFFU), (uint8_t)(VW_LIMIT >> 8U) };
    uint8_t rb[EXPECTED_READ16_ACK_LEN];
    uint8_t ack[EXPECTED_WRITE_ACK_LEN];
    simulate_servo_response(VW_LAST_UNICAST_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, ack);
    simulate_servo_response(VW_LAST_UNICAST_ID, STS_STATUS_OK, p, STS_DATA_LEN_16BIT, rb);
    const mock_step_t steps[] = { {ack, EXPECTED_WRITE_ACK_LEN, STS_OK}, {rb, EXPECTED_READ16_ACK_LEN, STS_OK} };
    use_vw_bus(steps, VW_COUNT(steps));
    vw_from_id(VW_LAST_UNICAST_ID);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id253, STS_REG_TORQUE_LIMIT);
}

void test_STS_SetIDVerified_Accepts_Id_253(void) {
    uint8_t ping[EXPECTED_WRITE_ACK_LEN];
    simulate_servo_response(VW_LAST_UNICAST_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, ping);
    const mock_step_t steps[] = { VW_ACK, {ping, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    const uint8_t ids[] = {TEST_VALID_ID, VW_LAST_UNICAST_ID};
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetIDVerified(&len_servo, VW_LAST_UNICAST_ID));
    assert_vw_traffic("WP", ids, STS_REG_ID);
    TEST_ASSERT_EQUAL_UINT8(VW_LAST_UNICAST_ID, len_servo.id);
}

void test_STS_SetIDVerified_Works_From_Id_253(void) {
    uint8_t ack[EXPECTED_WRITE_ACK_LEN];
    simulate_servo_response(VW_LAST_UNICAST_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, ack);
    const mock_step_t steps[] = { {ack, EXPECTED_WRITE_ACK_LEN, STS_OK}, {vw_ping_new, EXPECTED_WRITE_ACK_LEN, STS_OK} };
    const uint8_t ids[] = {VW_LAST_UNICAST_ID, VW_NEW_ID};
    use_vw_bus(steps, VW_COUNT(steps));
    vw_from_id(VW_LAST_UNICAST_ID);

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    assert_vw_traffic("WP", ids, STS_REG_ID);
    TEST_ASSERT_EQUAL_UINT8(VW_NEW_ID, len_servo.id);
}

/* A broadcast write to 0x05 would renumber every servo on the bus. */
void test_STS_SetIDVerified_Rejects_Broadcast_Handle(void) {
    use_vw_bus(NULL, 0U);
    len_servo.id = STS_ID_BROADCAST_SYNC;
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    len_servo.id = STS_ID_BROADCAST_ASYNC;
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
}

void test_STS_SetIDVerified_Reports_Null_Bus(void) {
    use_vw_bus(NULL, 0U);
    len_servo.bus = NULL;

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetIDVerified(&len_servo, VW_NEW_ID));
    TEST_ASSERT_EQUAL_UINT32(0U, script.tx_count);
    TEST_ASSERT_EQUAL_UINT8(TEST_VALID_ID, len_servo.id);
}

/* --- A servo flagging a fault still answers: a matching read-back is confirmed --- */

static uint8_t vw_rb8_hw[VW_REPLY_SLOTS][EXPECTED_READ8_ACK_LEN];
static uint8_t vw_rb16_hw[VW_REPLY_SLOTS][EXPECTED_READ16_ACK_LEN];

static const uint8_t *vw_read8_hw(uint8_t slot, uint8_t v) {
    simulate_servo_response(TEST_VALID_ID, STS_BIT_ERR_OVERHEAT, &v, PAYLOAD_LEN_1_BYTE, vw_rb8_hw[slot]);
    return vw_rb8_hw[slot];
}

static const uint8_t *vw_read16_hw(uint8_t slot, uint16_t v) {
    const uint8_t p[STS_DATA_LEN_16BIT] = { (uint8_t)(v & 0xFFU), (uint8_t)(v >> 8U) };
    simulate_servo_response(TEST_VALID_ID, STS_BIT_ERR_OVERHEAT, p, STS_DATA_LEN_16BIT, vw_rb16_hw[slot]);
    return vw_rb16_hw[slot];
}

#define VW_RB8_HW(slot, v)          {vw_read8_hw((slot), (v)), EXPECTED_READ8_ACK_LEN, STS_OK}
#define VW_RB16_HW(slot, v)         {vw_read16_hw((slot), (v)), EXPECTED_READ16_ACK_LEN, STS_OK}

void test_STS_Write8Verified_Faulted_Servo_Match_Reports_Hardware(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8_HW(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_Write8Verified_Faulted_Servo_Mismatch_Is_Verify(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8_HW(0U, VW_TORQUE_OFF) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write8Verified(&len_servo, STS_REG_TORQUE_ENABLE, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_Write16Verified_Faulted_Servo_Match_Reports_Hardware(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16_HW(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

void test_STS_Write16Verified_Faulted_Servo_Mismatch_Is_Verify(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16_HW(0U, VW_LIMIT_OTHER) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write16Verified(&len_servo, STS_REG_TORQUE_LIMIT, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

/* Torque off on an overheated servo: confirmed and not re-sent, with the fault still reported. */
void test_STS_SetTorqueEnableVerified_Off_On_Faulted_Servo_Not_Resent(void) {
    const mock_step_t steps[] = { {rt_ack_hw, EXPECTED_WRITE_ACK_LEN, STS_OK}, VW_RB8_HW(0U, VW_TORQUE_OFF),
                                  VW_ACK, VW_RB8(1U, VW_TORQUE_OFF) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_OFF));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueEnableVerified_Faulted_Servo_Mismatch_Resent(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB8_HW(0U, VW_TORQUE_ON), VW_ACK, VW_RB8_HW(1U, VW_TORQUE_OFF) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_OFF));
    assert_vw_traffic("WRWR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueLimitVerified_Faulted_Servo_Match_Not_Resent(void) {
    const mock_step_t steps[] = { VW_ACK, VW_RB16_HW(0U, VW_LIMIT), VW_ACK, VW_RB16(1U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_HARDWARE, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}

/* A noise byte ahead of a short faulted status packet: the parser finds the packet, but the reply in rx_buf
 * does not start at its header, so no value can be trusted and the read-back is not confirmed. */
#define VW_NOISE_ID                 3U
void test_STS_Write8Verified_Unreadable_Faulted_Reply_Is_Verify(void) {
    uint8_t ack[EXPECTED_WRITE_ACK_LEN];
    uint8_t status[EXPECTED_WRITE_ACK_LEN];
    uint8_t frame[EXPECTED_READ8_ACK_LEN];
    simulate_servo_response(VW_NOISE_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, ack);
    simulate_servo_response(VW_NOISE_ID, STS_BIT_ERR_OVERHEAT, NULL, PAYLOAD_LEN_NONE, status);
    frame[0] = 0x00U;                                   /* noise byte */
    memcpy(&frame[1], status, sizeof(status));
    const mock_step_t steps[] = { {ack, EXPECTED_WRITE_ACK_LEN, STS_OK}, {frame, EXPECTED_READ8_ACK_LEN, STS_OK} };
    const uint8_t ids[] = {VW_NOISE_ID, VW_NOISE_ID};
    use_vw_bus(steps, VW_COUNT(steps));
    vw_from_id(VW_NOISE_ID);

    /* The noise byte shifts the packet's ID (3) into the length position: stage 1 accepts 7 bytes. */
    TEST_ASSERT_EQUAL_HEX8(VW_NOISE_ID, frame[IDX_LENGTH_BYTE]);
    /* The value written equals the byte a misaligned read would take (the status, 0x04): only the alignment
     * check stops a false confirmation. */
    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write8Verified(&len_servo, STS_REG_ACCELERATION, STS_BIT_ERR_OVERHEAT));
    assert_vw_traffic("WR", ids, STS_REG_ACCELERATION);

    /* Writing 0: no value at all may be taken from the unreadable reply. */
    use_vw_bus(steps, VW_COUNT(steps));
    vw_from_id(VW_NOISE_ID);
    TEST_ASSERT_EQUAL_INT(STS_ERR_VERIFY, STS_Write8Verified(&len_servo, STS_REG_ACCELERATION, 0U));
    assert_vw_traffic("WR", ids, STS_REG_ACCELERATION);
}

/* A transport caller error fails the same way again: reported, with no re-send. */
void test_STS_SetTorqueEnableVerified_Caller_Error_Not_Resent(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_NULL_PTR}, {NULL, 0U, STS_ERR_NULL_PTR},
                                  VW_ACK, VW_RB8(0U, VW_TORQUE_ON) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTorqueEnableVerified(&len_servo, VW_TORQUE_ON));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_ENABLE);
}

void test_STS_SetTorqueLimitVerified_Caller_Error_Not_Resent(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_ERR_INVALID_PARAM}, {NULL, 0U, STS_ERR_INVALID_PARAM},
                                  VW_ACK, VW_RB16(0U, VW_LIMIT) };
    use_vw_bus(steps, VW_COUNT(steps));

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, STS_SetTorqueLimitVerified(&len_servo, VW_LIMIT));
    assert_vw_traffic("WR", vw_id10, STS_REG_TORQUE_LIMIT);
}
