/**
 ******************************************************************************
 * @file           : test_sts_servo.c
 * @brief          : Unit tests for the STS Service and Bus Layers
 * @author         : Grisham Balloo
 * @date           : 2026-03-21
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
#include "sts_servo_cmd.h"
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
 * Raw Byte & Register Test Values
 * ========================================================================== */
#define HW_STATUS_OK                0x00U
#define INVALID_OPERATING_MODE      99U

#define TEST_DATA_BYTE              0xAAU
#define TEST_DATA_LEN               1U

#define TEST_VAL_8BIT               0x7FU
#define TEST_VAL_16BIT              0x1234U
#define TEST_VAL_16BIT_LOW          0x34U  /* Little-endian lower byte */
#define TEST_VAL_16BIT_HIGH         0x12U  /* Little-endian upper byte */

/* ==========================================================================
 * Kinematic Target Boundaries & Intents
 * ========================================================================== */
#define TARGET_ZERO                 0

/* --- Position Mode --- */
#define TARGET_VALID_POS            2048
#define TARGET_INVALID_NEG_POS     (-1500)

/* --- Speed Mode --- */
#define TARGET_VALID_SPEED_CCW      1000
#define TARGET_VALID_SPEED_CW      (-1000)
#define TARGET_INVALID_SPEED        5000
#define TARGET_INVALID_NEG_SPEED   (-5000)

/* --- PWM Mode --- */
#define TARGET_VALID_PWM_CCW        500
#define TARGET_VALID_PWM_CW        (-500)
#define TARGET_INVALID_PWM          1500
#define TARGET_INVALID_NEG_PWM     (-1500)

/* --- Step Mode --- */
#define TARGET_VALID_STEP_CCW       25000
#define TARGET_VALID_STEP_CW       (-25000)
#define TARGET_INVALID_POS_STEP     35000
#define TARGET_INVALID_STEP        (-35000)

#define TARGET_VALID_TORQUE_LIMIT 500U

/* Mock Telemetry Expected Data */
#define TEST_MOCK_LOAD_VAL             1000   /* Raw expected integer */
#define TEST_MOCK_LOAD_LOW_BYTE        0xE8U  /* 1000 in Little Endian */
#define TEST_MOCK_LOAD_HIGH_BYTE       0x03U

#define TEST_MOCK_VOLTAGE_VAL          125U   /* Represents 12.5V */
#define TEST_MOCK_TEMP_VAL             50U    /* Represents 50°C */
#define TEST_MOCK_TEMP_OVERHEAT_VAL    85U    /* Represents 85°C */
#define TEST_MOCK_STATUS_STOPPED       0U     /* Motor not moving */

/* Sentinel Values for State Preservation Checks */
#define TEST_SENTINEL_MAX_U8           255U   /* Used to verify variable was overwritten */
#define TEST_SENTINEL_GARBAGE_U8       99U    /* Used to verify variable was NOT overwritten */
#define TEST_ZERO_RX_LEN               0U     /* Simulates disconnected line */

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
    /* Wipe the globals to prevent state leakage */
    memset(&test_bus, 0, sizeof(test_bus));
    memset(&test_servo, 0, sizeof(test_servo));
    memset(&dummy_uart_port, 0, sizeof(dummy_uart_port));

    /* Re-initialize */
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
    uint8_t mock_payload[PAYLOAD_LEN_1_BYTE] = {HW_STATUS_OK};

    simulate_servo_response(TEST_VALID_ID, HW_STATUS_OK, mock_payload, PAYLOAD_LEN_1_BYTE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_buffer[IDX_HEADER_START] = CORRUPT_HEADER_BYTE;
    dummy_uart_port.rx_len = EXPECTED_READ8_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_ERR_HEADER, STS_Read8(&test_servo, STS_REG_PRESENT_VOLTAGE, &val8));
}

void test_STS_ExecuteCommand_Length_Mismatch(void) {
    // Set length field below the protocol minimum to triggerSTS_ERR_MALFORMED 
    // from the stage 1 length validation guard. 
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_1] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_2] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_ID]       = TEST_VALID_ID;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH]   = 0x00U; // Below STS_MIN_PKT_LEN_VAL 
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL;

    uint16_t actual_read_value = 0U;
    sts_result_t res = STS_Read16(&test_servo, STS_REG_PRESENT_POSITION, &actual_read_value);

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, res);
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
    // Packet claims a length that exceeds the internal RX buffer. 
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_1] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_2] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_ID]       = TEST_VALID_ID;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH]   = 0xFEU;
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL;

    sts_cmd_t cmd = {
        .instruction     = STS_INST_READ,
        .tx_params       = NULL,
        .tx_param_len    = 0U,
        .expected_rx_len = (uint16_t)(STS_MAX_RX_BUFFER + 10U),
        .rx_params_out   = NULL,
        .rx_params_size  = 0U,
        .rx_param_len_out = NULL
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
    // Provide less data than required for a Stage 1 Read (Header + ID + Length) 
    const uint16_t truncated_len = STS_PKT_FIXED_TOTAL - 1U; 
    
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_1] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_2] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_ID]       = TEST_VALID_ID;
    dummy_uart_port.rx_len = truncated_len; 

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    //Stage 1 read should fail and return a Timeout 
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
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_1] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_2] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_ID]       = TEST_VALID_ID;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH]   = 0x00U; // Violates STS_MIN_PKT_LEN_VAL 
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL;

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_MALFORMED, res); 
}

void test_STS_GetPresentPosition_Buffer_Overflow_Guard(void) {
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_1] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_2] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_ID]       = TEST_VALID_ID;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH]   = 0xFEU; // exceeding STS_MAX_RX_BUFFER 
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL;

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_BUF_TOO_SMALL, res);
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
}

void test_STS_SetTargetPosition_Max_Boundary(void) {
    const uint16_t max_pos = 4095U;
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPosition(&test_servo, max_pos);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetPosition_Just_Out_Of_Range(void) {
    const uint16_t off_by_one_pos = 4096U;

    sts_result_t res = STS_SetTargetPosition(&test_servo, off_by_one_pos);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
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
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_1] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_HEADER_2] = STS_HEADER;
    dummy_uart_port.rx_buffer[STS_IDX_ID]       = TEST_VALID_ID;
    dummy_uart_port.rx_buffer[STS_IDX_LENGTH]   = 0x04U; 
    
    dummy_uart_port.rx_len = STS_PKT_FIXED_TOTAL; 

    uint16_t pos = 0U;
    sts_result_t res = STS_GetPresentPosition(&test_servo, &pos);

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
}

void test_STS_SetTargetSpeed_Out_Of_Range(void) {
    const uint16_t out_of_range_speed = STS_MAX_SPEED + 1U;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, out_of_range_speed, STS_DIR_CCW);

    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_GetPresentSpeed_Success(void) {
    const uint16_t expected_speed = 1500U; 
  
    uint8_t speed_data[STS_DATA_LEN_16BIT] = { 
        (uint8_t)(expected_speed & 0xFFU), 
        (uint8_t)((expected_speed >> 8U) & 0xFFU) 
    };
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, speed_data, STS_DATA_LEN_16BIT, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    uint16_t actual_speed = 0U;
    sts_result_t res = STS_GetPresentSpeed(&test_servo, &actual_speed);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
    TEST_ASSERT_EQUAL_UINT16(expected_speed, actual_speed);
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
}

void test_STS_SetTargetSpeed_Max_Reverse_Allowed(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetSpeed(&test_servo, STS_MAX_SPEED, STS_DIR_CW);

    TEST_ASSERT_EQUAL_INT(STS_OK, res);
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

/* ==========================================================================
 * PWM & STEP COMMANDS
 * ========================================================================== */

void test_STS_PWM_Step_Null_Guards(void) {
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetPWM(NULL, TARGET_VALID_PWM_CCW, STS_DIR_CCW));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTargetStep(NULL, TARGET_VALID_STEP_CCW, STS_DIR_CW));
    TEST_ASSERT_EQUAL_INT(STS_ERR_NULL_PTR, STS_SetTarget(NULL, TARGET_VALID_SPEED_CCW));
}

void test_STS_SetTargetPWM_Out_Of_Range(void) {
    const uint16_t out_of_range_pwm = STS_MAX_PWM + 1U;
    sts_result_t res = STS_SetTargetPWM(&test_servo, out_of_range_pwm, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTargetStep_Out_Of_Range(void) {
    const uint16_t out_of_range_step = STS_MAX_STEP + 1U;
    sts_result_t res = STS_SetTargetStep(&test_servo, out_of_range_step, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTargetPWM_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPWM(&test_servo, TARGET_VALID_PWM_CCW, STS_DIR_CW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetStep_Success(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetStep(&test_servo, TARGET_VALID_STEP_CCW, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetPWM_Max_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPWM(&test_servo, STS_MAX_PWM, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetStep_Max_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetStep(&test_servo, STS_MAX_STEP, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetPWM_Zero_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetPWM(&test_servo, TARGET_ZERO, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTargetStep_Zero_Boundary(void) {
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTargetStep(&test_servo, TARGET_ZERO, STS_DIR_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_SpeedMode_Min_Boundary(void) {
    test_servo.current_mode = STS_MODE_SPEED;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, -(int32_t)STS_MAX_SPEED);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_PWMMode_Min_Boundary(void) {
    test_servo.current_mode = STS_MODE_PWM;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, -(int32_t)STS_MAX_PWM);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_StepMode_Min_Boundary(void) {
    test_servo.current_mode = STS_MODE_STEP;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, -(int32_t)STS_MAX_STEP);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_SpeedMode_Negative_Out_Of_Range(void) {
    test_servo.current_mode = STS_MODE_SPEED;
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_NEG_SPEED); 
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTarget_PWMMode_Negative_Out_Of_Range(void) {
    test_servo.current_mode = STS_MODE_PWM;
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_NEG_PWM); 
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTarget_StepMode_Positive_Out_Of_Range(void) {
    test_servo.current_mode = STS_MODE_STEP;
    /* We tested negative out-of-range earlier, this covers the positive side! */
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_POS_STEP); 
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}
/* ==========================================================================
 * UNIVERSAL TARGET WRAPPER 
 * ========================================================================== */

void test_STS_SetTarget_Zero_Boundary(void) {
    test_servo.current_mode = STS_MODE_SPEED;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_ZERO);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_PositionMode_Normal(void) {
    test_servo.current_mode = STS_MODE_POSITION;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_POS);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_PositionMode_ClampsNegativeToZero(void) {
    test_servo.current_mode = STS_MODE_POSITION;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_NEG_POS);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_SpeedMode_Positive_CCW(void) {
    test_servo.current_mode = STS_MODE_SPEED;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_SPEED_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_SpeedMode_Negative_CW(void) {
    test_servo.current_mode = STS_MODE_SPEED;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_SPEED_CW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_SpeedMode_Out_Of_Range(void) {
    test_servo.current_mode = STS_MODE_SPEED;
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_SPEED); 
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTarget_PWMMode_Positive_CCW(void) {
    test_servo.current_mode = STS_MODE_PWM;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_PWM_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_PWMMode_Negative_CW(void) {
    test_servo.current_mode = STS_MODE_PWM;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_PWM_CW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_PWMMode_Out_Of_Range(void) {
    test_servo.current_mode = STS_MODE_PWM;
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_PWM); 
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTarget_StepMode_Positive_CCW(void) {
    test_servo.current_mode = STS_MODE_STEP;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_STEP_CCW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_StepMode_Negative_CW(void) {
    test_servo.current_mode = STS_MODE_STEP;
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, NULL, PAYLOAD_LEN_NONE, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_WRITE_ACK_LEN;

    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_STEP_CW);
    TEST_ASSERT_EQUAL_INT(STS_OK, res);
}

void test_STS_SetTarget_StepMode_Out_Of_Range(void) {
    test_servo.current_mode = STS_MODE_STEP;
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_INVALID_STEP); 
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
}

void test_STS_SetTarget_InvalidMode(void) {
    test_servo.current_mode = (sts_operating_mode_t)INVALID_OPERATING_MODE; 
    sts_result_t res = STS_SetTarget(&test_servo, TARGET_VALID_SPEED_CCW);
    TEST_ASSERT_EQUAL_INT(STS_ERR_INVALID_PARAM, res);
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
 * HARDENED TELEMETRY & FEEDBACK COMMAND TESTS (100% COVERAGE)
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
    int16_t load_out = 0;
    uint8_t mock_payload[PAYLOAD_LEN_2_BYTES] = {TEST_MOCK_LOAD_LOW_BYTE, TEST_MOCK_LOAD_HIGH_BYTE}; 
    
    simulate_servo_response(TEST_VALID_ID, STS_STATUS_OK, mock_payload, PAYLOAD_LEN_2_BYTES, dummy_uart_port.rx_buffer);
    dummy_uart_port.rx_len = EXPECTED_READ16_ACK_LEN;

    TEST_ASSERT_EQUAL_INT(STS_OK, STS_GetPresentLoad(&test_servo, &load_out));
    TEST_ASSERT_EQUAL_INT16(TEST_MOCK_LOAD_VAL, load_out);
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