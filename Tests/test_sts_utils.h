/**
 ******************************************************************************
 * @file           : test_sts_utils.h
 * @brief          : Header for STS test support utilities
 * @author         : Grisham Balloo
 * @date           : 2026-03-08
 * @version        : 1.1.0
 ******************************************************************************
 * @details
  * This file provides a shared testing harness for the STS driver.
 * It includes:
 * 1. Hardware Mocks: Simulating UART peripherals and HAL function pointers.
 *    mock_rx consumes rx_buffer incrementally to support multi-stage reads,
 *    and both mock functions track call counts via rx_call_count and
 *    tx_call_count for behavioural verification in tests.
 * 2. Protocol Simulators: Constructing raw byte streams to test the parser.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>
#include "sts_protocol.h"
#include "sts_servo.h"

#define TEST_DUMMY_BYTE_EE 0xEEU


/* Fills a buffer with a known byte to detect partial overwrites */
#define POISON_BUFFER(buf, size, fill_byte) memset((buf), (fill_byte), (size))

/* Compares a struct with a baseline snapshot*/
#define TEST_ASSERT_STRUCT_UNCHANGED(snapshot_ptr, actual_ptr, struct_type) \
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE((snapshot_ptr), (actual_ptr), sizeof(struct_type), "Struct has mutated unexpectedly")

    
/** * @brief A mock testing structure to simulate hardware data queues.
 * Used by mock_tx and mock_rx to inject and verify STS protocol packets.
 */
typedef struct {
    uint8_t rx_buffer[256]; /**< Buffer to hold fake responses */
    uint16_t rx_len;        /**< Number of bytes ready to be read */
    uint32_t rx_call_count; /**< Track how many times RX was called */

    uint8_t tx_buffer[256]; /**< Capture what the driver sends */
    uint16_t tx_len;        /**< Length of captured TX data */
    uint32_t tx_call_count; /**< Track how many times TX was called */

    uint32_t last_timeout_ms;
} mock_uart_t;

extern mock_uart_t dummy_uart_port;

/**
 * @brief Mock transmit function. Captures outgoing data into port->tx_buffer
 *        and increments tx_call_count. Always returns STS_OK.
 *
 * @param[in]  bus   Bus handle with port_handle pointing to a mock_uart_t.
 * @param[in]  data  Data buffer to transmit.
 * @param[in]  len   Number of bytes to transmit.
 *
 * @return STS_OK on success.
 * @return STS_ERR_NULL_PTR if bus, port_handle, or data is NULL.
 */
sts_result_t mock_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len);

/**
 * @brief Mock receive function. Consumes port->rx_buffer incrementally to
 *        support multi-stage reads. Returns STS_ERR_TIMEOUT if fewer than
 *        len bytes are available. Increments rx_call_count on each call.
 *
 * @param[in]  bus         Bus handle with port_handle pointing to a mock_uart_t.
 * @param[out] data        Buffer to write received bytes into.
 * @param[in]  len         Number of bytes to read.
 * @param[in]  timeout_ms  Timeout in milliseconds (unused in mock).
 *
 * @return STS_OK on success.
 * @return STS_ERR_NULL_PTR if bus, port_handle, or data is NULL.
 * @return STS_ERR_TIMEOUT if port->rx_len is less than len.
 */
sts_result_t mock_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms);

sts_result_t mock_tx_bare_metal(sts_bus_t *bus, const uint8_t *data, uint16_t len);

/** 
 * @brief Mock Transmit Function that simulates a hardware failure (e.g., cable disconnected).
 */
sts_result_t mock_tx_fail(sts_bus_t *bus, const uint8_t *data, uint16_t len);


/** 
 * @brief Mock Transmit Function that simulates a busy state (e.g., UART buffer full).
 */
sts_result_t mock_tx_busy(sts_bus_t *bus, const uint8_t *data, uint16_t len);

/**
 * @brief Manually constructs a raw byte array representing a servo response.
 * This skips the production "CreatePacket" logic so we can test the parser 
 * with intentionally corrupted or specific data.
 */
void simulate_servo_response(uint8_t id, uint8_t status, const uint8_t* params, uint16_t p_len, uint8_t* out_buf);


/**
 * @brief Asserts that a buffer has not been mutated from its initial sentinel state.
 * Reduces boilerplate and clarifies test intent.
 */
void TEST_ASSERT_BUFFER_UNCHANGED(const uint8_t *buffer, size_t size, uint8_t expected_fill);