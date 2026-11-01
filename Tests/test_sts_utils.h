/**
 ******************************************************************************
 * @file           : test_sts_utils.h
 * @brief          : Header for STS test support utilities
 * @author         : Grisham Balloo
 * @date           : 2026-03-08
 ******************************************************************************
 * @details
  * This file provides a shared testing harness for the STS driver.
 * It includes:
 * 1. Hardware Mocks: Simulating UART peripherals and HAL function pointers.
 *    mock_rx consumes rx_buffer incrementally to support multi-stage reads,
 *    and both mock functions track call counts via rx_call_count and
 *    tx_call_count for behavioural verification in tests.
 * 2. Protocol Simulators: Constructing raw byte streams to test the parser.
 * 3. Scripted Transport: mock_script_* bus callbacks with a reply (or none) per
 *    transmit, failure injection, flush counting and a log of every packet sent.
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


/* ==========================================================================
 * SCRIPTED TRANSPORT
 * Per-transmit replies, failure injection, and a log of every packet sent.
 * Bind a bus to mock_script_tx/rx/flush with port_handle pointing to a mock_script_t.
 * ========================================================================== */
#define MOCK_SCRIPT_QUEUE_SIZE  256U
#define MOCK_SCRIPT_PACKET_MAX  STS_MAX_TX_BUFFER  /* Longest packet the engine can send */
#define MOCK_SCRIPT_TX_MAX      300U  /* More transmits than one call can make: max_retries is a uint8_t */

/* Reasons passed to on_fail */
#define MOCK_SCRIPT_FAIL_CAP    "Scripted transport (test_sts_utils.c): transmit cap exceeded"
#define MOCK_SCRIPT_FAIL_PACKET "Scripted transport (test_sts_utils.c): packet longer than the log holds"
#define MOCK_SCRIPT_FAIL_QUEUE  "Scripted transport (test_sts_utils.c): receive queue overflow"

/** @brief What one transmit returns, and the reply it queues. */
typedef struct {
    const uint8_t *reply;      /**< Queued when the transmit succeeds; NULL = reply lost */
    uint16_t       reply_len;
    sts_result_t   tx_result;  /**< Returned by the transmit; anything but STS_OK queues nothing */
} mock_step_t;

/** @brief Scripted transport state. Step k applies to transmit k; past the last step, no reply. */
typedef struct {
    const mock_step_t *steps;
    uint16_t           step_count;
    uint8_t            discard_on_tx;   /**< 1 = each transmit clears unread bytes, as the STM32 port does */
    uint32_t           rx_fail_call;    /**< 1-based receive call that returns rx_fail_result; 0 = none */
    sts_result_t       rx_fail_result;
    uint32_t           tx_cap;          /**< Transmits allowed; 0 or above MOCK_SCRIPT_TX_MAX means MOCK_SCRIPT_TX_MAX */
    void             (*on_fail)(const char *reason); /**< Script errors; reset installs mock_script_default_fail */

    uint8_t  queue[MOCK_SCRIPT_QUEUE_SIZE];
    uint16_t queue_len;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t flush_count;
    uint8_t  tx_log[MOCK_SCRIPT_TX_MAX][MOCK_SCRIPT_PACKET_MAX];
    uint16_t tx_log_len[MOCK_SCRIPT_TX_MAX];
} mock_script_t;

/** @brief Default on_fail: fails the current test with the reason. */
void mock_script_default_fail(const char *reason);

/** @brief Clears all state, installs the steps and the default on_fail. */
void mock_script_reset(mock_script_t *script, const mock_step_t *steps, uint16_t step_count);

/**
 * @brief Logs the packet, clears unread bytes if discard_on_tx, then applies this transmit's step.
 *        A transmit past the cap or an over-long packet calls on_fail without being logged;
 *        a queue overflow calls on_fail after the packet is logged.
 * @return The step's tx_result (STS_OK past the last step); STS_ERR_TX_FAIL after on_fail returns;
 *         STS_ERR_NULL_PTR on a NULL argument.
 */
sts_result_t mock_script_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len);

/**
 * @brief Consumes len queued bytes. Returns rx_fail_result on receive call rx_fail_call, and
 *        STS_ERR_TIMEOUT when fewer than len bytes are queued; both leave the queue unchanged.
 */
sts_result_t mock_script_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms);

/** @brief Empties the queue and counts the call. */
sts_result_t mock_script_flush(sts_bus_t *bus);
