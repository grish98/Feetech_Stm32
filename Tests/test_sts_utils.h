/**
 ******************************************************************************
 * @file           : test_sts_utils.h
 * @brief          : Header for STS test support utilities
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 1.1.0
 ******************************************************************************
 * @details
 * This file provides a shared testing harness for the STS driver. 
 * It includes:
 * 1. Hardware Mocks: Simulating UART peripherals and HAL function pointers.
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


/** 
 * @brief A dummy struct to represent a Hardware Peripheral.
 * This mimics the memory layout of a STM32 UART peripheral.
 */
typedef struct {
    uint32_t cr1; /**< Control Register */
    uint32_t dr;  /**< Data Register */
    uint32_t sr;  /**< Status Register */
} mock_uart_t;

/* Extern declaration so all test files can point to the same 'hardware' */
extern mock_uart_t dummy_uart_port;

/** 
* @brief Mock Transmit Function
 * This simulates a successful transmission over the bus.
 */
sts_result_t mock_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len);

/**
 * @brief Mock Receive Function
 * This simulates a successful reception over the bus.
 */
sts_result_t mock_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief Manually constructs a raw byte array representing a servo response.
 * This skips the production "CreatePacket" logic so we can test the parser 
 * with intentionally corrupted or specific data.
 */
void simulate_servo_response(uint8_t id, uint8_t status, const uint8_t* params, uint16_t p_len, uint8_t* out_buf);