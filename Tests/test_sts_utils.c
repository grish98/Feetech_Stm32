/**
 ******************************************************************************
 * @file           : test_sts_utils.c
 * @brief          : Support utilities for STS Protocol Testing
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 1.1.0
 ******************************************************************************
 * @details
 * This module provides a shared testing harness for the STS driver.
 * It simulates hardware peripherals via mock_uart_t and provides protocol
 * packet injection via simulate_servo_response.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "test_sts_utils.h"
#include <string.h>

mock_uart_t dummy_uart_port = {0};


sts_result_t mock_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    (void)bus; 
    (void)data; 
    (void)len;
    
    return STS_OK;
}

sts_result_t mock_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    (void)bus; 
    (void)data; 
    (void)len; 
    (void)timeout_ms;
    
    return STS_OK;
}

void simulate_servo_response(uint8_t id, uint8_t status, const uint8_t* params, 
                            uint16_t p_len, uint8_t* out_buf) {
    out_buf[0] = 0xFF;
    out_buf[1] = 0xFF;
    out_buf[2] = id;
    out_buf[3] = (uint8_t)(p_len + 2U); 
    out_buf[4] = status;
   
    if (p_len > 0U && params != NULL) {
        (void)memcpy(&out_buf[5], params, p_len);
    }
    
    uint8_t cs = 0U;
    uint16_t total_len = (uint16_t)STS_MIN_PACKET_SIZE + p_len;
    (void)sts_calculate_checksum(out_buf, total_len, &cs);
    out_buf[total_len - 1U] = cs;
}