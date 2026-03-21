/**
 ******************************************************************************
 * @file           : test_sts_utils.c
 * @brief          : Support utilities for STS Protocol Testing
 * @author         : Grisham Balloo
 * @date           : 2026-03-08
 * @version        : 1.2.0
 ******************************************************************************
 * @details
 * This module provides a shared testing harness for the STS driver.
 * It simulates hardware peripherals via mock_uart_t and provides protocol
 * packet injection via simulate_servo_response. mock_rx consumes rx_buffer
 * incrementally to support multi-stage reads, and both mock functions track
 * call counts via rx_call_count and tx_call_count for behavioural
 * verification in tests.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "test_sts_utils.h"
#include <string.h>

mock_uart_t dummy_uart_port = {0};


sts_result_t mock_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    if (bus == NULL || bus->port_handle == NULL || data == NULL) {
        return STS_ERR_NULL_PTR;
    }
    
    mock_uart_t *port = (mock_uart_t *)bus->port_handle;
    
    port->tx_call_count++; 

    if (len <= sizeof(port->tx_buffer)) {
        memcpy(port->tx_buffer, data, len);
        port->tx_len = len;
    }
    
    return STS_OK;
}

sts_result_t mock_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    if (bus == NULL || bus->port_handle == NULL || data == NULL) {
        return STS_ERR_NULL_PTR; 
    }

    mock_uart_t *port = (mock_uart_t *)bus->port_handle;
    port->rx_call_count++; 

    if (port->rx_len < len) {
        return STS_ERR_TIMEOUT; 
    }

    memcpy(data, port->rx_buffer, len);

    uint16_t remaining = port->rx_len - len;
    if (remaining > 0) {
        memmove(port->rx_buffer, &port->rx_buffer[len], remaining);
    }
    
    port->rx_len = remaining; 

    return STS_OK;
}

sts_result_t mock_tx_fail(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    (void)bus; (void)data; (void)len;
    return STS_ERR_TX_FAIL;
}

sts_result_t mock_tx_busy(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    (void)bus; (void)data; (void)len;
    return STS_ERR_BUSY;
}

void simulate_servo_response(uint8_t id, uint8_t status, const uint8_t* params, 
                            uint16_t p_len, uint8_t* out_buf) {
    if (out_buf == NULL) {
        return;               
    }             
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