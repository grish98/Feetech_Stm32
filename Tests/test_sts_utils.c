/**
 ******************************************************************************
 * @file           : test_sts_utils.c
 * @brief          : Support utilities for STS Protocol Testing
 * @author         : Grisham Balloo
 * @date           : 2026-02-28
 * @version        : 1.0.0
 ******************************************************************************
 * @details
 * This module provides simulation helpers to bypass production guards during 
 * unit and integration testing. It allows for the construction of servo-side
 * response packets, including status/error bytes and valid checksums.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "test_sts_utils.h"
#include <string.h>

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