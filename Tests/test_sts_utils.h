/**
 ******************************************************************************
 * @file           : test_sts_utils.h
 * @brief          : Header for STS test support utilities
 * @author         : Grisham Balloo
 * @date           : 2026-02-28
 * @version        : 1.0.0
 ******************************************************************************
 * @details
 * Function prototypes for the STS protocol simulator. Used to inject raw
 * data into the parser for validation and error-handling verification.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
 
#pragma once

#include <stdint.h>
#include "sts_protocol.h"

/**
 * @brief Simulates a raw servo response packet.
 * Useful for injecting data into the parser without production guards.
 */
void simulate_servo_response(uint8_t id, uint8_t status, const uint8_t* params, uint16_t p_len, uint8_t* out_buf);