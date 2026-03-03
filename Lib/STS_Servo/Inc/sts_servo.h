/**
 ******************************************************************************
 * @file           : sts_servo.h
 * @brief          : STS Service Layer and Hardware Abstraction Definitions
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 0.1.0
 ******************************************************************************
 * @details
 * This header defines the Hardware Abstraction Layer  for the Feetech 
 * STS protocol. It utilizes an injected-dependency model (function pointers) 
 * to remain agnostic of the underlying MCU or UART implementation.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#pragma once
#include "sts_protocol.h"
#include <stdint.h>
#include <stddef.h>



/* Forward declaration of the bus for the function pointers */
typedef struct sts_bus_s sts_bus_t;

/**
 * @brief Context-Aware HAL Function Pointers.
 * The bus itself is passed in, allowing the HAL to access the port_handle.
 */
typedef sts_result_t (*sts_hal_transmit_t)(sts_bus_t *bus, const uint8_t *data, uint16_t len);
typedef sts_result_t (*sts_hal_receive_t)(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief STS Shared Bus Handle
 */
struct sts_bus_s {
    void *port_handle;           /**< Opaque pointer to the physical UART hardware */
    sts_hal_transmit_t transmit; /**< Injected TX function */
    sts_hal_receive_t receive;   /**< Injected RX function */
};

/**
 * @brief Initializes the hardware bus abstraction.
 */
sts_result_t STS_Bus_Init(sts_bus_t *bus, void *port_handle, sts_hal_transmit_t tx_func, sts_hal_receive_t rx_func);

