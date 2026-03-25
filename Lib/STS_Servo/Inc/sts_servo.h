/**
 ******************************************************************************
 * @file           : sts_servo.h
 * @brief          : STS Service Layer and Hardware Abstraction Definitions
 * @author         : Grisham Balloo
 * @date           : 2026-03-20
 * @version        : 0.2.0
 ******************************************************************************
 * @details
 * Defines the Hardware Abstraction Layer and public API for the Feetech STS
 * protocol service layer. Uses an injected-dependency model (function pointers)
 * to remain agnostic of the underlying MCU or UART implementation.
 *
 * Exposes the bus handle, servo handle, HAL typedefs, register access
 * primitives, and the ping service. The internal sts_cmd_t struct is
 * conditionally visible for white-box unit testing via STATIC_TESTABLE.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#pragma once
#include "sts_protocol.h"
#include <stdint.h>
#include <stddef.h>
#include "sts_registers.h"

#define STS_ONLINE  1
#define STS_OFFLINE 0

#define STS_MAX_TX_BUFFER       128U
#define STS_MAX_RX_BUFFER       128U
#define STS_DEFAULT_TIMEOUT_MS  10U



/* Forward declaration of the bus for the function pointers */
typedef struct sts_bus_s sts_bus_t;
typedef struct sts_servo_t sts_servo_t;
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

    uint8_t tx_buf[STS_MAX_TX_BUFFER];
    uint8_t rx_buf[STS_MAX_RX_BUFFER];
};

/**
 * @brief STS Servo Handle
 * Represents a single physical servo on the bus.
 */
typedef struct sts_servo_t {
    sts_bus_t *bus;    /**< Pointer to the bus this servo is connected to */
    uint8_t id;        /**< Unique ID of the servo (0-253) */
    uint8_t is_online; /**< Connection status (STS_ONLINE/STS_OFFLINE) */
    sts_operating_mode_t  current_mode;
} sts_servo_t;

/**
 * @brief Initializes the hardware bus abstraction.
 */
sts_result_t STS_Bus_Init(sts_bus_t *bus, void *port_handle, sts_hal_transmit_t tx_func, sts_hal_receive_t rx_func);

/**
 * @brief Initializes a servo handle.
 * @param servo Pointer to the servo handle to initialize.
 * @param bus   Pointer to an initialized bus handle.
 * @param id    The hardware ID of the servo (must be < 254).
 * @return STS_OK on success, 
 * STS_ERR_NULL_PTR if servo/bus are NULL,
 * STS_ERR_INVALID_PARAM if ID is 254 or 255.
 */
sts_result_t STS_Servo_Init(sts_servo_t *servo, sts_bus_t *bus, uint8_t id);

/**
 * @brief  Safe wrapper for HAL transmission.
 * @return STS_ERR_NULL_PTR if function pointer is missing, else HAL result.
 */
sts_result_t STS_Bus_Transmit(sts_bus_t *bus, const uint8_t *data, uint16_t len);

/**
 * @brief  Safe wrapper for HAL reception.
 * @return STS_ERR_NULL_PTR if function pointer is missing, else HAL result.
 */
sts_result_t STS_Bus_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout);


/**
* @brief Pings the servo to check if it's online.
* @param servo Pointer to the initialized servo handle.
* @return STS_OK if the servo responds correctly, error code otherwise.
* On success, the servo's is_online field is set to STS_ONLINE; on failure, it is set to STS_OFFLINE.
 */
sts_result_t STS_servo_ping(sts_servo_t *servo);


/* ==========================================================================
 * REGISTER ACCESS PRIMITIVES
 * ========================================================================== */

/**
 * @brief Writes a single byte (8-bit) to a specific servo register.
 */
sts_result_t STS_Write8(sts_servo_t *servo, uint8_t reg_addr, uint8_t value);

/**
 * @brief Writes a word (16-bit) to a specific servo register (Little-Endian).
 */
sts_result_t STS_Write16(sts_servo_t *servo, uint8_t reg_addr, uint16_t value);

/**
 * @brief Reads a single byte (8-bit) from a specific servo register.
 */
sts_result_t STS_Read8(sts_servo_t *servo, uint8_t reg_addr, uint8_t *value_out);

/**
 * @brief Reads a word (16-bit) from a specific servo register (Little-Endian).
 */
sts_result_t STS_Read16(sts_servo_t *servo, uint8_t reg_addr, uint16_t *value_out);


/** @cond INTERNAL */
/**
 * @brief Internal transaction context for the command engine.
 * Exposed in header strictly for unit testing visibility.
 */
typedef struct {
    uint8_t instruction;
    const uint8_t *tx_params;
    uint16_t tx_param_len;
    uint16_t expected_rx_len;
    uint8_t *rx_params_out;
    uint16_t rx_params_size;
    uint16_t *rx_param_len_out;
    uint32_t timeout_ms;
} sts_cmd_t;
/** @endcond */