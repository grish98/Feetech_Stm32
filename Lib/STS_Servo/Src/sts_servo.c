/**
 ******************************************************************************
 * @file           : sts_servo.c
 * @brief          : STS Service Layer Implementation
 * @author         : Grisham Balloo
 * @date           : 2026-03-05
 * @version        : 0.2.0
 ******************************************************************************
 * @details
 * This module implements the high-level service functions for the Feetech STS
 * protocol. It provides a hardware-agnostic interface for motor control through
 * the following functional areas:
 *
 * - Bus HAL: STS_Bus_Init maps platform transmit/receive function pointers
 *   onto a shared bus handle.
 * - Servo Handle: STS_Servo_Init binds a servo ID to a bus and initialises
 *   online state to offline.
 * - Command Engine: sts_execute_command (STATIC_TESTABLE) is the single
 *   transaction path — handles framing, TX, RX, and response parsing with
 *   internal buffer overflow guards and broadcast early-exit logic.
 * - Ping: STS_servo_ping issues a PING instruction and updates is_online,
 *   distinguishing communication failures from servo hardware errors.
 * - Register Primitives: STS_Write8, STS_Write16, STS_Read8, STS_Read16
 *   provide typed register access built on the command engine.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "sts_servo.h"
#include "sts_protocol.h"
#include "sts_registers.h"
#include <string.h>

#ifdef UNIT_TESTING
    #define STATIC_TESTABLE 
#else
    #define STATIC_TESTABLE static
#endif

/* ==========================================================================
 * STS Configuration & Limits
 * ========================================================================== */
#define STS_MAX_TX_BUFFER       128U
#define STS_MAX_RX_BUFFER       128U
#define STS_DEFAULT_TIMEOUT_MS  10U

/* ==========================================================================
 * STS Protocol Framing & Sizing
 * ========================================================================== */
/* Packet Overhead: 2 Header + 1 ID + 1 Length + 1 Status + 1 Checksum */
#define STS_ACK_BASE_LEN        6U  

/* Base sizes for protocol elements */
#define STS_REG_ADDR_LEN        1U  /* Register addresses are always 1 byte */
#define STS_DATA_LEN_8BIT       1U  /* 8-bit data payload */
#define STS_DATA_LEN_16BIT      2U  /* 16-bit data payload */

/* --- Derived Command Parameter Lengths --- 
 * TX Params for Read/Write commands always consist of the Register Address 
 * followed by the Data (or the Length of data to read).
 */
#define STS_WRITE8_PARAM_LEN    (STS_REG_ADDR_LEN + STS_DATA_LEN_8BIT)   /* 2U */
#define STS_WRITE16_PARAM_LEN   (STS_REG_ADDR_LEN + STS_DATA_LEN_16BIT)  /* 3U */
#define STS_READ_CMD_PARAM_LEN  (STS_REG_ADDR_LEN + STS_DATA_LEN_8BIT)   /* 2U (Address + Length byte) */

/**
 * @brief  Core command execution engine. Handles framing, TX, RX, and parsing safely.
 * @param  servo Pointer to the servo handle.
 * @param  cmd   Pointer to the command configuration struct.
 * @return STS_OK on success, or specific sts_result_t error code.
 */
STATIC_TESTABLE sts_result_t sts_execute_command(sts_servo_t *servo, const sts_cmd_t *cmd) {
    if (servo == NULL || cmd == NULL || servo->bus == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (servo->bus->transmit == NULL || servo->bus->receive == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (cmd->expected_rx_len > STS_MAX_RX_BUFFER) {
        return STS_ERR_BUF_TOO_SMALL; 
    }
    if (cmd->tx_param_len > (STS_MAX_TX_BUFFER - STS_MIN_PACKET_SIZE)) {
        return STS_ERR_BUF_TOO_SMALL;
    }

    /* Static buffers: deterministic bare-metal memory. Not thread-safe by design.
    * All transactions route through sts_execute_command — single-point change
    * when mutex support is added. */
    static uint8_t tx_buf[STS_MAX_TX_BUFFER];
    static uint8_t rx_buf[STS_MAX_RX_BUFFER];
    static uint8_t trash_bin[STS_MAX_RX_BUFFER];
    static uint16_t dummy_len;

    (void)memset(tx_buf, 0, sizeof(tx_buf));
    sts_result_t res = sts_create_packet(servo->id, cmd->instruction, cmd->tx_params, 
                                         cmd->tx_param_len, tx_buf, sizeof(tx_buf));
    if (res != STS_OK) {
        return res;
    }

    uint16_t total_tx_len = (uint16_t)STS_MIN_PACKET_SIZE + cmd->tx_param_len;
    res = servo->bus->transmit(servo->bus, tx_buf, total_tx_len);
    if (res != STS_OK) {
        return res;
    }

    if (servo->id == STS_ID_BROADCAST_SYNC || servo->id == STS_ID_BROADCAST_ASYNC || cmd->expected_rx_len == 0U) {
        return STS_OK;
    }

    (void)memset(rx_buf, 0, sizeof(rx_buf));
    res = servo->bus->receive(servo->bus, rx_buf, cmd->expected_rx_len, STS_DEFAULT_TIMEOUT_MS);
    if (res != STS_OK) {
        return res;
    }

    uint8_t *safe_out  = (cmd->rx_params_out != NULL) ? cmd->rx_params_out : trash_bin;
    uint16_t safe_size = (cmd->rx_params_out != NULL) ? cmd->rx_params_size : (uint16_t)sizeof(trash_bin);
    uint16_t *safe_len = (cmd->rx_param_len_out != NULL) ? cmd->rx_param_len_out : &dummy_len;

    return sts_parse_response(servo->id, rx_buf, cmd->expected_rx_len, 
                              safe_out, safe_size, safe_len);
}

sts_result_t STS_Bus_Init(sts_bus_t *bus, void *port_handle, sts_hal_transmit_t tx_func, sts_hal_receive_t rx_func) {
    if (bus == NULL || tx_func == NULL || rx_func == NULL) {
        return STS_ERR_NULL_PTR;
    }

    bus->port_handle = port_handle;
    bus->transmit = tx_func;
    bus->receive = rx_func;

    return STS_OK;
}

sts_result_t STS_Servo_Init(sts_servo_t *servo, sts_bus_t *bus, uint8_t id) {
    if (servo == NULL || bus == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (id >= STS_MAX_ID ) {
        return STS_ERR_INVALID_PARAM;
    }

    servo->bus = bus;
    servo->id = id;
    servo->is_online = STS_OFFLINE; 

    return STS_OK;
}

sts_result_t STS_servo_ping(sts_servo_t *servo) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (servo->id == STS_ID_BROADCAST_SYNC || servo->id == STS_ID_BROADCAST_ASYNC) {
        return STS_ERR_INVALID_PARAM; 
    }

    sts_cmd_t cmd = {
        .instruction      = STS_INST_PING,
        .tx_params        = NULL,
        .tx_param_len     = 0U,
        .expected_rx_len  = STS_ACK_BASE_LEN, 
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    sts_result_t res = sts_execute_command(servo, &cmd);

 if (res == STS_OK || res == STS_ERR_HARDWARE){ 
        servo->is_online = STS_ONLINE;
    } else {
        servo->is_online = STS_OFFLINE;
    }
    return res;
}

/* ==========================================================================
 * REGISTER ACCESS PRIMITIVES
 * ========================================================================== */

sts_result_t STS_Write8(sts_servo_t *servo, uint8_t reg_addr, uint8_t value) {
    uint8_t params[STS_WRITE8_PARAM_LEN] = {reg_addr, value};

    sts_cmd_t cmd = {
        .instruction      = STS_INST_WRITE,
        .tx_params        = params,
        .tx_param_len     = STS_WRITE8_PARAM_LEN,
        .expected_rx_len  = STS_ACK_BASE_LEN, 
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    return sts_execute_command(servo, &cmd);
}

sts_result_t STS_Write16(sts_servo_t *servo, uint8_t reg_addr, uint16_t value) {
    uint8_t params[STS_WRITE16_PARAM_LEN];
    params[0] = reg_addr;
    params[1] = (uint8_t)(value & 0xFFU);         
    params[2] = (uint8_t)((value >> 8U) & 0xFFU); 

    sts_cmd_t cmd = {
        .instruction      = STS_INST_WRITE,
        .tx_params        = params,
        .tx_param_len     = STS_WRITE16_PARAM_LEN,
        .expected_rx_len  = STS_ACK_BASE_LEN, 
        .rx_params_out    = NULL,
        .rx_params_size   = 0U,
        .rx_param_len_out = NULL
    };

    return sts_execute_command(servo, &cmd);
}

sts_result_t STS_Read8(sts_servo_t *servo, uint8_t reg_addr, uint8_t *value_out) {
    if (servo == NULL || value_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (servo->id == STS_ID_BROADCAST_SYNC || servo->id == STS_ID_BROADCAST_ASYNC) {
        return STS_ERR_INVALID_PARAM; 
    }

    uint8_t params[STS_READ_CMD_PARAM_LEN]  = {reg_addr, STS_DATA_LEN_8BIT};
    uint8_t rx_data[STS_DATA_LEN_8BIT]    = {0};
    uint16_t rx_len                         = 0U;

    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = params,
        .tx_param_len     = STS_READ_CMD_PARAM_LEN,
        .expected_rx_len  = STS_ACK_BASE_LEN + STS_DATA_LEN_8BIT,
        .rx_params_out    = rx_data,
        .rx_params_size   = STS_DATA_LEN_8BIT,
        .rx_param_len_out = &rx_len
    };

    sts_result_t res = sts_execute_command(servo, &cmd);
    
    if (res == STS_OK && rx_len == STS_DATA_LEN_8BIT) {
        *value_out = rx_data[0];
    }
    return res;
}

sts_result_t STS_Read16(sts_servo_t *servo, uint8_t reg_addr, uint16_t *value_out) {
    if (servo == NULL || value_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (servo->id == STS_ID_BROADCAST_SYNC || servo->id == STS_ID_BROADCAST_ASYNC) {
        return STS_ERR_INVALID_PARAM;
    }

    uint8_t params[STS_READ_CMD_PARAM_LEN]  = {reg_addr, STS_DATA_LEN_16BIT};
    uint8_t rx_data[STS_DATA_LEN_16BIT]   = {0};
    uint16_t rx_len                         = 0U;

    sts_cmd_t cmd = {
        .instruction      = STS_INST_READ,
        .tx_params        = params,
        .tx_param_len     = STS_READ_CMD_PARAM_LEN,
        .expected_rx_len  = STS_ACK_BASE_LEN + STS_DATA_LEN_16BIT, 
        .rx_params_out    = rx_data,
        .rx_params_size   = STS_DATA_LEN_16BIT,
        .rx_param_len_out = &rx_len
    };

    sts_result_t res = sts_execute_command(servo, &cmd);
    
    if (res == STS_OK && rx_len == STS_DATA_LEN_16BIT) {
        *value_out = (uint16_t)(rx_data[0] | (rx_data[1] << 8U)); 
    }
    return res;
}