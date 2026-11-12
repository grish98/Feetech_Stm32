/**
 ******************************************************************************
 * @file           : sts_servo.c
 * @brief          : STS Service Layer Implementation
 * @author         : Grisham Balloo
 ******************************************************************************
 * @details
 * This module implements the high-level service functions for the Feetech STS
 * protocol. It provides a hardware-agnostic interface for motor control through
 * the following functional areas:
 *
 * - Bus HAL: STS_Bus_Init maps platform transmit/receive function pointers
 *   onto a shared bus handle.
 *
 * - Servo Handle: STS_Servo_Init binds a servo ID to a bus and initialises
 *   online state to offline.
 *
 * - Command Engine: one attempt handles framing, TX, and a two-stage receive.
 *   Stage 1 reads the fixed header to extract the packet length field. Stage 2
 *   reads the remaining payload. If 4 + the length field differs from
 *   cmd->expected_rx_len or exceeds the RX buffer, the attempt fails as
 *   STS_ERR_MALFORMED before Stage 2.
 *   Broadcast IDs and expected_rx_len == 0 skip the receive entirely.
 *   Writes make one attempt (sts_execute_command); reads and ping retry up to
 *   max_read_retries times (sts_execute_read).
 *
 * - Ping: STS_servo_ping issues a PING instruction and updates is_online,
 *   distinguishing communication failures from servo hardware errors.
 *
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
    #include "sts_servo_internal.h" 
    #define STATIC_TESTABLE 
#else
    #define STATIC_TESTABLE static
#endif

sts_result_t STS_Bus_Transmit(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    if (bus == NULL || bus->transmit == NULL|| data == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (len == 0U) {
        return STS_OK; 
    }
    
    return bus->transmit(bus, data, len);
}


sts_result_t STS_Bus_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout) {
    if (bus == NULL || bus->receive == NULL|| data == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (len == 0U) {
        return STS_OK;
    }
    return bus->receive(bus, data, len, timeout);
}

sts_result_t STS_Bus_FlushRx(sts_bus_t *bus) {
    if (bus == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (bus->flush_rx == NULL) {
        return STS_OK;
    }
    return bus->flush_rx(bus);
}

/* Validates the command and frames it into tx_buf; nothing is sent or counted. */
static sts_result_t sts_prepare(sts_servo_t *servo, const sts_cmd_t *cmd, uint16_t *tx_len) {
    if (servo == NULL || cmd == NULL || servo->bus == NULL) {
        return STS_ERR_NULL_PTR;
    }

    /* max_read_retries + 1 must fit the 8-bit attempt count; 255 would wrap to 0 attempts. */
    if (servo->bus->max_read_retries > STS_MAX_READ_RETRIES) {
        return STS_ERR_INVALID_PARAM;
    }

    if (cmd->tx_param_len > (STS_MAX_TX_BUFFER - STS_MIN_PACKET_SIZE)) {
        return STS_ERR_BUF_TOO_SMALL;
    }

    *tx_len = (uint16_t)STS_MIN_PACKET_SIZE + cmd->tx_param_len;
    (void)memset(servo->bus->tx_buf, 0, STS_MAX_TX_BUFFER);
    return sts_create_packet(servo->id, cmd->instruction, cmd->tx_params,
                             cmd->tx_param_len, servo->bus->tx_buf, STS_MAX_TX_BUFFER);
}

/* One attempt: transmit the framed packet, then the two-stage receive and parse. Counts nothing. */
static sts_result_t sts_attempt(sts_servo_t *servo, const sts_cmd_t *cmd, uint16_t tx_len) {
    static uint8_t trash_bin[STS_MAX_RX_BUFFER];
    static uint16_t dummy_len;
    uint8_t *rx_buf = servo->bus->rx_buf;

    sts_result_t res = STS_Bus_Transmit(servo->bus, servo->bus->tx_buf, tx_len);
    if (res != STS_OK) return res;

    if (servo->id >= STS_ID_BROADCAST_SYNC || cmd->expected_rx_len == 0U) {
        return STS_OK;
    }

    (void)memset(rx_buf, 0, STS_MAX_RX_BUFFER);

    res = STS_Bus_Receive(servo->bus, rx_buf, STS_PKT_FIXED_TOTAL, cmd->timeout_ms);
    if (res != STS_OK) return res;

    uint8_t  packet_len_field    = rx_buf[STS_IDX_LENGTH];
    uint16_t total_expected_size = (uint16_t)STS_PKT_FIXED_TOTAL + (uint16_t)packet_len_field;

    /* Too long for rx_buf is malformed, like any unexpected length. */
    if (total_expected_size > STS_MAX_RX_BUFFER || total_expected_size != cmd->expected_rx_len) {
        STS_Bus_FlushRx(servo->bus);
        return STS_ERR_MALFORMED;
    }

    res = STS_Bus_Receive(servo->bus, &rx_buf[STS_PKT_FIXED_TOTAL],
                          (uint16_t)packet_len_field, cmd->timeout_ms);
    if (res != STS_OK) return res;

    uint8_t  *safe_out  = (cmd->rx_params_out    != NULL) ? cmd->rx_params_out    : trash_bin;
    uint16_t  safe_size = (cmd->rx_params_out    != NULL) ? cmd->rx_params_size   : (uint16_t)sizeof(trash_bin);
    uint16_t *safe_len  = (cmd->rx_param_len_out != NULL) ? cmd->rx_param_len_out : &dummy_len;

    return sts_parse_response(servo->id, rx_buf, total_expected_size, safe_out, safe_size, safe_len);
}

/**
 * @brief  Single-attempt transaction path, used by every write.
 *
 * @details
 * Frames and transmits a command packet once, then performs a two-stage receive:
 * Stage 1 reads the fixed header to extract the packet length field.
 * Stage 2 reads the remaining payload. If 4 + the length field differs from
 * cmd->expected_rx_len or exceeds the RX buffer, the attempt is
 * STS_ERR_MALFORMED and the RX buffer is flushed. Skips RX entirely on broadcast
 * IDs or expected_rx_len == 0. Never retries. If the command was sent and the
 * call fails, the outcome is unknown; argument errors send nothing. Read-back
 * advice: see sts_servo_cmd.h.
 *
 * @note Not thread-safe. For RTOS support, take the per-bus mutex here and in
 *       sts_execute_read, and hold it for the whole call, counter updates included.
 *
 * @param  servo  Pointer to an initialised servo handle.
 * @param  cmd    Pointer to the command configuration struct.
 *
 * @return STS_OK or specific error code
 */
STATIC_TESTABLE sts_result_t sts_execute_command(sts_servo_t *servo, const sts_cmd_t *cmd) {
    uint16_t tx_len = 0U;
    sts_result_t res = sts_prepare(servo, cmd, &tx_len);
    if (res != STS_OK) return res;

    servo->bus->total_transactions++;
    res = sts_attempt(servo, cmd, tx_len);

    if (res != STS_OK && res != STS_ERR_HARDWARE) {
        servo->bus->hard_failures++;
    }
    return res;
}

/* A caller error repeats on every attempt, so it ends the retries. */
static uint8_t sts_is_caller_error(sts_result_t res) {
    return (res == STS_ERR_BUF_TOO_SMALL || res == STS_ERR_INVALID_PARAM || res == STS_ERR_NULL_PTR) ? 1U : 0U;
}

/**
 * @brief  Transaction path for reads and ping: up to max_read_retries + 1 attempts.
 *
 * @details
 * Each attempt after the first flushes the RX buffer and resends the same packet.
 * Stops at STS_OK, STS_ERR_HARDWARE or a caller error; returns the last attempt's code.
 *
 * @note Not thread-safe. For RTOS support, hold the per-bus mutex for the whole call,
 *       retries and counter updates included.
 *
 * @param  servo  Pointer to an initialised servo handle.
 * @param  cmd    Pointer to the command configuration struct.
 *
 * @return STS_OK or specific error code
 */
STATIC_TESTABLE sts_result_t sts_execute_read(sts_servo_t *servo, const sts_cmd_t *cmd) {
    uint16_t tx_len = 0U;
    sts_result_t res = sts_prepare(servo, cmd, &tx_len);
    if (res != STS_OK) return res;

    servo->bus->total_transactions++;
    uint8_t max_attempts = servo->bus->max_read_retries + 1U;

    for (uint8_t attempt = 0U; attempt < max_attempts; attempt++) {
        if (attempt > 0U) {
            servo->bus->total_retries++;
            STS_Bus_FlushRx(servo->bus);
        }

        res = sts_attempt(servo, cmd, tx_len);

        /* Hardware error is a definitive servo response — do not retry. */
        if (res == STS_OK || res == STS_ERR_HARDWARE || sts_is_caller_error(res) != 0U) {
            if (res == STS_OK && attempt > 0U) {
                servo->bus->retry_saves++;
            }
            break;
        }
    }

    if (res != STS_OK && res != STS_ERR_HARDWARE) {
        servo->bus->hard_failures++;
    }
    return res;
}

sts_result_t STS_Bus_Init(sts_bus_t *bus, void *port_handle, sts_hal_transmit_t tx_func, sts_hal_receive_t rx_func) {
    if (bus == NULL || tx_func == NULL || rx_func == NULL) {
        return STS_ERR_NULL_PTR;
    }
    memset(bus, 0, sizeof(sts_bus_t));

    bus->port_handle = port_handle;
    bus->transmit = tx_func;
    bus->receive = rx_func;

    return STS_OK;
}

sts_result_t STS_Servo_Init(sts_servo_t *servo, sts_bus_t *bus, uint8_t id) {
    if (servo == NULL || bus == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (bus->transmit == NULL || bus->receive == NULL) {
    return STS_ERR_INVALID_PARAM;
    }

    if (id >= STS_MAX_ID ) {
        return STS_ERR_INVALID_PARAM;
    }

    servo->bus = bus;
    servo->id = id;
    servo->is_online = STS_OFFLINE;
    servo->current_mode = STS_MODE_POSITION;

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
        .rx_param_len_out = NULL,
        .timeout_ms      = STS_DEFAULT_TIMEOUT_MS
    };

    sts_result_t res = sts_execute_read(servo, &cmd);

 if (res == STS_OK || res == STS_ERR_HARDWARE){ 
        servo->is_online = STS_ONLINE;
    } else {
        servo->is_online = STS_OFFLINE;
    }
    return res;
}

sts_result_t STS_Bus_Scan(sts_bus_t *bus, uint8_t first_id, uint8_t last_id,
                          sts_scan_cb_t on_ping, void *ctx, uint8_t *found_id) {
    if (bus == NULL || found_id == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (bus->transmit == NULL || bus->receive == NULL || first_id > last_id || last_id >= STS_MAX_ID) {
        return STS_ERR_INVALID_PARAM;
    }

    sts_result_t  miss = STS_ERR_TIMEOUT;
    sts_servo_t   probe;

    /* uint16_t so the loop cannot wrap at the top of the ID range. */
    for (uint16_t id = first_id; id <= last_id; id++) {
        (void)STS_Servo_Init(&probe, bus, (uint8_t)id);

        sts_result_t res = STS_servo_ping(&probe);
        if (on_ping != NULL) {
            on_ping((uint8_t)id, res, ctx);
        }

        if (res == STS_OK || res == STS_ERR_HARDWARE) {
            *found_id = (uint8_t)id;
            return STS_OK;
        }
        if (res != STS_ERR_TIMEOUT) {
            miss = res;
        }
    }
    return miss;
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
        .rx_param_len_out = NULL,
        .timeout_ms      = STS_DEFAULT_TIMEOUT_MS
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
        .rx_param_len_out = NULL,
        .timeout_ms      = STS_DEFAULT_TIMEOUT_MS
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
        .rx_param_len_out = &rx_len,
        .timeout_ms      = STS_DEFAULT_TIMEOUT_MS
    };

    sts_result_t res = sts_execute_read(servo, &cmd);
    
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
        .rx_param_len_out = &rx_len,
        .timeout_ms      = STS_DEFAULT_TIMEOUT_MS
    };

    sts_result_t res = sts_execute_read(servo, &cmd);
    
    if (res == STS_OK && rx_len == STS_DATA_LEN_16BIT) {
        *value_out = (uint16_t)(rx_data[0] | (rx_data[1] << 8U)); 
    }
    return res;
}

/* ==========================================================================
 * VERIFIED WRITES
 * ========================================================================== */

/* After a read returned STS_ERR_HARDWARE: take the value from the reply in rx_buf if it is the complete,
 * valid reply from this servo (the fault flag is set, but the checksum passed). */
static uint8_t sts_faulted_reply_value(const sts_servo_t *servo, uint16_t data_len, uint16_t *value_out) {
    const uint8_t *rx = servo->bus->rx_buf;
    uint16_t total = (uint16_t)(STS_MIN_PACKET_SIZE + data_len);
    uint8_t cs = 0U;
    if (rx[STS_IDX_HEADER_1] != STS_HEADER || rx[STS_IDX_HEADER_2] != STS_HEADER || rx[STS_IDX_ID] != servo->id ||
        rx[STS_IDX_LENGTH] != (uint8_t)(data_len + 2U) || rx[STS_IDX_STATUS] == STS_HARDWARE_OK ||
        sts_calculate_checksum(rx, total, &cs) != STS_OK || cs != rx[total - 1U]) {
        return 0U;
    }
    *value_out = (data_len == STS_DATA_LEN_8BIT) ? (uint16_t)rx[STS_IDX_PARAM_START]
               : (uint16_t)(rx[STS_IDX_PARAM_START] | (rx[STS_IDX_PARAM_START + 1U] << 8U));
    return 1U;
}

/* Read-back outcome: STS_OK match, STS_ERR_HARDWARE match from a servo flagging a fault, STS_ERR_VERIFY mismatch. */
static sts_result_t sts_compare_read_back(const sts_servo_t *servo, sts_result_t res, uint16_t data_len,
                                          uint16_t read_back, uint16_t expected) {
    if (res == STS_ERR_HARDWARE) {
        uint16_t faulted = 0U;
        if (sts_faulted_reply_value(servo, data_len, &faulted) == 0U) {
            return STS_ERR_VERIFY;
        }
        return (faulted == expected) ? STS_ERR_HARDWARE : STS_ERR_VERIFY;
    }
    if (res != STS_OK) {
        return res;
    }
    return (read_back == expected) ? STS_OK : STS_ERR_VERIFY;
}

sts_result_t STS_Write8Verified(sts_servo_t *servo, uint8_t reg_addr, uint8_t value) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (servo->id >= STS_ID_BROADCAST_SYNC) {
        return STS_ERR_INVALID_PARAM;
    }

    /* The write's own result is not final: a lost acknowledgement may hide a write that took effect. */
    (void)STS_Write8(servo, reg_addr, value);

    uint8_t read_back = 0U;
    sts_result_t res = STS_Read8(servo, reg_addr, &read_back);
    return sts_compare_read_back(servo, res, STS_DATA_LEN_8BIT, read_back, value);
}

sts_result_t STS_Write16Verified(sts_servo_t *servo, uint8_t reg_addr, uint16_t value) {
    if (servo == NULL) {
        return STS_ERR_NULL_PTR;
    }
    if (servo->id >= STS_ID_BROADCAST_SYNC) {
        return STS_ERR_INVALID_PARAM;
    }

    (void)STS_Write16(servo, reg_addr, value);

    uint16_t read_back = 0U;
    sts_result_t res = STS_Read16(servo, reg_addr, &read_back);
    return sts_compare_read_back(servo, res, STS_DATA_LEN_16BIT, read_back, value);
}