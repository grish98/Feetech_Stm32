/**
 ******************************************************************************
 * @file           : sts_protocol.h
 * @brief          : Feetech STS Servo Protocol API
 * @author         : Grisham Balloo
 * @date           : 2026-02-28
 * @version        : 1.2.0
 ******************************************************************************
 * @details
 * This module defines the public interface for the Feetech STS Servo Protocol.
 * It implements a binary sliding-window packet parser and serialiser designed 
 * for robust half-duplex UART communication.
 * * Protocol Specifications:
 * - Sync Headers : 0xFF 0xFF
 * - Addressing   : 0-253 , 254 (Broadcast)
 * - Integrity    : 8-bit truncated sum-check 
 * - Payload      : 0-253 bytes variable parameters
 *
 * Design Philosophy:
 * - Zero dynamic memory allocation (Safe for deterministic real-time control).
 * - Stateless execution (Allows multiple servos to be parsed independently).
 * - Self-healing logic (Automatically recovers from bus collisions or noise).
 * - Status-Return Pattern: All functions return explicit error codes (sts_result_t)
 * to ensure deterministic error handling.
 *
 * * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* --- Protocol Structural Constants --- */
#define STS_HEADER                0xFFU
#define STS_HEADER_SIZE           2U
#define STS_CHECKSUM_SIZE         1U
#define STS_LENGTH_FIXED_OVERHEAD 2U   /* ID + Checksum  */
#define STS_PKT_FIXED_TOTAL       4U   /* Header(2) + ID(1) + Length(1) */
#define STS_MIN_PACKET_SIZE       6U   /* Header(2)+ID(1)+Len(1)+Inst(1)+CS(1) */
#define STS_MAX_PACKET_SIZE       255U /* Max total size including headers and checksum */

/* --- Packet Layout Indices --- */
#define STS_IDX_HEADER_1          0U
#define STS_IDX_HEADER_2          1U
#define STS_IDX_ID                2U
#define STS_IDX_LENGTH            3U
#define STS_IDX_INSTRUCTION       4U 
#define STS_IDX_STATUS            4U 
#define STS_IDX_PARAM_START       5U

/* --- Protocol Limits --- */
#define STS_MAX_ID                254U
#define STS_MAX_PARAM_LEN         253U
#define STS_HARDWARE_OK           0x00U
#define STS_MAX_INSTRUCTION       0x05U /* Valid range: 0x01 (Ping) to 0x05 (Reset) */


/**
 * @brief Return codes for STS protocol operations.
 */
typedef enum {
    STS_OK = 0,               /**< Operation successful */
    STS_ERR_NULL_PTR,         /**< Provided pointer was NULL */
    STS_ERR_INVALID_LEN,      /**< Length provided is outside protocol limits */
    STS_ERR_INVALID_PARAM,    /**< Invalid ID or Instruction code provided */
    STS_ERR_BUF_TOO_SMALL,    /**< Provided buffer cannot hold the generated packet */
    STS_ERR_TIMEOUT,          /**< Servo did not respond within the deadline */
    STS_ERR_HEADER,            /**< Packet does not start with 0xFF 0xFF */
    STS_ERR_ID_MISMATCH,      /**< Response ID does not match expected ID */
    STS_ERR_CHECKSUM,         /**< Calculated checksum does not match received byte */
    STS_ERR_MALFORMED,        /**< Packet length byte does not match actual bytes received */
    STS_ERR_HARDWARE,         /**< Servo reported hardware fault (Address 0x41) */
    STS_ERR_TX_FAIL,          /**< Hardware-level transmission failure */
    STS_ERR_BUSY,              /**< Interface is currently occupied */

} sts_result_t;

/**
 * @brief Calculates the Feetech STS Checksum.
 * Sums ID, Length, Instruction, and Parameters (skipping the initial 0xFF 0xFF headers), then returns the 
 * bitwise NOT of the 8-bit truncation.
 * @param[in]  pkt_buf      Pointer to the start of the packet buffer.
 * @param[in]  pkt_len      Total packet length to process.
 * @param[out] checksum_out Pointer to store the 8-bit NOT-sum result.
 * @return sts_result_t     STS_OK on success, error code otherwise
 */
sts_result_t sts_calculate_checksum(const uint8_t* pkt_buf, uint16_t pkt_len, uint8_t* checksum_out);

/**
 * @brief Constructs an STS packet in the provided buffer.
 *
 * @param id          Servo ID (0-253, 254 for broadcast).
 * @param instruction STS Instruction (e.g., 0x01 for Ping) or Status code.
 * @param param_buf   Pointer to parameter bytes (can be NULL if param_len is 0).
 * @param param_len   Number of parameters.
 * @param pkt_buf     The output buffer to store the packet.
 * @param pkt_buf_size The capacity of the output buffer.
 * @return sts_result_t  STS_OK or error code.
 */
sts_result_t sts_create_packet(uint8_t id, uint8_t instruction, const uint8_t* param_buf, uint16_t param_len, uint8_t* pkt_buf, uint16_t pkt_buf_size);

/**
 * @brief Parses a response packet from a servo, searching for valid headers in the buffer.
 *
 * This function  will skip invalid data until a valid packet for the expected ID is found or the buffer is exhausted.
 *
 * @param expected_id  The ID we are expecting a response from.
 * @param rx_buf         The raw data received from the UART.
 * @param rx_len         The length of data in rx_buf.
 * @param param_buf      [Out] Buffer to store extracted parameters.
 * @param param_buf_size The capacity of the param_buf buffer.
 * @param param_len      [Out] Number of bytes actually extracted.
 * @return sts_result_t STS_OK on success, or relevant error code.
 */
sts_result_t sts_parse_response(uint8_t expected_id, const uint8_t* rx_buf, uint16_t rx_len,  uint8_t* param_buf, uint16_t param_buf_size,uint16_t* param_len);
