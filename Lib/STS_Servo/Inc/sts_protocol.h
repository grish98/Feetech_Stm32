/**
 ******************************************************************************
 * @file           : sts_protocol.h
 * @brief          : Feetech STS Servo Protocol API
 * @author         : Grisham Balloo
 * @date           : 2026-02-22
 * @version        : 1.0.0
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
 * * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* --- Protocol Structural Constants --- */
#define STS_HEADER                0xFF
#define STS_HEADER_SIZE           2
#define STS_CHECKSUM_SIZE         1
#define STS_LENGTH_FIXED_OVERHEAD 2    /* ID + Checksum  */
#define STS_PKT_FIXED_TOTAL       4    /* Header(2) + ID(1) + Length(1) */
#define STS_MIN_PACKET_SIZE       6    /* Header(2)+ID(1)+Len(1)+Inst(1)+CS(1) */
#define STS_MAX_PACKET_SIZE       255 /* Max total size including headers and checksum */

/* --- Packet Layout Indices --- */
#define STS_IDX_HEADER_1          0
#define STS_IDX_HEADER_2          1
#define STS_IDX_ID                2
#define STS_IDX_LENGTH            3
#define STS_IDX_INSTRUCTION       4    /* Also used as Status byte in responses */
#define STS_IDX_PARAM_START       5

/* --- Protocol Limits --- */
#define STS_MAX_ID                254
#define STS_MAX_PARAM_LEN         253
#define STS_HARDWARE_OK           0x00


/**
 * @brief Return codes for STS protocol operations.
 */
typedef enum {
    STS_OK = 0,               /**< Operation successful */
    STS_ERR_NULL_PTR,         /**< Provided pointer was NULL */
    STS_ERR_INVALID_LEN,      /**< Length provided is outside protocol limits */
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
 * * Sums ID, Length, Instruction, and Parameters (skipping the initial 0xFF 0xFF headers), then returns the 
 * bitwise NOT of the 8-bit truncation.
 * * @param pkt_buf   Pointer to the start of the packet buffer.
 * @param pkt_len   Total packet length (including headers and checksum slot).
 * @return uint8_t The calculated checksum, or 0 if inputs are invalid.
 */
uint8_t sts_calculate_checksum(const uint8_t* pkt_buf, uint16_t pkt_len);

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

