#pragma once

#include <stdint.h>


#define STS_MIN_PACKET_SIZE  6   
#define STS_MAX_PACKET_SIZE 255


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
 * * Sums ID, Length, Instruction, and Parameters, then returns the 
 * bitwise NOT of the 8-bit truncation.
 * * @param pkt_buf   Pointer to the start of the packet buffer.
 * @param pkt_len   Total packet length (including headers and checksum slot).
 * @return uint8_t The calculated checksum, or 0 if inputs are invalid.
 */
uint8_t sts_calculate_checksum(const uint8_t* pkt_buf, uint16_t pkt_len);

/**
 * @brief Constructs an STS packet in the provided buffer.
 * * @param id          Servo ID (0-253, 254 for broadcast).
 * @param instruction STS Instruction (e.g., 0x01 for Ping).
 * @param param_buf   Pointer to parameter bytes (can be NULL if param_len is 0).
 * @param param_len   Number of parameters.
 * @param pkt_buf     The output buffer to store the packet.
 * @param pkt_buf_size The capacity of the output buffer.
 * @return sts_result_t  STS_OK or error code.
 */
sts_result_t sts_create_packet(uint8_t id, uint8_t instruction, const uint8_t* param_buf, uint16_t param_len, uint8_t* pkt_buf, uint16_t pkt_buf_size);

/**
 * @brief Parses a response packet from a servo.
 * * @param expected_id  The ID we are expecting a response from.
 * @param rx_buf       The raw data received from the UART.
 * @param rx_len       The length of data in rx_buf.
 * @param param_buf    Pointer to buffer where extracted parameters will be stored.
 * @param param_buf_size The capacity of the param_buf buffer.
 * @param param_len    Pointer to store the length of extracted data.
 * @return sts_result_t STS_OK on success, or hardware/protocol error code.
 */
sts_result_t sts_parse_response(uint8_t expected_id, const uint8_t* rx_buf, uint16_t rx_len,  uint8_t* param_buf, uint16_t param_buf_size,uint16_t* param_len);

