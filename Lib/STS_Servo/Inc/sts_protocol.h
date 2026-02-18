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
    STS_ERR_CHECKSUM,         /**< Calculated checksum does not match received byte */
    STS_ERR_TIMEOUT,          /**< Servo did not respond within the deadline */
    STS_ERR_TX_FAIL,          /**< Hardware-level transmission failure */
    STS_ERR_BUSY              /**< Interface is currently occupied */
} sts_result_t;

/**
 * @brief Calculates the Feetech STS Checksum.
 * * Sums ID, Length, Instruction, and Parameters, then returns the 
 * bitwise NOT of the 8-bit truncation.
 * * @param data   Pointer to the start of the packet buffer.
 * @param length Total packet length (including headers and checksum slot).
 * @return uint8_t The calculated checksum, or 0 if inputs are invalid.
 */
uint8_t sts_calculate_checksum(const uint8_t* data, uint16_t length);


/**
 * @brief Constructs an STS packet in the provided buffer.
 * * @param id          Servo ID (0-253).
 * @param instruction STS Instruction (e.g., 0x01 for Ping).
 * @param parameters  Pointer to parameter bytes (can be NULL if p_len is 0).
 * @param p_len       Number of parameters.
 * @param buffer      The output buffer to store the packet.
 * @param buf_size    The capacity of the output buffer.
 * @return sts_result_t STS_OK or error code.
 */
sts_result_t sts_create_packet(uint8_t id, uint8_t instruction, const uint8_t* parameters, uint16_t p_len, uint8_t* buffer, uint16_t buf_size);

