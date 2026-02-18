#include <stdint.h>
#include <stddef.h>
#include "sts_protocol.h"

#define STS_HEADER_SIZE 2
#define STS_CHECKSUM_SIZE 1
#define STS_LENGTH_FIXED_OVERHEAD  2

uint8_t sts_calculate_checksum(const uint8_t* data, uint16_t length) {
    if (data == NULL || length < STS_MIN_PACKET_SIZE) {
        return 0;
    }
    uint32_t sum = 0;
    // Skip the headers and stop before the checksum byte.
    uint16_t checksum_index = length - STS_CHECKSUM_SIZE;

    for (uint16_t i = STS_HEADER_SIZE; i < checksum_index; i++) {
        sum += data[i];
    } 
    return (uint8_t)(~(sum & 0xFF));
}

sts_result_t sts_create_packet(uint8_t id, uint8_t instruction, const uint8_t* parameters, uint16_t p_len, uint8_t* buffer, uint16_t buf_size) {
 
  if (buffer == NULL || (p_len > 0 && parameters == NULL)) {
    return STS_ERR_NULL_PTR;
  }
  if (id > 254 || p_len > 253) {
    return STS_ERR_INVALID_LEN;
  }
  const uint16_t total_packet_size = (uint16_t)STS_MIN_PACKET_SIZE + p_len;

  if (buf_size < total_packet_size){
      return STS_ERR_INVALID_LEN;
  }

    const uint8_t idx_id          = 2;
    const uint8_t idx_length      = 3;
    const uint8_t idx_instruction = 4;
    const uint8_t idx_param_start = 5;
    const uint16_t idx_checksum    = total_packet_size - STS_CHECKSUM_SIZE;

    const uint8_t protocol_len = p_len + STS_LENGTH_FIXED_OVERHEAD;

    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[idx_id]          = id;
    buffer[idx_length]      = protocol_len;
    buffer[idx_instruction] = instruction;
 
    if (p_len > 0 && parameters != NULL) {
        for (uint8_t i = 0; i < p_len; i++) {
            buffer[idx_param_start + i] = parameters[i];
        }
    }
    buffer[idx_checksum] = sts_calculate_checksum(buffer, total_packet_size);
    return STS_OK;
    
}