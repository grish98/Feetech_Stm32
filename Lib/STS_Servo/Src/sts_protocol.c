#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "sts_protocol.h"

/* Packet Layout Indices */
#define STS_IDX_HEADER_1     0
#define STS_IDX_HEADER_2     1
#define STS_IDX_ID           2
#define STS_IDX_LENGTH       3
#define STS_IDX_INSTRUCTION  4  
#define STS_IDX_STATUS       4  
#define STS_IDX_PARAM_START  5

#define STS_HEADER_SIZE 2
#define STS_CHECKSUM_SIZE 1
#define STS_LENGTH_FIXED_OVERHEAD  2
#define STS_PKT_FIXED_TOTAL  4

#define STS_MAX_ID 254
#define STS_MAX_PARAM_LEN 253

uint8_t sts_calculate_checksum(const uint8_t* pkt_buf, uint16_t pkt_len) {
    if (pkt_buf == NULL || pkt_len < STS_MIN_PACKET_SIZE) {
        return 0;
    }
    uint32_t sum = 0;
    // Skip the headers and stop before the checksum byte.
    uint16_t checksum_index = pkt_len - STS_CHECKSUM_SIZE;

    for (uint16_t i = STS_HEADER_SIZE; i < checksum_index; i++) {
        sum += pkt_buf[i];
    } 
    return (uint8_t)(~(sum & 0xFF));
}

sts_result_t sts_create_packet(uint8_t id, uint8_t instruction, const uint8_t* param_buf, uint16_t param_len, uint8_t* pkt_buf, uint16_t pkt_buf_size) {
 
  if (pkt_buf == NULL || (param_len > 0 && param_buf == NULL)) {
    return STS_ERR_NULL_PTR;
  }
  if (id > STS_MAX_ID || param_len > STS_MAX_PARAM_LEN) {
    return STS_ERR_INVALID_LEN;
  }
  const uint16_t total_packet_size = (uint16_t)STS_MIN_PACKET_SIZE + param_len;

  if (pkt_buf_size < total_packet_size){
      return STS_ERR_INVALID_LEN;
  }
    pkt_buf[STS_IDX_HEADER_1] = 0xFF;
    pkt_buf[STS_IDX_HEADER_2] = 0xFF;
    pkt_buf[STS_IDX_ID]          = id;
    pkt_buf[STS_IDX_LENGTH]      = param_len + STS_LENGTH_FIXED_OVERHEAD;
    pkt_buf[STS_IDX_INSTRUCTION] = instruction;
 
    if (param_len > 0) {
      memcpy(&pkt_buf[STS_IDX_PARAM_START], param_buf, param_len); 
    }

    pkt_buf[total_packet_size - STS_CHECKSUM_SIZE] = sts_calculate_checksum(pkt_buf, total_packet_size);
    return STS_OK;
  
}

static sts_result_t sts_validate_packet(uint8_t expected_id, const uint8_t* rx_buf, uint16_t rx_len) {
    if (rx_buf[STS_IDX_HEADER_1] != 0xFF || rx_buf[STS_IDX_HEADER_2] != 0xFF) {
      return STS_ERR_HEADER;
    }
    if (rx_buf[STS_IDX_ID] != expected_id)
    {
       return STS_ERR_ID_MISMATCH; 
    }

    uint8_t protocol_len = rx_buf[STS_IDX_LENGTH];
    if (protocol_len + STS_PKT_FIXED_TOTAL != rx_len){
       return STS_ERR_MALFORMED;
    }

    if (sts_calculate_checksum(rx_buf, rx_len) != rx_buf[rx_len - 1]) {
      return STS_ERR_CHECKSUM;
    }

    if (rx_buf[STS_IDX_STATUS] != 0x00) 
    {
      return STS_ERR_HARDWARE;
    }

    return STS_OK;
}

sts_result_t sts_parse_response(uint8_t expected_id, const uint8_t* rx_buf, uint16_t rx_len, uint8_t* param_buf, uint16_t param_buf_size, uint16_t* param_len) {

    if (rx_buf == NULL || param_buf == NULL || param_len == NULL) {
      return STS_ERR_NULL_PTR;
    }
    if (rx_len < STS_MIN_PACKET_SIZE) {
      return STS_ERR_INVALID_LEN;
    }

    sts_result_t status = sts_validate_packet(expected_id, rx_buf, rx_len);
    if (status != STS_OK) {
      return status;
    }

    uint16_t extracted_len = (uint16_t)(rx_buf[STS_IDX_LENGTH] - STS_LENGTH_FIXED_OVERHEAD);
    if (extracted_len > param_buf_size){
      return STS_ERR_INVALID_LEN;
    }

    memcpy(param_buf, &rx_buf[STS_IDX_PARAM_START], extracted_len);
    *param_len = extracted_len;
    return STS_OK;
}