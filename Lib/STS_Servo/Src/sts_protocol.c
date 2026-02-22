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
#define STS_Header 0xFF


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
    pkt_buf[STS_IDX_HEADER_1] = STS_Header;
    pkt_buf[STS_IDX_HEADER_2] = STS_Header;
    pkt_buf[STS_IDX_ID]          = id;
    pkt_buf[STS_IDX_LENGTH]      = param_len + STS_LENGTH_FIXED_OVERHEAD;
    pkt_buf[STS_IDX_INSTRUCTION] = instruction;
 
    if (param_len > 0) {
      memcpy(&pkt_buf[STS_IDX_PARAM_START], param_buf, param_len); 
    }

    pkt_buf[total_packet_size - STS_CHECKSUM_SIZE] = sts_calculate_checksum(pkt_buf, total_packet_size);
    return STS_OK;
  
}

/**
 * @brief Performs  validation of a potential STS packet.
 * * Checks for  headers, verifies the target ID, ensures the buffer length 
 * matches the protocol's length field, and validates the checksum. 
 * Inspects the status byte for  hardware errors.
 *
 * @param expected_id  The ID expected to see in the packet.
 * @param rx_buf       Pointer to the start of the packet.
 * @param rx_len       The  number of bytes provided for this packet.
 * @return sts_result_t STS_OK or specific error code.
 */
static sts_result_t sts_validate_packet(uint8_t expected_id, const uint8_t* rx_buf, uint16_t rx_len) {
    if (rx_buf[STS_IDX_HEADER_1] != STS_Header || rx_buf[STS_IDX_HEADER_2] != STS_Header) {
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

/**
 * @brief Scans a raw buffer to locate the STS sync headers (0xFF 0xFF).
 *
 * This helper implements a sliding window search. It ensures that any header found
 * has enough trailing buffer space to potentially contain a full STS packet, 
 * preventing out-of-bounds reads during subsequent validation.
 *
 * @param buf  Pointer to the beginning of the search area.
 * @param len  Remaining length of the buffer to scan.
 * @return     Pointer to the first 0xFF of the sync header, or NULL if no valid 
 * header-start is found within the remaining length.
 */
static const uint8_t* sts_find_packet_start(const uint8_t* buf, uint16_t len) {
    if (len < STS_MIN_PACKET_SIZE) {
      return NULL;
    }

    for (uint16_t i = 0; i <= (len - STS_MIN_PACKET_SIZE); i++) {
        if (buf[i] == STS_Header && buf[i+1] == STS_Header) {
            return &buf[i];
        }
    }
    return NULL;
}

sts_result_t sts_parse_response(uint8_t expected_id, const uint8_t* rx_buf, uint16_t rx_len, uint8_t* param_buf, uint16_t param_buf_size, uint16_t* param_len) {

  if (rx_buf == NULL || param_buf == NULL || param_len == NULL) {
    return STS_ERR_NULL_PTR;
  }

  const uint8_t* current_pos = rx_buf;
  uint16_t remaining_len = rx_len;

  sts_result_t last_error = STS_ERR_HEADER;

  while (remaining_len >= STS_MIN_PACKET_SIZE) {
          const uint8_t* packet_start = sts_find_packet_start(current_pos, remaining_len);
          
          if (packet_start == NULL){
            break; 
          }

          uint16_t skipped = (uint16_t)(packet_start - current_pos);
          remaining_len -= skipped;

          sts_result_t status = sts_validate_packet(expected_id, packet_start, remaining_len);
          
          if (status == STS_OK) {
              uint16_t extracted_len = (uint16_t)(packet_start[STS_IDX_LENGTH] - STS_LENGTH_FIXED_OVERHEAD);
              if (extracted_len > param_buf_size) {
                return STS_ERR_INVALID_LEN;
              }

              memcpy(param_buf, &packet_start[STS_IDX_PARAM_START], extracted_len);
              *param_len = extracted_len;
              return STS_OK;
          }

          last_error = status;

          current_pos = packet_start +1; 
          remaining_len -= 1;
      }

    return last_error;
}