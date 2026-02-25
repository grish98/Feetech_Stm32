/**
 ******************************************************************************
 * @file           : sts_protocol.c
 * @brief          : Feetech STS Servo Protocol Implementation
 * @author         : Grisham Balloo
 * @date           : 2026-02-22
 * @version        : 1.1.0
 ******************************************************************************
 * @details
 * This module implements the core logic for encoding and decoding Feetech STS
 * series servo packets.
 *
 * Key Features:
 * 1. Checksum Calculation: Implements the 8-bit NOT-sum logic required by 
 * the Feetech hardware specification.
 * 2. Packet Creation: Encodes instructions and parameters into valid, 
 * ready-to-transmit binary packets.
 * 3. Response Parsing: Synchronizes and extracts valid servo data from a 
 * raw byte stream, filtered for errors and noise.
 *
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "sts_protocol.h"


sts_result_t sts_calculate_checksum(const uint8_t* pkt_buf, uint16_t pkt_len, uint8_t* checksum_out) {
    if (pkt_buf == NULL || checksum_out == NULL) {
        return STS_ERR_NULL_PTR;
    }
    
    if (pkt_len < STS_MIN_PACKET_SIZE) {
        return STS_ERR_INVALID_LEN;
    }

    uint32_t sum = 0U; 
    uint16_t checksum_index = pkt_len - (uint16_t)STS_CHECKSUM_SIZE;

    for (uint16_t i = (uint16_t)STS_HEADER_SIZE; i < checksum_index; i++) {
        sum += (uint32_t)pkt_buf[i];
    } 

    *checksum_out = (uint8_t)(~((uint8_t)(sum & 0xFFU)));

    return STS_OK;
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
    pkt_buf[STS_IDX_HEADER_1] = STS_HEADER;
    pkt_buf[STS_IDX_HEADER_2] = STS_HEADER;
    pkt_buf[STS_IDX_ID]          = id;
    pkt_buf[STS_IDX_LENGTH]      = param_len + STS_LENGTH_FIXED_OVERHEAD;
    pkt_buf[STS_IDX_INSTRUCTION] = instruction;
 
    if (param_len > 0) {
      memcpy(&pkt_buf[STS_IDX_PARAM_START], param_buf, param_len); 
    }

   uint8_t calculated_cs = 0;
    sts_result_t res = sts_calculate_checksum(pkt_buf, total_packet_size, &calculated_cs);
    if (res != STS_OK) {
        return res;
    }
    pkt_buf[total_packet_size - 1] = calculated_cs;
    return STS_OK;
}

/**
 * @brief Performs  validation of STS packet.
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
  if (rx_len < STS_MIN_PACKET_SIZE) {
        return STS_ERR_MALFORMED;
    }
  if (rx_buf[STS_IDX_HEADER_1] != STS_HEADER || rx_buf[STS_IDX_HEADER_2] != STS_HEADER) {
    return STS_ERR_HEADER;
  }
  if (rx_buf[STS_IDX_ID] != expected_id)
  {
      return STS_ERR_ID_MISMATCH; 
  }

  uint8_t protocol_len = rx_buf[STS_IDX_LENGTH];
  uint16_t expected_total_len = (uint16_t)protocol_len + STS_PKT_FIXED_TOTAL;
  if (expected_total_len > rx_len){
      return STS_ERR_MALFORMED;
  }

  uint8_t calculated_cs = 0;
  sts_result_t res = sts_calculate_checksum(rx_buf, expected_total_len, &calculated_cs);
  if (res != STS_OK) {
      return res;
  }

  if (calculated_cs != rx_buf[expected_total_len - 1]) {
      return STS_ERR_CHECKSUM;
  }

  if (rx_buf[STS_IDX_INSTRUCTION] != STS_HARDWARE_OK) 
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
        if (buf[i] == STS_HEADER && buf[i+1] == STS_HEADER) {
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
          current_pos = packet_start;
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

          current_pos++; 
          remaining_len--;
      }

    return last_error;
}