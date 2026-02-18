#pragma once

#include <stdint.h>

/**
 * @brief Calculates the Feetech STS Checksum.
 * * Sums ID, Length, Instruction, and Parameters, then returns the 
 * bitwise NOT of the 8-bit truncation.
 * * @param data   Pointer to the start of the packet buffer.
 * @param length Total packet length (including headers and checksum slot).
 * @return uint8_t The calculated checksum, or 0 if inputs are invalid.
 */
uint8_t sts_calculate_checksum(const uint8_t* data, uint8_t length);


