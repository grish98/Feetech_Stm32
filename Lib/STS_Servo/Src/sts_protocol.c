#include <stdint.h>
#include <stddef.h>

#define STS_HEADER_SIZE 2
#define STS_CHECKSUM_SIZE 1
#define STS_MIN_PACKET_SIZE 6

uint8_t sts_calculate_checksum(const uint8_t* data, uint8_t length) {
    if (data == NULL || length < STS_MIN_PACKET_SIZE) {
        return 0;
    }
    uint32_t sum = 0;
    // Skip the headers and stop before the checksum byte.
    uint8_t checksum_index = length - STS_CHECKSUM_SIZE;
    for (uint8_t i = STS_HEADER_SIZE; i < checksum_index; i++) {
        sum += data[i];
    } 
    return (uint8_t)(~(sum & 0xFF));
}

