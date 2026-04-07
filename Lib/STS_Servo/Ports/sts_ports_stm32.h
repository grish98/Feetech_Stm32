#pragma once

sts_result_t STM32_UART_Transmit( sts_bus_t *bus, const uint8_t *data, uint16_t len);
sts_result_t STM32_UART_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout);
