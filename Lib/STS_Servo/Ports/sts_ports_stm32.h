#pragma once

sts_result_t STM32_UART_Transmit(sts_bus_t *bus, const uint8_t *data, uint16_t len);
sts_result_t STM32_UART_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout);
sts_result_t STM32_UART_FlushRx(sts_bus_t *bus);

/* Call once after STS_Bus_Init, before any servo communication.
 * enabled=1: STM32 half-duplex mode (direct servo wiring on PA2, no adapter).
 * enabled=0: full-duplex mode (Waveshare adapter, default). */
void STM32_UART_SetHalfDuplex(void *huart, uint8_t enabled);

void STM32_UART_IdleCallback(void);

void STS_Delay_ms(uint32_t ms);
uint32_t STS_GetTick_ms(void);