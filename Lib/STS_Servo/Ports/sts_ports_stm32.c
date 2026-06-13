#include "sts_protocol.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdint.h>

/*
 * Aborts any in-progress receive, clears STM32F1 UART error flags (ORE/FE/NE)
 * by the required SR-then-DR read sequence, then eats stale in-flight bytes
 * with a short per-byte timeout. The drain loop also provides ~2 ms of bus
 * settle time even when there are no stale bytes to consume.
 */
static void uart_drain_rx(UART_HandleTypeDef *huart) {
    HAL_UART_AbortReceive(huart);

    /* STM32F1 clears ORE/FE/NE by reading SR followed by DR */
    volatile uint32_t tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState   = HAL_UART_STATE_READY;

    uint8_t dummy;
    while (HAL_UART_Receive(huart, &dummy, 1, 2) == HAL_OK);
}

sts_result_t STM32_UART_Transmit(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    if (bus == NULL || data == NULL) {
        return STS_ERR_NULL_PTR;
    }

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;

    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->gState = HAL_UART_STATE_READY;

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)data, len, 10);

    if (status == HAL_OK) {
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
        HAL_Delay(3);
        return STS_OK;
    }
    return STS_ERR_TX_FAIL;
}

sts_result_t STM32_UART_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout) {
    if (bus == NULL || data == NULL) {
        return STS_ERR_NULL_PTR;
    }

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    HAL_StatusTypeDef status = HAL_UART_Receive(huart, (uint8_t*)data, len, timeout);

    if (status == HAL_OK) {
        return STS_OK;
    }

    uart_drain_rx(huart);

    switch (status) {
        case HAL_TIMEOUT: return STS_ERR_TIMEOUT;
        case HAL_BUSY:    return STS_ERR_BUSY;
        default:          return STS_ERR_RX_FAIL;
    }
}

sts_result_t STM32_UART_FlushRx(sts_bus_t *bus) {
    if (bus == NULL) {
        return STS_ERR_NULL_PTR;
    }
    uart_drain_rx((UART_HandleTypeDef *)bus->port_handle);
    return STS_OK;
}

void STS_Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

uint32_t STS_GetTick_ms(void) {
    return HAL_GetTick();
}