#include "sts_protocol.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdint.h>

static UART_HandleTypeDef  *s_huart    = NULL;
static volatile uint8_t     s_tx_done  = 0U;
static volatile uint8_t     s_rx_done  = 0U;
static volatile uint8_t     s_rx_error = 0U;
static volatile uint16_t    s_rx_bytes = 0U;

/* Called from USART2_IRQHandler on IDLE line. STM32F1 requires SR -> DR read to
 * clear the flag. NDTR must be captured before AbortReceive resets the dma. */
void STM32_UART_IdleCallback(void) {
    if (s_huart == NULL) { return; }

    __HAL_UART_DISABLE_IT(s_huart, UART_IT_IDLE);

    volatile uint32_t tmp = s_huart->Instance->SR;
    tmp = s_huart->Instance->DR;
    (void)tmp;

    s_rx_bytes = (uint16_t)(STS_MAX_RX_BUFFER -
                            (uint16_t)s_huart->hdmarx->Instance->CNDTR);

    HAL_UART_AbortReceive(s_huart);

    s_rx_done = 1U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        s_tx_done = 1U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* Full buffer consumed before IDLE fired — treat as completion. */
    if (huart->Instance == USART2) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        s_rx_bytes = STS_MAX_RX_BUFFER;
        s_rx_done  = 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        s_rx_error = 1U;
    }
}

/* Aborts DMA and clear STM32F1 error flags. */
static void uart_drain_rx(UART_HandleTypeDef *huart) {
    HAL_UART_AbortReceive(huart);

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
    s_huart = huart;

    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_AbortReceive(huart);
    volatile uint32_t tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState   = HAL_UART_STATE_READY;
    huart->gState    = HAL_UART_STATE_READY;

    s_tx_done  = 0U;
    s_rx_done  = 0U;
    s_rx_error = 0U;
    s_rx_bytes = 0U;

    /* RX must be armed before we transmit — the servo starts replying the moment
     * the half-duplex adapter switches direction, so DMA has to be ready first. */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    if (HAL_UART_Receive_DMA(huart, bus->rx_buf, STS_MAX_RX_BUFFER) != HAL_OK) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        return STS_ERR_RX_FAIL;
    }

    /* Wait for the TC interrupt before returning — the half-duplex adapter is still
     * driving the bus after DMA completes, and we can't switch to RX until it releases. */
    if (HAL_UART_Transmit_DMA(huart, (uint8_t *)data, len) != HAL_OK) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        HAL_UART_AbortReceive(huart);
        return STS_ERR_TX_FAIL;
    }

    uint32_t start = HAL_GetTick();
    while (!s_tx_done) {
        if ((HAL_GetTick() - start) >= 10U) {
            __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
            HAL_UART_AbortTransmit(huart);
            HAL_UART_AbortReceive(huart);
            return STS_ERR_TX_FAIL;
        }
    }

    return STS_OK;
}

sts_result_t STM32_UART_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout) {
    if (bus == NULL || data == NULL) {
         return STS_ERR_NULL_PTR;
        }

    (void)data;
    (void)len;

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;

    /* IDLE may have already fired during the transmit polling loop. */
    if (s_rx_done) { return STS_OK; }

    uint32_t start = HAL_GetTick();
    while (!s_rx_done && !s_rx_error) {
        if ((HAL_GetTick() - start) >= timeout) {
            __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
            HAL_UART_AbortReceive(huart);
            uart_drain_rx(huart);
            return STS_ERR_TIMEOUT;
        }
    }

    if (s_rx_error) {
        uart_drain_rx(huart);
        return STS_ERR_RX_FAIL;
    }

    if (s_rx_bytes < STS_PKT_FIXED_TOTAL) {
        uart_drain_rx(huart);
        return STS_ERR_RX_FAIL;
    }

    return STS_OK;
}

sts_result_t STM32_UART_FlushRx(sts_bus_t *bus) {
    if (bus == NULL) { return STS_ERR_NULL_PTR; }
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    uart_drain_rx(huart);
    return STS_OK;
}

void STS_Delay_ms(uint32_t ms)  { HAL_Delay(ms); }
uint32_t STS_GetTick_ms(void)   { return HAL_GetTick(); }
