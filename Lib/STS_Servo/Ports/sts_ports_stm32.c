#include "sts_protocol.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdint.h>

static UART_HandleTypeDef  *s_huart      = NULL;
static volatile uint8_t     s_tx_done    = 0U;
static volatile uint8_t     s_rx_done    = 0U;
static volatile uint8_t     s_rx_error   = 0U;
static volatile uint16_t    s_rx_bytes   = 0U;
static uint8_t              s_half_duplex = 0U;

/* Switch between direct-wired half-duplex (enabled=1) and full-duplex adapter (enabled=0).
 * HDSEL in CR3 is a protected bit on STM32F1; it can only be written while UE=0.
 * PA2 stays AF open-drain in half-duplex mode; UART TE/RE selects direction.
 * No internal pull-up is configured. */
void STM32_UART_SetHalfDuplex(void *huart_ptr, uint8_t enabled) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)huart_ptr;
    s_half_duplex = enabled;

    huart->Instance->CR1 &= ~USART_CR1_UE;   /* disable UART so CR3 write takes effect */

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2;

    if (enabled) {
        huart->Instance->CR3 |=  USART_CR3_HDSEL;
        /* Leave the bus released until the first transmission. */
        huart->Instance->CR1 = (huart->Instance->CR1 & ~USART_CR1_TE) | USART_CR1_RE;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    } else {
        huart->Instance->CR3 &= ~USART_CR3_HDSEL;
        huart->Instance->CR1 |=  (USART_CR1_TE | USART_CR1_RE);
        /* Restore AF push-pull for full-duplex (Waveshare adapter). */
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    }
    HAL_GPIO_Init(GPIOA, &gpio);

    
    huart->Instance->CR1 |= USART_CR1_UE;    /* re-enable */
}

/* Called from USART2_IRQHandler on IDLE line. STM32F1 requires SR -> DR read to
 * clear the flag. Capture the DMA count before AbortReceive. */
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
    /* Full buffer consumed before IDLE fired  */
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

/* Aborts DMA and clears STM32F1 error flags, then drains stale RX bytes. */
static void uart_drain_rx(UART_HandleTypeDef *huart) {
    HAL_UART_AbortReceive(huart);

    volatile uint32_t tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState   = HAL_UART_STATE_READY;

    uint8_t dummy;
    for (uint8_t i = 0U; i < 32U; i++) {
        if (HAL_UART_Receive(huart, &dummy, 1, 2) != HAL_OK) { 
            break; 
        }
    }
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

    if (!s_half_duplex) {
        /* Full-duplex (Waveshare adapter): arm RX before TX
         the adapter switches direction the moment  last byte goes out, so DMA must be ready first. */
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        if (HAL_UART_Receive_DMA(huart, bus->rx_buf, STS_MAX_RX_BUFFER) != HAL_OK) {
            __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
            return STS_ERR_RX_FAIL;
        }
    } else {
        HAL_HalfDuplex_EnableTransmitter(huart);
    }

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

    if (s_half_duplex) {
        /* Wait for the shift register to drain before switching direction */
        uint32_t tc_start = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - tc_start) >= 5U) { break; }
        }

        HAL_HalfDuplex_EnableReceiver(huart);

        /* No GPIO switch or turnaround flush; see docs/hardware-validation.md. */

        HAL_StatusTypeDef dma_rx = HAL_UART_Receive_DMA(huart, bus->rx_buf, STS_MAX_RX_BUFFER);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        if (dma_rx != HAL_OK) {
            __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
            return STS_ERR_RX_FAIL;
        }
    }

    return STS_OK;
}

sts_result_t STM32_UART_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout) {
    if (bus == NULL || data == NULL) {
        return STS_ERR_NULL_PTR;
    }

    /* data/len are polling-port parameters; DMA writes to bus->rx_buf directly. */
    (void)data;
    (void)len;

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;

    /* DMA + IDLE interrupt path (used for both full-duplex and half-duplex).
     * IDLE may have already fired before this call. */
    if (s_rx_done) {
        if (s_rx_bytes < STS_PKT_FIXED_TOTAL) {
            uart_drain_rx(huart);
            return STS_ERR_RX_FAIL;
        }
        return STS_OK;
    }

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
    if (bus == NULL) {
        return STS_ERR_NULL_PTR; 
        }
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    uart_drain_rx(huart);
    return STS_OK;
}

void STS_Delay_ms(uint32_t ms)  {
     HAL_Delay(ms); 
}

uint32_t STS_GetTick_ms(void)   { 
    return HAL_GetTick(); 
}
