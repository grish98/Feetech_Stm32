#include "sts_protocol.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdint.h>

/* Scope-trigger marker: pulses PA0 high for the duration of the AF_PP -> INPUT
 * switch on PA2, so the scope can trigger on PA0's rising edge. Used to confirm no transient
 * exists at the switch (see RETRACTION below, #10); retained for the
 * before/after captures in the pull-up + AF_OD change. Debug-only 
 * undefine once #10's Phase 1 captures are complete and this instrumentation
 * is no longer needed. */
// #define STS_DEBUG_MARKER

// #ifdef STS_DEBUG_MARKER
// #define STS_MARKER_PORT GPIOA
// #define STS_MARKER_PIN  GPIO_PIN_0
// #define STS_MARKER_HIGH() HAL_GPIO_WritePin(STS_MARKER_PORT, STS_MARKER_PIN, GPIO_PIN_SET)
// #define STS_MARKER_LOW()  HAL_GPIO_WritePin(STS_MARKER_PORT, STS_MARKER_PIN, GPIO_PIN_RESET)
// #else
// #define STS_MARKER_HIGH()
// #define STS_MARKER_LOW()
// #endif

static UART_HandleTypeDef  *s_huart      = NULL;
static volatile uint8_t     s_tx_done    = 0U;
static volatile uint8_t     s_rx_done    = 0U;
static volatile uint8_t     s_rx_error   = 0U;
static volatile uint16_t    s_rx_bytes   = 0U;
static uint8_t              s_half_duplex = 0U;

/* Switch between direct-wired half-duplex (enabled=1) and full-duplex adapter (enabled=0).
 * HDSEL in CR3 is a protected bit on STM32F1; it can only be written while UE=0.
 * PA2 must also be reconfigured: in half-duplex mode it must be AF open-drain so that
 * when the UART releases the pin between transmissions the line is high-Z, allowing the
 * servo to drive it. AF push-pull holds the line high and prevents servo RX. */
void STM32_UART_SetHalfDuplex(void *huart_ptr, uint8_t enabled) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)huart_ptr;
    s_half_duplex = enabled;

    huart->Instance->CR1 &= ~USART_CR1_UE;   /* disable UART so CR3 write takes effect */

#ifdef STS_DEBUG_MARKER
    GPIO_InitTypeDef gpio_marker = {0};
    gpio_marker.Pin   = STS_MARKER_PIN;
    gpio_marker.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_marker.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STS_MARKER_PORT, &gpio_marker);
    STS_MARKER_LOW();
#endif

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2;

    if (enabled) {
        huart->Instance->CR3 |=  USART_CR3_HDSEL;
        /* Start idle as floating input, output driver disabled, servo can drive the line.
         * Transmit() switches to AF_PP for TX, then back to INPUT for RX each cycle. */
        huart->Instance->CR1 = (huart->Instance->CR1 & ~USART_CR1_TE) | USART_CR1_RE;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
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

/* Aborts DMA and clear STM32F1 error flags. */
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
        /* Half-duplex: switch PA2 to AF push-pull for strong TX drive, then enable
         * transmitter. */
        GPIO_InitTypeDef gpio_tx = {0};
        gpio_tx.Pin   = GPIO_PIN_2;
        gpio_tx.Mode  = GPIO_MODE_AF_PP;
        gpio_tx.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio_tx);

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

        /* Switch PA2 back to floating input before enabling receiver */
        GPIO_InitTypeDef gpio_rx = {0};
        gpio_rx.Pin  = GPIO_PIN_2;
        gpio_rx.Mode = GPIO_MODE_INPUT;
        gpio_rx.Pull = GPIO_PULLUP;
        STS_MARKER_HIGH();
        HAL_GPIO_Init(GPIOA, &gpio_rx);
        STS_MARKER_LOW();

        HAL_HalfDuplex_EnableReceiver(huart);

  /* HDSEL in CR3 is a protected bit on STM32F1, writable only while UE=0.
 * PA2 idle state is INPUT + internal pull-up (40k); the TX path switches to
 * AF_PP per-packet, then back to INPUT for RX.
 *
 * OPEN DESIGN QUESTION (#10): the line currently relies on the internal pull-up
 * between transmissions. A GPIO-marker scope capture at the AF_PP->INPUT
 * turnaround found no transient at any resolution down to 50ns (see
 * RETRACTION below) so 40k is at minimum not producing an observable
 * fault at 1 Mbaud on this hardware, though its adequacy hasn't been
 * verified analytically or against a different servo/cable/capacitance.
 * Servo output topology: measured max 3.5V on the line, consistent with
 * open-drain (or at least non-overdriving); not independently confirmed.
 * Plan unchanged: fit an external pull-up, move to AF_OD full-time, delete
 * the per-packet GPIO switching, justified now by simplification and
 * measured noise-immunity gains (see #10) rather than by a disproven
 * transient. Works as-is: 240k+ transactions, zero failures (see #10).


 * RETRACTION: this used to be followed by a defensive SR/DR flush loop here,
 * draining RXNE/ORE/FE for up to 1ms before re-arming RX DMA, on the theory
 * that the AF_PP -> INPUT turnaround produced a capacitive transient
 * misread as a start bit. A GPIO-marker oscilloscope capture found no
 * transient at any zoom level (1us down to 50ns/div, peak-detect, full
 * switch-to-response window); the hypothesized mechanism does not occur.
 * A 200-run / 150,335-transaction stress campaign with the flush bypassed
 * completed clean (200/200, 0 retries, 0 hard failures), with every failure
 * path in sts_execute_command and uart_drain_rx confirmed to be counted, so
 * no silent absorption could have masked a live issue. The loop was masking
 * nothing. The specific character of garbage bytes observed during original
 * bring-up (on a different, now-destroyed board) was never recorded in the
 * commit history and cannot be independently verified. The pull-up sizing /
 * output-topology question above is a separate, still-open item.
 */

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

    /* data/len are polling-port parameters; DMA/polling write to bus->rx_buf directly. */
    (void)data;
    (void)len;

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;

    /* DMA + IDLE interrupt path (used for both full-duplex and half-duplex).
     * IDLE may have already fired during the transmit polling loop. */
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
