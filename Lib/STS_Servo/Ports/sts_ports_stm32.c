// Testing with Uart Polling FUll duplex

#include "sts_protocol.h"
#include "sts_servo.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdint.h>


//TODO: Implement UART DMA, non blocking send and receive functions
sts_result_t STM32_UART_Transmit(sts_bus_t *bus, const uint8_t *data, uint16_t len){
    if(bus == NULL || data == NULL) {
        return STS_ERR_NULL_PTR;
    }

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)data, len, 10);

    if (status == HAL_OK) {
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
        return STS_OK;
    }
    switch (status) {
        case HAL_BUSY:    return STS_ERR_BUSY;
        case HAL_TIMEOUT: return STS_ERR_TIMEOUT;
        default:          return STS_ERR_TX_FAIL;
    }
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

    switch (status) {
        case HAL_TIMEOUT: 
            return STS_ERR_TIMEOUT;
        case HAL_BUSY:    
            return STS_ERR_BUSY;
        default:          
            return STS_ERR_RX_FAIL;
    }
}

void STS_Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

uint32_t STS_GetTick_ms(void) {
    return HAL_GetTick();
}