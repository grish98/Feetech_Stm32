/**
 * @file  main.h (host test stub)
 * @brief Stands in for the CubeMX main.h when App/Src/app_main.c is built for host tests.
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint32_t unused;
} UART_HandleTypeDef;

void HAL_Delay(uint32_t ms);
