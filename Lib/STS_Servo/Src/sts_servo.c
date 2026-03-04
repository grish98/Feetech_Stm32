/**
 ******************************************************************************
 * @file           : sts_servo.c
 * @brief          : STS Service Layer Implementation
 * @author         : Grisham Balloo
 * @date           : 2026-03-03
 * @version        : 0.1.0
 ******************************************************************************
 * @details
 * This module implements the high-level service functions for the Feetech 
 * STS protocol. It handles the shared bus abstraction and manages individual 
 * servo handles, providing a hardware-agnostic interface for motor control.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "sts_servo.h"
#include "sts_protocol.h"

sts_result_t STS_Bus_Init(sts_bus_t *bus, void *port_handle, sts_hal_transmit_t tx_func, sts_hal_receive_t rx_func) {
    if (bus == NULL || tx_func == NULL || rx_func == NULL) {
        return STS_ERR_NULL_PTR;
    }

    bus->port_handle = port_handle;
    bus->transmit = tx_func;
    bus->receive = rx_func;

    return STS_OK;
}

sts_result_t STS_Servo_Init(sts_servo_t *servo, sts_bus_t *bus, uint8_t id) {
    if (servo == NULL || bus == NULL) {
        return STS_ERR_NULL_PTR;
    }

    if (id >= STS_MAX_ID ) {
        return STS_ERR_INVALID_PARAM;
    }

    servo->bus = bus;
    servo->id = id;
    servo->is_online = STS_OFFLINE; 

    return STS_OK;
}

