/**
 ******************************************************************************
 * @file           : sts_registers.h
 * @brief          : Feetech STS Series Register Map and Instruction Set
 * @author         : Grisham Balloo
 * @date           : 2026-03-19
 * @version        : 1.0.0
 ******************************************************************************
 * @details
 * This header defines the internal memory map and command set for the 
 * Feetech STS-series smart servos. 
 * * The register map is divided into two primary sections:
 * 1. EPROM (0x05 - 0x21): Persistent settings that survive power cycles. 
 * Requires unlocking via STS_REG_LOCK to modify.
 * 2. RAM (0x28 - 0x42): Volatile control and feedback registers updated 
 * in real-time. Changes are lost upon power-off.
 * * Data Types:
 * - 1-Byte registers: Standard uint8_t access.
 * - 2-Byte registers: Little-endian (Low byte first).
 * * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#pragma once

/** * @name Protocol Special IDs
 * @{ 
 */
#define STS_ID_BROADCAST_SYNC   0xFEU /**< 254: Broadcast to all servos (Execute immediately) */
#define STS_ID_BROADCAST_ASYNC  0xFFU /**< 255: Broadcast to all servos (Wait for ACTION instruction) */
/** @} */

/** * @name Protocol Instructions
 * @{ 
 */
#define STS_INST_PING       0x01 /**< Check servo online status */
#define STS_INST_READ       0x02 /**< Read data from register */
#define STS_INST_WRITE      0x03 /**< Write data to register */
#define STS_INST_REGWRITE   0x04 /**< Buffer write data for Action command */
#define STS_INST_ACTION     0x05 /**< Trigger buffered RegWrite commands */
#define STS_INST_RESET      0x06 /**< Factory reset EPROM parameters */
#define STS_INST_SYNCREAD   0x82 /**< Read from multiple servos simultaneously */
#define STS_INST_SYNCWRITE  0x83 /**< Write to multiple servos simultaneously */
/** @} */

/** * @name EPROM Registers 
 * @note Requires @ref STS_REG_LOCK to be 0 to modify.
 * @{ 
 */
#define STS_REG_ID              0x05 /**< Servo ID (0-253, 254 for broadcast) */
#define STS_REG_BAUDRATE        0x06 /**< Communication speed selector */
#define STS_REG_RETURN_DELAY    0x07 /**< Delay time before status return (0.5us units) */
#define STS_REG_RESPONSE_LEVEL  0x08 /**< 0: None, 1: Read commands only, 2: All */
#define STS_REG_MIN_ANGLE_LIMIT 0x09 /**< CW angle limit (2 bytes) */
#define STS_REG_MAX_ANGLE_LIMIT 0x0B /**< CCW angle limit (2 bytes) */
#define STS_REG_MAX_TEMP_LIMIT  0x0D /**< Max temperature threshold (°C) */
#define STS_REG_MAX_VOLT_LIMIT  0x0E /**< Max voltage threshold (0.1V units) */
#define STS_REG_MIN_VOLT_LIMIT  0x0F /**< Min voltage threshold (0.1V units) */
#define STS_REG_MAX_TORQUE_LIMIT 0x10 /**< Max output torque (2 bytes) */
#define STS_REG_P_GAIN          0x15 /**< Proportional gain for PID */
#define STS_REG_D_GAIN          0x16 /**< Differential gain for PID */
#define STS_REG_I_GAIN          0x17 /**< Integral gain for PID */
#define STS_REG_OFF_OFFSET      0x1F /**< Center position correction (2 bytes) */
#define STS_REG_OPERATION_MODE  0x21 /**< 0: Position, 1: Speed, 2: PWM */
/** @} */

/** * @name RAM Registers 
 * @{ 
 */
#define STS_REG_TORQUE_ENABLE   0x28 /**< 1: Torque On, 0: Free move */
#define STS_REG_ACCELERATION    0x29 /**< Acceleration/deceleration ramp */
#define STS_REG_GOAL_POSITION   0x2A /**< Target position (2 bytes, 0-4095) */
#define STS_REG_GOAL_TIME       0x2C /**< Target movement duration (2 bytes) */
#define STS_REG_GOAL_SPEED      0x2E /**< Target movement speed (2 bytes) */
#define STS_REG_LOCK            0x33 /**< 0: Unlock EPROM, 1: Lock EPROM */
/** @} */

/** * @name Feedback Registers 
 * @{ 
 */
#define STS_REG_PRESENT_POSITION 0x38 /**< Current encoder position (2 bytes) */
#define STS_REG_PRESENT_SPEED    0x3A /**< Current rotational speed (2 bytes) */
#define STS_REG_PRESENT_LOAD     0x3C /**< Current motor load (2 bytes) */
#define STS_REG_PRESENT_VOLTAGE  0x3E /**< Current input voltage (0.1V units) */
#define STS_REG_PRESENT_TEMP     0x3F /**< Internal temperature (°C) */
#define STS_REG_ASYNC_WRITE_FLAG 0x40 /**< Status of buffered RegWrite */
#define STS_REG_STATUS_ERROR     0x41 /**< Hardware error */
#define STS_REG_MOVING_FLAG      0x42 /**< 1: Moving, 0: Target reached */
/** @} */

/** * @name Hardware Error Bitmasks (Address 0x41)
 * @{ 
 */
 #define STS_STATUS_OK        0x00U /**< No hardware errors */
#define STS_BIT_ERR_VOLTAGE  (1 << 0) /**< Voltage outside safety limits */
#define STS_BIT_ERR_TEMP     (1 << 1) /**< Internal temperature sensor fault */
#define STS_BIT_ERR_OVERHEAT (1 << 2) /**< Over-temperature detected */
#define STS_BIT_ERR_LOAD     (1 << 3) /**< Overload/Stall detected */
/** @} */

/** * @name Common State Values 
 * @{ 
 */
#define STS_TORQUE_DISABLE  0x00U /**< Disable motor torque (free spin) */
#define STS_TORQUE_ENABLE   0x01U /**< Enable motor torque (hold position) */
/** @} */


/** * @name Speed Register Bitmasks (Addresses 0x2E and 0x3A)
 * @brief Masks for parsing target and present speed registers.
 * @{ 
 */
#define STS_SPEED_DIRECTION_BIT   0x8000U  /**< Bit 15: 0 = CCW (Forward), 1 = CW (Reverse) */
#define STS_SPEED_MAGNITUDE_MASK  0x7FFFU  /**< Bits 0-14: Raw speed magnitude value */
/** @} */
