#pragma once

/* Instructions*/
#define STS_INST_PING       0x01
#define STS_INST_READ       0x02
#define STS_INST_WRITE      0x03
#define STS_INST_REGWRITE   0x04
#define STS_INST_ACTION     0x05
#define STS_INST_RESET      0x06
#define STS_INST_SYNCREAD   0x82
#define STS_INST_SYNCWRITE  0x83

/*EPROM REGISTERS */
#define STS_REG_ID                  0x05 // Servo ID (0-253, 254 for broadcast)
#define STS_REG_BAUDRATE            0x06 // Baud rate selector
#define STS_REG_RETURN_DELAY        0x07 // Response delay time
#define STS_REG_RESPONSE_LEVEL      0x08 // Reply level
#define STS_REG_MIN_ANGLE_LIMIT     0x09 // CW angle limit (2 bytes)
#define STS_REG_MAX_ANGLE_LIMIT     0x0B // CCW angle limit (2 bytes)
#define STS_REG_MAX_TEMP_LIMIT      0x0D // Max temp (°C)
#define STS_REG_MAX_VOLT_LIMIT      0x0E // Max volt (0.1V units)
#define STS_REG_MIN_VOLT_LIMIT      0x0F // Min volt (0.1V units)
#define STS_REG_MAX_TORQUE_LIMIT    0x10 // Max torque (2 bytes)
#define STS_REG_P_GAIN              0x15 // Proportional gain
#define STS_REG_D_GAIN              0x16 // Differential gain
#define STS_REG_I_GAIN              0x17 // Integral gain
#define STS_REG_OFF_OFFSET          0x1F // Position correction (2 bytes)
#define STS_REG_OPERATION_MODE      0x21 // 0: Position, 1: Speed, 2: PWM

/* RAM REGISTERS*/
#define STS_REG_TORQUE_ENABLE       0x28 // 1: Enable, 0: Disable
#define STS_REG_ACCELERATION        0x29 // Movement acceleration
#define STS_REG_GOAL_POSITION       0x2A // Target position (2 bytes)
#define STS_REG_GOAL_TIME           0x2C // Movement time (2 bytes)
#define STS_REG_GOAL_SPEED          0x2E // Target speed (2 bytes)
#define STS_REG_LOCK                0x33 // 0: Unlock EPROM, 1: Lock

/*FEEDBACK REGISTERS */
#define STS_REG_PRESENT_POSITION    0x38 // Current position (2 bytes)
#define STS_REG_PRESENT_SPEED       0x3A // Current speed (2 bytes)
#define STS_REG_PRESENT_LOAD        0x3C // Current output load (2 bytes)
#define STS_REG_PRESENT_VOLTAGE     0x3E // Current input voltage
#define STS_REG_PRESENT_TEMP        0x3F // Internal temperature
#define STS_REG_ASYNC_WRITE_FLAG    0x40 // Flag for RegWrite/Action
#define STS_REG_STATUS_ERROR        0x41 // Hardware error byte
#define STS_REG_MOVING_FLAG         0x42 // 1: Moving, 0: Stopped

/* STATUS ERROR BITMASKS (Address 0x41) */
#define STS_ERR_VOLTAGE     (1 << 0) //  Voltage error
#define STS_ERR_TEMP        (1 << 1) //  Temperature sensor error
#define STS_ERR_OVERHEAT    (1 << 2) // Temperature current angle overload
#define STS_ERR_LOAD        (1 << 3) // Angle overload







