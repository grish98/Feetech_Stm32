# feetech-sts-driver

A portable, dependency-free C library for communicating with **Feetech STS series smart servos** over half-duplex UART. Implements the full STS binary packet protocol with robust noise-resilient parsing, strict error reporting, and zero dynamic memory allocation.

[![CI - Feetech Driver](https://github.com/grish98/Feetech_Stm32/actions/workflows/ci.yml/badge.svg)](https://github.com/grish98/Feetech_Stm32/actions/workflows/ci.yml)

---

## Features

- **Noise-resilient parsing** — sliding-window seeker recovers from bus collisions, false headers, and partial packet fragments
- **Zero heap allocation** — safe for deterministic real-time and RTOS environments
- **Stateless design** — thread-safe when provided unique buffers; supports multiple servos independently
- **Precise error reporting** — distinct error codes for every failure mode
- **Portable** — written in C11 with fixed-width types; no platform-specific dependencies
- **HAL-agnostic service layer** — injected function pointers decouple servo logic from MCU UART implementation; includes a full command set covering position, speed, acceleration, PWM, step, torque, telemetry, and EEPROM control
- **168 unit tests** — verified with the Unity framework via CTest

---

## Protocol Overview

The Feetech STS protocol is a binary half-duplex UART protocol. Every packet follows this structure:

| Byte(s) | Field       | Description                        |
|---------|-------------|------------------------------------|
| 0–1     | Header      | Always `0xFF 0xFF`                 |
| 2       | ID          | Servo ID (0–253, 254 = broadcast)  |
| 3       | Length      | Number of remaining bytes (excl. header + ID) |
| 4       | Instruction | Command or status byte             |
| 5..N    | Parameters  | Optional payload (0–253 bytes)     |
| N+1     | Checksum    | `~(ID + Length + Instruction + Params) & 0xFF` |

---

## Architecture

The library is split into two layers:

**Protocol Layer** (`sts_protocol`) — stateless packet framing, checksum, and response parsing. No knowledge of hardware or servo state.

**Service Layer** (`sts_servo`, `sts_servo_cmd`) — HAL-agnostic servo management built on top of the protocol layer. Handles bus wiring, servo handles, and all servo transactions through a single command engine. The command set (`sts_servo_cmd`) is part of this layer, providing motion control, telemetry reads, and configuration commands built directly on the register access primitives.

---

## Operating Modes

The STS servo supports four operating modes, set via `STS_SetOperatingMode`. The active mode determines how target commands are interpreted.

| Mode                | Description |
|---------------------|-------------|
| `STS_MODE_POSITION` | Moves the servo to an absolute position within `[0, STS_MAX_POSITION]`. The servo holds position under load. This is the default mode for most applications. |
| `STS_MODE_SPEED`    | Drives the servo continuously at a target speed within `[0, STS_MAX_SPEED]`. Direction is specified separately — positive values drive CCW, negative values drive CW. Useful for wheel-drive or conveyor applications. |
| `STS_MODE_PWM`      | Applies a raw PWM duty cycle to the motor within `[0, STS_MAX_PWM]`. No closed-loop control is active. Direction is specified separately. Use when direct motor voltage control is required. |
| `STS_MODE_STEP`     | Commands a relative step count within `[0, STS_MAX_STEP]`. Direction is specified separately. Useful for incremental motion without tracking absolute position. |

When using `STS_SetTarget`, the sign of the `target` argument encodes direction for speed, PWM, and step modes. Position mode ignores the sign and clamps negative values to zero.

---

## API Reference

Full Doxygen-generated documentation can be built locally (see [Building Docs](#building-docs)).

### Protocol Layer

#### `sts_calculate_checksum`
```c
sts_result_t sts_calculate_checksum(
    const uint8_t* pkt_buf,
    uint16_t       pkt_len,
    uint8_t*       checksum_out
);
```
Calculates the 8-bit NOT-sum checksum over a packet buffer. Skips the two sync headers and the final checksum byte.

---

#### `sts_create_packet`
```c
sts_result_t sts_create_packet(
    uint8_t        id,
    uint8_t        instruction,
    const uint8_t* param_buf,
    uint16_t       param_len,
    uint8_t*       pkt_buf,
    uint16_t       pkt_buf_size
);
```
Serialises a complete STS command packet into `pkt_buf`. Validates ID, instruction, parameter length, and buffer capacity before writing.

---

#### `sts_parse_response`
```c
sts_result_t sts_parse_response(
    uint8_t        expected_id,
    const uint8_t* rx_buf,
    uint16_t       rx_len,
    uint8_t*       param_buf,
    uint16_t       param_buf_size,
    uint16_t*      param_len
);
```
Scans a raw UART buffer for a valid response packet matching `expected_id`. Skips noise and false headers automatically. On success, extracted parameters are written to `param_buf`.

---

### Service Layer

#### `STS_Bus_Init`
```c
sts_result_t STS_Bus_Init(
    sts_bus_t*         bus,
    void*              port_handle,
    sts_hal_transmit_t tx_func,
    sts_hal_receive_t  rx_func
);
```
Initialises a shared bus handle by injecting platform-specific transmit and receive function pointers. `port_handle` is an opaque pointer passed through to the HAL functions — pass `NULL` if your transport does not require one.

---

#### `STS_Servo_Init`
```c
sts_result_t STS_Servo_Init(
    sts_servo_t* servo,
    sts_bus_t*   bus,
    uint8_t      id
);
```
Binds a servo handle to a bus and assigns it a hardware ID. Sets `is_online` to `STS_OFFLINE`. ID must be in the range 0–253.

---

#### `STS_servo_ping`
```c
sts_result_t STS_servo_ping(sts_servo_t* servo);
```
Sends a PING instruction and updates `servo->is_online`. Sets `STS_ONLINE` on `STS_OK` or `STS_ERR_HARDWARE` — a hardware fault means the servo responded, so comms are intact. Sets `STS_OFFLINE` on any communication failure. Broadcast IDs are rejected.

---

#### `STS_Write8` / `STS_Write16`
```c
sts_result_t STS_Write8 (sts_servo_t* servo, uint8_t reg_addr, uint8_t  value);
sts_result_t STS_Write16(sts_servo_t* servo, uint8_t reg_addr, uint16_t value);
```
Writes an 8-bit or 16-bit value to a servo register. 16-bit values are transmitted little-endian.

---

#### `STS_Read8` / `STS_Read16`
```c
sts_result_t STS_Read8 (sts_servo_t* servo, uint8_t reg_addr, uint8_t*  value_out);
sts_result_t STS_Read16(sts_servo_t* servo, uint8_t reg_addr, uint16_t* value_out);
```
Reads an 8-bit or 16-bit value from a servo register. Broadcast IDs are rejected — reads require a response from a single target.

---

### Service Layer — Commands

#### Motion Control

##### `STS_SetTorqueEnable`
```c
sts_result_t STS_SetTorqueEnable(sts_servo_t* servo, uint8_t enable);
```
Enables or disables servo torque. Any non-zero value enables torque; `0` disables it.

---

##### `STS_SetOperatingMode`
```c
sts_result_t STS_SetOperatingMode(sts_servo_t* servo, sts_operating_mode_t mode);
```
Sets the servo operating mode. Valid modes are `STS_MODE_POSITION`, `STS_MODE_SPEED`, `STS_MODE_PWM`, and `STS_MODE_STEP`. The `servo->current_mode` field is used by `STS_SetTarget` to route commands correctly.

---

##### `STS_SetTargetPosition`
```c
sts_result_t STS_SetTargetPosition(sts_servo_t* servo, uint16_t position);
```
Writes the goal position register. `position` must be within `[0, STS_MAX_POSITION]`.

---

##### `STS_GetPresentPosition`
```c
sts_result_t STS_GetPresentPosition(sts_servo_t* servo, uint16_t* position_out);
```
Reads the present position register.

---

##### `STS_SetTargetSpeed`
```c
sts_result_t STS_SetTargetSpeed(sts_servo_t* servo, uint16_t speed, sts_direction_t dir);
```
Writes the goal speed register with direction. `speed` must be within `[0, STS_MAX_SPEED]`. Direction is encoded into the register's direction bit.

---

##### `STS_GetPresentSpeed`
```c
sts_result_t STS_GetPresentSpeed(sts_servo_t* servo, uint16_t* speed_out);
```
Reads the present speed register.

---

##### `STS_SetTargetAcceleration`
```c
sts_result_t STS_SetTargetAcceleration(sts_servo_t* servo, uint8_t acceleration);
```
Writes the acceleration register. `acceleration` must be within `[0, STS_MAX_ACCELERATION]`.

---

##### `STS_SetTargetPWM`
```c
sts_result_t STS_SetTargetPWM(sts_servo_t* servo, uint16_t pwm, sts_direction_t dir);
```
Writes the goal speed register as a raw PWM value with direction. Used in `STS_MODE_PWM`. `pwm` must be within `[0, STS_MAX_PWM]`.

---

##### `STS_SetTargetStep`
```c
sts_result_t STS_SetTargetStep(sts_servo_t* servo, uint16_t steps, sts_direction_t dir);
```
Writes the goal position register as a step count with direction. Used in `STS_MODE_STEP`. `steps` must be within `[0, STS_MAX_STEP]`.

---

##### `STS_SetTarget`
```c
sts_result_t STS_SetTarget(sts_servo_t* servo, int32_t target);
```
Universal target dispatcher. Routes to the appropriate command based on `servo->current_mode`:

- `STS_MODE_POSITION` — calls `STS_SetTargetPosition`; negative values are clamped to 0
- `STS_MODE_SPEED` — calls `STS_SetTargetSpeed`; sign determines direction
- `STS_MODE_PWM` — calls `STS_SetTargetPWM`; sign determines direction
- `STS_MODE_STEP` — calls `STS_SetTargetStep`; sign determines direction

---

##### `STS_SetTorqueLimit`
```c
sts_result_t STS_SetTorqueLimit(sts_servo_t* servo, uint16_t limit);
```
Writes the torque limit register. `limit` must be within `[0, STS_MAX_TORQUE]`.

---

#### Telemetry

##### `STS_GetPresentLoad`
```c
sts_result_t STS_GetPresentLoad(sts_servo_t* servo, int16_t* load_out);
```
Reads the present load register as a signed 16-bit value.

---

##### `STS_GetPresentVoltage`
```c
sts_result_t STS_GetPresentVoltage(sts_servo_t* servo, uint8_t* voltage_out);
```
Reads the present voltage register.

---

##### `STS_GetPresentTemperature`
```c
sts_result_t STS_GetPresentTemperature(sts_servo_t* servo, uint8_t* temp_out);
```
Reads the present temperature register.

---

##### `STS_GetMovingStatus`
```c
sts_result_t STS_GetMovingStatus(sts_servo_t* servo, uint8_t* status_out);
```
Reads the moving flag register.

---

#### Configuration

##### `STS_SetEEPROMLock`
```c
sts_result_t STS_SetEEPROMLock(sts_servo_t* servo, uint8_t lock);
```
Locks or unlocks the EEPROM. Pass `EEPROM_LOCK` or `EEPROM_UNLOCK`. EEPROM must be unlocked before writing persistent configuration registers such as ID.

---

##### `STS_SetID`
```c
sts_result_t STS_SetID(sts_servo_t* servo, uint8_t new_id);
```
Writes a new ID to the servo's EEPROM ID register. `new_id` must be in the range `[0, STS_ID_BROADCAST_SYNC]`. The EEPROM must be unlocked with `STS_SetEEPROMLock` before calling this, and locked again immediately after to prevent flash wear.

---

### Error Codes

| Code                    | Meaning                                              |
|-------------------------|------------------------------------------------------|
| `STS_OK`                | Operation successful                                 |
| `STS_ERR_NULL_PTR`      | A required pointer argument was NULL                 |
| `STS_ERR_INVALID_LEN`   | Length value outside protocol limits                 |
| `STS_ERR_INVALID_PARAM` | Invalid ID, instruction, or out-of-range value       |
| `STS_ERR_BUF_TOO_SMALL` | Provided buffer cannot hold the packet or parameters |
| `STS_ERR_HEADER`        | Packet does not begin with `0xFF 0xFF`               |
| `STS_ERR_ID_MISMATCH`   | Response ID does not match expected ID               |
| `STS_ERR_CHECKSUM`      | Calculated checksum does not match received byte     |
| `STS_ERR_MALFORMED`     | Length field inconsistent with actual bytes received |
| `STS_ERR_HARDWARE`      | Servo reported a hardware fault in the status byte   |
| `STS_ERR_TIMEOUT`       | Servo did not respond within the deadline            |
| `STS_ERR_TX_FAIL`       | Hardware-level transmission failure                  |
| `STS_ERR_BUSY`          | Interface is currently occupied                      |

---

## Usage Example

### Protocol Layer (direct packet control)

```c
#include "sts_protocol.h"

/* Build a Ping command for servo ID 1 */
uint8_t tx_buf[STS_MIN_PACKET_SIZE];

sts_result_t res = sts_create_packet(
    0x01,           /* Servo ID      */
    0x01,           /* Ping command  */
    NULL, 0,        /* No parameters */
    tx_buf, sizeof(tx_buf)
);

if (res != STS_OK) { /* Handle error */ }

/* Transmit tx_buf over UART (platform-specific) */

uint8_t rx_buf[32];
uint8_t params[16];
uint16_t params_len = 0;

res = sts_parse_response(
    0x01,
    rx_buf, bytes_received,
    params, sizeof(params),
    &params_len
);
```

### Service Layer (HAL-injected servo control and commands)

```c
#include "sts_servo.h"

/* 1. Implement your platform HAL functions */
sts_result_t my_uart_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    return (HAL_UART_Transmit(huart, data, len, 10) == HAL_OK)
           ? STS_OK : STS_ERR_TX_FAIL;
}

sts_result_t my_uart_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    return (HAL_UART_Receive(huart, data, len, timeout_ms) == HAL_OK)
           ? STS_OK : STS_ERR_TIMEOUT;
}

/* 2. Initialise the bus and servo handles */
sts_bus_t   bus;
sts_servo_t servo;

STS_Bus_Init(&bus, &huart1, my_uart_tx, my_uart_rx);
STS_Servo_Init(&servo, &bus, 0x01);

/* 3. Ping to confirm the servo is online */
if (STS_servo_ping(&servo) == STS_OK) {
    /* servo.is_online == STS_ONLINE */
}

/* 4. Read and write registers directly */
uint16_t position = 0;
STS_Read16(&servo, STS_REG_PRESENT_POSITION, &position);
STS_Write16(&servo, STS_REG_GOAL_POSITION, 2048);
```

### High-Level Commands

```c
#include "sts_servo_cmd.h"

/* Set position mode and drive to centre */
STS_SetOperatingMode(&servo, STS_MODE_POSITION);
STS_SetTorqueEnable(&servo, 1);
STS_SetTargetAcceleration(&servo, 50);
STS_SetTargetPosition(&servo, 2048);

/* Poll until the servo stops moving */
uint8_t moving = 1;
while (moving) {
    STS_GetMovingStatus(&servo, &moving);
}

/* Read back telemetry */
uint16_t pos;
uint8_t  temp, voltage;
int16_t  load;

STS_GetPresentPosition(&servo, &pos);
STS_GetPresentTemperature(&servo, &temp);
STS_GetPresentVoltage(&servo, &voltage);
STS_GetPresentLoad(&servo, &load);

/* Universal dispatcher — sign encodes direction in speed/PWM/step modes */
STS_SetOperatingMode(&servo, STS_MODE_SPEED);
STS_SetTarget(&servo,  500);   /* CCW at speed 500 */
STS_SetTarget(&servo, -500);   /* CW  at speed 500 */

/* Persist a new ID (unlock EEPROM first, lock immediately after) */
STS_SetEEPROMLock(&servo, EEPROM_UNLOCK);
STS_SetID(&servo, 0x02);
STS_SetEEPROMLock(&servo, EEPROM_LOCK);
```

---

## Project Structure

```
feetech-sts-driver/
├── Lib/
│   └── Sts_Servo/
│       ├── Inc/
│       │   ├── sts_protocol.h      # Protocol layer API and type definitions
│       │   ├── sts_servo.h         # Service layer API and HAL typedefs
│       │   ├── sts_servo_cmd.h     # Service layer command API
│       │   └── sts_registers.h     # Servo register address definitions
│       └── Src/
│           ├── sts_protocol.c      # Protocol layer implementation
│           ├── sts_servo.c         # Service layer implementation
│           └── sts_servo_cmd.c     # Service layer command implementation
├── Tests/
│   ├── test_sts_protocol.c         # Protocol layer unit tests (Unity)
│   ├── test_sts_protocol.h         # Protocol test prototypes
│   ├── test_sts_servo.c            # Service and command layer unit tests (Unity)
│   ├── test_sts_servo.h            # Servo test prototypes
│   ├── test_sts_utils.c            # Response simulation helpers
│   ├── test_sts_utils.h            # Test utility prototypes
│   ├── test_runner_protocol.c      # Protocol layer CTest entry point
│   └── test_runner_servo.c         # Service and command layer CTest entry point
├── .github/
│   └── workflows/
│       └── ci.yml                  # CI pipeline
└── CMakeLists.txt
```

---

## Building and Testing

This library uses CMake with a dual-target build system. Tests run on the host machine using a native compiler — no hardware required.

### Prerequisites

- CMake 3.15+
- A C11 compiler (GCC or Clang)
- CTest (included with CMake)

### Build and run tests

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
cd build && ctest --output-on-failure
```

### Expected output

```
--- STS Checksum Tests ---
--- STS Creation Tests ---
--- STS Parser Tests ---
--- STS Protocol Integration Tests ---

40 Tests  0 Failures  0 Ignored
OK

--- STS Bus Initialisation Tests ---
--- STS Servo Initialisation Tests ---
--- STS Servo Read/Write Tests ---
--- STS Command Engine Tests ---
--- STS Servo Ping Tests ---
--- STS Torque Enable Tests ---
--- STS Position Control Tests ---
--- STS Speed & Acceleration Tests ---
--- STS Operating Mode Tests ---
--- STS PWM & Step Control Tests ---
--- STS Universal Target Routing Tests ---
--- STS Torque Limit Tests ---
--- STS Telemetry Tests ---
--- STS EEPROM & ID Config Tests ---

128 Tests  0 Failures  0 Ignored
OK
```

---

## Building Docs

Requires [Doxygen](https://www.doxygen.nl/).

```bash
cmake --build build --target docs
```

Generated HTML documentation will be output to `build/docs/html/index.html`.

---

## Design Notes

**Why no heap allocation?**
Dynamic memory allocation introduces non-deterministic behaviour that is unsafe in real-time control systems. All buffers are caller-provided, making memory ownership explicit.

**Why stateless parsing?**
The parser holds no internal state between calls. Each call to `sts_parse_response` is independent, making it safe to use for multiple servos simultaneously without synchronisation.

**Why does `sts_create_packet` reject instruction `0x00`?**
`0x00` is not a valid STS instruction byte. The status byte in servo responses uses `0x00` to indicate no hardware error — it has a different semantic role than a command instruction. Response packet construction for testing is handled by the `simulate_servo_response` helper in `test_sts_utils.c`.

**Why are hardware errors on ping treated as online?**
`STS_ERR_HARDWARE` means the servo responded — communication succeeded. The fault lies in the servo's internal state (overtemperature, overload, etc.), not the bus. Marking the servo offline in this case would be incorrect. Hardware error semantics belong in a higher application layer that has the context to make recovery decisions.

**Why a centralised command engine?**
All service layer transactions route through a single `sts_execute_command` function. This keeps TX framing, RX receive, and response parsing in one place, and provides a single point of change when features like mutex support or async IO are added later.

**Why does `STS_SetID` not bundle EEPROM lock/unlock?**
Bundling the lock/unlock sequence inside `STS_SetID` would hide repeated flash writes if the function were called incorrectly in a loop. Keeping the responsibility with the caller makes the flash write cost explicit and prevents accidental EEPROM wear.

**Why does `STS_SetTarget` clamp negative position values to zero instead of rejecting them?**
In position mode, a negative target has no physical meaning but a zero position is always valid. Clamping avoids a hard error for what is most likely an off-by-one in the caller's coordinate calculation, while still keeping the servo in a safe state. The other modes use sign to encode direction, so rejection there is appropriate.

---

## Compatibility

Developed and verified on a host machine via unit tests. The library has no platform-specific dependencies and is designed for portability to any target with a C11 toolchain, including STM32 Cortex-M platforms.

Fixed-width integer types (`uint8_t`, `uint16_t`, `uint32_t`) are used throughout for cross-architecture correctness.

> **Note:** Hardware validation against a physical Feetech STS servo is in progress. This section will be updated once confirmed on target hardware.

---

## Roadmap

- [x] Protocol layer — packet framing, checksum, noise-resilient parsing
- [x] Service layer — HAL-agnostic bus abstraction, command engine, register access primitives, ping, and full command set (position, speed, acceleration, PWM, step, torque, telemetry, EEPROM, ID)
- [ ] Sync Write and Bulk Read support
- [ ] HAL integration examples for STM32
- [ ] Hardware validation against physical Feetech STS servo

---

## License

Copyright (c) 2026 Grisham Balloo. All rights reserved.

---

## Author

**Grisham Balloo**