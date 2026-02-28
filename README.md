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
- **38 unit tests** — verified with the Unity framework via CTest

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

## API Reference

Full Doxygen-generated documentation can be built locally (see [Building Docs](#building-docs)).

### `sts_calculate_checksum`
```c
sts_result_t sts_calculate_checksum(
    const uint8_t* pkt_buf,
    uint16_t       pkt_len,
    uint8_t*       checksum_out
);
```
Calculates the 8-bit NOT-sum checksum over a packet buffer. Skips the two sync headers and the final checksum byte.

---

### `sts_create_packet`
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

### `sts_parse_response`
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

### Error Codes

| Code                    | Meaning                                              |
|-------------------------|------------------------------------------------------|
| `STS_OK`                | Operation successful                                 |
| `STS_ERR_NULL_PTR`      | A required pointer argument was NULL                 |
| `STS_ERR_INVALID_LEN`   | Length value outside protocol limits                 |
| `STS_ERR_INVALID_PARAM` | Invalid ID or instruction byte                       |
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

if (res != STS_OK) {
    /* Handle creation error */
}

/* Transmit tx_buf over UART (platform-specific) */

/* Parse the servo response */
uint8_t rx_buf[32];
uint8_t params[16];
uint16_t params_len = 0;

/* Populate rx_buf from UART receive (platform-specific) */

res = sts_parse_response(
    0x01,                       /* Expected servo ID    */
    rx_buf, bytes_received,     /* Raw UART data        */
    params, sizeof(params),     /* Output buffer        */
    &params_len
);

if (res == STS_OK) {
    /* params[0..params_len-1] contains the response payload */
} else if (res == STS_ERR_HARDWARE) {
    /* Servo reported a fault — inspect raw status byte in your servo layer */
}
```

---

## Project Structure

```
feetech-sts-driver/
├── Lib/
│   └── Sts_Servo/
│       ├── Inc/
│       │   └── sts_protocol.h      # Public API and type definitions
│       └── Src/
│           └── sts_protocol.c      # Protocol implementation
├── Tests/
│   ├── test_sts_protocol.c         # Unit test cases (Unity)
│   ├── test_sts_protocol.h         # Test function prototypes
│   ├── test_sts_utils.c            # Response simulation helpers
│   ├── test_sts_utils.h            # Test utility prototypes
│   └── test_runner.c               # CTest entry point
├── .github/
│   └── workflows/
│       └── tests.yml               # CI pipeline
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

38 Tests  0 Failures  0 Ignored
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

**Hardware error handling**
This library intentionally reports `STS_ERR_HARDWARE` without interpreting the fault bits. Hardware error semantics — overtemperature, overload, voltage fault — belong in a higher servo layer that has the context to make recovery decisions.

---

## Compatibility

TDeveloped and verified on a host machine via unit tests. The library has no 
platform-specific dependencies and is designed for portability to any target 
with a C11 toolchain, including STM32 Cortex-M platforms.

Fixed-width integer types (`uint8_t`, `uint16_t`, `uint32_t`) are used 
throughout for cross-architecture correctness.

> **Note:** Hardware validation against a physical Feetech STS servo is 
> in progress. This section will be updated once confirmed on target hardware.

---

## Roadmap

- [ ] Servo layer — high-level register abstraction and hardware error interpretation
- [ ] HAL integration examples for STM32
- [ ] Hardware validation against physical Feetech STS servo
- [ ] Sync Write and Bulk Read support

## License

Copyright (c) 2026 Grisham Balloo. All rights reserved.

---

## Author

**Grisham Balloo**