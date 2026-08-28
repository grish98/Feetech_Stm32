# Feetech STS Servo Driver (STM32)

A portable, dependency-free C11 core for **Feetech STS series smart servos** over half-duplex UART. The project includes an STM32F103 hardware port using DMA-driven half-duplex UART with IDLE-line reception. The core is covered by 198 host unit tests in CI, and the hardware port has completed 390,385 live transactions with zero hard failures.

[![CI - Feetech Driver](https://github.com/grish98/Feetech_Stm32/actions/workflows/ci.yml/badge.svg)](https://github.com/grish98/Feetech_Stm32/actions/workflows/ci.yml)

**Quick links:** [API documentation](https://grish98.github.io/Feetech_Stm32/) | [Hardware validation](docs/hardware-validation.md) | [Design decisions](docs/design-decisions.md) | [Porting guide](docs/porting.md) | [Engineering postmortem](https://github.com/grish98/Feetech_Stm32/issues/8)

---

## Status

- **Protocol and service core**: implemented and covered by **198 passing host unit tests** using Unity and CTest, gated in CI alongside a Cppcheck static-analysis pass.
- **STM32F103 port**: validated across four on-target campaigns totalling **390,385 transactions with zero retries and zero hard failures**. Full per-campaign figures and methodology are in [docs/hardware-validation.md](docs/hardware-validation.md).
- **In progress**: port simplification (external pull-up and AF open-drain, removing per-packet GPIO switching) and full-duplex adapter re-validation. Tracked in the issue tracker.

An intermittent test failure was traced to state persisting between test runs rather than the initially suspected bus EMI. The investigation and supporting campaign logs are documented as an engineering postmortem in **[issue #8](https://github.com/grish98/Feetech_Stm32/issues/8)**. The full hardware bring-up was merged in **[PR #11](https://github.com/grish98/Feetech_Stm32/pull/11)**.

---

## Features

- **Noise-resilient parsing**: a sliding-window parser resynchronises after malformed data, false headers, and partial packet fragments
- **Zero heap allocation**: all API buffers are caller-provided, making memory ownership explicit and keeping the core usable on targets without a heap
- **HAL-agnostic core**: three injected function pointers decouple servo logic from the MCU UART implementation
- **Instrumented bus**: per-transaction counters for transactions, retries, retry-saves, and hard failures support regression testing, hardware validation, and fault analysis
- **Granular error reporting**: separate result codes distinguish validation, protocol, transport, timeout, and device faults
- **Portable**: written in C11 with fixed-width types; no platform-specific dependencies in the core
- **Broad command coverage**: position, speed, acceleration, PWM, step, torque, telemetry, EEPROM, and ID control
- **198 host unit tests** (Unity/CTest), plus an on-target hardware integration and stress suite

---

## Architecture

Three layers, each independently testable:

**Protocol layer** (`sts_protocol`): stateless packet framing, checksum, and response parsing. It holds no knowledge of hardware or servo state, and each parse call has no shared parser state. Synchronisation of shared bus access remains the responsibility of the service or application layer.

**Service layer** (`sts_servo`, `sts_servo_cmd`): HAL-agnostic servo management built on the protocol layer. Handles bus wiring, servo handles, and all transactions through a single command engine (`sts_execute_command`), which also carries the transaction, retry, and failure counters. Platform UART is injected as function pointers via `sts_bus_t`, so the core has no MCU dependency. The command set (`sts_servo_cmd`) provides motion control, telemetry reads, and configuration built on the register-access primitives.

**Port layer** (`Ports/sts_ports_stm32.c`): the STM32F103 implementation of the injected transmit, receive, and flush contract: DMA transfers with IDLE-line variable-length reception, direct-wired half-duplex turnaround, and bounded error recovery. Implementing the transport callbacks ports the core to another MCU, with this file as a worked reference; see the [porting guide](docs/porting.md).

---

## Hardware Validation

The STM32F103 port is exercised by an on-target integration suite (`Hardware_Tests/`) over SEGGER RTT, covering protocol validation, position, speed and acceleration control, the torque state machine, and moving-status semantics. An instrumented stress runner drives repeated campaigns and reports pass, skip and fail counts, a per-test failure histogram, quiescence-gate activation, and bus-counter deltas.

| Campaign | Runs | Transactions | Retries | Hard failures |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 100 | 67,801 | 0 | 0 |
| Intermediate | 30 | 19,269 | 0 | 0 |
| Final | 200 | 152,980 | 0 | 0 |
| Flush removal | 200 | 150,335 | 0 | 0 |
| **Total** | **530** | **390,385** | **0** | **0** |

These figures characterise one bench configuration: a single MCU, servo, cable, and environment. They are direct observations, not a general reliability claim for the design. Methodology, per-campaign conditions, and the oscilloscope work that retracted an earlier transient hypothesis are documented in **[docs/hardware-validation.md](docs/hardware-validation.md)**.

---

## Selected Design Decisions

**Why are hardware errors on ping treated as online?**
`STS_ERR_HARDWARE` means the servo responded, so communication succeeded. The fault lies in the servo's internal state, such as overtemperature or overload, rather than the bus. Marking the servo offline in this case would be incorrect. Hardware error semantics belong in a higher application layer that has the context to make recovery decisions.

**Why a centralised command engine?**
All service layer transactions route through a single `sts_execute_command` function. This keeps TX framing, RX receive, and response parsing in one place, gives retry policy and the transaction counters a single home, and provides one point of change for future work such as mutex protection or asynchronous IO.

**Why caller-provided buffers?**
Dynamic allocation can introduce variable latency, fragmentation, and runtime allocation failures. Caller-provided API buffers make memory ownership and capacity explicit while keeping the core usable on targets without a heap.

The full set, including the operating-mode and direction-encoding semantics, is in **[docs/design-decisions.md](docs/design-decisions.md)**.

---

## Usage Example

The example below shows the injection contract with a minimal blocking transport. The production STM32 port in `Ports/sts_ports_stm32.c` implements the same contract over DMA with IDLE-line half-duplex reception. Full API documentation is published at **[grish98.github.io/Feetech_Stm32](https://grish98.github.io/Feetech_Stm32/)**.

```c
#include "sts_servo.h"
#include "sts_servo_cmd.h"

/* 1. Implement your platform transport functions */
sts_result_t my_uart_tx(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    return (HAL_UART_Transmit(huart, data, len, 10) == HAL_OK)
           ? STS_OK : STS_ERR_TX_FAIL;
}

/* Distinguish a silent bus from a transport fault; both are retryable, but only
   the caller can tell them apart afterwards. */
sts_result_t my_uart_rx(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)bus->port_handle;
    switch (HAL_UART_Receive(huart, data, len, timeout_ms)) {
        case HAL_OK:      return STS_OK;
        case HAL_TIMEOUT: return STS_ERR_TIMEOUT;
        default:          return STS_ERR_RX_FAIL;
    }
}

/* 2. Initialise the bus and servo handles */
sts_bus_t   bus;
sts_servo_t servo;

STS_Bus_Init(&bus, &huart2, my_uart_tx, my_uart_rx);
bus.flush_rx    = NULL;  /* optional: supply one to drain stale RX between retries */
bus.max_retries = 2U;    /* 0 = single attempt */

STS_Servo_Init(&servo, &bus, 0x01);

/* 3. Ping to confirm the servo is online */
if (STS_servo_ping(&servo) == STS_OK) {
    /* servo.is_online == STS_ONLINE */
}

/* 4. Drive the servo. Every call returns a result code; a short chain keeps the
      first failure rather than overwriting it. */
sts_result_t res = STS_SetOperatingMode(&servo, STS_MODE_POSITION);
if (res == STS_OK) { res = STS_SetTorqueEnable(&servo, 1); }
if (res == STS_OK) { res = STS_SetTargetAcceleration(&servo, 50); }
if (res == STS_OK) { res = STS_SetTargetPosition(&servo, 2048); }

if (res == STS_ERR_TIMEOUT) {
    /* Nothing answered: check wiring, servo ID, and baud rate. */
} else if (res == STS_ERR_HARDWARE) {
    /* The servo replied but reports a fault, such as overload or overtemperature.
       Communication is intact, so this is a servo-state problem, not a bus problem. */
}

/* 5. Read telemetry */
uint16_t pos  = 0U;
uint8_t  temp = 0U;
STS_GetPresentPosition(&servo, &pos);
STS_GetPresentTemperature(&servo, &temp);

/* 6. Read the bus counters at any time */
uint32_t failures = bus.hard_failures;
```

---

## Building and Testing

The library uses CMake with a dual-target build system. Host tests run on the development machine with a native compiler, so no hardware is required. The ARM firmware target is selected automatically when an `arm-none-eabi` toolchain is configured.

### Prerequisites

- CMake 3.22+
- A C11 compiler (GCC or Clang)
- CTest (included with CMake)

### Build and run tests

```bash
cmake -B build_native
cmake --build build_native --config Debug
ctest --test-dir build_native -C Debug --output-on-failure
```

All 198 tests should pass across two suites: 40 in the protocol layer and 158 in the service and command layers. The `--config` and `-C` flags are required by multi-config generators such as Visual Studio and are ignored by single-config generators such as Ninja and Unix Makefiles.

### Build the API documentation

Requires [Doxygen](https://www.doxygen.nl/). CI publishes the same output to GitHub Pages on every push to `main`.

```bash
cmake --build build_native --target docs
```

Generated HTML is written to `build_native/html/index.html`.

---

## Protocol Summary

The Feetech STS protocol is a binary half-duplex UART protocol. Every packet follows this structure:

| Byte(s) | Field       | Description                                    |
|---------|-------------|------------------------------------------------|
| 0–1     | Header      | Always `0xFF 0xFF`                             |
| 2       | ID          | Servo ID (0–253, 254 = broadcast)              |
| 3       | Length      | Number of remaining bytes (excl. header + ID)  |
| 4       | Instruction | Command or status byte                         |
| 5..N    | Parameters  | Optional payload (0–253 bytes)                 |
| N+1     | Checksum    | `~(ID + Length + Instruction + Params) & 0xFF` |

Responses are variable length, which is why the STM32 port frames them with UART IDLE-line detection rather than a fixed byte count. The servo supports four operating modes (position, speed, PWM, and step) selected through `STS_SetOperatingMode`; their semantics are described in [docs/design-decisions.md](docs/design-decisions.md).

---

## Roadmap

- [x] Protocol layer: packet framing, checksum, noise-resilient parsing
- [x] Service layer: HAL-agnostic bus abstraction, command engine, register access primitives, ping, and command coverage for position, speed, acceleration, PWM, step, torque, telemetry, EEPROM, and ID
- [x] STM32F103 port: DMA half-duplex with IDLE-line reception, hardware-validated
- [x] Transient-hypothesis measurement: oscilloscope capture found no turnaround transient, so the defensive RX flush loop was retracted and removed ([#10](https://github.com/grish98/Feetech_Stm32/issues/10))
- [ ] Port simplification: external pull-up and AF open-drain, removing per-packet GPIO switching ([#10](https://github.com/grish98/Feetech_Stm32/issues/10))
- [ ] Sync Write and Bulk Read support
- [ ] Portable on-target test suite: route `Hardware_Tests/` timing through the `STS_Delay_ms` and `STS_GetTick_ms` port hooks so the integration and stress suites can validate a new MCU port unmodified
- [ ] Full-duplex bus-adapter path re-validation ([#9](https://github.com/grish98/Feetech_Stm32/issues/9))

---

## Compatibility

The core (protocol and service layers) has no platform-specific dependencies and builds on any target with a C11 toolchain. Fixed-width integer types are used throughout for cross-architecture correctness. Porting to another MCU means supplying three transport callbacks, `transmit`, `receive`, and the optional `flush_rx`; no other platform code is required to drive servos. The [porting guide](docs/porting.md) documents the full contract, and the STM32F103 port in `Lib/STS_Servo/Ports/` is a worked reference implementation.

---

## License

Copyright (c) 2026 Grisham Balloo. All rights reserved.
