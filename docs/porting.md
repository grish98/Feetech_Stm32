# Porting Guide

The core (`sts_protocol`, `sts_servo`, `sts_servo_cmd`) has no platform dependencies. Porting to a new MCU means supplying transport callbacks and nothing else. `Lib/STS_Servo/Ports/sts_ports_stm32.c` is a worked reference implementation for STM32F103.

## The contract

Three function-pointer types, declared in `sts_servo.h`:

```c
typedef sts_result_t (*sts_hal_transmit_t)(sts_bus_t *bus, const uint8_t *data, uint16_t len);
typedef sts_result_t (*sts_hal_receive_t) (sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout_ms);
typedef sts_result_t (*sts_hal_flush_rx_t)(sts_bus_t *bus);
```

`transmit` and `receive` are required. `flush_rx` is optional; if left `NULL`, `STS_Bus_FlushRx` returns `STS_OK` and does nothing.

Every callback receives the bus handle, so platform state is reached through `bus->port_handle` rather than through globals. The core never dereferences `port_handle`; it is opaque and may be `NULL` if your transport does not need one.

### Wiring it up

`STS_Bus_Init` accepts only the two required callbacks. `flush_rx` and `max_retries` are assigned directly afterwards, because `STS_Bus_Init` zeroes the whole handle first:

```c
static sts_bus_t bus;

STS_Bus_Init(&bus, &huart2, MyPort_Transmit, MyPort_Receive);
bus.flush_rx    = MyPort_FlushRx;  /* optional */
bus.max_retries = 2U;              /* 0 = one attempt, no retries */
```

Assigning `flush_rx` before `STS_Bus_Init` has no effect: the init call clears it.

## Timing hooks

None are required. The core never asks the platform for the current time or for a delay, so there is no timing hook to implement in order to drive servos. The only timing requirement lives inside your own `receive` implementation, which needs some way to enforce `timeout_ms`, and how you obtain it is up to you.

`sts_ports_stm32.h` declares two further functions:

```c
void     STS_Delay_ms(uint32_t ms);
uint32_t STS_GetTick_ms(void);
```

Nothing currently calls them. The core does not use them, and the STM32 port calls `HAL_Delay` and `HAL_GetTick` directly. Treat them as vestigial rather than as part of the porting contract.

They are, however, the natural basis for one. The on-target suite in `Hardware_Tests/` performs roughly fifty delay and tick operations against the STM32 HAL directly, and that is effectively the only thing tying the suite to this MCU: apart from a single diagnostic line that prints a HAL error code, it otherwise talks only to the driver API. Routing those calls through the two hooks would let a new port reuse the integration tests and the instrumented stress runner as-is instead of rewriting them. That work is not done yet and is tracked on the roadmap.

## Buffer ownership and lifetime

The bus handle owns both buffers: `tx_buf` and `rx_buf`, each `128` bytes (`STS_MAX_TX_BUFFER` and `STS_MAX_RX_BUFFER`). The core never allocates.

- The `data` pointer passed to `transmit` points into `bus->tx_buf`.
- The `data` pointer passed to `receive` points into `bus->rx_buf`.
- **Neither pointer may be retained after the callback returns.** The engine clears `tx_buf` and `rx_buf` at the start of each transaction, so a DMA transfer still in flight would read or write a buffer being reset. The STM32 port satisfies this by blocking until the transfer completes before returning.

`sts_bus_t` is self-contained, so it can be a static or stack object with no further setup.

## Receive length and timeout behaviour

The engine performs a two-stage receive per transaction:

1. **Stage 1** requests exactly `STS_PKT_FIXED_TOTAL` (4) bytes into `rx_buf`: two header bytes, ID, and the length field.
2. The length field is validated against the RX buffer limit and the caller's `expected_rx_len`. A mismatch returns `STS_ERR_MALFORMED`, calls `flush_rx`, and consumes a retry attempt.
3. **Stage 2** requests `packet_len_field` further bytes into `&rx_buf[4]`.

Two implementation strategies both satisfy this:

- **Byte-count transports** (blocking or interrupt-driven) honour `data` and `len` literally, reading exactly `len` bytes into `data` for each stage.
- **Whole-packet transports** (the STM32 DMA port) ignore `data` and `len`, capture the entire response into `bus->rx_buf` in one operation, and return `STS_OK` on both stage calls once the packet has arrived. This works because both stages address the same buffer the DMA already filled, so the second call finds the data in place and returns immediately.

If you take the second approach, `receive` must still reject a response shorter than `STS_PKT_FIXED_TOTAL`; otherwise stage 1 will hand the engine an uninitialised length byte.

`timeout_ms` is supplied per command and defaults to `STS_DEFAULT_TIMEOUT_MS` (10 ms). Return `STS_ERR_TIMEOUT` when it expires. The engine treats a timeout as a retryable transport failure.

## Half-duplex turnaround

The core has no concept of bus direction. Turnaround is entirely the port's responsibility, and it must be complete before `transmit` returns, because the engine calls `receive` immediately afterwards with no delay in between.

A port for a single-wire bus must therefore, inside `transmit`:

1. Drive the line for transmission.
2. Send the packet.
3. Wait for the final stop bit to leave the shift register, not merely for the DMA or FIFO to drain.
4. Release the line and enable reception.
5. Arm the receiver before returning, so that a fast servo response is not missed.

The STM32 port does this by switching PA2 between alternate-function push-pull and input with pull-up, waiting on the UART `TC` flag between the two, then arming RX DMA and enabling the IDLE interrupt. Variable-length responses are framed by the IDLE line: the DMA is armed for the full buffer, and the byte count is recovered from the DMA counter when the line goes idle.

If your transport uses an external direction-switching adapter, arm reception *before* transmitting instead; the adapter may flip direction the instant the last byte goes out. The STM32 port takes this branch when configured for full duplex.

## Error-code mapping

Return these from the callbacks. The right-hand column is what the engine does with each.

| Code | Meaning | Engine behaviour |
| --- | --- | --- |
| `STS_OK` | Operation completed | Proceeds to the next stage |
| `STS_ERR_TX_FAIL` | Transmission failed at the hardware level | Retries if attempts remain |
| `STS_ERR_RX_FAIL` | Reception failed, or the response was too short | Retries if attempts remain |
| `STS_ERR_TIMEOUT` | No response within `timeout_ms` | Retries if attempts remain |
| `STS_ERR_NULL_PTR` | `bus` or `data` was `NULL` | Retries, but the fault is a caller bug |

Codes the port should **not** invent: `STS_ERR_HARDWARE`, `STS_ERR_CHECKSUM`, `STS_ERR_MALFORMED`, and `STS_ERR_ID_MISMATCH` are produced by the protocol layer from packet content. A transport that returns them would misreport a wiring fault as a servo fault.

Any result other than `STS_OK` or `STS_ERR_HARDWARE` on the final attempt increments `bus->hard_failures`. Keeping the mapping accurate is what makes the reliability counters meaningful.

## Thread safety and bus synchronisation

`sts_execute_command` is not thread-safe. It shares `tx_buf` and `rx_buf` across all servos on a bus with no locking, so two concurrent transactions on one bus will corrupt each other. The parser being stateless does not change this; the buffers are the shared resource.

For an RTOS port, take a per-bus mutex around the transaction. The command engine is the single point through which every transaction passes and is the intended place for that lock.

The STM32 port additionally keeps its UART state in file-scope statics, so it supports exactly one bus instance. A port supporting several buses must move that state into a per-bus context reached through `port_handle`.

## Bringing up a new MCU

1. **Configure the UART**: 1 Mbaud, 8 data bits, no parity, 1 stop bit. Confirm the baud rate error at your clock is within tolerance before suspecting the driver.
2. **Implement `transmit` and `receive`.** Start with the simplest blocking implementation; correctness first, DMA later.
3. **Implement `flush_rx`** if your peripheral latches error flags that survive an aborted read. It is called before each retry.
4. **Wire up the bus** as shown above, with `max_retries = 0` initially so failures surface immediately rather than being masked by retries.
5. **Ping a known ID.** `STS_servo_ping` is the smallest complete transaction: it exercises framing, transmit, both receive stages, and parsing. Treat only `STS_OK` or `STS_ERR_HARDWARE` as a confirmed response from that servo; `STS_ERR_HARDWARE` still counts, because the servo answered and merely reported an internal fault.
6. **Scan for the servo** if the ID is unknown, pinging each candidate and accepting the same two results. Do not treat "anything other than `STS_ERR_TIMEOUT`" as a hit: `STS_ERR_TX_FAIL`, `STS_ERR_RX_FAIL`, and `STS_ERR_NULL_PTR` are local faults that say nothing about whether a servo is present. A checksum, header, or ID error is weaker evidence again; it shows bytes arrived, but not that they came from the ID you asked for, so it is worth logging separately during bring-up rather than counting as a match. Note that the scan in `App/Src/app_main.c` currently uses the looser non-timeout test.
7. **Run the integration suite** (`STS_RunIntegrationTests`) once single transactions work, then a stress campaign. Note that the suite currently calls the STM32 HAL for its timing, so reusing it on another MCU means substituting those calls first. Watch `total_retries` and `hard_failures` rather than only the pass and fail counts: a port that works but retries constantly is a port with a marginal turnaround.
