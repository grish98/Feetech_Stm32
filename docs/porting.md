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

`receive` must never return the bytes `transmit` just sent. An echoed ping is itself a valid packet, so the engine would read it as a reply from a servo reporting a hardware fault. A half-duplex port whose transmitter disables the receiver cannot echo; a full-duplex adapter can. To check a port, ping an ID with no servo connected: a conforming port returns `STS_ERR_TIMEOUT`, an echoing one returns `STS_ERR_HARDWARE`.

### Wiring it up

`STS_Bus_Init` accepts only the two required callbacks. `flush_rx` and `max_read_retries` are assigned directly afterwards, because `STS_Bus_Init` zeroes the whole handle first:

```c
static sts_bus_t bus;

STS_Bus_Init(&bus, &huart2, MyPort_Transmit, MyPort_Receive);
bus.flush_rx    = MyPort_FlushRx;  /* optional */
bus.max_read_retries = 2U;         /* reads and ping only; 0 = one attempt */
```

Assigning `flush_rx` before `STS_Bus_Init` has no effect: the init call clears it. `max_read_retries` accepts 0 to `STS_MAX_READ_RETRIES` (254); with 255 every transaction, read or write, returns `STS_ERR_INVALID_PARAM` and sends nothing. Writes always make one attempt, because the engine cannot tell a lost command from a lost acknowledgement. If the command was sent and the call fails, the outcome is unknown; argument errors send nothing. Read the register back where it matters.

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
- TX must finish using `data` before `transmit` returns. The STM32 port waits for UART transmit completion.
- RX buffers must not be cleared or reused while DMA can still write to them. The STM32 port arms RX DMA inside `transmit`, so RX can remain active after that callback returns. `receive` waits for completion or aborts reception on failure.

**Current integration limitation:** the command engine clears `rx_buf` after `transmit` returns, even though the STM32 port has already armed DMA into that buffer. A sufficiently early response can therefore be partly or entirely erased. The buffer clear needs to happen before RX is armed; the current bench results do not eliminate this race. Commands that skip `receive` also leave the port's RX operation armed until an interrupt or the next transmit aborts it, so the bus must remain alive during that interval.

`sts_bus_t` is self-contained, so it can be a static or stack object with no further setup.

## Receive length and timeout behaviour

The engine performs a two-stage receive per transaction:

1. **Stage 1** requests exactly `STS_PKT_FIXED_TOTAL` (4) bytes into `rx_buf`: two header bytes, ID, and the length field.
2. The engine adds the four header bytes to the length field. That total must equal the caller's `expected_rx_len` and be at most `STS_MAX_RX_BUFFER` (128). Otherwise the attempt returns `STS_ERR_MALFORMED` and calls `flush_rx`; a read then retries if attempts remain.
3. **Stage 2** requests `packet_len_field` further bytes into `&rx_buf[4]`.

Two implementation strategies both satisfy this:

- **Byte-count transports** (blocking or interrupt-driven) honour `data` and `len` literally, reading exactly `len` bytes into `data` for each stage.
- **Whole-packet transports** (the STM32 DMA port) ignore `data` and `len`, capture the entire response into `bus->rx_buf` in one operation, and return `STS_OK` on both stage calls once the packet has arrived. This works because both stages address the same buffer the DMA already filled, so the second call finds the data in place and returns immediately.

If you take the second approach, `receive` must still reject a response shorter than `STS_PKT_FIXED_TOTAL`; otherwise stage 1 will hand the engine an uninitialised length byte.

`timeout_ms` is supplied per command and defaults to `STS_DEFAULT_TIMEOUT_MS` (10 ms). Return `STS_ERR_TIMEOUT` when it expires. A read retries after a timeout; a write returns it.

## Half-duplex turnaround

The core has no concept of bus direction. Turnaround is entirely the port's responsibility, and it must be complete before `transmit` returns, because the engine calls `receive` immediately afterwards with no delay in between.

A port for a single-wire bus must therefore, inside `transmit`:

1. Drive the line for transmission.
2. Send the packet.
3. Wait for the final stop bit to leave the shift register, not merely for the DMA or FIFO to drain.
4. Release the line and enable reception.
5. Arm the receiver before returning, so that a fast servo response is not missed.

The STM32 port does this by keeping PA2 in alternate-function open-drain and switching UART direction through TE/RE, waiting on the UART `TC` flag before enabling RX, then arming RX DMA and enabling the IDLE interrupt. Variable-length responses are framed by the IDLE line: the DMA is armed for the full buffer, and the byte count is recovered from the DMA counter when the line goes idle.

There is no per-packet GPIO reconfiguration or timed flush between TX and RX. The single SR/DR clear before a new transmission, the `uart_drain_rx` error-recovery routine, and the optional `STM32_UART_FlushRx` callback remain. The command engine calls that callback before read retries and after an unexpected response length.

The half-duplex configuration does not enable an STM32 internal pull-up. Fit **1.5 kOhm from PA2/DATA to 3.3 V** for the tested **1 Mbaud, 8N1** configuration. The latest report supersedes the earlier report of resistor-free operation: removing the external pull-up caused complete communication failure on this bench, despite the servo's measured weak idle bias. The failure mechanism is unresolved. Two 200-run campaigns passed with the resistor fitted; see [hardware validation](hardware-validation.md#current-af-open-drain-port) for measurements and remaining electrical evidence gaps.

If your transport uses an external direction-switching adapter, arm reception *before* transmitting instead; the adapter may flip direction the instant the last byte goes out. The STM32 port takes this branch when configured for full duplex.

### Current receive-timing limitations

Besides the buffer-clear race described above, the half-duplex path enables the receiver before starting RX DMA. This leaves a short setup window before DMA can service incoming bytes. The latest tests establish operation at the tested response timing, not a bound on the minimum response delay the port can support.

`USART2_IRQHandler` also checks the IDLE status flag without checking whether the IDLE interrupt is enabled. If IDLE is pending when another UART source triggers the handler, the IDLE callback can run outside the intended receive window. These are review findings in the current implementation; the AF_OD/comment update does not change their behavior.

[Issue #10](https://github.com/grish98/Feetech_Stm32/issues/10) tracks further receive-state hardening in Phase 2. The current IDLE callback dereferences `hdmarx` without a receive-armed guard and calls `HAL_UART_AbortReceive` in interrupt context, while thread-side paths also abort and reset HAL state. The planned work verifies HAL abort behavior, moves abort/cleanup to thread context, protects shared state against ISR interleaving (including already-pended interrupts), adds DMA/receive-armed guards, and standardizes both wiring paths on DMA-before-IDLE arming. The full-duplex path currently enables IDLE before starting DMA. These changes and their campaign gate remain pending.

Phase 3 will replace the blocking error drain with bounded register-level recovery and review whether the post-TX `TC` check is redundant. The existing drain permits up to 32 blocking receive calls with a 2 ms timeout each, but exits on the first unsuccessful call; this does not imply a fixed 64 ms delay. Phase 1 bench success does not establish that these software concerns are resolved. Adapter runtime validation remains separately tracked in [#9](https://github.com/grish98/Feetech_Stm32/issues/9).

## Error-code mapping

Return these from the callbacks. The right-hand column is what the engine does with each.

| Code | Meaning | Engine behaviour |
| --- | --- | --- |
| `STS_OK` | Operation completed | Proceeds to the next stage |
| `STS_ERR_TX_FAIL` | Transmission failed at the hardware level | A read retries if attempts remain; a write returns it |
| `STS_ERR_RX_FAIL` | Reception failed, or the response was too short | A read retries if attempts remain; a write returns it |
| `STS_ERR_TIMEOUT` | No response within `timeout_ms` | A read retries if attempts remain; a write returns it |
| `STS_ERR_NULL_PTR` | `bus` or `data` was `NULL` | Returned at once: the fault is a caller bug |

Codes the port should **not** invent: `STS_ERR_HARDWARE`, `STS_ERR_CHECKSUM`, `STS_ERR_MALFORMED`, and `STS_ERR_ID_MISMATCH` are produced by the protocol layer from packet content. A transport that returns them would misreport a wiring fault as a servo fault.

Any result other than `STS_OK` or `STS_ERR_HARDWARE` on the final attempt increments `bus->hard_failures`. Keeping the mapping accurate is what makes the reliability counters meaningful.

## Thread safety and bus synchronisation

The command engine is not thread-safe. Its two entry points, `sts_execute_command` (writes) and `sts_execute_read` (reads and ping), share `tx_buf` and `rx_buf` across all servos on a bus with no locking, so two concurrent transactions on one bus will corrupt each other. The parser being stateless does not change this; the buffers are the shared resource.

For an RTOS port, take a per-bus mutex in both entry points and hold it for the whole call. A read's retries and the bus counter updates must not interleave with another task's transaction, so locking around a single attempt is not enough.

The STM32 port additionally keeps its UART state in file-scope statics, so it supports exactly one bus instance. A port supporting several buses must move that state into a per-bus context reached through `port_handle`.

## Bringing up a new MCU

1. **Configure the UART**: 1 Mbaud, 8 data bits, no parity, 1 stop bit. Confirm the baud rate error at your clock is within tolerance before suspecting the driver.
2. **Implement `transmit` and `receive`.** Start with the simplest blocking implementation; correctness first, DMA later.
3. **Implement `flush_rx`** if your peripheral latches error flags that survive an aborted read. It is called before each read retry and after an unexpected response length.
4. **Wire up the bus** as shown above, with `max_read_retries = 0` initially so failures surface immediately rather than being masked by retries.
5. **Ping a known ID.** `STS_servo_ping` is the smallest complete transaction: it exercises framing, transmit, both receive stages, and parsing. Treat only `STS_OK` or `STS_ERR_HARDWARE` as a confirmed response from that servo; `STS_ERR_HARDWARE` still counts, because the servo answered and merely reported an internal fault.
6. **Scan for the servo** with `STS_Bus_Scan` if the ID is unknown. It pings each ID in a range once and stops at the first `STS_OK` or `STS_ERR_HARDWARE`. A ping is a read, so it is retried up to `max_read_retries` times, and the callback receives each ID's final result. With 0, a noise burst on the only reply hides that servo; with 1, as the example app uses, every absent ID costs two receive timeouts. No other result counts as a hit. Log each ping through the callback during bring-up.
7. **Run the integration suite** (`STS_RunIntegrationTests`) once single transactions work, then a stress campaign. Note that the suite currently calls the STM32 HAL for its timing, so reusing it on another MCU means substituting those calls first. Watch `total_retries` and `hard_failures` rather than only the pass and fail counts: a port that works but retries constantly is a port with a marginal turnaround.
