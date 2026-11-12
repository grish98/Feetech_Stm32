# Design Decisions

Rationale for the choices that shaped the driver. Three of these also appear in the README; the rest live here.

## Electrical interface

**Why permanent AF open-drain with an external pull-up?**
Keeping PA2 in AF_OD removes per-packet GPIO mode changes; UART TE/RE controls direction. The selected single-servo bench configuration uses 1.5 kOhm from PA2/DATA to 3.3 V at 1 Mbaud, 8N1. Two campaigns recorded 304,227 transactions with zero retries or hard communication failures. Removing the external resistor caused communication failure despite the servo's weak idle pull-up, so the resistor is required for this tested setup.

Direction-separated captures measured approximately 195 ns command rise time and 5 ns reply rise time. These results support retaining the tested configuration; they do not establish the original GPIO-transition failure mechanism, an optimal resistor value, or a guaranteed receiver timing margin. Settled-low and threshold-margin evidence remain outstanding. See [hardware validation](hardware-validation.md#current-af-open-drain-port) for the measurements, evidence links, and limits of the result.

## Memory and state

**Why caller-provided buffers?**
Dynamic allocation can introduce variable latency, fragmentation, and runtime allocation failures. Caller-provided API buffers make memory ownership and capacity explicit while keeping the core usable on targets without a heap. The bus handle carries its own `tx_buf` and `rx_buf`, both fixed at `STS_MAX_TX_BUFFER` and `STS_MAX_RX_BUFFER` (128 bytes each), so a `sts_bus_t` is entirely self-contained and can be a static or stack object.

**Why stateless parsing?**
The parser holds no internal state between calls. Each call to `sts_parse_response` is independent, so responses for multiple servo instances can be parsed without shared parser state. Access to a shared physical bus still requires coordination by the service or application layer; statelessness in the parser does not by itself make concurrent bus use safe.

The receive ring buffer's overwrite checks, counter limits and memory
requirements are explained in [Ring buffer design](ring-buffer.md).

## Transactions

**Why a centralised command engine?**
All service layer transactions route through one command engine: `sts_execute_command` makes a single attempt and serves every write, and `sts_execute_read` runs the same attempt in a retry loop for reads and ping. Both share one attempt function, so TX framing, RX receive, and response parsing live in one place, and the retry policy and the bus counters live in one module. That module is the one point of change for future work such as mutex protection (taken in both entry points) or asynchronous IO. Every register primitive and every high-level command is a thin wrapper that fills in an `sts_cmd_t` and calls one of the two.

**Why does the engine never retry writes?**
The engine cannot tell a lost command from a lost acknowledgement. Resending a write the servo already executed repeats it: a relative step moves twice, and an ID change is resent to the old ID and then reported as failed. Reads and ping have no side effects on the servo, so only they retry. If the command was sent and the call fails, the outcome is unknown; argument errors send nothing. Read the register back where it matters.

**How do I know a write took effect?**
Use a verified write. `STS_Write8Verified` and `STS_Write16Verified` write once, then read the register back, even when the write itself failed, since a lost acknowledgement can hide a write the servo applied. They return `STS_OK` when the read-back matches, `STS_ERR_HARDWARE` when it matches but the servo flags a fault in its status byte, `STS_ERR_VERIFY` when it differs, and the read's own code when the read-back fails; they never re-send. The named variants for torque enable, operating mode and torque limit re-send once when the first read-back differs or fails, because a repeat of an absolute register gives the same final state. A faulted servo flags every reply, so a matching read-back with the fault flag set still counts as confirmed and is not re-sent: turning torque off on an overheated servo need not be sent twice. `STS_SetIDVerified` never re-sends: it confirms the change with a ping at the new ID and then points the handle at it, so the new ID must be free on the bus. A write whose acknowledgement was lost still counts in `hard_failures`, even when the read-back confirms it. The read-back or ping follows the write at once, and how long a servo takes to answer after an EEPROM-area write (mode, ID) has not been measured: with no read retries configured, a slow answer would show as an unconfirmed change. Goal position has no verified variant yet: in step mode it is a relative count, and what the servo reports when it is read back after a step has not been measured.

**Why a two-stage receive?**
STS responses are variable length. The engine first reads the four fixed bytes (two header bytes, ID, and length), extracts the length field, checks it against the caller's `expected_rx_len` and the RX buffer limit, and only then reads the remaining payload. Reading a length-prefixed payload in two stages means a corrupt length field is rejected before it can be used to size a read, rather than after.

**Why is a hardware error never retried?**
`STS_ERR_HARDWARE` means the servo replied and reported a fault in its status byte. That is a definitive answer, not a transport failure, so the engine breaks out of the retry loop immediately. Retrying would waste bus time re-asking a question that has already been answered. The same reasoning drives the ping behaviour below.

**Why are hardware errors on ping treated as online?**
`STS_ERR_HARDWARE` means the servo responded, so communication succeeded. The fault lies in the servo's internal state, such as overtemperature or overload, rather than the bus. Marking the servo offline in this case would be incorrect. Hardware error semantics belong in a higher application layer that has the context to make recovery decisions.

**Why count retries separately from failures?**
The bus tracks `total_transactions`, `total_retries`, `retry_saves`, and `hard_failures` independently. Tolerance without measurement is blindness: a bus that silently retries its way to success looks identical to a healthy one unless the retries are counted. The separation is deliberately analogous to the transmit and receive error counters on a CAN peripheral. `retry_saves` in particular answers a question a pass or fail result cannot, namely how often the retry policy was actually load-bearing. Only reads and ping retry, so `total_retries` and `retry_saves` describe reads; `hard_failures` counts every failed transaction that reached the bus, writes included.

## Protocol layer

**Why does `sts_create_packet` reject instruction `0x00`?**
`0x00` is not a valid STS instruction byte. The status byte in servo responses uses `0x00` to indicate no hardware error, so it has a different semantic role than a command instruction. Allowing it in `sts_create_packet` would let a caller build a packet that is structurally a response rather than a command. Response construction for testing is handled by the `simulate_servo_response` helper in `test_sts_utils.c`.

**Why a sliding-window seeker rather than a strict header check?**
A half-duplex bus can present the receiver with collision fragments or noise before the real response. The parser scans for the `0xFF 0xFF` sync pair, validates the candidate packet at that offset, and advances by one byte on failure rather than discarding the buffer. A false header inside noise therefore costs one byte of progress instead of the whole response. It does not filter a local echo: an echoed ping or read is itself a valid packet, so keeping echo out is the transport's job (see the porting guide).

## Operating modes and target dispatch

The servo supports four operating modes, selected with `STS_SetOperatingMode`. The active mode determines how target commands are interpreted.

| Mode | Behaviour |
| --- | --- |
| `STS_MODE_POSITION` | Absolute position control via the servo's internal PID, within `[0, STS_MAX_POSITION]`. Target speed acts as a travel limit. The servo holds position under load. This is the default after `STS_Servo_Init`. |
| `STS_MODE_SPEED` | Continuous rotation velocity control within `[0, STS_MAX_SPEED]`. Absolute position commands are ignored. Useful for wheel-drive or conveyor applications. |
| `STS_MODE_PWM` | Open-loop voltage control within `[0, STS_MAX_PWM]`. The PID is disabled and the servo behaves as a standard DC motor. |
| `STS_MODE_STEP` | Relative position control within `[0, STS_MAX_STEP]`. Moves a specified number of steps from the current location without tracking an absolute target. |

Direction for speed, PWM, and step modes is supplied explicitly through the `dir` argument of the individual command functions, and is encoded into the relevant register's direction bit.

**Why does `STS_SetTarget` use the sign of its argument to encode direction?**
`STS_SetTarget` is a convenience dispatcher that routes to the correct command for the servo's current mode, so it needs one argument type that can express every mode's target. Speed, PWM, and step all need a direction; position does not. Taking a signed `int32_t` and reading the sign as direction gives a single uniform call for all four modes.

**Why does `STS_SetTarget` reject out-of-range targets instead of clamping them?**
A target outside the active mode's range returns `STS_ERR_INVALID_PARAM` and nothing is sent. This covers targets above the mode's maximum, of any size, and negative targets in position mode. Clamping would return `STS_OK` for a command that was not carried out as requested: in step mode a relative move would be silently shortened, and in speed mode a miscalculated target would become full speed. A caller on production equipment needs to learn that its target was wrong, so the wrapper applies the same rule as `STS_SetTargetPosition`, `STS_SetTargetSpeed`, `STS_SetTargetPWM` and `STS_SetTargetStep`. Before narrowing to the 16-bit register value, the wrapper rejects any magnitude above `UINT16_MAX`, so a large target cannot wrap into a valid one; each per-mode function then enforces its own, smaller limit.

## Configuration

**Why does `STS_SetID` not bundle EEPROM lock and unlock?**
Bundling the lock and unlock sequence inside `STS_SetID` would hide repeated flash writes if the function were called incorrectly in a loop. Keeping the responsibility with the caller makes the flash write cost explicit and prevents accidental EEPROM wear. The intended sequence is unlock, write, lock immediately.

## Known limitations

These are documented rather than resolved, and are listed here so that a reader evaluating the driver does not have to discover them from the source.

- **The command engine is not thread-safe.** Neither `sts_execute_command` (writes) nor `sts_execute_read` (reads and ping) takes a lock: there is no mutex around the shared `tx_buf` and `rx_buf`, and no bus-level locking. This is correct for the current single-threaded bare-metal use. RTOS use would require a per-bus lock in both entry points, held for the whole call so that a read's retries and the bus counters cannot interleave with another task's transaction.
- **The STM32 port holds file-static state.** `s_huart`, the completion flags, and the byte count are file-scope statics, so the port supports one UART instance. Supporting a second bus would require moving that state into a per-bus context.
- **The transaction API is fully blocking.** Every call spins until completion, timeout, or error. There is no asynchronous or queued variant.
- **Sync Write and Bulk Read are unimplemented.** The instruction codes are defined in the register map but the service layer has no corresponding commands, so multi-servo coordinated motion is not yet available.
- **The on-target test suite is tied to the STM32 HAL by its timing calls.** `Hardware_Tests/` calls `HAL_Delay` and `HAL_GetTick` directly for roughly fifty polling and delay operations. Apart from one diagnostic that prints a HAL error code, nothing else in the suite is MCU-specific, so routing timing through the `STS_Delay_ms` and `STS_GetTick_ms` hooks already declared in `sts_ports_stm32.h` would make the suite reusable across ports. Those two functions are currently declared and defined but unused.
