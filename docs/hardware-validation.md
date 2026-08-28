# Hardware Validation

Validation evidence for the STM32F103 port. Every figure here comes from on-target campaigns driven by the instrumented stress runner in `Hardware_Tests/`.

## Bench configuration

| Item | Value |
| --- | --- |
| MCU | STM32F103CBTx, USART2 |
| Servo | Feetech STS3215 |
| Bus | 1 Mbaud, 8 data bits, no parity, 1 stop bit |
| Wiring | Direct half-duplex, servo DATA to PA2, STM32 HDSEL mode |
| Reception | DMA with UART IDLE-line framing |
| Reporting | SEGGER RTT |
| Retry policy | `max_retries = 2` during campaigns |

USART1 on the development board is physically damaged, which is why USART2 is used. This was confirmed with an oscilloscope during bring-up.

## What the runner measures

`STS_RunStressTest` repeats the integration suite and aggregates results across runs. Each campaign reports:

- pass, skip, and fail counts per run
- a per-test failure histogram, so a repeated failure is attributable to a specific test rather than to the campaign as a whole
- quiescence-gate activations, counting how often the servo was still moving when a timed move was about to begin
- bus-counter deltas taken from `sts_bus_t`: `total_transactions`, `total_retries`, `retry_saves`, and `hard_failures`

The counters live in the command engine (`sts_execute_command`), so every transaction the service layer issues is counted regardless of which command produced it. A retry increments `total_retries`; a transaction that exhausted all attempts increments `hard_failures`. This distinction matters: a campaign with retries but no hard failures would indicate a bus that is degrading but recovering, which is a different result from a campaign that never needed a retry at all.

## Campaign results

| Campaign | Runs | Skips | Transactions | Retries | Retry saves | Hard failures |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Baseline | 100 | 80 | 67,801 | 0 | 0 | 0 |
| Intermediate | 30 | 4 | 19,269 | 0 | 0 | 0 |
| Final | 200 | 0 | 152,980 | 0 | 0 | 0 |
| Flush removal | 200 | 0 | 150,335 | 0 | 0 | 0 |
| **Total** | **530** | **84** | **390,385** | **0** | **0** | **0** |

The first three campaigns are recorded with their logs in [PR #11](https://github.com/grish98/Feetech_Stm32/pull/11). The fourth is described below.

Two configuration notes apply when reading the total:

1. The baseline and intermediate campaigns ran against an earlier test fixture whose skips are explained in the postmortem below. Their transactions are still valid observations of bus behaviour, because a skipped test still exchanged packets.
2. The first three campaigns ran with the defensive RX flush loop present in the port; the fourth ran without it. The total therefore spans two firmware configurations and should be read as cumulative bus exposure, not as 390,385 transactions of one fixed build.

## Scope of these numbers

These are direct observations from a single bench: one MCU, one servo, one cable, one power supply, one ambient environment. Repeated transactions across that bench are not independent trials. A systematic fault, such as a marginal pull-up or a temperature-dependent timing edge, would affect all transactions or none of them, so a zero-failure result cannot be extrapolated into a general per-transaction reliability figure for the design.

Across these campaigns, the command engine recorded no retries and no terminal transport or protocol failures. These results cover the instrumented paths and do not rule out faults that produce plausible but incorrect data.

Two limits on that coverage are worth stating explicitly. `uart_drain_rx` does not increment any counter, so error-recovery activity in the port is invisible to the totals. Some early returns in `sts_execute_command` also bypass `hard_failures`, including an oversized reported packet length, which returns `STS_ERR_BUF_TOO_SMALL` after the transaction has already been counted. A campaign that reports zero hard failures is therefore evidence about the paths that are counted, not proof that nothing went wrong anywhere.

## Postmortem: intermittent acceleration failure

The acceleration-timing test failed or skipped intermittently for several weeks. The working hypothesis was that EMI or UART corruption was altering servo register state during motion. Instrumentation did not support that explanation.

1. Bus counters recorded 67,801 transactions with zero retries and zero hard failures during the baseline campaign. No recorded UART or protocol failure accounted for the timing anomaly.
2. Corrected skip accounting showed that 80 of 100 baseline runs were skipped, making the anomaly the dominant behaviour rather than an occasional failure.
3. Skip telemetry showed every skipped run began at position `4095`. The fixture homed to `0`, the encoder wrap boundary, and its one-sided settle check could exit while the servo was still moving. The servo then coasted through `0` and stopped at `4095`.
4. Moving the home target to mid-range reduced the skip rate from 80/100 to 4/30.
5. The remaining failures were traced to inherited motion state. `Test_Accel` did not reset `GOAL_SPEED`, and a non-zero inherited value selects constant-speed behaviour. Residual motion also produced false arrival readings.
6. Explicit register resets, mid-range homing, double-read arrival confirmation, and a quiescence gate produced zero skips across 200 final runs.

The quiescence gate found the servo still moving at 160 of 600 gate entries, preventing those timed moves from beginning against residual motion. A smaller start-position-dependent timing offset remains in the final data. It is benign at the current operating point, does not affect pass or fail results, and is documented rather than claimed resolved.

The full investigation is recorded in [issue #8](https://github.com/grish98/Feetech_Stm32/issues/8).

## Retraction: the turnaround transient

The port previously ran a defensive flush loop after each half-duplex turnaround, draining `RXNE`, `ORE`, and `FE` for up to 1 ms before re-arming RX DMA. Its stated justification was that switching PA2 from `AF_PP` to `INPUT` produced a capacitive transient that the UART decoded as a start bit.

That mechanism was tested directly. A GPIO marker on PA0 was pulsed high for the duration of the pin-mode switch, giving the oscilloscope a hardware trigger on the exact transition. No transient was observed at any resolution from 1 us/div down to 50 ns/div, using peak detect, across the full window from the switch to the servo response.

The hypothesised mechanism does not occur on this hardware. The loop was removed, and the flush-removal campaign in the table above completed clean. The specific character of the garbage bytes observed during the original bring-up, on a different and now-destroyed board, was never recorded in the commit history and cannot be independently verified.

A separate question remains open: PA2 currently idles as an input with the internal pull-up, nominally around 40 kOhm. That value has not been justified analytically or tested against a different servo, cable, or bus capacitance. Fitting an external pull-up and moving to AF open-drain full time would remove the per-packet GPIO switching entirely. That work is tracked in [issue #10](https://github.com/grish98/Feetech_Stm32/issues/10), now motivated by simplification and noise margin rather than by a transient that was shown not to exist.

## Reproducing a campaign

Flash the firmware target and open an RTT viewer. `AppMain` scans IDs 1 to 15, reports the first servo that answers, enables torque, and runs the stress campaign:

```c
STS_RunStressTest(&servo_1, 200U);   /* iteration count */
```

Campaign totals are printed at the end of the run. Comparing `total_transactions` against `total_retries` and `hard_failures` gives the recorded counter totals reported above.
