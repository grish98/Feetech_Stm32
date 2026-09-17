# Hardware Validation

Validation evidence for the STM32F103 port, including historical campaigns driven by the instrumented stress runner in `Hardware_Tests/` and preliminary observations from the current AF open-drain bench tests. Pending logs and scope captures are identified separately.

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

The four tabulated campaigns below used the earlier per-packet AF_PP/input GPIO switching. The current port keeps PA2 in AF_OD, with UART TE/RE direction switching and no STM32 internal pull-up configured. Its separately reported test results are recorded under [Current AF open-drain port](#current-af-open-drain-port).

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

`uart_drain_rx` has no independent counter, but calls from `STM32_UART_Receive` return a transport error to the command engine, where retries or exhausted attempts are counted. Explicit flush callbacks also run before retries and after unexpected response lengths. Some early returns in `sts_execute_command` also bypass `hard_failures`, including an oversized reported packet length, which returns `STS_ERR_BUF_TOO_SMALL` after the transaction has already been counted. A campaign that reports zero hard failures is therefore evidence about the paths that are counted, not proof that nothing went wrong anywhere.

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

Those captures did not support the hypothesised mechanism on the tested hardware. The loop was removed in commit `315f279`, and the flush-removal campaign in the table above completed clean: 200 runs and 150,335 transactions, with no skips, retries, or hard failures. This removed the timed per-turnaround flush, not the error-recovery drain or retry flush callback. The specific character of the garbage bytes observed during the original bring-up, on a different and now-destroyed board, was never recorded in the commit history and cannot be independently verified.

The earlier implementation used input mode with an internal pull-up between transmissions. The subsequent AF_OD change removes that per-packet GPIO switching; the current configuration and preliminary measurements are described below. [Issue #10](https://github.com/grish98/Feetech_Stm32/issues/10) records the investigation history.

## Current AF open-drain port

PA2 now stays in AF open-drain mode for both transmission and reception. Direction changes use the UART TE/RE bits. The removed turnaround flush has not been reintroduced, and no STM32 internal pull-up is configured in this mode.

An external 1.5 kOhm pull-up was initially fitted because the servo was thought not to provide a pull-up. Subsequent bench operation without the added resistor showed it was unnecessary for communication on this setup. This is consistent with a pull-up already being present on the servo side; its value and circuit topology have not been independently characterised here.

The following preliminary results were reported by the maintainer, with the external **1.5 kOhm pull-up fitted**:

| Observation | Reported result |
| --- | --- |
| Hardware-test loops | 200 |
| Hardware-test failures | 0 |
| Mean rise time | Approximately 60-70 ns |
| Worst observed rise time | Approximately 200 ns |

The 200-loop result and these rise times do not describe the configuration without the external resistor. Operation without it was reported separately, without a quantified campaign or rise-time dataset. The new campaign is not included in the historical transaction totals above: its exact transaction count, skips, retries, retry saves, and hard failures await the full log.

### Data to attach

- Full stress-run output and the corresponding firmware revision/configuration.
- Scope captures and rise-time data, including measurement thresholds, sample count, probe setup, and pull-up supply voltage.
- Wiring/cable details and any separate measurements with the external pull-up removed.

The legacy scope-marker macros and PA0 initialization have been removed because the AF_OD path no longer switches GPIO mode per packet. The port no longer configures or drives PA0; the historical scope results above are retained. Current receive-timing review findings are described in the [porting guide](porting.md#current-receive-timing-limitations).

## Reproducing a campaign

Flash the firmware target and open an RTT viewer. `AppMain` scans IDs 1 to 15, reports the first servo that answers, enables torque, and runs the stress campaign:

```c
STS_RunStressTest(&servo_1, 200U);   /* iteration count */
```

Campaign totals are printed at the end of the run. Comparing `total_transactions` against `total_retries` and `hard_failures` gives the recorded counter totals reported above.
