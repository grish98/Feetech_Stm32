# Hardware Validation

Validation evidence for the STM32F103 port, including historical campaigns driven by the instrumented stress runner in `Hardware_Tests/` and the current AF open-drain Phase 1 report. Historical and AF_OD campaign totals are kept separate. Remaining evidence gaps are identified below.

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

The updated issue records a bring-up recollection of approximately six all-zero bytes before the drain was introduced. That account does not establish the GPIO-transition mechanism. The earlier implementation used input mode with an internal pull-up between transmissions. The subsequent AF_OD change removes that per-packet GPIO switching; the current configuration and measurements are described below. [Issue #10](https://github.com/grish98/Feetech_Stm32/issues/10) records the investigation history.

## Current AF open-drain port

PA2 now stays in AF open-drain mode for both transmission and reception. Direction changes use the UART TE/RE bits. The removed turnaround flush has not been reintroduced, and no STM32 internal pull-up is configured in this mode.

The [Phase 1 electrical-validation report](https://github.com/grish98/Feetech_Stm32/issues/10#issuecomment-5740164455), updated on 19 September 2026, describes **one STS3215-HS servo, 1 Mbaud UART (8N1), and an external 1.5 kOhm pull-up from PA2/DATA to 3.3 V**. Captures used a Siglent SDS804X HD with a 10x probe setting and a 1.65 V edge trigger. The trigger level is an acquisition setting, not a receiver input threshold.

**The external resistor is required in the tested configuration.** The updated report supersedes the earlier account of successful resistor-free operation: communication failed completely when the external pull-up was disconnected. Isolated-servo loading measurements indicate an approximately 10-11 kOhm idle pull-up to an effective source near 3.0 V. This characterizes idle bias, not the servo's transmitting output circuit. The precise failure mechanism without the external pull-up remains unresolved.

### Functional campaigns

| Campaign | Runs passed | Individual tests passed | Transactions | Retries | Hard failures |
| --- | ---: | ---: | ---: | ---: | ---: |
| Rise-time capture | 200/200 | 6,400/6,400 | 152,140 | 0 | 0 |
| Fall-time capture | 200/200 | 6,400/6,400 | 152,087 | 0 | 0 |
| **Total** | **400/400** | **12,800/12,800** | **304,227** | **0** | **0** |

No nonzero `uart_errors` entries were reported. Issue #10 records the Phase 1 functional non-regression gate as passed, including its no-skip criterion. These AF_OD results are separate from the 390,385 historical transactions above. They support the tested single-servo configuration; they do not establish general reliability across other bus loads or environments.

### Electrical measurements

The report's verified rise-time dataset contains **32,547 measurements** in two populations:

| Population | Samples | Share | 10-90% rise time |
| --- | ---: | ---: | --- |
| Fast | 25,175 | 77.3% | 5.17-52.43 ns |
| Slow | 7,372 | 22.7% | 151.55-197.44 ns |

The overall mean is **45.922 ns**, replacing the earlier preliminary 60-70 ns summary. A later command/response capture associates the slow population with **MCU commands (approximately 195 ns)** and the fast population with **servo replies (approximately 5 ns)**. The two populations are more informative than their combined mean. The reply edges suggest stronger drive during transmission, but the servo's internal output topology remains unconfirmed.

| Measurement | Reported result |
| --- | --- |
| Fall-time samples | 32,487 |
| Mean / maximum fall time | 10.29 ns / 12.22 ns |
| Settled high with external pull-up | 3.11-3.28 V |
| Waveform minimum, mean | 51.7 mV |
| Waveform minimum, range | -81.25 to +118.75 mV |

Waveform minima can include undershoot and do not establish settled-low voltage. The report uses the base STS3215 input limits of 2.0 V minimum high and 0.45 V maximum low as provisional references pending HS-specific confirmation. Servo input limits apply to commands; STM32 input limits must be checked separately for replies.

At 1 Mbaud, the bit period is 1,000 ns. The approximately 195 ns command rise is a 10-90% measurement, not the time to cross the valid-high threshold. A first-order RC model estimates release-to-2.0 V at approximately 84-92 ns for the measured high levels. Those values are calculated, not directly measured; the servo's sampling timing is unconfirmed, so no guaranteed timing margin is claimed. The results support retaining 1.5 kOhm to 3.3 V and 1 Mbaud for subsequent phases.

### Evidence and remaining closure

The [Phase 1 report](https://github.com/grish98/Feetech_Stm32/issues/10#issuecomment-5740164455) includes scope screenshots, calculations, and these exports:

- [Fall-time campaign log](https://github.com/user-attachments/files/32411134/18-09-2026-FallTimes.txt).
- [Rise-time CSV](https://github.com/user-attachments/files/32411171/Rise_Time_C1_19700101_144040.csv).
- [Fall-time CSV](https://github.com/user-attachments/files/32411168/Fall_Time_C1_19700101_124211.csv).
- [Minimum-voltage CSV](https://github.com/user-attachments/files/32411170/Min_C1_19700101_124219.csv).

The report's other link under "Campaign logs" points to a rise-time CSV rather than the named `13-09-2026-RiseTimes.txt` log. The rise-campaign text-log link needs correction or confirmation; it is not treated here as an attached campaign log. These results summarize the maintainer's report, rather than a new independent analysis of its raw exports.

Remaining Phase 1 evidence work is to link the tested firmware revision and complete campaign evidence, including the subsequent command/response capture, and document settled-low voltage and direct receiver-threshold margin. Existing captures may suffice; focused measurements are needed where evidence is missing. A further 200-run campaign is not required solely to document the completed functional gate. If electrical checks are deferred, record that acceptance-scope change explicitly.

Phase 2 receive-state hardening and its campaign gate remain pending, followed by Phase 3 cleanup and its final gate. The phase gates require at least 50 runs each with zero failures, skips, retries, hard communication failures, or unexpected UART error state, expected response lengths/parsing, and move-time distributions within the established baseline band. Full-duplex adapter runtime validation remains in [issue #9](https://github.com/grish98/Feetech_Stm32/issues/9).

Multi-servo operation, deployment cable lengths, sustained high-current motor operation, and supply/environmental variation remain separate qualification work. Response-delay measurement is needed if a specific timing guarantee is relied upon; the current report establishes none.

The legacy scope-marker macros and PA0 initialization have been removed because the AF_OD path no longer switches GPIO mode per packet. The port no longer configures or drives PA0; the historical scope results above are retained. Current receive-timing review findings are described in the [porting guide](porting.md#current-receive-timing-limitations).

## Reproducing a campaign

Flash the firmware target and open an RTT viewer. `AppMain` scans IDs 1 to 15, reports the first servo that answers, enables torque, and runs the stress campaign:

```c
STS_RunStressTest(&servo_1, 200U);   /* iteration count */
```

Campaign totals are printed at the end of the run. Comparing `total_transactions` against `total_retries` and `hard_failures` gives the recorded counter totals reported above.
