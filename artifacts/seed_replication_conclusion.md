# Direction-A Seed Replication Conclusion

## Cross-Seed Table

| seed | Arm A interior | Arm B interior / support median | Arm C interior / support median / channels | ddmin clean | ddmin support median | throttle/yaw residual | per-distinct clean cost |
| ---: | ---: | --- | --- | --- | ---: | --- | --- |
| 0 | 0 | 7 / 27.0 | 18 / 6.0 / pitch;pitch,roll;roll | 4/10 | 14.5 | 5/10 | C 11.43; ddmin 98.25 |
| 1 | 0 | 10 / 25.5 | 12 / 18.0 / pitch,roll;roll | 2/10 | 10.5 | 2/10 | C inf; ddmin 185.50 |
| 2 | 0 | 11 / 27.0 | 6 / 7.5 / pitch;pitch,roll;roll | 4/10 | 9.5 | 2/10 | C 80.00; ddmin 90.25 |

Trigger signatures use the pre-registered discrete fingerprint:
active channel set, active window band, and per-channel sign/amplitude-bin
envelope for `|theta| > 0.1`.

## Pre-Registered Criteria Check

- Arm A: replicated on all seeds. Seeds 0/1/2 all have 0 interior robust violations.
- Arm C channels: replicated on all seeds. Arm C interior robust violations use only subsets of `{roll,pitch}`.
- Arm C count advantage over Arm B: replicated on seeds 0 and 1, not seed 2. Seed 2 has Arm C 6 versus Arm B 11.
- Arm C clean support in the 4-8 range: replicated on seed 0 only. Seed 1 has support 10-20, median 18. Seed 2 has median 7.5 but support reaches 18.
- ddmin failure: seed 0 and seed 2 support the failure claim under the distinct-trigger cost accounting. Seed 1 still shows poor ddmin yield and high cost, but the pre-registered Arm C clean-output comparison is not established because Arm C produced 0 distinct clean triggers under the fixed support <=8 definition.

## One-Line Judgment

The spine is only partially cross-seed stable: thin-target/random-miss and relevant-channel directionality survive, but the strong seed-0 claim that Arm C reliably yields many 4-8-support clean triggers does not survive seeds 1 and 2 unchanged.
