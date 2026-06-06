# Direction-A Seed Replication Summary

Trigger signature: active channel set, active window band, and per-channel sign/amplitude-bin envelope for |theta| > 0.1.

## Cross-Seed Table

| seed | Arm A interior | Arm B interior/support | Arm C interior/support/channels | ddmin clean | ddmin support median | throttle/yaw residual | per-distinct cost |
| ---: | ---: | --- | --- | --- | ---: | --- | --- |
| 0 | 0 | 7 / med 27.0 | 18 / med 6.0 / pitch;pitch,roll;roll | 4/10 | 14.5 | 5/10 | C 11.43; ddmin 98.25 |
| 1 | 0 | 10 / med 25.5 | 12 / med 18.0 / pitch,roll;roll | 2/10 | 10.5 | 2/10 | C inf; ddmin 185.50 |
| 2 | 0 | 11 / med 27.0 | 6 / med 7.5 / pitch;pitch,roll;roll | 4/10 | 9.5 | 2/10 | C 80.00; ddmin 90.25 |

## Criteria

- seed 0: pass=arm_a_zero_interior,arm_c_count_exceeds_arm_b,arm_c_support_4_to_8,arm_c_channels_subset_roll_pitch,ddmin_failure_relative_to_arm_c; fail=none
- seed 1: pass=arm_a_zero_interior,arm_c_count_exceeds_arm_b,arm_c_channels_subset_roll_pitch; fail=arm_c_support_4_to_8,ddmin_failure_relative_to_arm_c
- seed 2: pass=arm_a_zero_interior,arm_c_channels_subset_roll_pitch,ddmin_failure_relative_to_arm_c; fail=arm_c_count_exceeds_arm_b,arm_c_support_4_to_8

## Distinct Costs

| seed | method | total J=5 points | clean triggers | distinct clean triggers | J=5 per distinct clean trigger |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0 | CADET_arm_C | 80 | 18 | 7 | 11.43 |
| 0 | ddmin | 393 | 4 | 4 | 98.25 |
| 1 | CADET_arm_C | 80 | 0 | 0 | inf |
| 1 | ddmin | 371 | 2 | 2 | 185.50 |
| 2 | CADET_arm_C | 80 | 3 | 1 | 80.00 |
| 2 | ddmin | 361 | 4 | 4 | 90.25 |
