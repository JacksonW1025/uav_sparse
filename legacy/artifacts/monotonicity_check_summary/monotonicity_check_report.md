# Monotonicity Check Report

Label: exploratory / verification. Platform: PX4. Scenario: px4_position. Seed: 0. J=5.

## Inputs

| source | kind | eval_id | signature | theta | archived rho mean/std |
| --- | --- | --- | --- | --- | --- |
| ArmC_eval164 | known_internal_violation | 164 | channels=pitch,roll;time=w07-w09;signs=pitch:+<bar>roll:+ | `artifacts/direction_a_px4_position_seed0_v0/thetas/C_00164_0556702a8ad39731.npy` | -0.215798 / 0.027387 |
| ArmC_eval174 | known_internal_violation | 174 | channels=pitch;time=w04-w08;signs=pitch:- | `artifacts/direction_a_px4_position_seed0_v0/thetas/C_00174_f7d1cf8cc272f027.npy` | -0.275694 / 0.018787 |
| ArmC_eval182 | known_internal_violation | 182 | channels=pitch,roll;time=w06-w09;signs=pitch:+<bar>roll:+ | `artifacts/direction_a_px4_position_seed0_v0/thetas/C_00182_36c4ad75ca8b372b.npy` | -0.605722 / 0.031542 |
| ArmC_eval199 | known_internal_violation | 199 | channels=pitch,roll;time=w06-w09;signs=pitch:-<bar>roll:+ | `artifacts/direction_a_px4_position_seed0_v0/thetas/C_00199_a98d7690b189a629.npy` | -0.635278 / 0.037294 |
| ArmC_eval234 | known_internal_violation | 234 | channels=roll;time=w04-w08;signs=roll:+ | `artifacts/direction_a_px4_position_seed0_v0/thetas/C_00234_aec082d57a938c50.npy` | -0.282551 / 0.024340 |
| G01_roll_plus_full | saturated_control | NA | G01 C1 roll_plus_full; roll:+; active [0,5] then neutral | `artifacts/contract_grid_summary/thetas/G01_roll_plus_full_b96ead7c8720e928.npy` | NA |
| G02_pitch_plus_full | saturated_control | NA | G02 C1 pitch_plus_full; pitch:+; active [0,5] then neutral | `artifacts/contract_grid_summary/thetas/G02_pitch_plus_full_a08edfccf9989958.npy` | NA |

## Metric Definitions

- Grid C1: absolute [11,13] s peak |v_xy|, threshold 1.0 m/s from contract-grid C1 Brake xy_velocity, rho = threshold - peak, label by rho_mean +/- 2*rho_std.
- Original xy_velocity: existing `compute_robustness(..., post_neutral_xy_velocity, ...)`, tail from `t_neutral_s` to log end, peak |v_xy|, threshold 1.0 m/s from config.

## Per-Theta Results

| source | grid peak [11,13] mean/std | grid rho mean/std | grid label | orig fresh rho mean/std | orig label | archived orig rho | [5,7] | [7,9] | [9,11] | [11,13] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ArmC_eval164 | 0.044 / 0.027 | 0.956 / 0.027 | robust_safe | 0.124 / 0.038 | robust_safe | -0.216 | 0.876 | 0.080 | 0.040 | 0.044 |
| ArmC_eval174 | 0.050 / 0.012 | 0.950 / 0.012 | robust_safe | 0.075 / 0.072 | noise_band | -0.276 | 0.925 | 0.073 | 0.047 | 0.050 |
| ArmC_eval182 | 0.044 / 0.007 | 0.956 / 0.007 | robust_safe | -0.205 / 0.049 | robust_violation | -0.606 | 1.205 | 0.209 | 0.048 | 0.044 |
| ArmC_eval199 | 0.045 / 0.009 | 0.955 / 0.009 | robust_safe | -0.120 / 0.070 | noise_band | -0.635 | 1.120 | 0.238 | 0.052 | 0.045 |
| ArmC_eval234 | 0.053 / 0.028 | 0.947 / 0.028 | robust_safe | 0.037 / 0.069 | noise_band | -0.283 | 0.963 | 0.076 | 0.048 | 0.053 |
| G01_roll_plus_full | 0.305 / 0.025 | 0.695 / 0.025 | robust_safe | -6.150 / 0.145 | robust_violation | NA | 7.150 | 3.924 | 0.362 | 0.305 |
| G02_pitch_plus_full | 0.323 / 0.016 | 0.677 / 0.016 | robust_safe | -6.222 / 0.122 | robust_violation | NA | 7.222 | 3.682 | 0.274 | 0.323 |

## Run-To-Run Check

| source | archived orig rho mean | fresh orig rho mean | fresh - archived |
| --- | --- | --- | --- |
| ArmC_eval164 | -0.216 | 0.124 | 0.340 |
| ArmC_eval174 | -0.276 | 0.075 | 0.350 |
| ArmC_eval182 | -0.606 | -0.205 | 0.401 |
| ArmC_eval199 | -0.635 | -0.120 | 0.515 |
| ArmC_eval234 | -0.283 | 0.037 | 0.319 |

## Archive Hygiene

- `*.ulg` files are removed from this artifact directory after the run; parsed telemetry CSV/parquet and JSON metadata are retained.

## Judgment

口径不一致: internal theta not robust-violating under grid [11,13]: ArmC_eval164, ArmC_eval174, ArmC_eval182, ArmC_eval199, ArmC_eval234; saturated controls violate the original metric: G01_roll_plus_full, G02_pitch_plus_full.
