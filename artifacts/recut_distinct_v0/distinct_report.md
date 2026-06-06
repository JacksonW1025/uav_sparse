# Distinct Recut v0

Scope: EXPLORATORY post-processing only. Single platform: PX4. Single property: xy_velocity / post_neutral_xy_velocity. Confirmation requires alt_drift and seed3. No simulation was run; this recut reads archived CSV/JSON and existing theta npy files only.

## Inputs And Data Gaps

| seed | probe groups.csv | ddmin groups.csv | mapping used | groups |
| ---: | --- | --- | --- | ---: |
| 0 | no | no | reused seed1/seed2 row-identical D=40 frozen mapping | 40 |
| 1 | yes | yes | artifacts/direction_a_px4_position_seed1_v0/groups.csv | 40 |
| 2 | yes | yes | artifacts/direction_a_px4_position_seed2_v0/groups.csv | 40 |

Seed1 and seed2 probe/ddmin groups.csv files were checked row-for-row identical across the frozen D=40 configuration. Seed0 lacks groups.csv in the archived probe/ddmin directories, so the seed1/seed2 mapping is reused for seed0 because the parameterization is frozen; this is not imputation of trigger data.
Checked groups files: artifacts/direction_a_px4_position_seed1_v0/groups.csv, artifacts/direction_a_ddmin_px4_position_seed1_v0/groups.csv, artifacts/direction_a_px4_position_seed2_v0/groups.csv, artifacts/direction_a_ddmin_px4_position_seed2_v0/groups.csv.

Signature definitions:
- Main CADET/ddmin distinct signature: sorted roll/pitch channel set + active window band + per-channel sign; amplitude and bisection iter are ignored.
- CADET main signatures parse `env####_deg###_w##_d##` labels, with `w` and `d` defining the band.
- ddmin signatures map final active group ids to (window, channel), read signs from existing final theta npy, and use the min/max active window band.
- Cross-check fingerprint: previous pre-registered theta fingerprint with sign/amplitude bins and active window band, but without the support<=8 filter unless stated.

Data gaps or validation issues: no missing labels or theta files; no ddmin group/channel mismatches.

## Step 1 - CADET Arm C Distinct Channel-Pure Maneuvers

| seed | raw channel-pure Arm C | D_C main | raw/D_C | prereg distinct no support | prereg support<=8 | support<=8 expected | main signatures |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | 18 | 7 | 2.571 | 7 | 7 | 7 | channels=pitch,roll;time=w06-w09;signs=pitch:+|roll:+<br>channels=pitch,roll;time=w06-w09;signs=pitch:-|roll:+<br>channels=pitch,roll;time=w07-w09;signs=pitch:+|roll:+<br>channels=pitch;time=w04-w08;signs=pitch:-<br>channels=pitch;time=w06-w09;signs=pitch:+<br>channels=roll;time=w04-w08;signs=roll:+<br>channels=roll;time=w06-w09;signs=roll:- |
| 1 | 12 | 4 | 3 | 5 | 0 | 0 | channels=pitch,roll;time=w00-w07;signs=pitch:+|roll:+<br>channels=pitch,roll;time=w00-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w04-w08;signs=pitch:-|roll:+<br>channels=roll;time=w00-w09;signs=roll:- |
| 2 | 6 | 3 | 2 | 3 | 1 | 1 | channels=pitch,roll;time=w00-w07;signs=pitch:-|roll:-<br>channels=pitch;time=w04-w08;signs=pitch:+<br>channels=roll;time=w00-w07;signs=roll:- |

## Step 2 - ddmin Distinct Channel-Pure Maneuvers

| seed | raw is_roll_pitch_only | D_dd main | raw/D_dd | total ddmin J5 | main signatures |
| ---: | ---: | ---: | ---: | ---: | --- |
| 0 | 5 | 5 | 1 | 393 | channels=pitch,roll;time=w03-w09;signs=pitch:+|roll:-<br>channels=pitch,roll;time=w05-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w06-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w07-w09;signs=pitch:+|roll:+<br>channels=roll;time=w06-w09;signs=roll:+ |
| 1 | 8 | 7 | 1.143 | 371 | channels=pitch,roll;time=w01-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w02-w09;signs=pitch:+|roll:+<br>channels=pitch,roll;time=w02-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w03-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w04-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w05-w09;signs=pitch:-|roll:-<br>channels=pitch;time=w04-w09;signs=pitch:+ |
| 2 | 8 | 8 | 1 | 361 | channels=pitch,roll;time=w01-w09;signs=pitch:+|roll:-<br>channels=pitch,roll;time=w02-w09;signs=pitch:+|roll:+<br>channels=pitch,roll;time=w02-w09;signs=pitch:+|roll:-<br>channels=pitch,roll;time=w03-w09;signs=pitch:-|roll:-<br>channels=pitch,roll;time=w04-w09;signs=pitch:+|roll:+<br>channels=pitch,roll;time=w04-w09;signs=pitch:+|roll:-<br>channels=pitch,roll;time=w05-w09;signs=pitch:+|roll:+<br>channels=roll;time=w05-w09;signs=roll:+ |

## Step 3 - End-To-End Cost Per Distinct Maneuver

| seed | CADET raw cost | CADET distinct cost | CADET raw inflation | ddmin raw direct/e2e | ddmin distinct direct/e2e | ddmin raw inflation | ddmin/CADET direct | ddmin/CADET e2e |
| ---: | ---: | ---: | ---: | --- | --- | ---: | ---: | ---: |
| 0 | 4.444 | 11.429 | 2.571x | 78.6 / 94.6 | 78.6 / 94.6 | 1x | 6.877x | 8.277x |
| 1 | 6.667 | 20 | 3x | 46.375 / 56.375 | 53 / 64.429 | 1.143x | 2.65x | 3.221x |
| 2 | 13.333 | 26.667 | 2x | 45.125 / 55.125 | 45.125 / 55.125 | 1x | 1.692x | 2.067x |

Raw denominator inflation is visible mostly on CADET: seed0 18 raw points collapse to 7 maneuvers, seed1 12 to 4, and seed2 6 to 3. ddmin is less inflated under this band+sign signature: 5 to 5, 8 to 7, and 8 to 8.

## Step 4 - Cross-Checks

| seed | prereg no-support >= support<=8 | no-support | support<=8 | expected support<=8 | channel-pure D_C/D_dd | support<=8 D_C/D_dd |
| ---: | --- | ---: | ---: | ---: | --- | --- |
| 0 | OK | 7 | 7 | 7 | 7 / 5 | 7 / 4 |
| 1 | OK | 5 | 0 | 0 | 4 / 7 | 0 / 2 |
| 2 | OK | 3 | 1 | 1 | 3 / 8 | 1 / 4 |

## Step 5 - Verdict

Step 5 判决：B) 脊柱仅成本占优、数量不占优。EXPLORATORY、PX4、xy_velocity；确证需 alt_drift/seed3。distinct 通道纯口径下 CADET 不是 3/3 数量占优，只能主张更便宜，不能主张更多/更好。
数量：seed0 D_C/D_dd=7/5, seed1 D_C/D_dd=4/7, seed2 D_C/D_dd=3/8；CADET 只在 seed0 >= ddmin，seed1/seed2 翻车，因此通道纯 + 数量这部分脊柱不活。
成本：端到端 ddmin/CADET = seed0 8.277x, seed1 3.221x, seed2 2.067x；直接 ddmin/CADET = seed0 6.877x, seed1 2.65x, seed2 1.692x。CADET 每 distinct 机动成本 3/3 低于 ddmin e2e，但倍数从 seed0 到 seed2 衰减到约 2.067x，措辞应收缩为成本优势。

All conclusions are EXPLORATORY, PX4-only, xy_velocity-only, and require alt_drift/seed3 confirmation.