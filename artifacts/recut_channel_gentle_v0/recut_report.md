# Recut Channel/Gentle v0

Scope: EXPLORATORY post-processing only. Single platform: PX4. Single property: post_neutral_xy_velocity / xy_velocity. Confirmation requires new data, especially alt_drift and seed 3. No PX4 or simulator runs were launched; this report reads only archived JSON/CSV under artifacts/.

## Inputs And Gaps

| seed | probe groups.csv | probe groups | ddmin groups.csv | ddmin groups |
| --- | --- | --- | --- | --- |
| 0 | no |  | no |  |
| 1 | yes | 40 | yes | 40 |
| 2 | yes | 40 | yes | 40 |

Data gaps reported without imputation:
- artifacts/direction_a_ddmin_px4_position_seed0_v1/groups.csv
- artifacts/direction_a_px4_position_seed0_v0/groups.csv

Channel purity counts use the archived active_channels fields in the summary/CSV. Where ddmin final active group ids and groups.csv both exist, I cross-checked the group-id-derived channels against the archived channel strings; no mismatches were found for seed1/2. Probe summaries do not include active group ids, so probe channel strings cannot be independently reconstructed from groups.csv without reading theta arrays, which this recut does not do.

## Step 1 - Channel Purity Recut

Definition: EXPLORATORY recut clean prime = active channels subset of {roll,pitch}; this drops support<=8. This criterion is channel-purity-only, on PX4 xy_velocity, and needs alt_drift/seed3 confirmation.

| seed | Arm A pure/interior | Arm B pure/interior | Arm C pure/interior | ddmin RP-only | CADET J5/pure | ddmin J5/RP | ddmin J5/RP e2e | ddmin/CADET | ddmin e2e/CADET |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 0/0 (NA) | 0/7 (0.0%) | 18/18 (100.0%) | 5/10 (50.0%) | 4.444 | 78.6 | 94.6 | 17.68x | 21.28x |
| 1 | 0/0 (NA) | 0/10 (0.0%) | 12/12 (100.0%) | 8/10 (80.0%) | 6.667 | 46.375 | 56.375 | 6.96x | 8.46x |
| 2 | 0/0 (NA) | 0/11 (0.0%) | 6/6 (100.0%) | 8/10 (80.0%) | 13.333 | 45.125 | 55.125 | 3.38x | 4.13x |

EXPLORATORY conclusion, PX4 xy_velocity only, confirm with alt_drift/seed3: the channel axis is stable for Arm C and absent in Arm B under this recut. The ddmin route partially recovers roll/pitch-only outputs, but at higher J=5 cost per output. Channel-pure distinct triggers were not recovered because summary/CSV do not contain enough stable theta/signature information for the support-unbounded recut.

## Step 2 - Gentlest Amplitude Stability

| seed | gentlest max\|theta\| | support | channels | \|channels\| | duration support/\|ch\| | label d | match | rho_mean | rho_std |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 0.40625 | 8 | pitch,roll | 2 | 4 | 4 | yes | -0.0923 | 0.0212 |
| 1 | 0.28125 | 20 | pitch,roll | 2 | 10 | 10 | yes | -0.0484 | 0.0235 |
| 2 | 0.445312 | 6 | pitch | 1 | 6 | 5 | no | -0.0224 | 0.0096 |

| seed | Arm C min | Arm C p25 | Arm C median |
| --- | --- | --- | --- |
| 0 | 0.40625 | 0.46875 | 0.5 |
| 1 | 0.28125 | 0.414062 | 0.617188 |
| 2 | 0.445312 | 0.578125 | 0.75 |

EXPLORATORY conclusion, PX4 xy_velocity only, confirm with alt_drift/seed3: the gentlest max|theta| values have range 0.164062 and sample CV 22.7% across the three seeds. That is not a tight lower-bound band for a title-level gentle claim. Duration cross-check is exact for seed0/seed1 gentlest points but seed2 has support/|channels|=6 vs label d=5; report duration should keep this caveat.

## Step 3 - Duration-Amplitude Tradeoff

| pair | Spearman rho | p-value | n |
| --- | --- | --- | --- |
| max\|theta\| vs duration | -0.316 | 0.0604 | 36 |
| max\|theta\| vs support | -0.3682 | 0.0272 | 36 |

| seed | min duration support/\|ch\| | min label d | min support |
| --- | --- | --- | --- |
| 0 | 3 | 3 | 4 |
| 1 | 6 | 5 | 10 |
| 2 | 6 | 5 | 6 |

EXPLORATORY conclusion, PX4 xy_velocity only, confirm with alt_drift/seed3: lower max|theta| trends negative against duration and is negatively associated with support in the Arm C interior set. The duration relation is marginal by the t approximation, while support is below 0.05; this supports a gentle-vs-sparse tension, but not a definitive universality claim.

## Step 4 - Distinct Clean Reconciliation

| seed | CADET distinct clean support<=8 | ddmin distinct clean support<=8 | ddmin >= CADET | CADET J5/distinct | ddmin J5/distinct |
| --- | --- | --- | --- | --- | --- |
| 0 | 7 | 4 | no | 11.429 | 98.25 |
| 1 | 0 | 2 | yes | inf | 185.5 |
| 2 | 1 | 4 | yes | 80 | 90.25 |

EXPLORATORY wording discipline, PX4 xy_velocity only, confirm with alt_drift/seed3: under the pre-registered support<=8 clean definition, ddmin recovers distinct clean counts >= CADET in 2/3 seeds, including seed1 where CADET=0. Therefore the claim that channel direction is necessary is falsified in this archive. What survives is a channel-purity cost/reliability difference, not necessity.

## Step 5 - Verdict

Q1 verdict: EXPLORATORY, single-platform PX4, single-property xy_velocity; confirmation needs new alt_drift and seed 3 data. The narrative should move to state (iii): product = channel-pure + return-to-neutral triggers, and method contribution = low-cost direct synthesis of channel-pure triggers. The channel axis is stable across seeds (Arm C seed0 18/18, seed1 12/12, seed2 6/6; Arm B seed0 0/7, seed1 0/10, seed2 0/11; ddmin partial recovery seed0 5/10, seed1 8/10, seed2 8/10), while the pre-registered support/sparsity axis is not stable beyond seed0 and the gentlest amplitude floor is not tight enough for a title claim. Delete 'necessary'; the surviving main beam is cost plus bug-class existence.

Q2 verdict: EXPLORATORY, single-platform PX4, single-property xy_velocity; confirmation needs new alt_drift and seed 3 data. Do not claim sparse and gentle on the same trigger: Spearman max|theta| vs duration is -0.316 (p=0.06, marginal) and vs support is -0.368 (p=0.027), so lower amplitude tends to require longer/higher-support triggers. If forced to choose between gentle and sparse, choose gentle only as a secondary exploratory characterization; sparsity is less stable across seeds, and the main paper claim should stay on channel purity rather than saying both are achieved.

Q3 verdict: EXPLORATORY, single-platform PX4, single-property xy_velocity; confirmation needs new alt_drift and seed 3 data. Hardest surviving sentence: On archived Direction-A seeds 0/1/2, CADET's channel-directed Arm C produced roll/pitch-only interior robust xy_velocity violations in 100% of raw Arm C cases (seed0 18/18, seed1 12/12, seed2 6/6) while Arm B produced essentially none (seed0 0/7, seed1 0/10, seed2 0/11); per roll/pitch-only output, ddmin cost seed0 17.68x, seed1 6.96x, seed2 3.38x as many J=5 points as CADET, or seed0 21.28x, seed1 8.46x, seed2 4.13x when the Arm B starting-budget is included.

## Output CSVs

- channel_purity_costs.csv
- gentlest_amplitude.csv
- tradeoff_points.csv
- pre_registered_distinct_clean_reconciliation.csv
- summary_statistics.csv
- groups_file_status.csv
