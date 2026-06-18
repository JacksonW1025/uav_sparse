VERDICT: GO

# Energy Headwind Feasibility Probe

Decision branch: `clean_flip_in_legal_envelope`. At least one same-D got_home True->False flip occurs inside 0-12 m/s wind.

## Four checks

- Battery RTL triggered/executed: **True**; no-wind got_home all true=True; cleanup audit clean=True.
- D reached: **True**; min achieved/commanded=1.024.
- Return headwind coupling: **True**; D=100 median return groundspeeds: 0 m/s -> 7.88 m/s, 6 m/s -> 6.48 m/s, 12 m/s -> 3.05 m/s.
- Legal-envelope got_home flip: **True**; D=80: 9 True -> 12 False, D=100: 6 True -> 9 False, D=120: 6 True -> 9 False.
- Boundary sigma: D=100, wind=9, n=5, mean shortfall=0.74 m, sigma=0.071 m, nontrivial=True.

## Fixed Scenario

`BATT_LOW_MAH=220.0`, `BATT_FS_LOW_ACT=2`, `BATT_FS_CRT_ACT=1`, `RTL_ALT=2000`, `WPNAV_SPEED=800`, `AVOID_ENABLE=0`, `FENCE_ENABLE=0`, `SIM_WIND_DIR=270`.
Outbound is east/downwind; RTL return is west/headwind.

## got_home vs wind

| D m | wind 0 | wind 6 | wind 9 | wind 12 |
| ---: | --- | --- | --- | --- |
| 40 | True | n/a | n/a | n/a |
| 80 | True | True | True | False |
| 100 | True | True | False | False |
| 120 | True | True | False | False |

Figure: ![](planc/analysis/energy_headwind_feasibility_got_home_vs_wind.png)

## Runs

| run | D | wind | got_home | shortfall m | final dist m | RTL | clean | return median GS |
| --- | ---: | ---: | --- | ---: | ---: | --- | --- | ---: |
| ehf_D040_W00_r00 | 40 | 0 | True | 0.00 | 0.01 | True | True | 5.98 |
| ehf_D080_W00_r00 | 80 | 0 | True | 0.00 | 0.00 | True | True | 7.55 |
| ehf_D080_W06_r00 | 80 | 6 | True | 0.00 | 0.05 | True | True | 6.40 |
| ehf_D080_W09_r00 | 80 | 9 | True | 0.00 | 0.37 | True | True | 4.77 |
| ehf_D080_W12_r00 | 80 | 12 | False | 10.13 | 25.13 | True | True | 3.05 |
| ehf_D100_W00_r00 | 100 | 0 | True | 0.00 | 0.01 | True | True | 7.88 |
| ehf_D100_W06_r00 | 100 | 6 | True | 0.00 | 0.34 | True | True | 6.48 |
| ehf_D100_W09_r00 | 100 | 9 | False | 0.68 | 15.68 | True | True | 4.78 |
| ehf_D100_W09_r01 | 100 | 9 | False | 0.85 | 15.85 | True | True | 4.78 |
| ehf_D100_W09_r02 | 100 | 9 | False | 0.74 | 15.74 | True | True | 4.78 |
| ehf_D100_W09_r03 | 100 | 9 | False | 0.73 | 15.73 | True | True | 4.78 |
| ehf_D100_W09_r04 | 100 | 9 | False | 0.67 | 15.67 | True | True | 4.78 |
| ehf_D100_W12_r00 | 100 | 12 | False | 30.45 | 45.45 | True | True | 3.05 |
| ehf_D120_W00_r00 | 120 | 0 | True | 0.00 | 0.01 | True | True | 8.01 |
| ehf_D120_W06_r00 | 120 | 6 | True | 0.00 | 7.27 | True | True | 6.51 |
| ehf_D120_W09_r00 | 120 | 9 | False | 20.40 | 35.40 | True | True | 4.78 |
| ehf_D120_W12_r00 | 120 | 12 | False | 50.02 | 65.02 | True | True | 3.05 |

No decisive tag was created.
