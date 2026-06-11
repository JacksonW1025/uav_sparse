# MANTIS PX4 Headroom Boundary Plan

## Goal

Find H* = weakest admissible PX4 headroom condition.

## Headroom Axis

This round uses CA_ROTOR0_CT through CA_ROTOR3_CT scaling as the first headroom axis.

The current known safe point is scale = 0.70. The boundary audit first tries lower scales 0.65, 0.60, 0.55, and 0.50 using only default-parameter gates. If a scale fails, the boundary is bracketed between the last safe scale and the first bad scale, then refined by three bisection points. If all scales down to 0.50 remain safe, H* is 0.50 and the default-safe boundary is reported as not reached.

## H* Admissibility

H* is admissible iff all hold:

- M0@P0@H* is safe under C_recover and C_track.
- Msmall@P0@H* is safe under C_recover and C_track.
- All selected Mstrong@P0@H* are safe under C_recover and C_track.
- No crash, failsafe, landing, or mode-harness failure is caused by H*.
- Nonlinear telemetry is observable.

If H causes P0 strong maneuvers to violate, classify it as BAD_HEADROOM, not a bug.

## Accepted Witness Criteria

An accepted witness still requires:

- Mstrong@P0@H* safe.
- M0@P@H* safe.
- Msmall@P@H* safe.
- Mstrong@P@H* violates C_recover or C_track.
- Nonlinear activation overlaps the violation window.
- Repeat-robust confirmation.

No thresholds may be changed. No new contracts may be added. Saturation alone is not a violation.
