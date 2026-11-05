# Temporal scaling: fixed vs CAD-scaled timing (L2, 5 Nov plans)

Offline preflight of the physical runner (`run_full_cycle_physical.py --level L2 --temporal fixed|cad`,
plans `stage10_depthfix`, registration 537b2d8c). No robot motion. The path is identical in both conditions;
only the time stamps of INSERT/ROTATE change (tau = max(1, s_i / s_V1), tab-path length, `temporal_scaling.py`).
Executed duration = after the physical 0.14 rad/s per-joint guard and the 1.5x INSERT minimum slowdown.

| Variant | INSERT fixed | INSERT cad | ROTATE fixed | ROTATE cad | What decides the INSERT time |
|---|---|---|---|---|---|
| V1 | 19.5 s | 19.5 s | 20.1 s | 20.1 s | tau = 1 (reference) |
| V2 | 19.5 s | 19.5 s | 19.8 s | 19.8 s | tau = 1 (same depth as V1) |
| V3 | 19.5 s | **43.2 s** | 19.8 s | 19.8 s | CAD timing (deeper bore, longer tab path) |
| V4 | 57.9 s | 57.9 s | 17.0 s | 17.0 s | speed guard (fixed needs 4.45x, cad 1.60x) |
| V5 | 124.3 s | 124.3 s | 17.0 s | 17.0 s | speed guard (fixed needs 9.56x, cad 1.67x) |

Findings
- Only V3 executes differently: CAD timing makes its INSERT 2.2x slower (planned peak joint speed 0.040 -> 0.018 rad/s).
- For V4 and V5 the robot's speed guard already stretches the fixed-timing INSERT to the same duration that
  CAD timing requests, so the executed motion is identical. CAD timing reaches that duration by design
  (requested peak 0.22-0.23 rad/s, guard factor 1.6-1.7x) instead of by the safety limit (guard factor 4.5-9.6x).
- V1/V2 are unaffected (tau = 1).
- Physical evidence (5 Nov) used fixed timing only. CAD timing is evaluated here kinematically; the one
  physical test that would show an effect is V3 (e.g. CAD or L1 level with `--temporal cad`, where fixed timing
  made contact: 41.6 N / 39.2 N).
