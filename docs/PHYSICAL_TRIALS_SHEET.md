# Physical trials sheet (L2, staged)

One row per trial in `results/physical/trials_<date>.csv`. Copy the header from
`results/physical/trials_template.csv`; never edit the template itself. **This sheet records PHYSICAL results only;
simulation results are reported separately.** Fill a row even when a trial stops early. Leave a cell **empty when you
did not observe it**. Do not write `0` or `n` for something you did not check.

| Column | Values | How to fill it |
|---|---|---|
| `date` | `YYYY-MM-DD` | Lab day |
| `variant` | `V1`…`V5` | One variant per run |
| `trial` | 1, 2, … | Count per variant, including stopped trials |
| `calibration_session` | e.g. `20261103_0930` | The `config/calibration/<session>` that `UPDATE_SIM.sh` applied |
| `level` | `L2` | Tuesday's runner executes the L2 plans only (no L0/L1 yet) |
| `align_insert_centred` | y / n | At the `--until ALIGN_INSERT` stop: part centred over the bore |
| `align_insert_upright` | y / n | Same stop, by eye. This is the **part's** look. The tool tilt meter shows TOOL tilt, not part tilt |
| `align_insert_tabs_on_slots` | y / n | Same stop: tabs over the fitting slots |
| `inserted` | y / n | Part reached insertion depth (end of INSERT) |
| `rotated_locked` | y / n | ROTATE completed and the bayonet is locked (twist by hand gently: it does not come out) |
| `seated` | y / n | Part flat on its seat after RELEASE (no gap or tilt visible) |
| `outcome` | `ASSEMBLED` / `NOT_ASSEMBLED` / `STOPPED` | `ASSEMBLED` only if inserted, rotated/locked and seated are all `y` |
| `stop_reason` | text | Runner's `STOPPED:` line, or the operator's reason; empty if none |
| `retries` | integer | Re-runs of any stage within this trial |
| `protective_stops` | integer | Count shown on the pendant during this trial |
| `max_abs_force_N` | number or empty | Largest `max_abs_force_N` over the trial's stages in the run's `wrench_summary.json`; **empty if `available` is false** |
| `max_abs_torque_Nm` | number or empty | Same for `max_abs_torque_Nm` |
| `notes` | text | Include the run log folder `results/physical/full_cycle/V<i>_<time>/` |
| `photo_ids` | e.g. `IMG_0412;IMG_0413` | Separate IDs with `;`. Include the ALIGN_INSERT photo |

## Reading the force columns

- The wrench is **recorded only**. The runner does no contact detection, no force-controlled insertion and no
  force-triggered stopping. A high value did not stop anything; the operator and the existing gates did.
- The values are the UR's built-in wrench estimate, **raw and not zeroed**. They include sensor bias and the weight of
  the RG2 and part. Compare a stage against its own start (`max_abs_force_change_from_stage_start_N` in
  `wrench_summary.json`) or across trials, not against zero.
- The UR driver rotates the values into the pendant's TCP frame (`tool0_controller`). The `frame_id` in the log is the
  broadcaster's label (`tool0` in ur_onrobot).
- `stale: true` or a large `max_gap_s` means part of the stage had no fresh data. Say so in `notes`.

## Pickup setup-consistency runs

`scripts/lab/pickup_consistency.py` writes its own folder (`results/physical/pickup_consistency/<time>/`). Its
spread is **setup consistency only**. It does not measure part-to-gripper repeatability or slip, so do not enter
it on this sheet as a trial.
