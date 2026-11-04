# Lab day checklist (Tuesday 3 Nov 2026)

Keep this page open. Steps run in order; **stop at any FAIL** and fix it before going on. Nothing on this
page runs the robot without an explicit typed confirmation at every stage. Keep the pendant E-stop in reach.

## 0. Bring
- Both new printed plates (Basea + PlaceAssem, 143 mm, holes at 33/83/133 mm) and parts V1–V5
- M5 bolts + T-nuts for the 6 mm slots, hex key; calipers; ruler
- Laptop with the repo at `static-variant-scene` (pushed, up to date), charger, Ethernet cable

## 1. Mount the fixture (robot off or in protective stop)
1. Bolt both plates to the aluminium plate: **prongs pushed against the far edge**, every hole over a slot.
2. Measure the **aluminium plate thickness** at its edge with calipers: ______ mm (sim assumes 12.5).
3. Parts V1–V5 in their pockets; fittings V1–V5 seated.
4. Photograph the setup (top view + side view).

## 2. Start the robot (27–29 Oct method)
1. Laptop Ethernet on the robot network (static 192.168.0.10/24); `ping 192.168.0.192`.
2. Terminal A: `bash scripts/lab/start_robot.sh` (UR3e + RG2 driver, robot calibration `~/ur3e_calibration.yaml`),
   then press **Play** on the pendant's External Control program.
   Check the robot calibration matches the one the simulation uses: `config/ur3e_lab_calibration.yaml` should hold the
   same kinematics as `~/ur3e_calibration.yaml`.
3. Terminal B: MoveIt for the real robot (only needed for the physical runner in step 6, not for calibration).
4. **Check the elbow joint limit** on the pendant (Installation → Safety → Joint limits): ______ °.
   The plans keep the elbow within ±160° (max used ≈ 95°).
5. Low speed slider (it limits commanded motion only, **not** how fast you move the arm by hand in freedrive:
   move slowly). Terminal C: `bash START_LAB.sh` → all OK.

**Freedrive and gripper while the driver runs** (arm stationary when switching):
- `bash scripts/lab/freedrive.sh`: type FREEDRIVE; guide the arm by hand; **Ctrl+C** ends freedrive and switches the arm
  back to `scaled_joint_trajectory_controller` (it prints the controller states: check it says active).
- `bash scripts/lab/rg2.sh open` / `bash scripts/lab/rg2.sh grip` (works during freedrive; prints the measured width.
  On 27 Oct an 8.5 mm command read ≈ 10.1 mm on the part = contact with the grip block).

## 2b. Grip test session (before calibrating, ~45 min)
Terminals: A driver, C `bash scripts/lab/freedrive.sh`, D `python3 scripts/lab/tilt_meter.py`, E `rg2.sh`.
1. **Pre-open close to the part:** `rg2.sh 12` (≈1 mm each side of the ~10 mm grip block: easier to line up
   than 40 mm). Freedrive down over V1, tilt meter ≤ 0.5°.
2. **Find the grip width:** `rg2.sh grip` (8.2 mm). Note the printed width (on the part ≈ 10.1 mm; empty ≈ 8.2).
   Also try `rg2.sh 9` and `rg2.sh 7` (open in between): whether a smaller command squeezes harder depends on the
   driver, so check by feel which holds firmly without marking or bending the print. Width used: ______ mm, reads ______ mm on the part.
3. **Force:** check if it can be set: `ros2 control list_hardware_interfaces | grep -i -e finger -e force`,
   `ros2 param list | grep -i force`. Settable → start 20–25 N, up to 40 N if it slips. Fixed → just note it.
4. **Lift-and-look by hand** (×3): grip, lift ~30 mm, the part must hang straight and the plate must not lift.
5. **Freedrive carry:** move it around by hand, over to its fitting and down to entry depth (2–3 mm in, no
   twist), back. Does it stay straight in the jaws? Repeat for V5 (tallest, helical).
6. **Powered carry (robot moves, not freedrive):** part gripped in its pocket, **Ctrl+C freedrive** (arm back on
   `scaled_joint_trajectory_controller`), then
   `python3 scripts/lab/hold_test.py --execute`
   It moves from where the arm is: LIFT 80 mm straight up → CARRY ±50 mm sideways → TWIST wrist ±20° → LOWER back
   into the pocket, each typed (`RUN_LIFT` …), 0.14 rad/s (same as the runner); after each it asks if the part is
   still straight. Then once faster: `... --execute --speed 0.3`. Log: `results/physical/hold_test/<time>/`.
   Preview anytime without ROS: `python3 scripts/lab/hold_test.py`.
If the part tilts or slips at any step: change the grip (width/force/where the fingers sit) before calibrating.
Optional, read-only: `python3 scripts/lab/pickup_consistency.py` records TCP, TOOL tilt and RG2 width at the closed
checkpoint over 5 hand pickups (setup consistency only, not part slip). Keep `python3 scripts/lab/tilt_meter.py` open
in a spare terminal while freedriving: it shows the TOOL tilt (not the part's) live. See docs/PRELAB_TOOLS.md.

## 3. Calibrate (read-only, ~20 min)
Terminal C: `bash scripts/lab/freedrive.sh` (keep it running). Terminal D: `bash CALIBRATE_PLATES.sh` and follow the prompts;
close/open the gripper with `bash scripts/lab/rg2.sh grip|open` in Terminal E. When done, Ctrl+C in Terminal C
(arm back on `scaled_joint_trajectory_controller`). Per variant: freedrive the gripper onto the part in its
pocket, close the RG2 with `rg2.sh grip`, save; lift it and guide it into **its own** fitting: tabs on the slots,
**lower only to the entry position; if it catches or needs pressure, stop** (never push with the arm), save; put it back, `rg2.sh open`.
Limits it checks: pickup RMS ≤ 1.5 mm · placement RMS ≤ 0.5 mm · prong mismatch ≤ 1.5 mm · plate yaw diff ≤ 0.5°.
Anything outside → redo that variant with `--resume <session> --variants <i>`. A failed save asks retry/skip/quit.
After each save it prints the TOOL tilt of that pose; above 1.0° it recommends and offers a redo (a warning, not a limit).
- PICKUP pose over 1°: redo it (open the RG2, straighten the tool with tilt_meter.py ≤ 0.5°, grip, save).
- PLACE pose over 1°: answer **N**. At the fitting the tool tilt just follows the pickup grasp; "straightening"
  there would push the part sideways. Only worry if PLACE and PICKUP tilt differ by > ~0.5° (part moved in the jaws).
**PASS here = the poses fit the plate model; it does not prove insertion accuracy** (clearance 0.30–0.35 mm):
the above-entry inspection in step 6 still decides.

## 4. Update the simulation (offline, ~10–15 min)
`bash UPDATE_SIM.sh config/calibration/<session>` → registration, scene check, Stage 9, Stage 10 (arc-height
rule), release/retreat, pre-planned RViz plans. Expect 5/5 everywhere; read any FAIL before going on.
Undo: `cp config/scene/scene_registration.before_<session>.yaml config/scene/scene_registration.yaml`.

## 5. Fake hardware first (ROS_DOMAIN_ID=73, separate terminals)
Restart the fake robot + MoveIt so RViz loads the measured plates, then
`python3 scripts/phase9_variants/run_variant_insert_sim.py --variant all --level L2 --full --run --speed 3`
→ 5× functional PASS, no collisions. Do not continue with any variant that fails here.

## 6. Physical, staged (ROS_DOMAIN_ID=0)
0. Freedrive must be OFF and `scaled_joint_trajectory_controller` active (`bash START_LAB.sh` checks it).
1. Load the scene into the real robot's MoveIt (collision objects only, no motion):
   `python3 scripts/phase6_pickup_sim/planning_scene_tools.py --load --physical`
2. Preflight (no motion): `python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1`
3. **One run per trial** (`--from` now refuses to start unless the part is attached in MoveIt and held, but one
   continuous run is still simpler):
   `python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1 --execute`
   Type `RUN_<STAGE>` for each stage after checking the real robot.
   - After GRIP: **look that the part is in the jaws** before `RUN_LIFT`. GRIP now also stops the run unless the RG2
     reads 9.0–11.5 mm (fingers stopped on the part; empty ≈ 8.2 mm). If your grip test reads outside that band on
     the part, pass `--grip-band <min> <max>` (mm).
   - At ALIGN_INSERT: **inspect the part above its fitting** (centred? upright? tabs on the slots?). Photograph it.
     Type `RUN_INSERT` only if it is good; otherwise Ctrl+C (the arm stays above the fitting).
4. Before the first run, check the wrench topic once (missing = logged as unavailable, run unaffected):
   `ros2 topic info -v /force_torque_sensor_broadcaster/wrench` and `ros2 topic hz /force_torque_sensor_broadcaster/wrench`
5. Record one row per trial on the sheet in docs/PHYSICAL_TRIALS_SHEET.md (assembled and locked? seated? retries,
   protective stops, max |F| / |T| from the run's wrench_summary.json, photos). The runner logs joints,
   gripper and stage results to `results/physical/full_cycle/V1_<time>/`.
6. Then V2 and V3, then V4 and V5 (same way).
7. Plan: **pilot** = one successful run per variant (~1 h); then, if time allows, **5 runs per variant** (~3 h).
8. **Comparison conditions** (after the L2 pilot works), same command plus `--level`:
   - `--level CAD` on V1–V5: CAD-only baseline (same CAD path, generic timing, no demo) vs L2 (SQ3).
   - `--level L1` on V1–V3: CDMP goal scaled to CAD; V4/V5 are refused (they collide in simulation).
   - `--level L0` on V1 only: demo replay reference; V2–V5 are refused.
   The runner refuses any variant+level that does not assemble and seat in simulation. Watch each new level once on
   fake hardware first (`run_variant_insert_sim.py --variant all --level CAD --full --run --speed 3`).
   Logs: `results/physical/full_cycle/V<i>_<level>_<time>/`; note the level on the trials sheet.
   Between runs: robot at HOME, take the part out of the fitting by hand and put it back in its pocket.

Speeds: every stage is slowed uniformly to ≤ 0.14 rad/s per joint (the pilot guard); the path is unchanged.
A full V1 cycle is ≈ 3.5 min of arm motion plus reviews; V5's helical insertion alone ≈ 2.2 min.

## Stop rules
- Any FAIL in START_LAB / calibration limits / UPDATE_SIM / fake hardware → stop and fix, do not skip.
- Live start mismatch, MoveIt invalid state, controller error, gripper width wrong → the runner stops itself;
  inspect before anything else. Ctrl+C cancels the active goal (not an E-stop).
- Part not centred at ALIGN_INSERT, any unexpected contact or sound → stop, do not insert.
- Force mode is a separate, later experiment; not part of this checklist.

## Report separately
Physical outcomes (this page) are reported separately from the kinematic simulation and fake-hardware results.
