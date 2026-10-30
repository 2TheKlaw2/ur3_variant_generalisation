# Lab day checklist (Thursday 5 Nov 2026)

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

## 2. Start the robot (as for the 29 Oct physical V1 run)
1. Laptop Ethernet on the robot network (static 192.168.0.10/24); `ping 192.168.0.192`.
2. Start the real UR3e + RG2 driver and MoveIt **the same way as the 29 Oct run**, then press Play on
   External Control on the pendant. `ROS_DOMAIN_ID` must be **0** (unset) in every robot terminal.
3. **Check the elbow joint limit** on the pendant (Installation → Safety → Joint limits): ______ °.
   The plans keep the elbow within ±160° (max used ≈ 95°); write down what the controller says.
4. Set a low speed slider for the first runs.
5. New terminal: `bash START_LAB.sh` → all OK.

## 3. Calibrate (read-only, ~20 min)
`bash CALIBRATE_PLATES.sh` and follow the prompts. Per variant: freedrive the gripper onto the part in its
pocket, close the RG2 from the pendant, save; lift it and guide it into **its own** fitting (tabs on the
slots, entry depth, no twist, never force), save; put it back, open.
Limits it checks: pickup RMS ≤ 1.5 mm · placement RMS ≤ 0.5 mm · prong mismatch ≤ 1.5 mm · plate yaw diff ≤ 0.5°.
Anything outside → redo that variant with `--resume <session> --variants <i>`.

## 4. Update the simulation (offline, ~10–15 min)
`bash UPDATE_SIM.sh config/calibration/<session>` → registration, scene check, Stage 9, Stage 10 (arc-height
rule), release/retreat, pre-planned RViz plans. Expect 5/5 everywhere; read any FAIL before going on.
Undo: `cp config/scene/scene_registration.before_<session>.yaml config/scene/scene_registration.yaml`.

## 5. Fake hardware first (ROS_DOMAIN_ID=73, separate terminals)
Restart the fake robot + MoveIt so RViz loads the measured plates, then
`python3 scripts/phase9_variants/run_variant_insert_sim.py --variant all --level L2 --full --run --speed 3`
→ 5× functional PASS, no collisions. Do not continue with any variant that fails here.

## 6. Physical, staged (ROS_DOMAIN_ID=0)
1. Load the scene into the real robot's MoveIt (collision objects only, no motion):
   `python3 scripts/phase6_pickup_sim/planning_scene_tools.py --load --physical`
2. Preflight (no motion): `python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1`
3. **First run stops before insertion:**
   `python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1 --until ALIGN_INSERT --execute`
   Type `RUN_<STAGE>` for each stage after checking the real robot. At ALIGN_INSERT **inspect the part above
   its fitting** (centred? upright? tabs on the slots?). Photograph it.
4. If the alignment is good: `... --variant 1 --from INSERT --execute` (insert, rotate, release, retreat, home).
5. Record: assembled and locked? (yes/no), seated? retries, protective stops, photos. The runner logs joints,
   gripper and stage results to `results/physical/full_cycle/V1_<time>/`.
6. Then V2 and V3, then V4 and V5 (same two-step way).

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
