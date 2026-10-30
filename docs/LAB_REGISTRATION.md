# Lab registration: making the simulation match the real fixtures

The simulation places everything relative to the robot from measured robot poses.

- **Pickup plate:** a fit to 5 measured pickup poses (RMS 2.1 mm).
- **Placement plate:** **one** pose (V1 insertion end) plus an **assumed** yaw (parallel to the pickup plate). Fittings V2–V5 are extrapolated up to 200 mm from that one point.
  - A 0.5° yaw error moves V5 by about 1.7 mm. The radial clearance is 0.25 mm.
  - So the placement plate must be measured before physical V2–V5 trials.
- **Fixture plate thickness:** 12.5 mm, taken from the trolley model. Measure it.
- **Ruler measurements (30 Oct, ±1–2 mm):**
  - The aluminium fixture plate is about 495 × 320 mm. The blue plates' outer ends line up with its ends, and the gap between the blue plates is about 50 mm (the simulation has 51.6 mm).
  - The far edge of each blue plate is 76–77 mm from the far aluminium edge at three places. So both blue plates are parallel to that edge, and to each other, to roughly ±0.3° (about ±1 mm over 200 mm).
  - This supports the simulation's assumption that the plates are parallel (they already share one yaw). At V5, ±0.3° is still about ±1 mm against 0.25 mm of clearance, so the robot-based registration below is still needed before physical V2–V5 trials.
  - `fixture_plate` in the scene now uses these dimensions.
- **Trolley mesh in RViz:** placed approximately, for looks only. The aluminium plate in the trolley mesh does not line up with the real one. The blue plates are what the robot uses. The simulation already has about 51.6 mm between the blue plates, matching the roughly 50 mm in the lab.

## Procedure (about 20 min, staged, low speed)
1. **Check that nothing has moved.** The blue plates are screwed to the slotted plate. If either plate was moved since the recordings, redo the pickup poses too (`v1_pickup_V{i}_geometry`).
2. **Measure the fixture plate.** Measure the total thickness of the aluminium plate at its edge, then run:
   ```bash
   python3 scripts/scene/compute_scene_registration.py --fixture-thickness <metres>
   ```
3. **Register the placement plate with the robot's own grasp.** The part is the probe, so any grasp offset is the same as during insertion.
   1. Run the normal V1 pickup on the robot (or close the RG2 on V1 in its pocket exactly as the program does).
   2. In freedrive, guide V1 into **each** fitting V1…V5 until it sits centred in the bore. The bore centres it: entry depth with tabs on the slot, no twist.
   3. Save each pose:
      ```bash
      bash scripts/save_named_pose.sh v1_place_V1_geometry
      bash scripts/save_named_pose.sh v1_place_V2_geometry
      ...                                   # up to v1_place_V5_geometry
      ```
   4. The same V1 part fits every bore (all discs are Ø27.5), so one grasp is used for all five.
4. **Recompute the registration and validate:**
   ```bash
   python3 scripts/scene/compute_scene_registration.py --fixture-thickness <metres>
   python3 scripts/scene/validate_scene.py
   ```
   `compute_scene_registration.py` uses the `v1_place_V*` poses automatically when at least 2 exist. It fits the placement xy and yaw and reports the residuals: **expect well under 0.5 mm RMS**.
5. **Re-run the simulations on the new registration** (offline Stage 9, then fake hardware). Only then run the physical trials, slowly and staged, with the E-stop ready.

## Why this works
Every motion is planned relative to where the robot itself measured the parts and fittings, using the same grasp it will insert with. So the simulation and the robot share one frame, and an error in the trolley model does not matter.
