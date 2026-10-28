# Physical lab inputs and data capture checklist

## Verify against the real CAD / workcell

- [ ] Actual fitting centre coordinates (x,y,z) in one fixture frame. **Do not assume 50 mm centre-to-centre** until measured and clearance-checked.
- [ ] Direction of plate in robot base frame; fixture orientation and registered base-to-fixture transform.
- [ ] Clockwise sign when viewed from above and robot rotation-axis convention.
- [ ] V1/V2 top reference height and +5 mm top offsets on V3/V4/V5.
- [ ] Tab-centre radius 28.8 mm and clearance of BOTH tabs throughout the entire groove.
- [ ] Initial angular alignment and actual start/finish angles for each fitting.
- [ ] V5 slope: 10° clockwise while descending 5 mm; validate groove bottom/ledge and end seated position.
- [ ] Fixed transformation from the robot's gripper TCP to the rotating piece's axis and pin centres.
- [ ] Updated collision model for plate/fittings, gripper and carried part.
- [ ] Real UR3e calibration YAML correction file location and which robot it was extracted from.
- [ ] Correct real robot home configuration, verify robot and fixture have not moved.

## Demonstration

- [ ] One V1 kinesthetic demonstration of insertion 5 mm and rotation 10° clockwise (plus separate approach/grasp if needed).
- [ ] Save untouched raw CSV to `demonstrations/raw/`; processed copy to `demonstrations/processed/`.
- [ ] Record phase boundary, initial TCP orientation, fixture registration and gripper setting.

## Evaluation

- [ ] Test synthetic sample first, then dry-run fake hardware/MoveIt plan-only; only then supervised approved-speed physical trial.
- [ ] Measure success, path deviation, angular error, endpoint pose error, timings, setup/re-demonstration effort and failure reason.
- [ ] Keep previous experiments in `../ur3_lfd/` intact.
