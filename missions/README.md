# Working missions

Run from the repository root with `scrimmage run missions/<file> --headless`.
Replay the printed run directory with `scrimmage replay runs/runNNN`.
These eleven XML files use the compiled built-ins. They are examples and regression
inputs, not a copy of every upstream demo.

| Mission | What to try or verify |
| --- | --- |
| `straight-no-gui.xml` | Opposing aircraft, collision, removal, and team scores. |
| `fixed-wing-6dof.xml` | Two aerodynamic aircraft: level eastbound and tilted northbound with wind. A reference fixture, not a trimmed flight demonstration. |
| `networks-local-global.xml` | NoisyState over LocalNetwork; Boundary over GlobalNetwork turns both aircraft; a third aircraft hits the ground. Set `boundary_control:=false` or `ground_team:=1` to see the difference. |
| `noisy-state.xml` | Independent own-state noise and belief feedback. Not sample-for-sample C++ RNG parity. |
| `waypoints-aircraft.xml` | Two aircraft follow a looping route; a shared GlobalNetwork replacement at 12 s changes both goals. Set `update_at:=100` to suppress that update. |
| `waypoints-point-agents.xml` | Two SingleIntegrator agents receive a replacement at 3 s and stop at the final point. Set `update_at:=100` to retain the original route. |
| `test_missions/straight_cpu.xml` | Longer deterministic aircraft reference run. |
| `test_missions/straight_cpu_mul.xml` | Preserves upstream's `motion_multipler` typo; this does **not** enable substeps. |
| `test_missions/straight_cpu_threaded.xml` | Selects workers through the XML setting. |
| `verification/aircraft-substeps-spawning.xml` | Actual substeps, update rates, scheduled spawning, and randomized state. Known Ubuntu spawn RNG mismatch. |
| `verification/noisy-state-bias.xml` | Deterministic NoisyState means/attitude and closed-loop feedback against C++. |

WaypointFollower and WaypointBroadcast are small Rust replacements, not ports of
MotorSchemas or the legacy waypoint composition. Their routes use local ENU
meters (`x,y,z; x,y,z`), not latitude/longitude. The point example uses
`max_speed=-1` to follow commanded velocity; a nonnegative SingleIntegrator
`max_speed` forces that speed magnitude, matching C++ rather than acting as a cap.
Fixed-wing aircraft cannot stop at a point; their example repeats the route.

The original unsupported demos, including `straight.xml`, remain in sibling
`../scrimmage/missions` on `Ubuntu-24.04`. Use `straight-no-gui.xml` for the small
collision demo or `networks-local-global.xml` for boundary/ground behavior here.
Malformed/missing-plugin cases live under `crates/*/tests/`, not this folder.
The copied C++ guides still describe upstream inputs and are intentionally unchanged.

See [verification evidence](../docs/EVIDENCE.md) for commands, expected results,
and comparison limits; [model scope](../docs/MODEL_SCOPE.md) explains the selection.
