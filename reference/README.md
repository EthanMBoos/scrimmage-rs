# Docker reference checks

Build the C++ `Ubuntu-24.04` reference in Docker and compare it with the current
Rust workspace:

```sh
python3 reference/reference_check.py
```

Requires Python 3.10+, a running Docker engine, Rust/Cargo, and sibling
`../scrimmage` with a local `Ubuntu-24.04` branch. No native C++ build is needed.
The first build downloads a large dependency image and compiles C++; later
checks reuse Docker's build cache. Allow several GB of disk space.

Checks are saved automatically as `runs/reference-check000`, `001`, and so on.
Use `--output path/to/new-check`, `--source path/to/scrimmage`, or `--jobs 4`
to override locations or compilation parallelism. Existing output directories
are never reused. `--timeout` controls each mission/comparison; `--build-timeout`
controls each Docker/Cargo build. Both are in seconds.

## What gets built

The script reads the exact
[upstream slim dependency Dockerfile](https://github.com/gtri/scrimmage/blob/Ubuntu-24.04/ci/dockerfiles/ubuntu-24.04-slim-dependency-only)
from the local branch snapshot and builds it without edits. Despite its name,
that image includes VTK/Qt and other integration dependencies. Its JSBSim setup
downloads amd64 packages, so the workflow explicitly uses `linux/amd64`, including
emulation on Apple Silicon. These packages are installed inside Docker only.

A second image builds a `git archive` of the local branch. It does not copy
uncommitted changes, modify the checkout, or fetch/update your local branch.
The branch is not pinned in this project;
each check records the actual commit, source archive hash, Dockerfile hashes,
image IDs, Linux package/compiler versions, Rust compiler, and Rust source hashes.
The upstream dependency Dockerfile itself clones GitHub for its setup scripts;
that is distinct from the archived source used for the reference executable.

[Dockerfile](Dockerfile) disables optional VTK, gRPC, OpenCV, ROS, and GPU build
paths and builds the executable plus the thirteen upstream C++ plugins needed by the current
matrix. This is a headless comparison build, not verification of the full C++
plugin catalog. No C++ simulator logic or random generator is patched.
Multirotor additionally uses our small `fixtures/MotorSpeeds.cpp` command driver:
the old upstream PID writes a different control interface. This fixture supplies
constant motor speeds to the unchanged upstream model; it is not an autopilot.
Fixture source hashes are recorded alongside the upstream source provenance.
`VTK_FOUND=OFF` is set explicitly because upstream CMake expands that variable
inside conditionals even when VTK discovery is disabled.

Docker image tags/layers remain cached. There is no automatic Docker pruning.
Mission containers run without network access and can write only to their
dedicated host output mount; they are removed after the run.

## Default comparison matrix

| Mission | Coverage |
| --- | --- |
| `straight-no-gui.xml` | Opposing aircraft, collision and removal |
| `test_missions/straight_cpu.xml` | Longer single-aircraft run |
| `test_missions/straight_cpu_mul.xml` | Original mission including its `motion_multipler` typo |
| `verification/aircraft-substeps-spawning.xml` | Actual five-substep execution, rates, randomized state, and scheduled spawning |
| `verification/noisy-state-bias.xml` | NoisyState deterministic bias/attitude equations and belief feedback, without stochastic sequence differences |
| `networks-local-global.xml` | Boundary response over GlobalNetwork, local own-state feedback, ground removal and metrics |
| `fixed-wing-6dof.xml` | Aerodynamic flight, tilted/northbound attitude, nonzero rates and wind |
| `multirotor.xml` | Quad hover, tilted flight, unequal motor speeds and a six-rotor configuration |

Each case runs C++ headless and single-threaded. Rust runs with 1, 2, and 8
workers without Rerun, plus an eight-worker headless Rerun recording.

For just the Multirotor port, run
`python3 reference/reference_check.py --mission missions/multirotor.xml` and
`cargo test -p scrimmage-core multirotor`. The initial four-vehicle check compared
4,001 frames / 16,004 states: maximum position error 8.63e-15 m, velocity error
4.82e-13 m/s, matching summaries, and byte-identical Rust worker/recording outputs.
This establishes the selected open-loop cases, not a working upstream autopilot
or external-force/landing integration. Model-specific quirks are commented inline.

- C++/Rust frames and team summaries use `reference/compare.py`.
- Rust frames, events, and summaries must be byte-identical across worker counts
  and with recording enabled/disabled. The recording must be nonempty.
- Direct C++ event-stream comparison is not implemented. Headless recording is
  not a native-viewer playback or screenshot test; use `AGENTS.md` for visual QA.

`--mission path/to/mission.xml` can be repeated to replace the default matrix.
Additional missions must use plugins included in this build and have expanded
XML: XIncludes are rejected explicitly. The script saves the input and effective
C++ mission, with all headless/logging overrides recorded. It does not change
physics, seeds, rates, parameter defaults, or the multiplier typo.

The build also includes NoisyState. Its deterministic bias/attitude fixture can
be checked with `--mission missions/verification/noisy-state-bias.xml`.
`missions/noisy-state.xml` uses Rust's independent sensor random streams and is
not expected to match C++ sample-for-sample. Keep stochastic mismatch reports;
use the deterministic fixture and sensor unit tests to verify equations/noise
without pretending the random sequences are identical.

## Compare saved runs

Comparison is development tooling, not a simulator subcommand. It needs only
Python 3.10+ (standard library); no Docker, Cargo, Rerun, or Python packages:

```sh
python3 reference/compare.py path/to/cpp-run path/to/rust-run \
  --report runs/comparison.json
```

Both directories must contain `frames.bin` and `summary.csv`. JSON is printed
to stdout; `--report` also saves it, including mismatches, to a new file. Parent
directories are created automatically; existing reports are never overwritten.
Exit codes are 0 for a match, 1 for a mismatch, and 2 for invalid inputs or I/O
errors. Invalid inputs do not produce a comparison report.

The ported comparator keeps the original tolerances: position 0.01 m, velocity
0.001 m/s, orientation 0.001 rad, angular velocity 0.001 rad/s, timestamp 1e-9 s,
and summary values 1e-6. Frames are paired by index, retaining repeated terminal
timestamps; entities and summary rows are matched by ID. Quaternion signs are
equivalent. Malformed/nonfinite state is rejected. `frames.py` decodes the
length-delimited proto3 frame subset from the reference branch's schemas; it is
not a general protobuf library. Direct C++ events are still not compared.

## Results and failure handling

The command exits zero only when every requested case passes. `report.json`
contains the overall result and per-mission checks; case directories retain
`comparison.json`, input missions, C++ logs, Rust variants, hashes, and recordings.
Build logs and source/image provenance are kept at the check root. A failed
mission does not prevent the remaining cases from being checked.

Old native macOS comparisons are historical evidence, not proof of Ubuntu
parity. In particular, the current Rust legacy RNG reproduces Apple libc++.
Ubuntu's libstdc++ can use a different default random engine and normal sampler.
Randomized missions may therefore fail even when deterministic dynamics agree.
Treat those as compatibility gaps: do not change seeds, disable variance, or
relax comparator tolerances to manufacture a pass.

Reproduction commands and recorded outcomes are in the commit-ready
[evidence guide](../docs/EVIDENCE.md). The first completed Docker check was
saved locally as `runs/reference-check001`; it is not required to use this repo.
The straight mission and both CPU missions passed. The randomized spawning
fixture failed (maximum position difference 8.41934 m, already different at
initialization). Inspection of the image's libstdc++ headers confirmed a
different engine multiplier and normal-variate ordering from Rust's current
Apple-compatible implementation. All four Rust worker-count and recording
equality checks passed. The overall checker correctly exited with status 1;
Ubuntu stochastic parity is still unfinished.

The archive has no `.git` directory, so upstream's optional Git metadata
collection prints warnings in C++ console logs. `source.json` and image
provenance identify the actual source instead. The unused JSBSim integration
also warns about its unset model path; no JSBSim mission is tested here.

The upstream base tag and package repositories are mutable. Saved image IDs
and package inventories identify a check's environment; rebuilding months later
is not guaranteed to reproduce that exact dependency image byte-for-byte.

Test the checker itself without Docker:

```sh
python3 -m unittest discover -s reference -p 'test_*.py'
```

## Preliminary benchmark

After building the reference image with the checker above:

```sh
python3 reference/benchmark.py --output runs/new-benchmark
```

This builds a Release core-only Rust harness and runs both implementations in
one `linux/amd64` container, with C++ always single-threaded and Rust at 1/8 workers.
Defaults: 128 entities, 1,000 steps, one warmup and three measured runs each.
`--entities`, `--steps`, and `--repetitions` override those sizes. The script
checks reference output and repeatability; a mismatch is a failure, not a timing
result to advertise. No Rerun is built into the harness. Timing includes process
startup, parsing, simulation, and logging, but excludes builds/container startup.
The [benchmark code](benchmark.py) records the measured baseline and limitations;
each run saves raw samples and environment details in its output directory.
