# C++ plugin audit: can downstream users avoid core changes?

Source review, 2026-09-24. C++: sibling `../scrimmage`, branch `Ubuntu-24.04`,
commit `4bdf41fb06facaea358d478d9a07aa4d77fcac27`. Rust: commit
`9614581bc0d43a2ac78d689fccb62d104edc8cd4`, plus the working tree. These identify
the reviewed snapshots, not required pins. Existing untracked documentation was
left alone. The original audit changed no simulator code; the implementation
update below records the subsequently selected additions.

ROS 1/2 and ArduPilot remain separate future adapter work (see [TODO.md](TODO.md)).
Their upstream plugins are listed in the catalog for completeness; they are not
core plugin gaps for this audit.

## Assessment

**The seven-role engine is a useful extensible baseline, but it is not yet a
general replacement for the C++ core, which is not the project goal.** Many new
algorithms can be ordinary downstream plugins today. Some established C++ use
cases still cross boundaries that Rust does not expose. The highest-value next
work is to exercise those
boundaries with a few composed cases, rather than port every plugin.

The existing scheduler, registry, control chain, immutable world snapshots,
typed messages, owned belief, interactions, and team metrics are good foundations.
This review does not find a reason to discard their design. It does find reasons
not to freeze the API or promise that every future user will need only plugins.

“Plugin-only” below means an implementation using today's public Rust API, with
explicit Rust behavior where appropriate. It does **not** mean unchanged C++
configuration, numerical parity, identical callbacks, or automatic support for
every optional mode. A plugin can often work around a missing shared capability
using its own configuration and messages; that is different from all stock
plugins interoperating through an established core contract.

## Selected additions (user decision)

Only these two workstreams were accepted and are now implemented:

| Workstream | Coverage | Shared code changed |
| --- | --- | --- |
| NoisyContacts | Other-agent measurements, next-tick consumption, independent noise, spawn/removal | Registration/module wiring only; existing sensor RNG, messages and observations suffice. |
| SphereNetwork + AuctionAssign | 3D range/altitude-plane filtering, loss, start/bid/result delivery, deterministic outcomes | Registration/module wiring plus engine-provided, mission-seeded per-plugin streams on agent and network contexts. |

The other proposals are **not selected and are not implementation backlog**:
force/contact response, richer dynamics telemetry, energy/battery models, games,
Boids, more motion/controller models, terrain/rays, and maritime workflows.
The source findings below remain documented limitations, not prerequisites for
this supported simulation subset. Do not expand core interfaces to cover them
without a new concrete request. See [model scope](MODEL_SCOPE.md) and
[verification evidence](EVIDENCE.md) for the shipped behavior.

## Source findings and limits outside the selected subset

### 1. Physical interactions cannot supply loads through a common motion interface

C++ [ExternalForceField](../../scrimmage/src/plugins/interaction/ExternalForceField/ExternalForceField.cpp)
calls `motion()->set_external_force` and `set_external_moment`.
[GroundCollision](../../scrimmage/src/plugins/interaction/GroundCollision/GroundCollision.cpp)
has a non-removal force mode, and
[BulletCollision](../../scrimmage/src/plugins/interaction/BulletCollision/BulletCollision.cpp)
also supplies contact forces. The C++
[MotionModel interface](../../scrimmage/include/scrimmage/motion/MotionModel.h)
exposes these operations, mass, external velocity and teleport.

Rust [MotionContext](../crates/core/src/motion.rs) exposes mutable kinematic truth,
messages, entity identity and time. It has no load input or motion reset hook.
[InteractionContext](../crates/core/src/entity_interaction.rs) can change truth or
health, but cannot apply a load to an arbitrary stock motion model.
[Multirotor](../crates/core/src/plugin/motion/multirotor/multirotor.rs) explicitly
assumes zero external contact force; GroundCollision rejects force response.

**Consequence:** landing/contact without deletion, physical disturbances, and
shared world forces are not covered by successful free-flight tests. Directly
changing velocity is not generally equivalent to applying a force through an
integrator. Changing public truth also does not define how every model's private
state is reset; Multirotor intentionally retains private body rates/velocities.

Small direction: explicit physical load data and its commit/consumption rule,
plus reset behavior when a real consumer needs it. Specify whether multiple
interactions replace or accumulate loads and whether a load persists through
all motion substeps. C++ models differ and sometimes clear forces after one
substep; do not silently “fix” that while claiming parity. A paired custom
interaction/motion implementation can exchange messages today, but that does
not make stock physics accept loads or establish general interoperability.

### 2. Rich dynamics output is unavailable to sensors at the sensor phase

C++ [RigidBody6DOFStateSensor](../../scrimmage/src/plugins/sensor/RigidBody6DOFStateSensor/RigidBody6DOFStateSensor.cpp)
reads body acceleration, angular acceleration and wind from a motion-model
interface, in addition to ordinary state. That richer sample is useful for
simulated inertial sensing and motion diagnostics independently of any external
flight stack.

Rust [KinematicState](../crates/core/src/math/state.rs) contains pose and linear/
angular velocity only. [SensorContext](../crates/core/src/sensor.rs) cannot read
additional committed motion outputs. Motion can publish a typed message, but
the [network phase](../crates/core/src/simcontrol.rs) runs **after** sensors, so
that message cannot supply current-tick dynamics to a sensor in the same tick.

**Consequence:** a plugin can implement a finite-difference accelerometer, but
it is not automatically equivalent to the model's acceleration at its physics
sampling interval. Multi-rate simulated IMU experiments encounter this boundary.

Small direction: expose the selected committed dynamics sample, with explicit
frame, time and meaning. Distinguish inertial acceleration from specific force
and specify gravity handling. No need for arbitrary casts into concrete motion
plugins or for copying all private ODE state into core state.

### 3. Perception of other agents is feasible, but truth isolation is optional

[NoisyContacts](../../scrimmage/src/plugins/sensor/NoisyContacts/NoisyContacts.cpp)
adds noise to other agents and publishes a contact map. Rust sensors already
have post-motion contacts, independent sensor RNG and arbitrary typed payloads.
The Rust port now uses these APIs, including a latest local snapshot and typed
LocalNetwork publication. Its contract tests consume biased measurements in
autonomy and cover empty, newly spawned, and removed contacts.

However, [AgentContext](../crates/core/src/plugin_manager/entity_plugin.rs)
also exposes `contacts_truth`. A consumer can choose measurements, but the API
does not enforce that choice. Own-state belief is not a per-agent estimated
world model. The tested measurement consumer does not establish enforced truth isolation.

[ContactBlobCamera](../../scrimmage/src/plugins/sensor/ContactBlobCamera/ContactBlobCamera.cpp)
adds range/FOV, detection errors, mounting pose, and target-size projection.
Its bounding boxes depend on `Contact::radius()`. Rust
[EntitySnapshot](../crates/core/src/entity.rs) has no radius/geometry field.
Sensor-specific mounting pose can live in plugin configuration, but obtaining
arbitrary targets' sizes requires shared metadata or an explicitly supplied
model-specific map. A bearings/range-only sensor is easier than full camera
behavior; omitting target sizes is a change in model scope.

### 4. Environmental geometry is a separate missing contract

[RayTrace::step](../../scrimmage/src/plugins/sensor/RayTrace/RayTrace.cpp) does
not perform ray casting. It defines rays and configuration. BulletCollision
discovers sensors, transforms their rays, maintains geometry and produces point
clouds; it also registers `get_ray_tracing` as a global service.
[LOSSensor](../../scrimmage/src/plugins/sensor/LOSSensor/LOSSensor.cpp) can use a
flat-earth calculation or consume the collision system's point clouds.

Rust has no shared geometric scene/query API, sensor-mount registry, or global
service interface. **A full lidar/occlusion/obstacle world is not just a new
Sensor implementation against today's stock core.** It is possible to build
a self-contained sensor with private geometry, or a cooperating plugin group
with its own messages. Those are narrower capabilities with explicit timing.

Terrain is a useful smaller starting point:
[TerrainGenerator](../../scrimmage/src/plugins/interaction/TerrainGenerator/TerrainGenerator.cpp)
publishes a height map and
[AltitudeAboveTerrain](../../scrimmage/src/plugins/sensor/AltitudeAboveTerrain/AltitudeAboveTerrain.cpp)
caches and queries it. Typed immutable map messages can fit Rust today.
Late-spawned subscribers need an explicit delivery strategy: stock messages are
not latched. The C++ one-shot publication is not itself proof of late-spawn support.
Physical terrain must remain independent of viewer terrain loading.

### 5. A controller can stop the run, but cannot directly retire its own entity

[MotionBattery](../../scrimmage/src/plugins/controller/MotionBattery/MotionBattery.cpp)
discovers connected ports, limits depleted outputs, accepts recharge messages,
publishes charge, offers a synchronous `get_battery_charge` service, and calls
`parent()->collision()` on depletion.
[TrajectoryRecordPlayback](../../scrimmage/src/plugins/autonomy/TrajectoryRecordPlayback/TrajectoryRecordPlayback.cpp)
sets its parent's health to zero at the end of playback.

Rust [Update](../crates/core/src/plugin_manager/entity_plugin.rs) offers Applied
and Stop; Stop ends the **simulation**, not just that agent. Agent contexts have
no health/removal command. Interactions can set health, so an explicit lifecycle
message and cooperating interaction can implement removal today, with delivery
latency. That is not the same immediate contract as the C++ method call.

The normal controller chain already supports an explicit-port energy limiter.
[UUV6DOFLinearEnergy](../../scrimmage/src/plugins/controller/UUV6DOFLinearEnergy/UUV6DOFLinearEnergy.cpp)
is the clean first test: it consumes throttle and forwards limited throttle plus
elevator/rudder. A full battery port does not justify a universal RPC framework;
first decide whether charge observations and an entity-local retirement request
cover the research use case. Port discovery can also be replaced by explicit
configured channels rather than rebuilding C++ reflection.

### 6. Computation is more extensible than stock recording

[SimpleCaptureMetrics](../../scrimmage/src/plugins/metrics/SimpleCaptureMetrics/SimpleCaptureMetrics.cpp)
and [FlagCaptureMetrics](../../scrimmage/src/plugins/metrics/FlagCaptureMetrics/FlagCaptureMetrics.cpp)
can map to typed messages and Rust's arbitrary per-team metric columns.
Interactions can implement their health/game logic without new simulator branches.

But [EventKind](../crates/core/src/simcontrol.rs) is a closed core enum, and the
stock event log only retains those built-in events. A downstream capture or
battery message will not automatically appear in `events.json`.
[MetricReport](../crates/core/src/metrics.rs) is organized by team, whereas C++
[CPA](../../scrimmage/src/plugins/metrics/CPA/CPA.cpp) and
[OpenAIRewards](../../scrimmage/src/plugins/metrics/OpenAIRewards/OpenAIRewards.cpp)
write per-entity records. Plugins also lack an application-provided output
directory/diagnostic sink in their contexts.

**Consequence:** users can calculate these results and write explicitly
configured files, but cannot obtain every custom event, entity table, or debug
shape through the existing common output path. Keep model-specific records in
the application/output boundary; do not add a core EventKind for every new game.
This is an output API issue, not a reason to copy VTK or build a statistics framework.

### 7. Neighborhood algorithms fit; the common RNG surface is narrower

[SphereNetwork](../../scrimmage/src/plugins/network/SphereNetwork/SphereNetwork.cpp)
needs endpoint positions, range/plane filtering and transmission probability.
Rust [Transmission](../crates/core/src/pubsub/messages.rs) supplies identities
and contacts, and routing supports drop/delay. A snapshot distance scan is enough
for an initial implementation. [Boids](../../scrimmage/src/plugins/autonomy/Boids/Boids.cpp)
can likewise scan contacts. C++'s R-tree is not an architectural prerequisite;
its pre-motion indexing also requires care in post-motion comparisons.

The engine gives every sensor, autonomy, controller, and network instance its
own mission-seeded [PluginRandom](../crates/core/src/common/plugin_random.rs)
stream, keyed by entity ID and a stable plugin identity. SphereNetwork and
AuctionAssign draw from these, so the run's `<seed>` changes loss and bids as in
C++. Sensor sequences, spawn RNG, and scheduling are unchanged. This is not C++
shared-RNG sequence parity.

Two smaller shared-data limits also deserve an explicit decision. Port **names**
are extensible, but [Unit](../crates/core/src/common/variable_io.rs) is a closed
enum without force, torque or angular-acceleration units. Normalized actuator
commands fit today; accurately declaring a new physical port may require a small
core addition. GPS and the geographic waypoint plugins use C++'s shared local
projection. Rust plugin contexts do not provide a geographic origin/projection;
a plugin can own an explicitly configured conversion, while a shared origin
belongs at the later application/configuration boundary. Neither issue calls
for a large unit system or GIS framework.

### 8. C++ plugin composition is not the current Rust composition contract

[MotorSchemas](../../scrimmage/src/plugins/autonomy/MotorSchemas/MotorSchemas.cpp)
constructs nested behaviors through the plugin manager and uses runtime parameters.
[AutonomyExecutor](../../scrimmage/src/plugins/autonomy/AutonomyExecutor/AutonomyExecutor.cpp)
constructs named child autonomies, switches active groups and reinitializes them.
TrajectoryRecordPlayback inspects another autonomy's selected desired state.

Rust [PluginStack](../crates/core/src/entity/plugin_stack.rs) validates a fixed
stack, merges autonomy outputs in source order, and connects a controller chain.
Autonomies cannot consume another autonomy's ports through this API, inspect
other instances, or create independently scheduled/subscribed child plugins.
This is not equivalent to C++ active-autonomy selection.

**Do not restore nested plugin machinery just to port these names.** Ordinary
Rust structs, a state enum and behavior helpers can implement goal following,
avoidance and switching inside one plugin, consistent with
[the existing composition guidance](FIGHTING_AUTONOMY_PLUGIN_BLOAT.md).
GraphvizFSM's transition/message logic can also be a normal plugin. Exact
legacy nested lifecycle/configuration behavior remains unsupported. Runtime
gain updates can use a specific typed message when needed; a global parameter
server is not a prerequisite for basic controller development.

### 9. Lifecycle requirements

External sessions, peer readiness, connection identity, and connection-triggered
creation belong to the planned ROS/ArduPilot adapter work in [TODO.md](TODO.md),
not to this plugin audit. Current mission-scheduled spawning remains supported.

C++ also has posthumous/readiness/reset hooks beyond Rust's complete-tick stop
and removal contract. No concrete `posthumous` override was found in the audited
seven-category plugin tree; do not invent a port requirement without a consumer.
MOOSAutonomy provides a concrete readiness example, but external middleware
sessions are excluded. For reinforcement-learning experiments, reward
accumulation is straightforward; OpenAIRewards is not evidence of a complete
episode/reset/action interface. Reconstructing a simulation per episode is a
possible starting point, not a claim of an implemented RL adapter.

## Catalog coverage

The directory inventory covers **110 C++ implementation directories**: 29 autonomy,
26 controller, 17 motion, 13 sensor, 16 interaction, 4 network and 5 metrics.
These are source directories, not a claim that all are enabled, buildable or
complete plugins. Public/core dependencies were screened across the tree and
the distinctive cases above were traced in their implementations. This is not
a numerical correctness review of every equation or an audit of private
downstream plugins that are absent from this checkout.

| Category | Names/families | Disposition |
| --- | --- | --- |
| Motion | SimpleAircraft, SingleIntegrator, FixedWing6DOF, Multirotor | Present in Rust with selected behavior; shared force and dynamics-output contracts still matter for simulation. |
| Motion | Unicycle, DoubleIntegrator, DubinsAirplane, HarmonicOscillator | Ordinary model ports; useful when a research case needs them. Teleport/private-state behavior must be specified separately. |
| Motion | SimpleCar, Unicycle3D, DubinsAirplane3D | Free-motion equations fit; complete external-force behavior needs the load contract. |
| Motion | UUV6DOF | Meaningful maritime addition; verify equations, force usage and frame conversions rather than inferring fidelity from its name. |
| Motion | SimpleQuadrotor | Old controller-vector access can become named ports; limited extra coverage after Multirotor. |
| Motion | Ballistic, RigidBody6DOF | Low priority; specific suspicious equations described below. |
| Motion | JSBSimControl, JSBSimModel | Excluded production integrations; retain offline-verification direction. |
| Controller | AircraftPIDController, SimpleAircraftControllerPID, SingleIntegratorControllerSimple | Already selected in Rust, with documented option limits. |
| Controller | DirectController, AircraftToSingleIntegratorController, DoubleIntegratorControllerVelYaw, DoubleIntegratorControllerWaypoint, SingleIntegratorControllerWaypoint, UnicycleControllerPoint, UnicyclePID, SimpleCarControllerHeading, HarmonicOscillatorConstController, FixedWing6DOFControllerPID, RigidBody6DOFControllerPID, SimpleQuadrotorControllerLQR, UUV6DOFPIDController | Mostly port/configuration/control equations. Translate legacy desired-state/u-vector access into explicit ports; audit units and selected plant compatibility. |
| Controller | UUV6DOFLinearEnergy, MotionBattery | Not selected; battery's full contract differs as above. |
| Controller | MultirotorControllerOmega, MultirotorControllerPID | Omega's PWM conversion is useful; model parameters can be explicit config. PID is not a finished stabilization controller in this source. |
| Controller | FixedWing6DOFControllerROS, RigidBody6DOFControllerROS | ROS integrations; separate future adapter work, not core plugin ports. |
| Controller | JoystickController | Peripheral input demo; not required for the mission workflow. |
| Controller | JSBSimControlControllerHeadingPID, JSBSimModelControllerDirect, JSBSimModelControllerHeadingPID | Coupled to excluded production JSBSim workflows. |
| Sensor | NoisyState | Present; stochastic Rust/C++ streams intentionally differ. |
| Sensor | NoisyContacts | Implemented with typed measured contacts and explicit C++ differences. |
| Sensor | RigidBody6DOFStateSensor | Not selected; would need the dynamics-output contract above. |
| Sensor | ContactBlobCamera | Useful later for perception; target size, mount/geometry, uniform randomness and output need choices. OpenCV windows are not necessary. |
| Sensor | GPS, SimpleINS | Geographic conversion and intermittent navigation case; SimpleINS has legacy numerical/RNG quirks. |
| Sensor | AltitudeAboveTerrain, RayTrace, LOSSensor | Environment-data/query cases, with flat-earth LOS as a small subset. |
| Sensor | ROSAltimeter, ROSCompass, ROSIMUSensor, AirSimSensor | External integrations; separate future adapter work. Reuse mathematical sensor ideas for a concrete simulation experiment. |
| Network | GlobalNetwork, LocalNetwork | Present, selected immediate-delivery modes. |
| Network | SphereNetwork | Implemented with snapshot distance checks, altitude plane, and mission-seeded loss. |
| Network | GPUSphereNetwork | Excluded OpenCL path; not a core-coverage requirement. |
| Interaction | Boundary, GroundCollision, SimpleCollision | Present with deliberate geometry/contact limits. |
| Interaction | ExternalForceField, BulletCollision | Physical load/shared-query boundaries. Not selected for implementation. |
| Interaction | SimpleCapture, FlagCaptureInteraction, CaptureInBoundaryInteraction, EnforceBoundaryInteraction, RandomAttrit | Feasible health/game/message plugins; boundary shapes, RNG and custom recording limits apply. |
| Interaction | TerrainGenerator, MapGen2D, GraphInteraction | Environment/map/graph data producers. Own assets/data types in models/application; full physical geometry and rendering are separate. |
| Interaction | ROSClockServer, ROSShapeViz, GRPCCommandString | External integration/control paths excluded; no clock bridge, remote visualization bridge, or string-command service requirement. |
| Metrics | SimpleCollisionMetrics | Present. |
| Metrics | SimpleCaptureMetrics, FlagCaptureMetrics | Useful plugin-only game scoring. |
| Metrics | CPA, OpenAIRewards | Calculations fit; per-entity records need an explicit output path. |
| Autonomy | Straight | Present with deliberate excluded options. |
| Autonomy | AuctionAssign | Implemented as one random-bid auction over a configurable registered network. No task execution or auction framework. |
| Autonomy | Boids, Predator, TakeFlag, BoundaryDefense, AvoidWalls, follow | Not selected. Algorithms could use snapshots/messages with explicit Rust adaptations. |
| Autonomy | GoToWaypoint, WaypointDispatcher, WaypointGenerator | Existing Rust-native waypoint flow covers part of this need; exact legacy routes/geographic behavior remain different. |
| Autonomy | MotorSchemas, MoveToGoalMS, AvoidEntityMS, TrailMS, AutonomyExecutor, GraphvizFSM | Reuse behavior ideas with ordinary Rust composition; do not port nested plugin lifecycle machinery by default. |
| Autonomy | TrajectoryRecordPlayback | Exposes desired-command recording, arbitration and entity-local retirement gaps. Command playback alone is simpler. |
| Autonomy | APITester, ShapeDraw, Control3D, CommandStringRelay, JoystickAutonomy | API/visual/input demos, not reasons to recreate legacy framework services. |
| Autonomy | ArduPilot, ROSAutonomy, ROSControl | Runtime integrations; separate future adapter work, not core plugin ports. |
| Autonomy | ROSAirSim, MOOSAutonomy, FlightGearMultiplayer | Other integration ecosystems; outside selected runtime scope. |

The header-only SimplePubSub directory is not an additional implementation in
`src/plugins`; source-directory counts should not be mistaken for XML/header counts.

## Source quality cautions

These are observed reasons to select behavior carefully, not permission to patch
the C++ reference or change existing Rust parity contracts:

- MultirotorControllerPID computes velocity feedback but writes the same
  `sqrt(mass * gravity / (rotor_count * thrust_coefficient))` to each rotor.
  Attitude/yaw corrections are commented out. It is not evidence of working
  closed-loop navigation or stabilization.
- RigidBody6DOF sets gravity to zero, disables some translational derivatives,
  and assigns P/Q/R by clamping U. Prefer the existing more concrete flight models.
- Ballistic initializes acceleration to zero and puts `-mass * g` in the
  **derivative of vertical acceleration**. A literal port is not ordinary
  constant-gravity projectile motion.
- SimpleCar's `turn_rate` input is used as a steering angle inside `tan`, and
  its gravity branch multiplies acceleration by mass. Port names alone do not
  establish units or physical correctness.
- UUV6DOF transforms and clears external force into `force_ext_body_`, but the
  reviewed model equations do not use that field. Do not infer functioning
  disturbance response from the base interface or the assignment alone.
- SimpleINS divides its velocity difference by a hard-coded `.01` and uses
  `rand()` for timing. LOSSensor calls `srand(time(0))`. Neither is an acceptable
  source of an implicit deterministic/multi-rate guarantee.

## Supported downstream-plugin claim

The selected cases exercise actual measured-contact consumption, spawn/removal,
network geometry/loss, independent subscribers, and a start/bid/result exchange.
Tests compare measurements and message histories as well as frame/event/summary
outputs across workers. Stock recording does not archive arbitrary plugin messages.

The supported claim is: **“Users can add compiled plugins across all seven roles
for the supported snapshot/control/message workflows.”** These additions needed
no scheduler, entity-lifecycle, physics-state, or context redesign. They do not
prove that arbitrary future physics or unavailable private C++ use cases can
avoid core changes. The unselected findings document those limits; they are not
release gates. The later library-first work remains a separate user decision.

## Original audit validation (before the additions)

- Inventoried all seven C++ implementation categories and inspected Rust's
  public contexts, plugin stack, state, routing, lifecycle and output interfaces.
- Ran `cargo test -p scrimmage-core --test plugin_contracts --locked --offline`:
  **12 passed**. This confirms today's tested public extension contracts, not
  the missing capabilities identified here.
- No fresh C++ missions, broad numerical parity run, adapter test or visual QA
  was performed. Existing [evidence](EVIDENCE.md) remains scoped to its recorded
  cases; spawn-RNG mismatch and lack of direct C++ event-stream comparison remain.
- Some historical evidence predates FixedWing6DOF/Multirotor. Current source
  and [MODEL_SCOPE.md](MODEL_SCOPE.md) were used for present model availability.
