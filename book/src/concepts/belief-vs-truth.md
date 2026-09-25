# Belief vs truth

A real robot never knows exactly where it is. It has GPS that is a few meters
off, an IMU that drifts, and a compass disturbed by its motors. Its software acts
on an *estimate*. To test autonomy honestly, a simulator has to keep two things
apart:

- **Truth:** where the vehicle really is. The motion model computes it, and
  collisions and scoring use it.
- **Belief:** where the vehicle *thinks* it is. Sensors produce it, and
  autonomy and controllers act on it.

scrimmage-rs keeps these as two separate values on every entity.

## Who reads and writes what

| Plugin | Reads | Writes |
| --- | --- | --- |
| Motion model | Truth | Truth |
| Sensor | Truth | Belief (optional), observations, messages |
| Autonomy, controller | Belief, as `context.state` | Commands on its ports |
| Interaction | Truth of all entities | Truth, health |
| Metrics | Events and messages | Scores |

The key line is the autonomy row. `context.state` is the entity's **belief**.
Autonomy and controller code doesn't need to know whether a noisy sensor is
installed: it always reads `context.state`.

## Ideal feedback by default

If no sensor provides an estimate, belief simply *is* truth. `context.state`
reads the true state, as if the vehicle had perfect sensors. That's the
right starting point when you're developing a new behavior: first make it work
with perfect information, then add noise and see what breaks.

## Adding noise with a sensor

A sensor installs an estimate with `set_belief`. The built-in
[NoisyState](https://github.com/EthanMBoos/scrimmage-rs/blob/main/crates/core/src/plugin/sensor/noisy_state/noisy_state.rs)
sensor does exactly this. Each tick it:

1. copies the true state,
2. adds Gaussian noise to position, velocity, and attitude,
3. calls `context.set_belief(noisy_state)`,
4. also publishes the noisy state as a message on `LocalNetwork`.

Add it to an entity in a mission:

```xml
<sensor>NoisyState</sensor>
```

From then on, that vehicle's autonomy and controller act on the noisy state,
while collisions and scoring still use the truth. Try
`missions/noisy-state.xml` to see the difference.

A sensor can call `context.clear_belief()` to go back to ideal feedback.

## Timing: sensors run after motion

Within one tick, the order is:

```text
autonomy -> controllers -> motion -> sensors -> interactions -> ...
```

Sensors sample the state *after* the vehicle has moved. The autonomy sees that
new estimate on its **next** tick. This one-tick delay is realistic: software
always acts on a measurement taken slightly in the past.

## One estimator per vehicle

The belief persists between samples. If several sensors call `set_belief`, they
run in mission order and **the last one wins**. The simulator does not fuse them.

To model a realistic navigation system, use one sensor as the estimator:

- GPS, IMU, and compass sensors each publish their measurements as messages on
  `LocalNetwork`.
- A single estimator sensor (for example, a Kalman filter) subscribes to those
  messages, combines them, and is the **only** plugin that calls `set_belief`.

Every autonomy and controller then acts on the fused estimate without changes.

## Other vehicles are still truth

`context.contacts_truth` gives every entity's **true** state, as the name says.
It's there for simple behaviors and for porting C++ plugins that relied on it.

If your research is about perceiving *other* vehicles (detection, tracking,
noisy contacts), don't read `contacts_truth` in your autonomy. Instead:

1. write a sensor that reads `contacts_truth`,
2. applies range limits, noise, or missed detections,
3. publishes the result as a message,

and have the autonomy subscribe to that message. That keeps "what the world is"
and "what this vehicle knows" separate, the same way belief does for its own
state.

## Summary

- Truth is physics; belief is what the software thinks.
- `context.state` is belief. It equals truth until a sensor installs an estimate.
- Sensors run after motion; autonomy sees the estimate next tick.
- Use one estimator sensor per vehicle; the last `set_belief` wins.
- `contacts_truth` is exact. Put a sensor in between if perception matters.
