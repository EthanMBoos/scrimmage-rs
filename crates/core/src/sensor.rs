//! Sensor interface and observation delivery; concrete models live in plugin/sensor.
use crate::{
    EntitySnapshot, KinematicState,
    common::SensorRandom,
    plugin_manager::entity_plugin::{EntityInfo, Plugin, StepTime, Update},
    pubsub::Messages,
};
use anyhow::Result;

mod observations;
pub use observations::{Observation, Observations};

pub struct SensorContext<'a> {
    pub messages: &'a mut Messages,
    pub entity: EntityInfo,
    pub time: StepTime,
    /// Sensors sample committed truth after all motion substeps.
    pub truth: &'a KinematicState,
    pub contacts_truth: &'a [EntitySnapshot],
    pub random: &'a mut SensorRandom,
    pub(crate) belief: &'a mut Option<KinematicState>,
    pub(crate) pending: &'a mut Observations,
}
impl SensorContext<'_> {
    /// Replace the entity's estimate, without modifying physical truth.
    /// Autonomy/controllers see it on their next update; it persists between samples.
    /// Multiple writers run in mission order, with the last update winning.
    pub fn set_belief(&mut self, state: KinematicState) {
        *self.belief = Some(state);
    }

    /// Return to ideal state feedback until another sensor supplies an estimate.
    pub fn clear_belief(&mut self) {
        *self.belief = None;
    }

    /// Queue an entity-local observation for the network/delivery phase.
    /// This is not a simulated GlobalNetwork transmission.
    pub fn publish_local<T: Send + Sync + 'static>(&mut self, topic: &str, value: T) -> Result<()> {
        self.pending
            .publish(topic, self.time.time_s + self.time.dt_s, value)
    }
}

pub trait Sensor: Plugin {
    fn initialize(&mut self, _context: &mut SensorContext<'_>) -> Result<()> {
        Ok(())
    }
    fn step(&mut self, context: &mut SensorContext<'_>) -> Result<Update>;
}

#[cfg(test)]
mod tests {
    use super::{Observations, SensorContext};
    use crate::plugin::{EntityInfo, Messages, SensorRandom, StepTime};
    use crate::{KinematicState, Vec3};

    #[test]
    fn installing_and_clearing_belief_never_changes_truth() {
        let truth = KinematicState {
            position_world_m: Vec3::new(1.0, 2.0, 3.0),
            ..KinematicState::default()
        };
        let mut estimate = truth.clone();
        estimate.position_world_m.z = 40.0;
        let mut belief = None;
        let mut messages = Messages::default();
        let mut pending = Observations::default();
        let mut random = SensorRandom::new(123, 1, "test");
        let mut context = SensorContext {
            messages: &mut messages,
            entity: EntityInfo {
                id: 1,
                team_id: 1,
                sub_swarm_id: 0,
            },
            time: StepTime {
                time_s: 0.0,
                dt_s: 0.1,
            },
            truth: &truth,
            contacts_truth: &[],
            random: &mut random,
            belief: &mut belief,
            pending: &mut pending,
        };
        context.set_belief(estimate);
        assert_eq!(context.belief.as_ref().unwrap().position_world_m.z, 40.0);
        assert_eq!(context.truth.position_world_m.z, 3.0);
        context.clear_belief();
        assert!(context.belief.is_none());
        assert_eq!(truth.position_world_m.z, 3.0);
    }
}
