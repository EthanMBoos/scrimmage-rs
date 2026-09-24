//! Motion-model interface; concrete dynamics live in plugin/motion.
use crate::{
    KinematicState,
    common::PluginIo,
    plugin_manager::entity_plugin::{EntityInfo, Plugin, StepTime, Update},
    pubsub::Messages,
};
use anyhow::Result;

pub struct MotionContext<'a> {
    pub messages: &'a mut Messages,
    pub entity: EntityInfo,
    pub time: StepTime,
    pub truth: &'a mut KinematicState,
}

pub trait MotionModel: Plugin {
    fn initialize(&mut self, _context: &mut MotionContext<'_>) -> Result<()> {
        Ok(())
    }
    fn step(&mut self, context: &mut MotionContext<'_>, io: &mut PluginIo) -> Result<Update>;
}
