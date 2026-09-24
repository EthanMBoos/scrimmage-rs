//! Autonomy interface; concrete algorithms live in plugin/autonomy.
use crate::{
    common::PluginIo,
    plugin_manager::entity_plugin::{AgentContext, Plugin, Update},
};
use anyhow::Result;

pub trait Autonomy: Plugin {
    fn initialize(&mut self, _context: &mut AgentContext<'_>) -> Result<()> {
        Ok(())
    }
    fn step(&mut self, context: &mut AgentContext<'_>, io: &mut PluginIo) -> Result<Update>;
}
