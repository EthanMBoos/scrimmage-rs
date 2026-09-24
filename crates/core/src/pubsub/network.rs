//! Network interface; concrete communication policies live in plugin/network.
use super::NetworkContext;
use crate::plugin_manager::entity_plugin::{Plugin, Update, WorldContext};
use anyhow::Result;

pub trait Network: Plugin {
    fn initialize(&mut self, _context: &mut WorldContext<'_>) -> Result<()> {
        Ok(())
    }
    fn step(&mut self, context: &mut NetworkContext<'_, '_>) -> Result<Update>;
}
