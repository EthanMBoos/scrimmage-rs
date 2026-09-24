//! Plugin registration and typed configuration factories.
mod builtins;
pub(crate) mod entity_plugin;
mod registry;

pub use registry::PluginRegistry;
pub(crate) use registry::{Catalog, CompiledPlugin};
#[cfg(test)]
mod supported_options;
