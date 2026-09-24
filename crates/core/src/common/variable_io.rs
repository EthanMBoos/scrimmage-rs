use anyhow::{Context, Result, ensure};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Dimensionless,
    Meters,
    MetersPerSecond,
    MetersPerSecondSquared,
    Radians,
    RadiansPerSecond,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame {
    None,
    World,
    Body,
    Model,
}

/// Names are extensible; adding a channel does not require an engine enum change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Port {
    pub name: &'static str,
    pub unit: Unit,
    pub frame: Frame,
}
impl Port {
    pub const fn new(name: &'static str, unit: Unit, frame: Frame) -> Self {
        Self { name, unit, frame }
    }
}

#[derive(Clone, Default, Debug)]
pub struct Ports {
    pub(crate) inputs: Vec<Port>,
    pub(crate) outputs: Vec<Port>,
}
impl Ports {
    pub fn input(mut self, port: Port) -> Self {
        self.inputs.push(port);
        self
    }
    pub fn output(mut self, port: Port) -> Self {
        self.outputs.push(port);
        self
    }
    pub(crate) fn validate(&self) -> Result<()> {
        for ports in [&self.inputs, &self.outputs] {
            let mut names = std::collections::BTreeSet::new();
            for port in ports {
                ensure!(
                    !port.name.is_empty() && names.insert(port.name),
                    "empty or duplicate port {}",
                    port.name
                );
            }
        }
        Ok(())
    }
    pub(crate) fn connect(&self, upstream: &[Port], plugin_name: &str) -> Result<()> {
        for input in &self.inputs {
            let output = upstream
                .iter()
                .find(|port| port.name == input.name)
                .with_context(|| {
                    format!("{plugin_name}: missing upstream output '{}'", input.name)
                })?;
            ensure!(
                input.unit == output.unit && input.frame == output.frame,
                "{plugin_name}: port '{}' expects {:?}/{:?}, upstream provides {:?}/{:?}",
                input.name,
                input.unit,
                input.frame,
                output.unit,
                output.frame
            );
        }
        Ok(())
    }
}

pub(crate) type Signals = BTreeMap<&'static str, f64>;

/// Inputs and outputs are distinct, including when a controller reuses a channel name.
/// Values persist when a rate-limited plugin does not execute.
pub struct PluginIo {
    inputs: Signals,
    pub(crate) outputs: Signals,
}
impl PluginIo {
    pub(crate) fn new(ports: &Ports) -> Self {
        Self {
            inputs: ports.inputs.iter().map(|port| (port.name, 0.0)).collect(),
            outputs: ports.outputs.iter().map(|port| (port.name, 0.0)).collect(),
        }
    }
    pub fn read(&self, name: &str) -> Result<f64> {
        self.inputs
            .get(name)
            .copied()
            .with_context(|| format!("undeclared input '{name}'"))
    }
    pub fn write(&mut self, name: &str, value: f64) -> Result<()> {
        ensure!(value.is_finite(), "nonfinite output '{name}'");
        *self
            .outputs
            .get_mut(name)
            .with_context(|| format!("undeclared output '{name}'"))? = value;
        Ok(())
    }
    pub(crate) fn receive(&mut self, upstream: &Signals) -> Result<()> {
        for (name, value) in &mut self.inputs {
            *value = *upstream
                .get(name)
                .with_context(|| format!("missing connected input '{name}'"))?;
        }
        Ok(())
    }
}
