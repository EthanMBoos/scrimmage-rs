//! Same-entity communication, not distance-limited communication.
//!
//! C++ counterpart: src/plugins/network/LocalNetwork/LocalNetwork.cpp.
//! Network phase: inspect endpoint owners, route same-entity messages immediately.

use anyhow::{Result, ensure};

use crate::plugin::{Delivery, Network, NetworkContext, Plugin, PluginParams, Update};

pub struct LocalNetwork;

impl Plugin for LocalNetwork {
    type Config = ();

    fn configure(params: &PluginParams<'_>) -> Result<()> {
        ensure!(
            !params.boolean("is_stochastic_delay", false)?,
            "stochastic delay is not implemented by LocalNetwork"
        );
        // C++ uses a negative default to mean no communication delay.
        let delay_s = params.number("comm_delay", -1.0)?;
        ensure!(
            delay_s < 0.0,
            "legacy queued comm_delay is not yet ported; use a custom Network for explicit delay semantics"
        );
        Ok(())
    }

    fn new(_config: &()) -> Self {
        Self
    }
}

impl Network for LocalNetwork {
    fn step(&mut self, context: &mut NetworkContext<'_, '_>) -> Result<Update> {
        context.route(|link| {
            let Some(sender_entity_id) = link.sender.entity_id else {
                return Ok(Delivery::Drop);
            };
            if link.receiver.entity_id != Some(sender_entity_id) {
                return Ok(Delivery::Drop);
            }
            Ok(Delivery::After { delay_s: 0.0 })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{LocalNetwork, Plugin, PluginParams};
    use crate::Params;

    #[test]
    fn negative_legacy_delay_is_accepted() -> anyhow::Result<()> {
        let params = Params::from([("comm_delay".into(), "-2".into())]);
        LocalNetwork::configure(&PluginParams(&params))?;
        Ok(())
    }

    #[test]
    fn unsupported_delay_modes_remain_errors() {
        for (key, value) in [
            ("comm_delay", "0"),
            ("comm_delay", "1"),
            ("is_stochastic_delay", "true"),
        ] {
            let params = Params::from([(key.into(), value.into())]);
            assert!(LocalNetwork::configure(&PluginParams(&params)).is_err());
        }
    }
}
