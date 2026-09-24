//! Rust-native mission route publisher: one initial route and an optional timed replacement.
//! Interaction phase; ordinary typed GlobalNetwork messages, no services or spawn commands.

use anyhow::{Result, ensure};

use crate::common::{WAYPOINT_TOPIC, WaypointList};
use crate::plugin::{Interaction, InteractionContext, Plugin, PluginParams, Update};

pub struct WaypointBroadcastConfig {
    initial: WaypointList,
    replacement: Option<WaypointList>,
    update_at_s: f64,
}

pub struct WaypointBroadcast {
    initial: Option<WaypointList>,
    replacement: Option<WaypointList>,
    update_at_s: f64,
}

impl Plugin for WaypointBroadcast {
    type Config = WaypointBroadcastConfig;

    fn configure(params: &PluginParams<'_>) -> Result<Self::Config> {
        ensure!(
            params.text("update_waypoints").is_some() == params.text("update_at_s").is_some(),
            "provide both update_waypoints and update_at_s"
        );
        Ok(WaypointBroadcastConfig {
            initial: WaypointList::parse(params.text("waypoints").unwrap_or("0,0,200"))?,
            replacement: params
                .text("update_waypoints")
                .map(WaypointList::parse)
                .transpose()?,
            update_at_s: params.number("update_at_s", 0.0)?,
        })
    }

    fn new(config: &Self::Config) -> Self {
        Self {
            initial: Some(config.initial.clone()),
            replacement: config.replacement.clone(),
            update_at_s: config.update_at_s,
        }
    }
}

impl Interaction for WaypointBroadcast {
    fn step(&mut self, context: &mut InteractionContext<'_>) -> Result<Update> {
        // Known limitation (Rust-native, mirrors C++ Boundary's publish-once pattern):
        // entities spawned later never receive the route and keep their own configured one.
        // A possible fix is republishing the current route when an entity is generated.
        if let Some(route) = self.initial.take() {
            context
                .messages
                .publish("GlobalNetwork", WAYPOINT_TOPIC, route)?;
        }
        if context.time.time_s >= self.update_at_s
            && let Some(route) = self.replacement.take()
        {
            context
                .messages
                .publish("GlobalNetwork", WAYPOINT_TOPIC, route)?;
        }
        Ok(Update::Applied)
    }
}
