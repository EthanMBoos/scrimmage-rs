//! Fixed-distance collision detection, matching the C++ SimpleCollision plugin.
//!
//! C++ counterpart: src/plugins/interaction/SimpleCollision/SimpleCollision.cpp.
//! Startup checks candidate positions; the interaction phase marks collisions and emits events.

use anyhow::{Result, ensure};

use crate::plugin::{Interaction, InteractionContext, Plugin, PluginParams, Update};
use crate::{Entity, EventKind, Vec3};

#[derive(Clone)]
pub struct CollisionConfig {
    collision_range_m: f64,
    startup_collision_range_m: f64,
    startup_only: bool,
    same_team_enabled: bool,
    opposing_team_enabled: bool,
    altitude_deconfliction: bool,
}

pub struct SimpleCollision {
    config: CollisionConfig,
}

impl Plugin for SimpleCollision {
    type Config = CollisionConfig;

    fn configure(params: &PluginParams<'_>) -> Result<CollisionConfig> {
        let collision_range_m = params.number("collision_range", 0.0)?;
        let startup_collision_range_m =
            params.number("startup_collision_range", collision_range_m)?;
        ensure!(
            collision_range_m >= 0.0 && startup_collision_range_m >= 0.0,
            "collision ranges must be nonnegative"
        );
        Ok(CollisionConfig {
            collision_range_m,
            startup_collision_range_m,
            startup_only: params.boolean("startup_collisions_only", false)?,
            same_team_enabled: params.boolean("enable_team_collisions", true)?,
            opposing_team_enabled: params.boolean("enable_non_team_collisions", true)?,
            altitude_deconfliction: params.boolean("init_alt_deconflict", false)?,
        })
    }

    fn new(config: &CollisionConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }
}

impl Interaction for SimpleCollision {
    fn collision_exists(&self, entities: &[Entity], position_world_m: Vec3) -> bool {
        entities.iter().any(|entity| {
            let separation_world_m = position_world_m - entity.truth().position_world_m;
            if self.config.altitude_deconfliction {
                separation_world_m.z.abs() <= self.config.startup_collision_range_m
            } else {
                separation_world_m.norm() <= self.config.startup_collision_range_m
            }
        })
    }

    fn step(&mut self, context: &mut InteractionContext<'_>) -> Result<Update> {
        if self.config.startup_only {
            return Ok(Update::Applied);
        }

        for first_index in 0..context.entities.len() {
            for second_index in first_index + 1..context.entities.len() {
                // Split the slice so each contact is borrowed mutably and exclusively.
                // Pair order is observable when one collision disables later pairs.
                let (preceding, following) = context.entities.split_at_mut(second_index);
                let first = &mut preceding[first_index];
                let second = &mut following[0];
                if first.health() <= 0 || second.health() <= 0 {
                    continue;
                }

                let same_team = first.team_id() == second.team_id();
                let enabled = if same_team {
                    self.config.same_team_enabled
                } else {
                    self.config.opposing_team_enabled
                };
                let separation_m =
                    (first.truth().position_world_m - second.truth().position_world_m).norm();
                if !enabled || separation_m >= self.config.collision_range_m {
                    continue;
                }

                first.set_health(0);
                second.set_health(0);
                let ids = vec![first.id(), second.id()];
                let kind = if same_team {
                    EventKind::TeamCollision
                } else {
                    EventKind::NonTeamCollision
                };
                context.emit(kind, ids)?;
            }
        }
        Ok(Update::Applied)
    }
}
