//! Mission-controlled generation schedules.

use super::{Event, EventKind, Simulation};
use crate::parse::{EntityConfig, integer, number};
use crate::{common::random::Normal, entity::Entity, math::Vec3};
use anyhow::{Context, Result, ensure};

#[derive(Clone)]
pub(crate) struct Generator {
    pub(crate) definition_index: usize,
    heading_world_deg: f64,
    pub(crate) remaining: usize,
    pub(crate) next_spawn_times_s: Vec<f64>,
    pub(crate) rate_hz: f64,
    pub(crate) time_stddev_s: f64,
}

impl Simulation {
    pub(super) fn generate_entities(&mut self) -> Result<()> {
        let mut pending_definitions = Vec::new();
        for generator in &mut self.generators {
            for next_spawn_s in &mut generator.next_spawn_times_s {
                if self.time_s < *next_spawn_s || generator.remaining == 0 {
                    continue;
                }
                generator.remaining -= 1;
                pending_definitions.push(generator.definition_index);

                if generator.rate_hz > 0.0 {
                    let expected_next_spawn_s = self.time_s + 1.0 / generator.rate_hz;
                    *next_spawn_s = self
                        .rng
                        .normal(expected_next_spawn_s, generator.time_stddev_s);
                    if *next_spawn_s <= self.time_s {
                        *next_spawn_s = expected_next_spawn_s;
                    }
                }
            }
        }

        // Legacy compatibility requires draw order to follow source definition IDs.
        // Worker order never participates. New sensor noise will use explicit sensor identities.
        for definition_index in pending_definitions {
            let definition = &self.definitions[definition_index];
            let spawn = &definition.spawn;
            let origin_world_m = spawn.position_world_m;
            // C++ writes each sampled heading back into the entity template.
            // Keep that legacy evolution in generator state, not validated configuration.
            let heading_world_deg = legacy_decimal_roundtrip(self.rng.normal(
                self.generators[definition_index].heading_world_deg,
                spawn.heading_variance_deg2.sqrt(),
            ));
            self.generators[definition_index].heading_world_deg = heading_world_deg;
            let mut position_world_m = origin_world_m;

            if self.startup_collision_exists(position_world_m) || spawn.randomize_every_spawn {
                let mut x_normal = Normal::default();
                let mut y_normal = Normal::default();
                let mut z_normal = Normal::default();
                let position_stddev_world_m = spawn.position_variance_world_m2.map(f64::sqrt);
                let mut found_free_position = false;

                for _ in 0..1_000_000 {
                    position_world_m = Vec3::new(
                        x_normal.sample(&mut self.rng, origin_world_m.x, position_stddev_world_m.x),
                        y_normal.sample(&mut self.rng, origin_world_m.y, position_stddev_world_m.y),
                        z_normal.sample(&mut self.rng, origin_world_m.z, position_stddev_world_m.z),
                    );
                    if !self.startup_collision_exists(position_world_m) {
                        found_free_position = true;
                        break;
                    }
                }
                ensure!(
                    found_free_position,
                    "unable to find collision-free spawn position"
                );
            }

            let mut id = spawn.requested_id.unwrap_or(0);
            while !self.used_ids.insert(id) {
                id = self
                    .used_ids
                    .last()
                    .copied()
                    .unwrap_or(0)
                    .checked_add(1)
                    .context("entity ID exhausted")?;
            }

            let entity = Entity::new(
                id,
                i32::try_from(definition_index).context("entity block ID exceeds i32")?,
                definition,
                position_world_m.map(legacy_decimal_roundtrip),
                legacy_decimal_roundtrip(heading_world_deg),
                self.time_s,
                self.config.seed,
            )?;
            self.entity_teams.insert(id, entity.team_id);
            let event = Event {
                time_s: self.time_s,
                kind: EventKind::EntityGenerated,
                entity_ids: vec![id],
            };
            self.messages.time_s = self.time_s;
            self.messages
                .publish("GlobalNetwork", event.kind.topic(), event.clone())?;
            self.events.push(event);
            self.entities.push(entity);
        }
        Ok(())
    }
}

fn legacy_decimal_roundtrip(value: f64) -> f64 {
    // C++ sends spawn coordinates through std::to_string before parsing them again.
    format!("{value:.6}")
        .parse()
        .expect("formatted finite number must parse")
}
impl Generator {
    pub(crate) fn new(
        config: &EntityConfig,
        definition_index: usize,
        start_s: f64,
    ) -> Result<Self> {
        let params = &config.params;
        let count = integer(params, "count", 1)?;
        let mut rate_hz = -1.0;
        let mut batch = count;
        let mut first_spawn_s = start_s - 1.0;
        if let (Some(rate_text), Some(_)) =
            (params.get("generate_rate"), params.get("generate_count"))
        {
            let values: Vec<f64> = rate_text
                .split('/')
                .map(|s| s.trim().parse())
                .collect::<std::result::Result<_, _>>()?;
            let rate_value_hz = match values.as_slice() {
                [a, b] => a / b,
                [a] => *a,
                _ => anyhow::bail!("invalid generate_rate"),
            };
            let batch_size = integer(params, "generate_count", 1)?;
            if rate_value_hz > 0.0 && batch_size > 0 {
                rate_hz = rate_value_hz;
                batch = batch_size;
                first_spawn_s = number(params, "generate_start_time", 0.0)?;
            }
        }
        let time_stddev_s = number(params, "generate_time_variance", 0.0)?;
        ensure!(time_stddev_s >= 0.0, "negative generation variance");
        Ok(Self {
            definition_index,
            heading_world_deg: number(params, "heading", 0.0)?,
            remaining: count,
            next_spawn_times_s: vec![first_spawn_s; batch],
            rate_hz,
            time_stddev_s,
        })
    }
}
