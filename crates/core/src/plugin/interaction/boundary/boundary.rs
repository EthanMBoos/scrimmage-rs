//! Publish an axis-aligned mission boundary; autonomies decide how to respond.
//! C++ counterpart: interaction/Boundary. Publishes once in the interaction phase.
//! Geometry is ordinary Rust data, independent of rendering and physical collision.

use anyhow::{Result, ensure};

use crate::Vec3;
use crate::plugin::{Interaction, InteractionContext, Plugin, PluginParams, Update};

pub const BOUNDARY_TOPIC: &str = "Boundary";

#[derive(Clone, Copy, Debug)]
pub struct BoundaryRegion {
    pub center_world_m: Vec3,
    pub lengths_m: Vec3,
}

impl BoundaryRegion {
    /// Faces are outside, matching the C++ cuboid's strict comparisons.
    pub fn contains(&self, position_world_m: Vec3) -> bool {
        let offset_m = position_world_m - self.center_world_m;
        for axis in 0..3 {
            if offset_m[axis].abs() >= self.lengths_m[axis] / 2.0 {
                return false;
            }
        }
        true
    }
}

pub struct BoundaryConfig {
    region: BoundaryRegion,
    network: String,
}

pub struct Boundary {
    region: BoundaryRegion,
    network: String,
    published: bool,
}

impl Plugin for Boundary {
    type Config = BoundaryConfig;

    fn configure(params: &PluginParams<'_>) -> Result<BoundaryConfig> {
        ensure!(
            params.text("type").unwrap_or("cuboid") == "cuboid",
            "Boundary supports cuboid only"
        );
        ensure!(
            params.vector("rpy", [0.0; 3])? == [0.0; 3],
            "Boundary supports axis-aligned cuboids only"
        );
        ensure!(
            !params.boolean("show_boundary", false)?,
            "Boundary drawing is not implemented; use show_boundary=false"
        );
        let lengths_m = Vec3::from(params.vector("lengths", [10.0; 3])?);
        ensure!(
            lengths_m.iter().all(|length| *length > 0.0),
            "Boundary lengths must be positive"
        );
        Ok(BoundaryConfig {
            region: BoundaryRegion {
                center_world_m: Vec3::from(params.vector("center", [0.0; 3])?),
                lengths_m,
            },
            network: params
                .text("network_name")
                .unwrap_or("GlobalNetwork")
                .to_owned(),
        })
    }

    fn new(config: &BoundaryConfig) -> Self {
        Self {
            region: config.region,
            network: config.network.clone(),
            published: false,
        }
    }
}

impl Interaction for Boundary {
    fn step(&mut self, context: &mut InteractionContext<'_>) -> Result<Update> {
        // Matches C++ for now, though we don't consider it ideal: publishing once means
        // entities spawned after the first tick never receive the boundary.
        if !self.published {
            context
                .messages
                .publish(&self.network, BOUNDARY_TOPIC, self.region)?;
            self.published = true;
        }
        Ok(Update::Applied)
    }
}

#[cfg(test)]
mod tests {
    use super::BoundaryRegion;
    use crate::Vec3;

    #[test]
    fn cuboid_contains_interior_but_not_its_faces() {
        let region = BoundaryRegion {
            center_world_m: Vec3::new(10.0, 20.0, 30.0),
            lengths_m: Vec3::new(2.0, 4.0, 6.0),
        };
        assert!(region.contains(region.center_world_m));
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut point = region.center_world_m;
                point[axis] += sign * region.lengths_m[axis] / 2.0;
                assert!(!region.contains(point));
            }
        }
    }
}
