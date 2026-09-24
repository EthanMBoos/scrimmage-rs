//! Legacy protobuf I/O. Decode and validate once before exposing domain state.

use std::{collections::BTreeSet, fs, io::Write, path::Path};

use anyhow::{Context, Result, bail, ensure};
use prost::Message;

use crate::{
    entity::{EntityKind, EntitySnapshot},
    math::{KinematicState, Quaternion, Vec3},
    simcontrol::SimulationFrame,
};

use super::wire;

pub fn read_frames(path: &Path) -> Result<Vec<SimulationFrame>> {
    let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    let mut remaining = bytes.as_slice();
    let mut frames = Vec::new();

    while !remaining.is_empty() {
        let wire_frame = wire::Frame::decode_length_delimited(&mut remaining)
            .context("invalid legacy frame encoding")?;
        frames.push(SimulationFrame::try_from(wire_frame)?);
    }
    Ok(frames)
}

pub fn write_frame(writer: &mut impl Write, frame: &SimulationFrame) -> Result<()> {
    let wire_frame = wire::Frame::from(frame);
    let mut bytes = Vec::with_capacity(wire_frame.encoded_len() + 10);
    wire_frame.encode_length_delimited(&mut bytes)?;
    writer.write_all(&bytes)?;
    Ok(())
}

impl From<Vec3> for wire::Vector3 {
    fn from(vector: Vec3) -> Self {
        Self {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        }
    }
}

impl From<&SimulationFrame> for wire::Frame {
    fn from(frame: &SimulationFrame) -> Self {
        let mut contact = Vec::with_capacity(frame.entities.len());
        for entity in &frame.entities {
            let truth = &entity.truth;
            let orientation = truth.orientation_world_from_body;
            let contact_type = match entity.kind {
                EntityKind::Aircraft => 0,
                EntityKind::Quadrotor => 1,
                EntityKind::Sphere => 2,
                EntityKind::Mesh => 3,
                EntityKind::Unknown => 4,
            };
            contact.push(wire::Contact {
                id: Some(wire::Id {
                    id: entity.id,
                    sub_swarm_id: entity.sub_swarm_id,
                    team_id: entity.team_id,
                }),
                state: Some(wire::State {
                    position: Some(truth.position_world_m.into()),
                    orientation: Some(wire::Quaternion {
                        w: orientation.w,
                        x: orientation.x,
                        y: orientation.y,
                        z: orientation.z,
                    }),
                    linear_velocity: Some(truth.velocity_world_mps.into()),
                    angular_velocity: Some(truth.angular_velocity_body_radps.into()),
                }),
                r#type: contact_type,
                active: entity.active,
            });
        }
        Self {
            time: frame.time_s,
            contact,
        }
    }
}

fn decode_vector(value: Option<wire::Vector3>, field: &str) -> Result<Vec3> {
    let vector = value.with_context(|| format!("missing {field}"))?;
    let vector = Vec3::new(vector.x, vector.y, vector.z);
    ensure!(
        vector.iter().all(|value| value.is_finite()),
        "nonfinite {field}"
    );
    Ok(vector)
}

impl TryFrom<wire::Frame> for SimulationFrame {
    type Error = anyhow::Error;

    fn try_from(frame: wire::Frame) -> Result<Self> {
        ensure!(frame.time.is_finite(), "nonfinite frame timestamp");
        let mut entities = Vec::with_capacity(frame.contact.len());
        let mut ids = BTreeSet::new();

        for contact in frame.contact {
            let id = contact.id.context("missing contact ID")?;
            ensure!(ids.insert(id.id), "duplicate entity ID {}", id.id);
            let state = contact.state.context("missing contact state")?;
            let orientation = state.orientation.context("missing orientation")?;
            let coefficients = [orientation.w, orientation.x, orientation.y, orientation.z];
            ensure!(
                coefficients.iter().all(|value| value.is_finite()),
                "nonfinite orientation"
            );
            let norm_squared: f64 = coefficients.iter().map(|value| value * value).sum();
            ensure!(
                (norm_squared - 1.0).abs() < 1e-8,
                "orientation is not a unit quaternion"
            );

            let kind = match contact.r#type {
                0 => EntityKind::Aircraft,
                1 => EntityKind::Quadrotor,
                2 => EntityKind::Sphere,
                3 => EntityKind::Mesh,
                4 => EntityKind::Unknown,
                invalid => bail!("invalid contact type {invalid}"),
            };
            entities.push(EntitySnapshot {
                id: id.id,
                sub_swarm_id: id.sub_swarm_id,
                team_id: id.team_id,
                truth: KinematicState {
                    position_world_m: decode_vector(state.position, "position")?,
                    orientation_world_from_body: Quaternion {
                        w: orientation.w,
                        x: orientation.x,
                        y: orientation.y,
                        z: orientation.z,
                    },
                    velocity_world_mps: decode_vector(state.linear_velocity, "linear velocity")?,
                    angular_velocity_body_radps: decode_vector(
                        state.angular_velocity,
                        "angular velocity",
                    )?,
                },
                kind,
                active: contact.active,
            });
        }
        Ok(Self {
            time_s: frame.time,
            entities,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_contact_state_is_rejected_at_the_input_boundary() {
        let frame = wire::Frame {
            time: 0.1,
            contact: vec![wire::Contact {
                id: Some(wire::Id {
                    id: 1,
                    sub_swarm_id: 0,
                    team_id: 1,
                }),
                ..wire::Contact::default()
            }],
        };
        let error = SimulationFrame::try_from(frame).unwrap_err();
        assert!(error.to_string().contains("missing contact state"));
    }
}
