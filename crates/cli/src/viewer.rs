//! Rerun observes the run; playback and debug panels never change simulation state.
use anyhow::{Result, ensure};
use rerun::{RecordingStream, RecordingStreamBuilder};
use scrimmage_core::SimulationFrame;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
    time::Duration,
};

mod blueprint;
mod map;

struct Track {
    positions_world_m: Vec<[f32; 3]>,
    color: [u8; 3],
}

pub(crate) struct Viewer {
    recording: RecordingStream,
    previous_ids: BTreeSet<i32>,
    tracks: BTreeMap<i32, Track>,
    first_time_s: Option<f64>,
    last_time_ns: Option<i64>,
}
impl Viewer {
    pub fn new(path: &Path, launch: bool) -> Result<Self> {
        ensure!(
            !path.exists(),
            "Rerun output already exists: {}",
            path.display()
        );
        let mut sinks: Vec<Box<dyn rerun::sink::LogSink>> =
            vec![Box::new(rerun::sink::FileSink::new(path)?)];
        if launch {
            let viewer = rerun::spawn(&rerun::SpawnOptions {
                new: true,
                hide_welcome_screen: true,
                ..Default::default()
            })?;
            let uri = format!("rerun+http://127.0.0.1:{}/proxy", viewer.port).parse()?;
            sinks.push(Box::new(rerun::sink::GrpcSink::new(uri)));
        }
        let recording = RecordingStreamBuilder::new("scrimmage-rs")
            .enabled(true)
            .set_sinks(sinks)?;
        recording.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;
        blueprint::send(&recording)?;
        Ok(Self {
            recording,
            previous_ids: BTreeSet::new(),
            tracks: BTreeMap::new(),
            first_time_s: None,
            last_time_ns: None,
        })
    }

    pub fn log_frame(&mut self, index: usize, frame: &SimulationFrame) -> Result<()> {
        let first_time_s = *self.first_time_s.get_or_insert(frame.time_s);
        let time_ns = playback_time_ns(frame.time_s - first_time_s, self.last_time_ns);
        self.last_time_ns = Some(time_ns);
        self.recording
            .set_time_sequence("frame", i64::try_from(index)?);
        self.recording
            .set_duration_secs("time", time_ns as f64 * 1e-9);

        let current_ids: BTreeSet<_> = frame.entities.iter().map(|entity| entity.id).collect();
        for id in self.previous_ids.difference(&current_ids) {
            self.recording
                .log(format!("world/entities/{id}"), &rerun::Clear::recursive())?;
            self.recording
                .log(format!("map/entities/{id}"), &rerun::Clear::recursive())?;
        }
        for entity in &frame.entities {
            let path = format!("world/entities/{}", entity.id);
            let state = &entity.truth;
            let position = state.position_world_m;
            let orientation = state.orientation_world_from_body;
            let color = team_color(entity.team_id);
            let label = format!("{} · team {}", entity.id, entity.team_id);
            let point = [position.x as f32, position.y as f32, position.z as f32];
            self.tracks
                .entry(entity.id)
                .or_insert_with(|| Track {
                    positions_world_m: Vec::new(),
                    color,
                })
                .positions_world_m
                .push(point);

            self.recording.log(
                path.as_str(),
                &rerun::Transform3D::from_translation_rotation(
                    point,
                    rerun::Quaternion::from_wxyz([
                        orientation.w as f32,
                        orientation.x as f32,
                        orientation.y as f32,
                        orientation.z as f32,
                    ]),
                ),
            )?;
            self.recording.log(
                path.as_str(),
                &rerun::Points3D::new([[0.0, 0.0, 0.0]])
                    .with_colors([color])
                    .with_radii([rerun::Radius::new_ui_points(6.0)])
                    .with_labels([label.clone()]),
            )?;
            self.recording.log(
                format!("{path}/heading"),
                &rerun::Arrows3D::from_vectors([[8.0, 0.0, 0.0]]).with_colors([color]),
            )?;

            let map_path = format!("map/entities/{}", entity.id);
            let position_map_m = map::project(point);
            let heading_rad = orientation.yaw_world_from_body_rad();
            self.recording.log(
                map_path.as_str(),
                &rerun::Points2D::new([position_map_m])
                    .with_colors([color])
                    .with_radii([rerun::Radius::new_ui_points(6.0)])
                    .with_labels([label.clone()]),
            )?;
            self.recording.log(
                format!("{map_path}/heading"),
                &rerun::Arrows2D::from_vectors([[
                    8.0 * heading_rad.cos() as f32,
                    -8.0 * heading_rad.sin() as f32,
                ]])
                .with_origins([position_map_m])
                .with_colors([color]),
            )?;
            for (quantity, value) in [
                ("speed_mps", state.velocity_world_mps.norm()),
                ("altitude_m", position.z),
            ] {
                let plot_path = format!("debug/{quantity}/{}", entity.id);
                if !self.previous_ids.contains(&entity.id) {
                    self.recording.log_static(
                        plot_path.as_str(),
                        &rerun::SeriesLines::new()
                            .with_colors([color])
                            .with_names([label.clone()]),
                    )?;
                }
                self.recording
                    .log(plot_path.as_str(), &rerun::Scalars::new([value]))?;
            }
        }
        self.recording.log(
            "dashboard/status",
            &rerun::TextDocument::from_markdown(status_text(index, frame, time_ns)),
        )?;
        self.previous_ids = current_ids;
        Ok(())
    }

    pub fn finish(self) -> Result<()> {
        map::log_grid(
            &self.recording,
            self.tracks
                .values()
                .flat_map(|track| track.positions_world_m.iter().copied()),
        )?;
        // Full-run paths keep the map fitted to every agent across the whole replay,
        // including late spawns and entities removed before the terminal frame.
        for (id, track) in &self.tracks {
            let color = [track.color[0], track.color[1], track.color[2], 100];
            let map_points: Vec<_> = track
                .positions_world_m
                .iter()
                .map(|point| map::project(*point))
                .collect();
            self.recording.log_static(
                format!("map/tracks/{id}"),
                &rerun::LineStrips2D::new([map_points])
                    .with_colors([color])
                    .with_radii([rerun::Radius::new_ui_points(1.0)]),
            )?;
            self.recording.log_static(
                format!("world/tracks/{id}"),
                &rerun::LineStrips3D::new([track.positions_world_m.clone()])
                    .with_colors([color])
                    .with_radii([rerun::Radius::new_ui_points(1.0)]),
            )?;
        }
        // Activate a fresh layout after all data arrives so initial framing includes the full run.
        blueprint::send(&self.recording)?;
        self.recording.flush_with_timeout(Duration::from_secs(10))?;
        Ok(())
    }
}

fn team_color(team_id: i32) -> [u8; 3] {
    match team_id.rem_euclid(3) {
        1 => [77, 140, 255],
        2 => [255, 90, 77],
        _ => [90, 220, 130],
    }
}

fn playback_time_ns(elapsed_s: f64, previous_ns: Option<i64>) -> i64 {
    let time_ns = (elapsed_s.max(0.0) * 1e9).round() as i64;
    // Legacy output repeats the final timestamp. Preserve both visual samples without
    // changing frame files or pretending sequence indices are seconds.
    previous_ns.map_or(time_ns, |previous| time_ns.max(previous.saturating_add(1)))
}

fn status_text(index: usize, frame: &SimulationFrame, replay_time_ns: i64) -> String {
    let teams: BTreeSet<_> = frame.entities.iter().map(|entity| entity.team_id).collect();
    let mut text = format!(
        "# Mission status\n\nReplay **{:.2} s** · frame **{index}**\n\nLegacy frame label **{:.2} s**\n\n**{} agents** · **{} teams**\n\n",
        replay_time_ns as f64 * 1e-9,
        frame.time_s,
        frame.entities.len(),
        teams.len()
    );
    text.push_str("Local ENU map: East → / North ↑, meters.\n\nMarkers = current agents; arrows = heading. Faint tracks = complete run (including future).\n\n");
    text.push_str("| ID | Team | Speed m/s | Alt. m |\n|---:|---:|---:|---:|\n");
    for entity in frame.entities.iter().take(12) {
        text.push_str(&format!(
            "| {} | {} | {:.1} | {:.1} |\n",
            entity.id,
            entity.team_id,
            entity.truth.velocity_world_mps.norm(),
            entity.truth.position_world_m.z
        ));
    }
    if frame.entities.len() > 12 {
        text.push_str(&format!(
            "\n{} additional agents shown on the map.\n",
            frame.entities.len() - 12
        ));
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn playback_uses_seconds_and_retains_the_repeated_terminal_sample() {
        assert_eq!(playback_time_ns(0.1, Some(0)), 100_000_000);
        assert_eq!(playback_time_ns(0.1, Some(100_000_000)), 100_000_001);
        assert_eq!(playback_time_ns(0.2, Some(100_000_001)), 200_000_000);
    }
}
