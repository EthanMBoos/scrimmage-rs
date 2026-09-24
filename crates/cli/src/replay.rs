//! Open an existing recording; replay never constructs or advances a simulation.

use std::{fs, path::Path, process::Command};

use anyhow::{Context, Result, ensure};

pub(crate) fn replay(path: &Path) -> Result<()> {
    let recording = if path.is_dir() {
        path.join("recording.rrd")
    } else {
        path.to_path_buf()
    };
    // An absolute path is also safe for filenames beginning with a dash.
    let recording = recording
        .canonicalize()
        .with_context(|| format!("cannot open recording {}", recording.display()))?;
    ensure!(
        fs::metadata(&recording)?.is_file(),
        "recording is not a file: {}",
        recording.display()
    );

    let status = Command::new("rerun").arg(&recording).status().context(
        "could not launch Rerun; install the matching viewer and ensure rerun is on PATH",
    )?;
    ensure!(status.success(), "Rerun viewer exited with {status}");
    Ok(())
}
