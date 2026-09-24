use anyhow::Result;
use clap::{Parser, Subcommand};
use std::path::PathBuf;
mod replay;
mod run;
mod viewer;

#[derive(Parser)]
#[command(
    name = "scrimmage",
    version,
    about = "SCRIMMAGE Rust port with Rerun visualization"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Run a mission, recording frames, metrics, and Rerun data.
    Run(run::RunOptions),
    /// Open a saved Rerun recording without rerunning the mission.
    Replay {
        /// Run directory containing recording.rrd, or the recording file itself.
        path: PathBuf,
    },
}
fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Run(options) => run::run(options)?,
        Command::Replay { path } => replay::replay(&path)?,
    }
    Ok(())
}
