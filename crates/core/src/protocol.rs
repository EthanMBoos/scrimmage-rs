//! Legacy protobuf frame compatibility and validated serialization.
mod frame;
mod wire;

pub use frame::{read_frames, write_frame};
