//! Typed publications, independent subscriber queues, and network delivery.
pub(crate) mod messages;
mod network;

pub use network::Network;

pub use messages::{Delivery, Message, MessageEndpoint, Messages, NetworkContext, Transmission};
