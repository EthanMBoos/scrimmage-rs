#[path = "network/global_network/global_network.rs"]
mod global_network;
#[path = "network/local_network/local_network.rs"]
mod local_network;
pub use global_network::GlobalNetwork;
pub use local_network::LocalNetwork;
