#[path = "network/global_network/global_network.rs"]
mod global_network;
#[path = "network/local_network/local_network.rs"]
mod local_network;
pub use global_network::GlobalNetwork;
pub use local_network::LocalNetwork;
#[path = "network/sphere_network/sphere_network.rs"]
mod sphere_network;
pub use sphere_network::SphereNetwork;
