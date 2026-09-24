//! Motion implementations selected by mission XML.
#[path = "motion/simple_aircraft/simple_aircraft.rs"]
mod simple_aircraft;
pub use simple_aircraft::SimpleAircraft;
#[path = "motion/single_integrator/single_integrator.rs"]
mod single_integrator;
pub use single_integrator::SingleIntegrator;
