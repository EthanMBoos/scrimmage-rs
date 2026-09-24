//! Controller implementations selected by mission XML.
#[path = "controller/simple_aircraft_pid/simple_aircraft_pid.rs"]
mod simple_aircraft_pid;
pub use simple_aircraft_pid::SimpleAircraftControllerPid;
#[path = "controller/single_integrator_simple/single_integrator_simple.rs"]
mod single_integrator_simple;
pub use single_integrator_simple::SingleIntegratorControllerSimple;
