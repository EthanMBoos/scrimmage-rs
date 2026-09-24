//! Legacy proto3 wire layout. Field names and numbers match the reference schemas.

use prost::Message;

#[derive(Clone, PartialEq, Message)]
pub(super) struct Vector3 {
    #[prost(double, tag = "1")]
    pub x: f64,
    #[prost(double, tag = "2")]
    pub y: f64,
    #[prost(double, tag = "3")]
    pub z: f64,
}

#[derive(Clone, PartialEq, Message)]
pub(super) struct Quaternion {
    #[prost(double, tag = "1")]
    pub w: f64,
    #[prost(double, tag = "2")]
    pub x: f64,
    #[prost(double, tag = "3")]
    pub y: f64,
    #[prost(double, tag = "4")]
    pub z: f64,
}

#[derive(Clone, PartialEq, Message)]
pub(super) struct State {
    #[prost(message, optional, tag = "1")]
    pub position: Option<Vector3>,
    #[prost(message, optional, tag = "2")]
    pub orientation: Option<Quaternion>,
    #[prost(message, optional, tag = "3")]
    pub linear_velocity: Option<Vector3>,
    #[prost(message, optional, tag = "4")]
    pub angular_velocity: Option<Vector3>,
}

#[derive(Clone, PartialEq, Message)]
pub(super) struct Id {
    #[prost(int32, tag = "1")]
    pub id: i32,
    #[prost(int32, tag = "2")]
    pub sub_swarm_id: i32,
    #[prost(int32, tag = "3")]
    pub team_id: i32,
}

#[derive(Clone, PartialEq, Message)]
pub(super) struct Contact {
    #[prost(message, optional, tag = "1")]
    pub id: Option<Id>,
    #[prost(message, optional, tag = "2")]
    pub state: Option<State>,
    #[prost(int32, tag = "3")]
    pub r#type: i32,
    #[prost(bool, tag = "4")]
    pub active: bool,
}

#[derive(Clone, PartialEq, Message)]
pub(super) struct Frame {
    #[prost(double, tag = "1")]
    pub time: f64,
    #[prost(message, repeated, tag = "2")]
    pub contact: Vec<Contact>,
}
