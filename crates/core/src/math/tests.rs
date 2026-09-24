use super::*;
use std::f64::consts::{PI, TAU};

#[test]
fn positive_yaw_rotates_body_forward_toward_world_y() {
    let orientation = Quaternion::from_euler(EulerAngles {
        yaw_world_from_body_rad: PI / 2.0,
        ..EulerAngles::default()
    });
    let forward_world = orientation.rotate_body_to_world(Vec3::new(1.0, 0.0, 0.0));
    assert!(norm(forward_world - Vec3::new(0.0, 1.0, 0.0)) < 1e-12);
    assert!((orientation.yaw_world_from_body_rad() - PI / 2.0).abs() < 1e-12);
}

#[test]
fn angle_wrapping_preserves_reference_positive_pi_boundary() {
    assert_eq!(angle_pi(-PI), PI);
    assert!((angle_2pi(-0.1) - (TAU - 0.1)).abs() < 1e-12);
}

#[test]
fn rk4_integrates_constant_velocity() {
    let mut position_m = [0.0];
    rk4(&mut position_m, 0.1, |_| [21.0]);
    assert!((position_m[0] - 2.1).abs() < 1e-12);
}
