//! Fourth-order Runge-Kutta with the legacy arithmetic order.

/// Advance coordinates by `dt_s`; derivatives must use the same coordinate order.
pub fn rk4<const N: usize>(
    coordinates: &mut [f64; N],
    dt_s: f64,
    derivative: impl Fn(&[f64; N]) -> [f64; N],
) {
    let k1 = derivative(coordinates);
    let k2 = derivative(&std::array::from_fn(|i| {
        coordinates[i] + dt_s * 0.5 * k1[i]
    }));
    let k3 = derivative(&std::array::from_fn(|i| {
        coordinates[i] + dt_s * 0.5 * k2[i]
    }));
    let k4 = derivative(&std::array::from_fn(|i| coordinates[i] + dt_s * k3[i]));

    // Boost odeint's scale_sum5 uses this summation order.
    for i in 0..N {
        coordinates[i] = coordinates[i]
            + (dt_s / 6.0) * k1[i]
            + (dt_s / 3.0) * k2[i]
            + (dt_s / 3.0) * k3[i]
            + (dt_s / 6.0) * k4[i];
    }
}
