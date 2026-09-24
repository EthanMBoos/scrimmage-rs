//! Apple libc++ minstd_rand and generate_canonical<double,53> compatibility.
//! All draws are coordinator-owned, never scheduled on worker threads.
#[derive(Clone, Debug)]
pub struct LegacyRng {
    state: u64,
}
impl LegacyRng {
    pub fn new(seed: u32) -> Self {
        let state = seed as u64 % 2_147_483_647;
        Self {
            state: if state == 0 { 1 } else { state },
        }
    }
    pub fn next_u32(&mut self) -> u32 {
        self.state = self.state * 48_271 % 2_147_483_647;
        self.state as u32
    }
    pub fn canonical(&mut self) -> f64 {
        let r = 2_147_483_646.0;
        let lo = (self.next_u32() - 1) as f64;
        let hi = (self.next_u32() - 1) as f64;
        (lo + hi * r) / (r * r)
    }
    pub fn uniform(&mut self, low: f64, high: f64) -> f64 {
        (high - low) * self.canonical() + low
    }
    /// A temporary normal_distribution discards its cached second variate.
    pub fn normal(&mut self, mean: f64, sigma: f64) -> f64 {
        Normal::default().sample(self, mean, sigma)
    }
}
#[derive(Default)]
pub struct Normal {
    cached: Option<f64>,
}
impl Normal {
    pub fn sample(&mut self, rng: &mut LegacyRng, mean: f64, sigma: f64) -> f64 {
        let value = if let Some(v) = self.cached.take() {
            v
        } else {
            loop {
                let u = rng.uniform(-1.0, 1.0);
                let v = rng.uniform(-1.0, 1.0);
                let s = u * u + v * v;
                if s > 1.0 || s == 0.0 {
                    continue;
                }
                let f = (-2.0 * s.ln() / s).sqrt();
                self.cached = Some(v * f);
                break u * f;
            }
        };
        value * sigma + mean
    }
}
