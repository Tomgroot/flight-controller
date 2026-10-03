use fc_core::{RcChannels, RC_MAX, RC_MIN};

/// Scripted transmitter: disarmed for 1 s, armed at idle until 2 s,
/// then ramps throttle to 50 % over 2 s and holds.
#[derive(Default)]
pub struct RcSource;

impl RcSource {
    pub fn sample(&self, t: f32) -> RcChannels {
        let mut rc = RcChannels::default();
        if t < 1.0 {
            return rc;
        }
        rc.0[4] = RC_MAX;
        let throttle = ((t - 2.0) / 2.0).clamp(0.0, 1.0) * 0.5;
        rc.0[2] = RC_MIN + (throttle * (RC_MAX - RC_MIN) as f32) as u16;
        rc
    }
}
