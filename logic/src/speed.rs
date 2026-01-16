use core::ops::Mul;

use embassy_time::Duration;

use crate::step::SubStep;

/// Encoder speed
/// Internally stored as sub-steps per 2^20 microseconds
///```rust
/// use pio_speed_encoder_logic::Speed;
/// use pio_speed_encoder_logic::SubStep;
/// use embassy_time::Duration;
/// let second = Duration::from_secs(1);
///
/// assert_eq!(Speed::max()*second,SubStep::new(2047999999));
/// assert_eq!(Speed::min()*second,SubStep::new(-2048000000));
///
/// //NOTE I am not sure where this off by one error is comeing from
/// assert_eq!(Speed::new(SubStep::new(2),second)*second, SubStep::new(1));
/// assert_eq!(Speed::new(SubStep::new(500),second)*second, SubStep::new(499));
///
/// let minimum_frequencey = Duration::from_micros(1 << 20);
/// let slowest_non_zero_speed = Speed::new(SubStep::new(1),minimum_frequencey);
/// assert_eq!(slowest_non_zero_speed* minimum_frequencey,SubStep::new(1));
///
/// let below_minimum_frequencey = minimum_frequencey -Duration::from_micros(1);
/// let stopped= Speed::new(SubStep::new(1),minimum_frequencey);
/// assert_eq!(stopped* below_minimum_frequencey, SubStep::new(0));
///```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Speed(i32);

fn clamp_cast(value: i64) -> i32 {
    value
        .clamp(i32::MIN.into(), i32::MAX.into())
        .try_into()
        .expect("Bounds checked by clamp")
}

impl Speed {
    ///Create a new speed reading.
    pub fn new(delta: SubStep, duration: Duration) -> Self {
        let sub_steps = i64::from(delta.raw());
        let micro_seconds = duration.as_micros();
        if micro_seconds > i64::MAX as u64 {
            //Division would round to zero
            Self::stopped()
        } else {
            let micro_seconds =
                i64::try_from(micro_seconds).expect("Allready checked that value will fit");
            let speed = (sub_steps << 20) / micro_seconds;
            Self(clamp_cast(speed))
        }
    }
    pub fn stopped() -> Self {
        Self(0)
    }

    ///Maximum speed that is possible to represent
    pub const fn max() -> Self {
        Speed(i32::MAX)
    }
    ///Maximum speed that is possible to represent
    pub const fn min() -> Self {
        Speed(i32::MIN)
    }
}

impl Mul<Duration> for Speed {
    type Output = SubStep;

    fn mul(self, rhs: Duration) -> Self::Output {
        #[expect(
            clippy::suspicious_arithmetic_impl,
            reason = "Internal storage is in units of 2^20 microseconds"
        )]
        //TODO: I don't like this wrapping multiply
        SubStep::new(((self.0 as u64).wrapping_mul(rhs.as_micros()) >> 20) as i32)
    }
}
#[cfg(test)]
mod test {
    use super::Speed;
    use crate::step::SubStep;
    use embassy_time::Duration;

    #[test]
    fn ticks_per_second() {
        let speed = Speed::new(SubStep::new(50), Duration::from_secs(1));
        //Note there is a bit of rounding.
        assert_eq!(speed * Duration::from_secs(1), SubStep::new(49));
    }
    #[test]
    fn negative_speed() {
        let speed = Speed::new(SubStep::new(-50), Duration::from_secs(1));
        assert_eq!(speed * Duration::from_secs(1), SubStep::new(-50));
    }
    #[test]
    fn multiplication() {
        let four_ticks_per_us = Speed::new(SubStep::new(4), Duration::from_micros(1));
        assert_eq!(
            four_ticks_per_us * Duration::from_micros(3),
            SubStep::new(12)
        );
        let neg_four_ticks_per_us = Speed::new(SubStep::new(-4), Duration::from_micros(1));
        assert_eq!(
            neg_four_ticks_per_us * Duration::from_micros(3),
            SubStep::new(-12)
        );
    }
}
