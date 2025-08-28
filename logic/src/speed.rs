use core::ops::Mul;

use embassy_time::Duration;

use crate::encodeing::SubStep;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Speed(i32);

///Internally stored as sub-steps per 2^20 micro seconds
impl Speed {
    pub fn new(delta: SubStep, duration: Duration) -> Self {
        let sub_steps = delta.val() as i64;
        let micro_seconds = duration.as_micros() as i64;
        let speed = (sub_steps << 20) / micro_seconds;
        Self(speed as i32)
    }
    pub fn stopped() -> Self {
        Self(0)
    }
    //TODO: test
    pub fn ticks_per_second(&self) -> i32 {
        ((self.0 as i64 * 62500i64) >> 16) as i32
    }
}
impl Mul<Duration> for Speed {
    type Output = SubStep;

    fn mul(self, rhs: Duration) -> Self::Output {
        SubStep::new(((self.0 as u64).wrapping_mul(rhs.as_micros()) >> 20) as i32)
    }
}
#[cfg(test)]
mod test {
    use super::Speed;
    use crate::encodeing::SubStep;
    use embassy_time::Duration;

    #[test]
    fn ticks_per_second() {
        let speed = Speed::new(SubStep::new(50), Duration::from_secs(1));
        //Note there is a bit of rounding.
        assert_eq!(speed.ticks_per_second(), 49);
    }
    #[test]
    fn negative_speed() {
        let speed = Speed::new(SubStep::new(-50), Duration::from_secs(1));
        assert_eq!(speed.ticks_per_second(), -50)
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
