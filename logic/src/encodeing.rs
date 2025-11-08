use super::Direction;
use crate::CalibrationData;
use core::{
    num::Wrapping,
    ops::{Add, Sub},
};

use embassy_time::Duration;
/// The pio program always takes 13 clock cycles for each loop.
const LOOP_DURATION: u32 = 13;

///An encoder step. (4 steps per encoder cycle)
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Step(Wrapping<u32>);
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct SubStep(Wrapping<u32>);

#[cfg(feature = "defmt")]
mod defmt_impl {
    use super::*;
    impl defmt::Format for Step {
        fn format(&self, fmt: defmt::Formatter) {
            self.0.0.format(fmt);
        }
    }
    impl defmt::Format for SubStep {
        fn format(&self, fmt: defmt::Formatter) {
            self.0.0.format(fmt);
        }
    }
}

impl Step {
    pub fn new(step: i32) -> Self {
        #[allow(
            clippy::cast_sign_loss,
            reason = "we are casting to a u32 specifly to take advantage of the wrapping behavior. This casting is inverted before the raw value ever leaves this modual"
        )]
        Self(Wrapping(step as u32))
    }
    fn phase(self) -> usize {
        //Get raw steps remainder when divided by 4
        (self.0.0 & 3) as usize
    }
    pub fn lower_bound(self, calibration: &CalibrationData) -> SubStep {
        self.start_position(calibration)
    }
    pub fn upper_bound(self, calibration: &CalibrationData) -> SubStep {
        Self(self.0 + Wrapping(1)).start_position(calibration)
    }
    //returns both (lower_bound,upper_bound)
    pub fn bounds(&self, calibration: &CalibrationData) -> (SubStep, SubStep) {
        (self.lower_bound(calibration), self.upper_bound(calibration))
    }

    fn start_position(self, calibration: &CalibrationData) -> SubStep {
        //Extract the whole number of cycles
        let whole_cycles = self.0 / Wrapping(4);
        let partial_cycle = Wrapping(u32::from(calibration[self.phase()]));
        SubStep((whole_cycles << 8) + partial_cycle)
    }
    pub fn val(&self) -> i32 {
        #[allow(
            clippy::cast_possible_wrap,
            reason = "Inverting cast done in constructor"
        )]
        {
            self.0.0 as i32
        }
    }
}

impl SubStep {
    pub fn new(step: i32) -> Self {
        #[allow(
            clippy::cast_sign_loss,
            reason = "we are casting to a u32 specifly to take advantage of the wrapping behavior. This casting is inverted before the raw value ever leaves this modual"
        )]
        Self(Wrapping(step as u32))
    }
    pub fn val(&self) -> i32 {
        #[allow(
            clippy::cast_possible_wrap,
            reason = "Inverting cast done in constructor"
        )]
        {
            self.0.0 as i32
        }
    }
}

impl Sub for SubStep {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}
impl Add for SubStep {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

/// Contains the direction of the last encoder tick and how long ago that happened.
///
/// ```md
/// let C = cycles since last encoder tick;
/// If moving clockwise value = 0 - C.
/// If moving counterclockwise value = 2^31 - C .
///
/// NOTE: the cycles counter **can** overflow (i32 are not infinite).
/// In that case the direction will flip and the duration will reset to zero.
/// This happens afer about 3.5 miniutes (assuming 125Mhz clock speed)
///
/// However, overflows will not cause any faulty readings.
/// The duration is not used in speed calculations if the encoder is in a stoped state.
/// The caller of the crate is repsocible for repeatedly callling the update function
/// (10hz at least)
/// So we will always be in a stoped state before an overflow could occur.
/// stae.
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DirectionDuration(pub i32);
impl DirectionDuration {
    pub fn new(val: i32) -> Self {
        Self(val)
    }
    pub fn decode(self, clocks_per_us: u32) -> (Direction, Duration) {
        let (iterations, direction) = if self.0 < 0 {
            (0_i32.wrapping_sub(self.0), Direction::CounterClockwise)
        } else {
            (i32::MIN.wrapping_sub(self.0), Direction::Clockwise)
        };
        let iterations = {
            #[expect(
                clippy::cast_sign_loss,
                reason = "sign infomation is used to store direction and has allrady been extracted"
            )]
            {
                iterations as u32
            }
        };

        // By the time we have hit u32::Max cycles the encoder should be in a stopped state.
        // So saturating here should not affect anything (aside from preventing an overflow).
        let cycles = (iterations).saturating_mul(LOOP_DURATION);
        let duration = Duration::from_micros((cycles / clocks_per_us).into());
        (direction, duration)
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        EQUAL_STEPS,
        encodeing::{Step, SubStep},
    };

    use super::Direction;
    use embassy_time::Duration;

    use super::DirectionDuration;

    #[test]
    fn incrementing() {
        assert_eq!(
            DirectionDuration(0 - 50).decode(10),
            (Direction::CounterClockwise, Duration::from_micros(65))
        );
    }
    #[test]
    fn decrimenting() {
        assert_eq!(
            DirectionDuration(((1u32 << 31) - 50) as i32).decode(10),
            (Direction::Clockwise, Duration::from_micros(65))
        );
    }

    #[test]
    fn lower_upper_bounds() {
        assert_eq!(
            Step::new(-4).bounds(&EQUAL_STEPS),
            (SubStep::new(-256), SubStep::new(-192))
        );
        assert_eq!(Step::new(-3).lower_bound(&EQUAL_STEPS), SubStep::new(-192));
        assert_eq!(Step::new(-2).lower_bound(&EQUAL_STEPS), SubStep::new(-128));
        assert_eq!(Step::new(-1).lower_bound(&EQUAL_STEPS), SubStep::new(-64));
        assert_eq!(Step::new(0).lower_bound(&EQUAL_STEPS), SubStep::new(0));
        assert_eq!(Step::new(1).lower_bound(&EQUAL_STEPS), SubStep::new(64));
        assert_eq!(Step::new(2).lower_bound(&EQUAL_STEPS), SubStep::new(128));
        assert_eq!(Step::new(3).lower_bound(&EQUAL_STEPS), SubStep::new(192));
        assert_eq!(Step::new(4).lower_bound(&EQUAL_STEPS), SubStep::new(256));
    }

    #[test]
    fn into_i32() {
        //This test is here to confirm we can convert between our internal representation and
        //external representation
        assert_eq!(Step::new(-1).val(), -1);
        assert_eq!(Step::new(0).val(), 0);
        assert_eq!(Step::new(1).val(), 1);
    }
    #[test]
    fn sub_step_arithmatic() {
        assert_eq!(SubStep::new(1) + SubStep::new(1), SubStep::new(2));
        assert_eq!(SubStep::new(1) - SubStep::new(1), SubStep::new(0));
    }
}
