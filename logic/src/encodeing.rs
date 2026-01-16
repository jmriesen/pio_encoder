use super::Direction;
use crate::CalibrationData;
use core::{
    num::Wrapping,
    ops::{Add, Range, Sub},
};
use embassy_time::Duration;

/// The number of clock cycles it takes for the pio loop to for one iteration.
///
/// The pio program always takes 13 clock cycles for each loop.
const LOOP_DURATION: u32 = 13;

///An encoder step. (4 steps per encoder cycle)
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Step(Wrapping<u32>);
/// 4 `Step` = 1 cycle = 255 `SubStep`s.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct SubStep(Wrapping<u32>);

#[cfg(feature = "defmt")]
/// NOTE: I have created a pull request for wrapping to impl defmt::format.
/// It should be present in the next defmt release.
/// If so this impl can be defeated
mod defmt_impl {
    use super::*;
    #[mutants::skip]
    impl defmt::Format for Step {
        fn format(&self, fmt: defmt::Formatter) {
            self.0.0.format(fmt);
        }
    }
    #[mutants::skip]
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
        //Extract the whole number of cycles
        let whole_cycles = self.0 / Wrapping(4);
        let partial_cycle = Wrapping(u32::from(calibration[self.phase()]));
        SubStep((whole_cycles << 8) + partial_cycle)
    }
    pub fn upper_bound(self, calibration: &CalibrationData) -> SubStep {
        Self(self.0 + Wrapping(1)).lower_bound(calibration)
    }

    pub fn substep_range(&self, calibration: &CalibrationData) -> Range<SubStep> {
        Range {
            start: self.lower_bound(calibration),
            end: self.upper_bound(calibration),
        }
    }

    pub fn raw(&self) -> i32 {
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
    pub fn raw(&self) -> i32 {
        #[allow(
            clippy::cast_possible_wrap,
            reason = "Inverting cast done in constructor"
        )]
        {
            self.0.0 as i32
        }
    }
}

// TODO: adding role over will not be the same foe subsets and steps.
// Will this cause problems?
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
/// This encoding works by splitting the i32 in half.
/// - Counterclockwise range = [0, `i32::MIN`)
/// - Clockwise range = [`i32::MIN`,0)
/// When a tick is register the counter is reset to the top of its respective range (Clockwise or
/// Counterclockwise)
/// After every subsequent loop if a step was not detected we decrement the counter.
///
/// NOTE: the cycles counter **can** overflow (i32 are not infinite).
/// In that case the direction will flip and the duration will reset to zero.
/// This happens after about 3.5 minutes (assuming 125Mhz clock speed)
///
/// However, overflows will not cause any faulty readings.
/// The duration is not used in speed calculations if the encoder is in a stopped state.
/// The caller of the crate is responsible for repeatedly calling the update function,
/// (10hz at least)
/// so we will always be in a stopped state before an overflow could occur.
/// Represents the encoders current direction and how many PIO loop has run since the last step.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DirectionDuration(pub i32);

// The value that means 0 PIO loops since last step.
fn loop_count_start(direction: Direction) -> i32 {
    match direction {
        Direction::CounterClockwise => 0,
        Direction::Clockwise => i32::MIN,
    }
}

impl DirectionDuration {
    pub fn new(val: i32) -> Self {
        Self(val)
    }
    /// Split into direction and duration.
    pub fn decode(self, clock_ticks_per_us: u32) -> (Direction, Duration) {
        let direction = if self.0 < 0 {
            Direction::CounterClockwise
        } else {
            Direction::Clockwise
        };
        let iterations = loop_count_start(direction).wrapping_sub(self.0);
        let iterations = {
            #[expect(
                clippy::cast_sign_loss,
                reason = "sign infomation is used to store direction and has already been extracted"
            )]
            {
                iterations as u32
            }
        };

        // By the time we have hit u32::Max cycles the encoder should be in a stopped state.
        // So saturating here should not affect anything (aside from preventing an overflow).
        let cycles = (iterations).saturating_mul(LOOP_DURATION);
        let duration = Duration::from_micros((cycles / clock_ticks_per_us).into());
        (direction, duration)
    }
}

#[cfg(test)]
mod tests {
    use super::{Direction, DirectionDuration};
    use crate::{
        EQUAL_STEPS,
        encodeing::{LOOP_DURATION, Step, SubStep, loop_count_start},
    };
    use embassy_time::Duration;
    /// Do to how clockwise and counterclockwise ranges were defined the "start" values on there
    /// own correspond to -1 cycles
    /// Should never be an issue since pio program always increments the cycle count after setting
    /// one of the "start" values
    #[test]
    fn edge_case() {
        for direction in [Direction::Clockwise, Direction::CounterClockwise] {
            assert_eq!(
                DirectionDuration(loop_count_start(direction)).decode(1),
                (direction.invert(), Duration::from_micros(u32::MAX as u64))
            );
            assert_eq!(
                DirectionDuration(loop_count_start(direction).wrapping_sub(1)).decode(1),
                (direction, Duration::from_micros(u64::from(LOOP_DURATION)))
            );
        }
    }

    #[test]
    fn decode() {
        for direction in [Direction::Clockwise, Direction::CounterClockwise] {
            for ticks_per_ms in [1, 5, 10] {
                for cycles in [1, 5, 10] {
                    assert_eq!(
                        DirectionDuration(
                            loop_count_start(direction)
                                .wrapping_sub((cycles * ticks_per_ms) as i32)
                        )
                        .decode(ticks_per_ms),
                        (
                            direction,
                            Duration::from_micros(u64::from(cycles * LOOP_DURATION))
                        )
                    );
                }
            }
        }
    }

    #[test]
    fn lower_upper_bounds() {
        assert_eq!(
            Step::new(-4).substep_range(&EQUAL_STEPS),
            SubStep::new(-256)..SubStep::new(-192)
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
        assert_eq!(Step::new(-1).raw(), -1);
        assert_eq!(Step::new(0).raw(), 0);
        assert_eq!(Step::new(1).raw(), 1);
    }
    #[test]
    fn sub_step_arithmatic() {
        assert_eq!(SubStep::new(1) + SubStep::new(1), SubStep::new(2));
        assert_eq!(SubStep::new(1) - SubStep::new(1), SubStep::new(0));
    }
}
