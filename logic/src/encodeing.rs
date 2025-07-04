use std::{
    num::Wrapping,
    ops::{Add, Mul, Sub},
};

use crate::CalibrationData;

use super::Direction;
use embassy_time::{Duration, Instant};
/// The pio program always takes 13 clock cycles for each loop.
const LOOP_DURATION: u32 = 13;

///An encoder step. (4 steps per encoder cycle)
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Step(Wrapping<u32>);
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct SubStep(Wrapping<u32>);

impl Step {
    fn new(step: i32) -> Self {
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

    fn start_position(self, calibration: &CalibrationData) -> SubStep {
        //Extract the whole number of cycles
        let whole_cycles = self.0 / Wrapping(4);
        let partial_cycle = Wrapping(calibration[self.phase()] as u32);
        SubStep((whole_cycles << 8) + partial_cycle)
    }
    pub fn val(&self) -> i32 {
        self.0.0 as i32
    }
}

impl SubStep {
    pub fn new(step: i32) -> Self {
        Self(Wrapping(step as u32))
    }
    pub fn val(&self) -> i32 {
        self.0.0 as i32
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
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DirectionDuration(i32);
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
        //NOTE: num iterations is always positive.
        let cycles = iterations as u32 * LOOP_DURATION;
        let duration = Duration::from_micros((cycles / clocks_per_us).into());
        (direction, duration)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Mesurement {
    pub steps: Step,
    pub direction: Direction,
    pub step_time: embassy_time::Instant,
    pub sub_step_time: embassy_time::Instant,
}
impl Mesurement {
    pub fn new(
        dir_dur: DirectionDuration,
        steps: Step,
        instant: Instant,
        clocks_per_us: u32,
    ) -> Self {
        let (direction, duration) = dir_dur.decode(clocks_per_us);
        Self {
            steps,
            direction,
            step_time: instant - duration,
            sub_step_time: instant,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        EQUAL_STEPS,
        encodeing::{Mesurement, Step, SubStep},
    };

    use super::Direction;
    use embassy_time::{Duration, Instant};

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
    fn mesurment() {
        let time = Instant::from_secs(1);
        assert_eq!(
            Mesurement::new(DirectionDuration(0 - 50), Step::new(42), time, 10),
            Mesurement {
                steps: Step::new(42),
                direction: Direction::CounterClockwise,
                step_time: time - Duration::from_micros(65),
                sub_step_time: time
            }
        );
    }
    #[test]
    fn lower_upper_bounds() {
        assert_eq!(Step::new(-4).lower_bound(&EQUAL_STEPS), SubStep::new(-256));
        assert_eq!(Step::new(-3).lower_bound(&EQUAL_STEPS), SubStep::new(-192));
        assert_eq!(Step::new(-2).lower_bound(&EQUAL_STEPS), SubStep::new(-128));
        assert_eq!(Step::new(-1).lower_bound(&EQUAL_STEPS), SubStep::new(-64));
        assert_eq!(Step::new(0).lower_bound(&EQUAL_STEPS), SubStep::new(0));
        assert_eq!(Step::new(1).lower_bound(&EQUAL_STEPS), SubStep::new(64));
        assert_eq!(Step::new(2).lower_bound(&EQUAL_STEPS), SubStep::new(128));
        assert_eq!(Step::new(3).lower_bound(&EQUAL_STEPS), SubStep::new(192));
        assert_eq!(Step::new(4).lower_bound(&EQUAL_STEPS), SubStep::new(256));
    }
}
