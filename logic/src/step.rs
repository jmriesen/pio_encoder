use crate::{CalibrationData, Direction};
use core::{
    num::Wrapping,
    ops::{Add, Range, Sub},
};
///An encoder step. (4 steps per encoder cycle)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Step(Wrapping<u32>);
/// 4 `Step` = 1 cycle = 255 `SubStep`s.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
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
    /// Returns the direction of other relative to self via the shortest path.
    /// Returns None if the values are the same.
    pub fn comp(&self, other: Self) -> Option<Direction> {
        use Direction as D;
        use core::cmp::Ordering as E;
        let delta = other.0 - (self.0);
        //Compare to u32's halfway point
        match delta.0.cmp(&(1u32 << 31)) {
            E::Equal => None,
            E::Less => Some(D::CounterClockwise),
            E::Greater => Some(D::Clockwise),
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

#[cfg(test)]
mod tests {
    use std::num::Wrapping;

    use super::{Step, SubStep};
    use crate::EQUAL_STEPS;

    #[test]
    fn check_substep_ranges() {
        assert_eq!(
            Step::new(-4).substep_range(&EQUAL_STEPS),
            SubStep::new(-256)..SubStep::new(-192)
        );
        assert_eq!(
            Step::new(-3).substep_range(&EQUAL_STEPS),
            SubStep::new(-192)..SubStep::new(-128)
        );
        assert_eq!(
            Step::new(-2).substep_range(&EQUAL_STEPS),
            SubStep::new(-128)..SubStep::new(-64)
        );
        assert_eq!(
            Step::new(-1).substep_range(&EQUAL_STEPS),
            SubStep::new(-64)..SubStep::new(0)
        );
        assert_eq!(
            Step::new(0).substep_range(&EQUAL_STEPS),
            SubStep::new(0)..SubStep::new(64)
        );
        assert_eq!(
            Step::new(1).substep_range(&EQUAL_STEPS),
            SubStep::new(64)..SubStep::new(128)
        );
        assert_eq!(
            Step::new(2).substep_range(&EQUAL_STEPS),
            SubStep::new(128)..SubStep::new(192)
        );
        assert_eq!(
            Step::new(3).substep_range(&EQUAL_STEPS),
            SubStep::new(192)..SubStep::new(256)
        );
        assert_eq!(
            Step::new(4).substep_range(&EQUAL_STEPS),
            SubStep::new(256)..SubStep::new(320)
        );
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

    #[test]
    fn substeps_role_over_before_steps_do() {
        for i in (32 - 6)..32 {
            dbg!(i);
            assert_eq!(
                Step(Wrapping(10 + 0)).lower_bound(&EQUAL_STEPS),
                Step(Wrapping(10 + (1 << i))).lower_bound(&EQUAL_STEPS),
            );
        }
    }
}
