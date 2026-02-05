//! This crate contains all the logic assisted with parsing pio messages and calculating speed.
//! This crate specificity does **not** depend on embassy-rs.
//! Depending on embassy-rs would prevent me from running the unit test on my base machine.
//#![cfg_attr(not(test), no_std)]
#![warn(clippy::pedantic)]
#![allow(clippy::must_use_candidate)]
use core::ops::Index;

use embassy_time::Duration;
pub mod encodeing;
mod speed;
pub use speed::Speed;
mod measurement;
pub use encodeing::DirectionDuration;
pub use measurement::Measurement;
mod step;
use crate::calibration::EQUAL_STEPS;
pub use step::{Step, SubStep};
mod calibration;

#[cfg(test)]
mod mock;
mod runner;

pub trait EncoderStateMachine {
    /// Returns whatever data is currently stored in the PIO State Machine.
    /// Since this reflects real world data, repeated calls to read ***CAN RESULT IN Different
    /// VALUES***
    fn read(&self) -> Measurement;
}

//Calibration data is really a mapping from phase to subsets
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct CalibrationData([SubStep; 4]);
impl Index<usize> for CalibrationData {
    type Output = SubStep;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}
impl Direction {
    pub fn invert(&self) -> Self {
        match self {
            Direction::Clockwise => Direction::CounterClockwise,
            Direction::CounterClockwise => Direction::Clockwise,
        }
    }
}

/// A speed encoder
///
/// This trait exists as a seam so that a mock encoder can be injected when unit testing application
/// code.
pub trait Encoder {
    // Update is used by the encoder to update its internal state.
    // It should be called regularly.
    fn update(&mut self);
    fn speed(&self) -> Speed;
    fn position(&self) -> SubStep;
    fn ticks(&self) -> Step;
}
