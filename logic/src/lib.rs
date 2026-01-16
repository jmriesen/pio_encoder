//! This crate contains all the logic assisted with parsing pio messages and calculating speed.
//! This crate specificity does **not** depend on embassy-rs.
//! Depending on embassy-rs would prevent me from running the unit test on my base machine.
#![cfg_attr(not(test), no_std)]
#![warn(clippy::pedantic)]
#![allow(clippy::must_use_candidate)]
use embassy_time::Duration;
pub use encodeing::{Step, SubStep};
pub mod encodeing;
mod speed;
pub use speed::Speed;
mod measurement;
pub use encodeing::DirectionDuration;
pub use measurement::Measurement;

type CalibrationData = [u8; 4];
/// Default calibration value that assumes each encoder tick is the same size
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];

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

/// Stores all the logical state required for the sub-step encoder.
///
///NOTE: this intentionally does not rely on `embasy_rp` as that would prevent me from running the unit tests on my host machine.
pub struct EncoderState<const IDLE_STOPING_TIME_MS: u64> {
    calibration_data: CalibrationData,
    last_known_speed: Speed,
    prev_measurement: Measurement,
}
impl<const IDLE_STOPING_TIME_MS: u64> EncoderState<IDLE_STOPING_TIME_MS> {
    /// Get current encoder speed
    pub fn speed(&self) -> Speed {
        self.last_known_speed
    }
    /// Get last estimated position in subsets
    pub fn position(&self) -> SubStep {
        self.prev_measurement.transition(&self.calibration_data)
            + self.last_known_speed * (self.prev_measurement.time_since_transition())
    }
    /// Get the current encoder step
    pub fn steps(&self) -> Step {
        self.prev_measurement.step
    }
    pub fn idel_stopping_time() -> Duration {
        Duration::from_millis(IDLE_STOPING_TIME_MS)
    }

    fn calculate_new_speed(&self, measurement: Measurement) -> Speed {
        if measurement.time_since_transition() >= Self::idel_stopping_time() {
            Speed::stopped()
        } else {
            Measurement::estimate_speed(
                self.last_known_speed,
                self.prev_measurement,
                measurement,
                &self.calibration_data,
            )
        }
    }

    ///Process a new reading.
    pub fn update(&mut self, measurement: Measurement) {
        let new_speed = self.calculate_new_speed(measurement);
        *self = EncoderState {
            last_known_speed: new_speed,
            prev_measurement: measurement,
            calibration_data: self.calibration_data,
        }
    }

    ///Initialize a new encoder state.
    pub fn new(inital_conditions: Measurement) -> Self {
        let calibration_data = EQUAL_STEPS;
        EncoderState {
            calibration_data,
            // set so we start in the stopped state.
            last_known_speed: Speed::stopped(),
            prev_measurement: inital_conditions,
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

#[cfg(test)]
mod tests {
    use crate::{
        Direction::Clockwise,
        EQUAL_STEPS, EncoderState,
        encodeing::{Step, SubStep},
        measurement::{
            Measurement,
            tests::{Event, sequence_events},
        },
        speed::Speed,
    };
    use embassy_time::{Duration, Instant};

    #[test]
    fn testing_is_stoped() {
        let mesurements = sequence_events(
            (Step::new(0), Clockwise, Instant::from_millis(0)),
            vec![
                // we start off stopped
                (Instant::from_millis(0), Event::Mesurement),
                // Start moving
                (Instant::from_millis(10), Event::Step(1)),
                // Use real speed
                (Instant::from_millis(10), Event::Mesurement),
                // Use estimated speed based off of speed bounds.
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Mesurement),
                (Instant::from_millis(39), Event::Mesurement),
                // Time out and consider the encoder stopped
                (Instant::from_millis(40), Event::Mesurement),
            ],
        );

        let mut encoder_state = EncoderState::<30>::new(mesurements[0]);
        assert_eq!(encoder_state.speed(), Speed::stopped());

        encoder_state.update(mesurements[1]);
        assert_ne!(encoder_state.speed(), Speed::stopped());

        for mesurement in &mesurements[2..=4] {
            encoder_state.update(*mesurement);
            assert_ne!(encoder_state.speed(), Speed::stopped());
        }

        encoder_state.update(mesurements[5]);
        assert_eq!(encoder_state.speed(), Speed::stopped());
    }

    #[test]
    fn calculating_speed() {
        let mesurements = sequence_events(
            (Step::new(0), Clockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(0), Event::Mesurement),
                (Instant::from_millis(10), Event::Step(1)),
                (Instant::from_millis(10), Event::Mesurement),
                (Instant::from_millis(20), Event::Step(2)),
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Step(4)),
                (Instant::from_millis(30), Event::Mesurement),
            ],
        );
        let mut encoder_state = EncoderState::<30>::new(mesurements[0]);
        assert_eq!(encoder_state.speed(), Speed::stopped());
        // Start moving
        encoder_state.update(mesurements[1]);
        assert_eq!(
            encoder_state.speed(),
            Speed::new(SubStep::new(64), Duration::from_millis(10))
        );
        encoder_state.update(mesurements[2]);
        assert_eq!(
            encoder_state.speed(),
            Speed::new(SubStep::new(64), Duration::from_millis(10))
        );

        encoder_state.update(mesurements[3]);
        assert_eq!(
            encoder_state.speed(),
            Speed::new(SubStep::new(128), Duration::from_millis(10))
        );
    }

    #[test]
    fn example_from_source_documentation() {
        //This is the example taken from the readme of the code.
        //https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder_substep
        let mesurements = sequence_events(
            (Step::new(3), Clockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(0), Event::Mesurement),
                (Instant::from_millis(21), Event::Step(4)),
                (Instant::from_millis(30), Event::Mesurement),
                (Instant::from_millis(34), Event::Step(5)),
                (Instant::from_millis(40), Event::Mesurement),
                (Instant::from_millis(49), Event::Step(7)),
                (Instant::from_millis(50), Event::Mesurement),
            ],
        );
        let mut encoder = EncoderState::<30>::new(mesurements[0]);
        encoder.update(mesurements[1]);
        encoder.update(mesurements[2]);
        assert_eq!(
            encoder.last_known_speed,
            Speed::new(SubStep::new(64), Duration::from_millis(13))
        );
        encoder.update(mesurements[3]);
        assert_eq!(
            encoder.last_known_speed,
            Speed::new(SubStep::new(128), Duration::from_millis(15))
        );
    }
    #[test]
    fn inital_position() {
        let inital_measurement = Measurement {
            step: Step::new(3),
            direction: Clockwise,
            step_instant: Instant::from_millis(0),
            sample_instant: Instant::from_millis(0),
        };
        let encoder = EncoderState::<30>::new(inital_measurement);
        // The encoder is initialized assuming we are in a stopped position,
        // so the position estimate is just the initial measured position
        assert_eq!(
            encoder.position(),
            inital_measurement.transition(&EQUAL_STEPS)
        )
    }
    /// Test helper function that initializes an encoder that is
    /// - Moving at one step per 10 milliseconds
    /// - Currently at step steps_per_10_millis *3
    /// - moving clockwise
    fn const_speed_encoder(steps_per_10_millis: i32) -> EncoderState<30> {
        //Renaming to something shorter so the formatter keeps each vec entry on one line;
        let step_delta = steps_per_10_millis;
        let mesurements = sequence_events(
            (Step::new(0), Clockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(0), Event::Mesurement),
                (Instant::from_millis(10), Event::Step(step_delta * 1)),
                (Instant::from_millis(10), Event::Mesurement),
                (Instant::from_millis(20), Event::Step(step_delta * 2)),
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Step(step_delta * 3)),
                (Instant::from_millis(30), Event::Mesurement),
            ],
        );
        let mut encoder = EncoderState::new(mesurements[0]);
        // Get the encoder moving at one step per 10 milliseconds
        for measurement in &mesurements[1..] {
            encoder.update(*measurement);
        }
        assert_eq!(
            encoder.last_known_speed,
            Speed::new(
                SubStep::new(steps_per_10_millis * 64),
                Duration::from_millis(10)
            )
        );
        encoder
    }
    #[test]
    fn estimate_substep_posotion() {
        //Check estimate after a short time
        let mut encoder = const_speed_encoder(1);
        encoder.update(Measurement {
            step: Step::new(3),
            direction: Clockwise,
            step_instant: Instant::from_millis(30),
            sample_instant: Instant::from_millis(35),
        });
        assert_eq!(
            encoder.position(),
            // The estimated position should be halfway between 3 and 4 (-1 due to rounding)
            Step::new(3).lower_bound(&EQUAL_STEPS) + SubStep::new(64 / 2 - 1)
        );
    }

    #[test]
    fn estimated_position_respects_step_bounds() {
        //Position estimate should still be bounded by the step bounds
        let mut encoder = const_speed_encoder(5);
        encoder.update(Measurement {
            step: Step::new(15),
            direction: Clockwise,
            step_instant: Instant::from_millis(30),
            sample_instant: Instant::from_millis(39),
        });
        //(-1 due to rounding)
        assert_eq!(
            encoder.position(),
            Step::new(15).upper_bound(&EQUAL_STEPS) - SubStep::new(1)
        )
    }
}
