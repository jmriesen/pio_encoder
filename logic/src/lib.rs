//! This crate contains all the logic assisted with parsing pio messages and calculating speed.
//! This crate specificity does **not** depend on embassy-rs.
//! Depending on embassy-rs would prevent me from running the unit test on my base machine.
#![cfg_attr(not(test), no_std)]
#![warn(clippy::pedantic)]
#![allow(clippy::must_use_candidate)]
use embassy_time::Duration;
pub mod encodeing;
mod speed;
pub use speed::Speed;
mod measurement;
pub use encodeing::DirectionDuration;
pub use measurement::Measurement;
mod step;
pub use step::{Step, SubStep};

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

    ///Process a new reading.
    pub fn update(&mut self, measurement: Measurement) {
        let new_speed = if measurement.time_since_transition() >= Self::idel_stopping_time() {
            Speed::stopped()
        } else {
            Measurement::estimate_speed(
                self.last_known_speed,
                self.prev_measurement,
                measurement,
                &self.calibration_data,
            )
        };
        self.last_known_speed = new_speed;
        self.prev_measurement = measurement;
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
        Direction::CounterClockwise,
        EQUAL_STEPS, EncoderState,
        measurement::{
            Measurement,
            tests::{Event, sequence_events},
        },
        speed::Speed,
        step::{Step, SubStep},
    };
    use embassy_time::{Duration, Instant};
    fn simulate_assert(
        measurements: Vec<Measurement>,
        speeds: Vec<Speed>,
        positions: Vec<SubStep>,
    ) {
        // Check that we did not forget to pass anything.
        assert_eq!(measurements.len(), speeds.len());
        assert_eq!(measurements.len(), positions.len());
        let mut iter = measurements
            .into_iter()
            .zip(speeds.into_iter())
            .zip(positions.into_iter());

        let ((inital, speed), position) = iter.next().unwrap();
        let mut encoder_state = EncoderState::<30>::new(dbg!(inital));
        dbg!(inital);
        dbg!(encoder_state.speed());
        dbg!(encoder_state.position());
        assert_eq!(speed, encoder_state.speed());
        assert_eq!(position, encoder_state.position());

        for ((measurement, speed), position) in iter {
            dbg!(measurement);
            encoder_state.update(measurement);
            dbg!(encoder_state.speed());
            dbg!(encoder_state.position());
            assert_eq!(speed, encoder_state.speed());
            assert_eq!(position, encoder_state.position());
        }
    }

    #[test]
    fn estimate_between_ticks() {
        let measurements = sequence_events(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                // we start off stopped
                (Instant::from_millis(0), Event::Mesurement),
                // Start moving
                (Instant::from_millis(35), Event::Step(3)),
                // Use real speed
                (Instant::from_millis(40), Event::Mesurement),
                // Use keep using last speed estimate.
                (Instant::from_millis(45), Event::Mesurement),
                // Using last speed estimate would push position into the next step.
                // Estimate current speed is the max possible that does not push position into the
                // next step
                (Instant::from_millis(50), Event::Mesurement),
                // Last ms before time out.
                (Instant::from_millis(64), Event::Mesurement),
                // Time out and consider the encoder stopped
                (Instant::from_millis(65), Event::Mesurement),
            ],
        );

        let speeds = vec![
            Speed::stopped(),
            Speed::new(SubStep::new(64 * 3), Duration::from_millis(35)),
            Speed::new(SubStep::new(64 * 3), Duration::from_millis(35)),
            //Reducing speed estimate since we need to stay in the same tick
            Speed::new(SubStep::new(64), Duration::from_millis(15)),
            Speed::new(SubStep::new(64), Duration::from_millis(29)),
            // Timed out
            Speed::stopped(),
        ];
        let positions = vec![
            SubStep::new(0),
            Step::new(3).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(5),
            Step::new(3).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(10),
            // Clamp position at the end of the step.
            Step::new(3).upper_bound(&EQUAL_STEPS) - SubStep::new(1),
            Step::new(3).upper_bound(&EQUAL_STEPS) - SubStep::new(1),
            // Revert position back to last known transition once we are stopped.
            Step::new(3).lower_bound(&EQUAL_STEPS),
        ];
        simulate_assert(measurements, speeds, positions);
    }

    #[test]
    fn step_and_mesurement_happen_at_the_same_time() {
        let measurements = sequence_events(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
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
        let speeds = vec![
            Speed::stopped(),
            Speed::new(SubStep::new(64), Duration::from_millis(10)),
            Speed::new(SubStep::new(64), Duration::from_millis(10)),
            Speed::new(SubStep::new(64 * 2), Duration::from_millis(10)),
        ];
        let positions = vec![
            SubStep::new(0),
            Step::new(1).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(0),
            Step::new(2).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(0),
            Step::new(4).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(0),
        ];

        simulate_assert(measurements, speeds, positions);
    }

    #[test]
    fn example_from_source_documentation() {
        //This is the example taken from the readme of the code.
        //(https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder_substep)
        let measurements = sequence_events(
            (Step::new(3), CounterClockwise, Instant::from_millis(0)),
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
        let speeds = vec![
            Speed::stopped(),
            Speed::new(SubStep::new(64), Duration::from_millis(21)),
            Speed::new(SubStep::new(64), Duration::from_millis(13)),
            Speed::new(SubStep::new(128), Duration::from_millis(15)),
        ];
        let positions = vec![
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(4).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(9),
            Step::new(5).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(6),
            Step::new(7).lower_bound(&EQUAL_STEPS) + speeds[3] * Duration::from_millis(1),
        ];
        simulate_assert(measurements, speeds, positions);
    }

    #[test]
    fn hovering_over_a_transition_is_not_considered_movement() {
        let measurements = sequence_events(
            (Step::new(3), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(0), Event::Mesurement),
                (Instant::from_millis(10), Event::Step(2)),
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Step(3)),
                (Instant::from_millis(40), Event::Mesurement),
                (Instant::from_millis(50), Event::Step(2)),
                (Instant::from_millis(60), Event::Mesurement),
            ],
        );
        let speeds = vec![
            Speed::stopped(),
            Speed::stopped(),
            Speed::stopped(),
            Speed::stopped(),
        ];
        let positions = vec![
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(3).lower_bound(&EQUAL_STEPS),
        ];
        simulate_assert(measurements, speeds, positions);
    }
    #[test]
    fn always_use_larger_delta_time_for_estiments() {
        let measurements = sequence_events(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                // larger delta happens first
                (Instant::from_millis(035), Event::Mesurement),
                (Instant::from_millis(050), Event::Step(10)),
                (Instant::from_millis(060), Event::Mesurement),
                //---resetting
                (Instant::from_millis(060), Event::Step(-1)),
                (Instant::from_millis(100), Event::Step(0)),
                // larger delta happens after
                (Instant::from_millis(145), Event::Mesurement),
                (Instant::from_millis(150), Event::Step(10)),
                (Instant::from_millis(160), Event::Mesurement),
                //---resetting
                (Instant::from_millis(160), Event::Step(-1)),
                (Instant::from_millis(200), Event::Step(0)),
                // Same time delta
                (Instant::from_millis(240), Event::Mesurement),
                (Instant::from_millis(250), Event::Step(10)),
                (Instant::from_millis(260), Event::Mesurement),
            ],
        );
        let speeds = vec![
            // Larger delta happens first use speed from the last two steps
            Speed::stopped(),
            Speed::new(
                Step::new(10).lower_bound(&EQUAL_STEPS) - Step::new(0).upper_bound(&EQUAL_STEPS),
                Duration::from_millis(15),
            ),
            // larger delta happens after
            Speed::stopped(),
            dbg!(Speed::new(SubStep::new(64), Duration::from_millis(10))),
            // Same time delta
            Speed::stopped(),
            Speed::new(SubStep::new(64), Duration::from_millis(10)),
        ];
        let positions = vec![
            // larger delta happens first
            Step::new(0).lower_bound(&EQUAL_STEPS),
            Step::new(10).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(10),
            // Larger delta happens after
            Step::new(0).lower_bound(&EQUAL_STEPS),
            Step::new(10).lower_bound(&EQUAL_STEPS) + speeds[3] * Duration::from_millis(10),
            // Same time delta
            Step::new(0).lower_bound(&EQUAL_STEPS),
            Step::new(10).lower_bound(&EQUAL_STEPS) + speeds[5] * Duration::from_millis(10),
        ];
        simulate_assert(measurements, speeds, positions);
    }
}
