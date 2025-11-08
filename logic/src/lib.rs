#![cfg_attr(not(test), no_std)]
#![warn(clippy::pedantic)]
#![allow(clippy::must_use_candidate)]
use encodeing::{Step, SubStep};

pub mod encodeing;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}
mod speed;
pub use speed::Speed;
mod measurement;
pub use encodeing::DirectionDuration;
pub use measurement::Measurement;
use measurement::{calculate_speed, calculate_speed_bounds};

type CalibrationData = [u8; 4];
/// Default calibration value that assumes each encoder tick is the same size
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];
/// The number of samples that need to be read before we conclude the encoder has stopped.
const IDLE_STOP_SAMPLES: u32 = 3;

/// Stores all the logical state required for the sub-step encoder.
///
///NOTE: this specific does not rely on embasy_rp that would prevent me from compiling unit tests.
pub struct EncoderState {
    calibration_data: CalibrationData,
    idle_stop_samples_count: u32,
    position: SubStep,
    speed: Speed,
    prev_measurement: Measurement,
}
impl EncoderState {
    /// Get current encoder speed
    pub fn speed(&self) -> Speed {
        self.speed
    }
    /// Get last estimated position in subsets
    pub fn position(&self) -> SubStep {
        self.position
    }
    /// Get the current encoder step
    pub fn steps(&self) -> Step {
        self.prev_measurement.step
    }
    /// The encoder is considered stopped if there have been `IDLE_STOP_SAMPLES` measurements
    /// without the step count changing.
    pub fn is_stopped(&self) -> bool {
        self.idle_stop_samples_count >= IDLE_STOP_SAMPLES
    }
    /// Helper method for `update_state` that guaranties we are not modifying the current state
    /// while generating the next one.
    fn calculate_next_state(&self, new_data: Measurement) -> Self {
        let idle_count = if self.prev_measurement.step == new_data.step {
            self.idle_stop_samples_count + 1
        } else {
            0
        };
        let speed = {
            let (speed_lower_bound, speed_upper_bound) =
                calculate_speed_bounds(self.prev_measurement, new_data, &self.calibration_data);
            if self.is_stopped() {
                Speed::stopped()
            } else if self.prev_measurement.step != new_data.step {
                calculate_speed(self.prev_measurement, new_data, &self.calibration_data)
            } else {
                self.speed
            }
            .clamp(speed_lower_bound, speed_upper_bound)
        };

        let position = self
            .prev_measurement
            .measured_position(&self.calibration_data)
            + speed * (new_data.sample_instant - new_data.step_instant);
        Self {
            calibration_data: self.calibration_data,
            idle_stop_samples_count: idle_count,
            position,
            speed,
            prev_measurement: new_data,
        }
    }

    ///Process a new reading.
    pub fn update_state(&mut self, measurement: Measurement) {
        *self = self.calculate_next_state(measurement);
    }

    ///Initialize a new encoder state.
    pub fn new(inital_conditions: Measurement) -> Self {
        let calibration_data = EQUAL_STEPS;
        EncoderState {
            calibration_data,
            // set so we start in the stopped state.
            idle_stop_samples_count: IDLE_STOP_SAMPLES + 1,
            position: inital_conditions.measured_position(&calibration_data),
            speed: Speed::stopped(),
            prev_measurement: inital_conditions,
        }
    }
}

#[cfg(test)]
mod tests {
    use embassy_time::{Duration, Instant};

    use crate::{
        Direction::Clockwise,
        EQUAL_STEPS, EncoderState, IDLE_STOP_SAMPLES,
        encodeing::{Step, SubStep},
        measurement::Measurement,
        speed::Speed,
    };

    fn measurement(steps: Step, time: u64) -> Measurement {
        Measurement {
            step: steps,
            direction: Clockwise,
            step_instant: Instant::from_millis(time),
            sample_instant: Instant::from_millis(time),
        }
    }

    #[test]
    fn testing_is_stoped() {
        let mut encoder_state = EncoderState::new(measurement(Step::new(0), 0));
        // we start off stopped
        assert!(encoder_state.is_stopped());
        // Start moving
        encoder_state.update_state(measurement(Step::new(1), 10));
        assert!(!encoder_state.is_stopped());
        // Next few readings don't show any movement
        for i in 0..IDLE_STOP_SAMPLES as u64 {
            assert!(!encoder_state.is_stopped());
            encoder_state.update_state(measurement(Step::new(1), i * 10 + 11));
        }
        assert!(encoder_state.is_stopped());
    }

    #[test]
    fn calculating_speed() {
        let mut encoder_state = EncoderState::new(measurement(Step::new(0), 0));
        assert_eq!(encoder_state.speed, Speed::stopped());
        // Start moving
        encoder_state.update_state(measurement(Step::new(1), 10));
        // There is a lag between first measurement that changes step and when we start "moving"
        // This delay is to insure the speed calculations have a valid previous position data.
        assert_eq!(encoder_state.speed, Speed::stopped());
        encoder_state.update_state(measurement(Step::new(2), 20));
        assert_eq!(
            encoder_state.speed,
            Speed::new(SubStep::new(64), Duration::from_millis(10))
        );

        encoder_state.update_state(measurement(Step::new(4), 30));
        assert_eq!(
            encoder_state.speed,
            Speed::new(SubStep::new(128), Duration::from_millis(10))
        );
    }
    #[test]
    fn wait_for_multiple_readings_before_concluding_movement() {
        // We need at least two in movement measurement to get a good speed estimate.
        let mut encoder_state = EncoderState::new(measurement(Step::new(0), 0));
        assert_eq!(encoder_state.speed, Speed::stopped());
        // See a new tick
        encoder_state.update_state(measurement(Step::new(1), 10));
        assert_eq!(encoder_state.speed, Speed::stopped());
        // Stay on the current tick
        encoder_state.update_state(measurement(Step::new(1), 20));
        assert_eq!(
            encoder_state.speed,
            Speed::new(SubStep::new(0), Duration::from_millis(10))
        );
    }

    #[test]
    fn example_from_source_documentation() {
        //This is the example taken from the readme of the original code.
        //https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder_substep
        let mut encoder = EncoderState::new(Measurement {
            step: Step::new(3),
            direction: Clockwise,
            step_instant: Instant::from_millis(0),
            sample_instant: Instant::from_millis(0),
        });
        encoder.update_state(Measurement {
            step: Step::new(4),
            direction: Clockwise,
            step_instant: Instant::from_millis(21),
            sample_instant: Instant::from_millis(30),
        });
        encoder.update_state(Measurement {
            step: Step::new(5),
            direction: Clockwise,
            step_instant: Instant::from_millis(34),
            sample_instant: Instant::from_millis(40),
        });
        assert_eq!(
            encoder.speed,
            Speed::new(SubStep::new(64), Duration::from_millis(13))
        );
        encoder.update_state(Measurement {
            step: Step::new(7),
            direction: Clockwise,
            step_instant: Instant::from_millis(49),
            sample_instant: Instant::from_millis(50),
        });
        assert_eq!(
            encoder.speed,
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
        let encoder = EncoderState::new(inital_measurement);
        // The encoder is initialized assuming we are in a stopped position,
        // so the position estimate is just the initial measured position
        assert_eq!(
            encoder.position,
            inital_measurement.measured_position(&EQUAL_STEPS)
        )
    }
    /// Test helper function that initializes an encoder that is
    /// - Moving at one step per 10 milliseconds
    /// - Currently at step steps_per_10_millis *3
    /// - moving clockwise
    fn const_speed_encoder(steps_per_10_millis: i32) -> EncoderState {
        let mut encoder = EncoderState::new(measurement(Step::new(0), 0));
        // Get the encoder moving at one step per 10 milliseconds
        encoder.update_state(measurement(Step::new(steps_per_10_millis), 10));
        encoder.update_state(measurement(Step::new(steps_per_10_millis * 2), 20));
        encoder.update_state(measurement(Step::new(steps_per_10_millis * 3), 30));
        assert_eq!(
            encoder.speed,
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
        encoder.update_state(Measurement {
            step: Step::new(3),
            direction: Clockwise,
            step_instant: Instant::from_millis(30),
            sample_instant: Instant::from_millis(35),
        });
        assert_eq!(
            encoder.position,
            // The estimated position should be halfway between 3 and 4 (-1 due to rounding)
            Step::new(3).lower_bound(&EQUAL_STEPS) + SubStep::new(64 / 2 - 1)
        );
    }

    #[test]
    fn estimated_position_respects_step_bounds() {
        //Position estimate should still be bounded by the step bounds
        let mut encoder = const_speed_encoder(5);
        encoder.update_state(Measurement {
            step: Step::new(15),
            direction: Clockwise,
            step_instant: Instant::from_millis(30),
            sample_instant: Instant::from_millis(39),
        });
        //(-1 due to rounding)
        assert_eq!(
            encoder.position,
            Step::new(15).upper_bound(&EQUAL_STEPS) - SubStep::new(1)
        )
    }
}
