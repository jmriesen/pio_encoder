use encodeing::{Step, SubStep};

pub mod encodeing;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}
mod speed;
pub use speed::Speed;
mod mesurement;
use mesurement::{Mesurement, calculate_speed, calculate_speed_bounds};

type CalibrationData = [u8; 4];
/// Default calibration value that assumes each encoder tick is the same size
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];
/// The number of samples that need to be read before we conclude the encoder has stopped.
const IDLE_STOP_SAMPLES: u32 = 3;

/// Stores all the logical state required for the sub-step encoder.
///
///NOTE: this specific dose not rely on embasy_rp since I want it unit testable.
pub struct EncoderState {
    calibration_data: CalibrationData,
    idle_stop_samples_count: u32,
    pub position: SubStep,
    speed: Speed,
    prev_mesurement: Mesurement,
}
impl EncoderState {
    /// The encoder is considered stopped if there have been `IDLE_STOP_SAMPLES` measurements
    /// without the step count changing.
    pub fn is_stopped(&self) -> bool {
        self.idle_stop_samples_count >= IDLE_STOP_SAMPLES
    }
    /// helper method for update_state that guarantiess we are not modifying the current state
    /// while generateing the next one.
    fn calculate_next_state(&self, new_data: Mesurement) -> Self {
        let speed = if self.is_stopped() {
            Speed::stopped()
        } else {
            calculate_speed(new_data, self.prev_mesurement, &self.calibration_data)
        };
        let idle_count = if self.prev_mesurement.steps == new_data.steps {
            self.idle_stop_samples_count + 1
        } else {
            0
        };
        Self {
            calibration_data: self.calibration_data,
            idle_stop_samples_count: idle_count,
            position: self.position,
            speed,
            prev_mesurement: new_data,
        }
    }
    pub fn update_state(&mut self, mesurement: Mesurement) {
        *self = self.calculate_next_state(mesurement);
        /*
                self.position = {
                    let new_position = self
                        .prev_mesurement
                        .mesured_position(&self.calibration_data)
                        + self.speed * (mesurement.sample_instant - mesurement.step_instant);
                    let (pos_lower_bound, pos_upper_bound) =
                        mesurement.steps.bounds(&self.calibration_data);
                    new_position.clamp(pos_lower_bound, pos_upper_bound)
                };
                let (speed_lower_bound, speed_upper_bound) =
                    calculate_speed_bounds(self.prev_mesurement, mesurement, &self.calibration_data);
                self.speed = self.speed.clamp(speed_lower_bound, speed_upper_bound);
        */
    }

    ///Initialize a new encoder state.
    pub fn new(inital_conditions: Mesurement) -> Self {
        let calibration_data = EQUAL_STEPS;
        EncoderState {
            calibration_data,
            idle_stop_samples_count: IDLE_STOP_SAMPLES + 1,
            position: inital_conditions.mesured_position(&calibration_data),
            speed: Speed::stopped(),
            prev_mesurement: inital_conditions,
        }
    }
}
#[cfg(test)]
mod tests {
    use embassy_time::{Duration, Instant};

    use crate::{
        EncoderState, IDLE_STOP_SAMPLES,
        encodeing::{Step, SubStep},
        mesurement::Mesurement,
        speed::Speed,
    };

    fn mesurement(steps: Step, time: u64) -> Mesurement {
        Mesurement {
            steps,
            direction: crate::Direction::Clockwise,
            step_instant: Instant::from_millis(time),
            sample_instant: Instant::from_millis(time),
        }
    }

    #[test]
    fn testing_is_stoped() {
        let mut encoder_state = EncoderState::new(mesurement(Step::new(0), 0));

        // we start off stopped
        assert!(encoder_state.is_stopped());
        // Start moving
        encoder_state.update_state(mesurement(Step::new(1), 10));
        assert!(!encoder_state.is_stopped());
        // Next few readings don't show any movement
        for i in 0..IDLE_STOP_SAMPLES as u64 {
            assert!(!encoder_state.is_stopped());
            encoder_state.update_state(mesurement(Step::new(1), i * 10 + 11));
        }
        assert!(encoder_state.is_stopped());
    }

    #[test]
    fn calculating_speed() {
        let mut encoder_state = EncoderState::new(mesurement(Step::new(0), 0));
        assert_eq!(encoder_state.speed, Speed::stopped());
        // Start moving
        encoder_state.update_state(mesurement(Step::new(1), 10));
        // There is a lag between first measurement that changes step and when we start "moving"
        // This delay is to insure the speed calculations have a valid previous position data.
        assert_eq!(encoder_state.speed, Speed::stopped());
        encoder_state.update_state(mesurement(Step::new(2), 20));
        assert_eq!(
            encoder_state.speed,
            Speed::new(SubStep::new(64), Duration::from_millis(10))
        );

        encoder_state.update_state(mesurement(Step::new(4), 30));
        assert_eq!(
            encoder_state.speed,
            Speed::new(SubStep::new(128), Duration::from_millis(10))
        );
    }
    #[test]
    fn wait_for_multiple_readings_before_concluding_movement() {
        // We need at least two in movement measurement to get a good speed estimate.
        let mut encoder_state = EncoderState::new(mesurement(Step::new(0), 0));
        assert_eq!(encoder_state.speed, Speed::stopped());
        // See a new tick
        encoder_state.update_state(mesurement(Step::new(1), 10));
        assert_eq!(encoder_state.speed, Speed::stopped());
        // Stay on the current tick
        encoder_state.update_state(mesurement(Step::new(1), 20));
        assert_eq!(
            encoder_state.speed,
            Speed::new(SubStep::new(0), Duration::from_millis(10))
        );
    }
}
