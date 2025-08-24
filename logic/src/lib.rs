use embassy_time::Duration;
use encodeing::{Step, SubStep};

pub mod encodeing;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}
mod speed;
use speed::Speed;
mod mesurement;
use mesurement::Mesurement;

type CalibrationData = [u8; 4];
/// Default calibration value that assumes each encoder tick is the same size
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];
/// The number of samples that need to be read before we conclude the encoder has stopped.
const IDLE_STOP_SAMPLES: u32 = 3;

/// Stores all the logical state required for the sub-step encoder.
///
///NOTE: this specific dose not rely on embasy_rp since I want it unit testable.
struct EncoderState {
    calibration_data: CalibrationData,
    idle_stop_samples_count: u32,
    is_stopped: bool,
    prev_trans_pos: SubStep,
    prev_trans_us: embassy_time::Instant,
    prev_sample_time: embassy_time::Instant,
    pub position: SubStep,
    speed: Speed,
    prev_step: Step,
    prev_low: SubStep,
    prev_high: SubStep,
}
impl EncoderState {
    fn update_state(&mut self, mesurement: Mesurement) {
        //Updates stopped state
        if self.prev_step == mesurement.steps {
            self.idle_stop_samples_count += 1;
        } else {
            self.idle_stop_samples_count = 0;
        }
        if !self.is_stopped && self.idle_stop_samples_count >= IDLE_STOP_SAMPLES {
            self.speed = Speed::stopped();
            self.is_stopped = true;
        }

        if self.prev_step != mesurement.steps {
            let transition_pos = mesurement.mesured_position(&EQUAL_STEPS);

            if !self.is_stopped {
                self.speed = Speed::new(
                    transition_pos - self.prev_trans_pos,
                    mesurement.step_instant - self.prev_trans_us,
                )
            }

            self.is_stopped = false;
            self.prev_trans_us = mesurement.step_instant;
            self.prev_trans_pos = transition_pos;
        }
    }
    ///Initialize a new encoder state.
    pub fn new(inital_conditions: Mesurement) -> Self {
        let calibration_data = EQUAL_STEPS;
        let (prev_low, prev_high) = inital_conditions.steps.bounds(&calibration_data);
        EncoderState {
            calibration_data,
            idle_stop_samples_count: 0,
            is_stopped: true,
            prev_trans_pos: inital_conditions.mesured_position(&calibration_data),
            prev_trans_us: inital_conditions.step_instant,
            prev_sample_time: inital_conditions.sample_instant,
            position: inital_conditions.mesured_position(&calibration_data),
            speed: Speed::stopped(),
            prev_step: inital_conditions.steps,
            prev_low,
            prev_high,
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
        assert!(encoder_state.is_stopped);
        // Start moving
        encoder_state.update_state(mesurement(Step::new(1), 10));
        assert!(!encoder_state.is_stopped);
        // Next few readings don't show any movement
        for i in 0..IDLE_STOP_SAMPLES as u64 {
            assert!(!encoder_state.is_stopped);
            encoder_state.update_state(mesurement(Step::new(0), i * 0));
        }
        assert!(encoder_state.is_stopped);
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
