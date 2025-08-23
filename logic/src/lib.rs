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
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];

/// Stores all the logical state required for the sub-step encoder.
///
///NOTE: this specific dose not rely on embasy_rp since I want it unit testable.
struct EncoderState {
    calibration_data: CalibrationData,
    idle_stop_samples: u32,
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
        //NOTE: label A
        if self.prev_step == mesurement.steps {
            self.idle_stop_samples_count += 1;
        } else {
            self.idle_stop_samples_count = 0;
        }
        if !self.is_stopped && self.idle_stop_samples_count >= self.idle_stop_samples {
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

        if !self.is_stopped {}
    }
}
#[cfg(test)]
mod tests {
    use embassy_time::Instant;

    use crate::{
        EQUAL_STEPS, EncoderState,
        encodeing::{Step, SubStep},
        mesurement::Mesurement,
        speed::Speed,
    };

    #[test]
    fn testing_is_stoped() {
        let mut encoder_state = EncoderState {
            calibration_data: EQUAL_STEPS,
            idle_stop_samples: 3,
            idle_stop_samples_count: 0,
            is_stopped: true,
            prev_trans_pos: SubStep::new(0),
            prev_trans_us: Instant::from_millis(0),
            prev_sample_time: Instant::from_millis(0),
            position: SubStep::new(0),
            speed: Speed::stopped(),
            prev_step: Step::new(0),
            prev_low: SubStep::new(0),
            prev_high: SubStep::new(0),
        };
        // we start off stopped
        assert!(encoder_state.is_stopped);
        // Start moving
        encoder_state.update_state(Mesurement {
            steps: Step::new(1),
            direction: crate::Direction::Clockwise,
            step_instant: Instant::from_millis(10),
            sample_instant: Instant::from_millis(11),
        });
        assert!(!encoder_state.is_stopped);
        // Next few readings don't show any movement
        for i in 0..encoder_state.idle_stop_samples as u64 {
            assert!(!encoder_state.is_stopped);
            encoder_state.update_state(Mesurement {
                steps: Step::new(0),
                direction: crate::Direction::Clockwise,
                step_instant: Instant::from_millis(0),
                sample_instant: Instant::from_millis(i * 0),
            });
        }
        assert!(encoder_state.is_stopped);
    }
}
