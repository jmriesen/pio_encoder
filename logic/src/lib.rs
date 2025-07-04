use std::ops::Mul;

use embassy_time::Duration;
use encodeing::{Mesurement, Step, SubStep};

pub mod encodeing;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}

type CalibrationData = [u8; 4];
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];
///Internally stored as sub-steps per 2^20 micro seconds
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Speed(i32);

impl Speed {
    fn new(delta: SubStep, duration: Duration) -> Self {
        Self(delta.val() << 20 / duration.as_micros())
    }
    fn stopped() -> Self {
        Self(0)
    }
    //TODO: test
    fn ticks_per_second(&self) -> i32 {
        ((self.0 as i64 * 2500i64) >> 16) as i32
    }
}
impl Mul<Duration> for Speed {
    type Output = SubStep;

    fn mul(self, rhs: Duration) -> Self::Output {
        SubStep::new((self.0 as u64 * rhs.as_micros() >> 20) as i32)
    }
}
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
    prev_step_us: embassy_time::Instant,
    pub position: SubStep,
    speed: Speed,
    prev_step: Step,
    prev_low: SubStep,
    prev_high: SubStep,
}
impl EncoderState {
    fn update_state(&mut self, mesurement: Mesurement) {
        let lower_bound = mesurement.steps.lower_bound(&EQUAL_STEPS);
        let upper_bound = mesurement.steps.upper_bound(&EQUAL_STEPS);

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
            //TODO consider pulling out into method
            let transition_pos = match mesurement.direction {
                Direction::Clockwise => lower_bound,
                Direction::CounterClockwise => upper_bound,
            };
            if !self.is_stopped {
                self.speed = Speed::new(
                    transition_pos - self.prev_trans_pos,
                    mesurement.step_time - self.prev_trans_us,
                )
            }

            self.is_stopped = false;
            self.prev_trans_us = mesurement.step_time;
            self.prev_trans_pos = transition_pos;
        }
        if !self.is_stopped {
            let (ticks_upper_bound, ticks_lower_bound, delta_time) = if self.prev_trans_us
                > self.prev_step_us
                && self.prev_trans_us - self.prev_step_us
                    > mesurement.step_time - self.prev_trans_us
            {
                let delta_time = self.prev_trans_us - self.prev_step_us;
                (
                    self.prev_trans_pos - self.prev_low,
                    self.prev_trans_pos - self.prev_high,
                    delta_time,
                )
            } else {
                let delta_time = mesurement.step_time - self.prev_trans_us;
                (
                    upper_bound - self.prev_trans_pos,
                    lower_bound - self.prev_trans_pos,
                    delta_time,
                )
            };

            let speed_high = Speed::new(ticks_upper_bound, delta_time);
            let speed_low = Speed::new(ticks_lower_bound, delta_time);
            self.speed = self.speed.clamp(speed_low, speed_high);
            //TODO check if this math works there is a lot of type casting going on.
            self.position = self.prev_trans_pos
                + self.speed * (mesurement.step_time - mesurement.sub_step_time);
            self.position = self.position.clamp(lower_bound, upper_bound);
        }
    }
}
