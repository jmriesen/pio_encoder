#![allow(dead_code)]
use defmt::info;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
/// Contains logic for parsing the pio messages into logical values
mod encodeing;
mod pio;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}

pub use pio::PioEncoderProgram;
use pio::{EncoderStateMachine, RawSteps};
type CalibrationData = [u32; 4];

pub fn substep_calc_speed(delta_substep: u32, delta_us: embassy_time::Duration) -> i32 {
    //TODO Check this calculation.
    info!("{}", delta_us);
    ((delta_substep << 20) as u64 / delta_us.as_micros()) as i32
}

/// Pio Backed quadrature encoder reader
pub struct PioEncoder<'d, T: Instance, const SM: usize> {
    sm: EncoderStateMachine<'d, T, SM>,
    calibration_data: CalibrationData,
    idle_stop_samples: u32,
    idle_stop_samples_count: u32,
    is_stopped: bool,
    prev_trans_pos: u32,
    prev_trans_us: embassy_time::Instant,
    prev_step_us: embassy_time::Instant,
    pub position: u32,
    pub speed: i32,
    speed_2_20: i32,
    raw_steps: RawSteps,
    prev_low: u32,
    prev_high: u32,
}

//Setting to assume every step takes the same time.
const EQUAL_STEPS: CalibrationData = [0, 64, 128, 192];

impl<'d, T: Instance, const SM: usize> PioEncoder<'d, T, SM> {
    pub fn new(
        pio: &mut Common<'d, T>,
        sm: StateMachine<'d, T, SM>,
        pin_a: impl PioPin,
        pin_b: impl PioPin,
        program: &PioEncoderProgram<'d, T>,
    ) -> Self {
        let mut sm = EncoderStateMachine::new(pio, sm, pin_a, pin_b, program);
        let inial_data = sm.pull_data();
        let calibration_data = EQUAL_STEPS;
        let position = inial_data.steps.lower_bound(&calibration_data);
        Self {
            sm: sm,
            calibration_data,
            //NOTEs check calculation (dividing by a million +rounding.)
            idle_stop_samples: 3,
            idle_stop_samples_count: 0,
            is_stopped: true,
            position,
            //todo check how this is initialized
            speed: 0,
            speed_2_20: 0,
            prev_trans_pos: 0,
            prev_trans_us: inial_data.transition_time,
            prev_step_us: inial_data.step_time,
            raw_steps: inial_data.steps,
            prev_low: 0,
            prev_high: 0,
        }
    }
    pub fn update(&mut self) {
        let new_data = self.sm.pull_data();
        let lower_bound = new_data.steps.lower_bound(&self.calibration_data);
        let upper_bound = new_data.steps.upper_bound(&self.calibration_data);

        if self.raw_steps == new_data.steps {
            self.idle_stop_samples_count += 1;
        } else {
            self.idle_stop_samples_count = 0;
        }

        if !self.is_stopped && self.idle_stop_samples_count >= self.idle_stop_samples {
            self.speed = 0;
            self.speed_2_20 = 0;
            self.is_stopped = true;
        }
        if self.raw_steps != new_data.steps {
            let transition_pos = match new_data.direction {
                Direction::Clockwise => lower_bound,
                Direction::CounterClockwise => upper_bound,
            };
            if !self.is_stopped {
                self.speed_2_20 = substep_calc_speed(
                    transition_pos - self.prev_trans_pos,
                    new_data.transition_time - self.prev_trans_us,
                )
            }

            self.is_stopped = false;
            self.prev_trans_us = new_data.transition_time;
            self.prev_trans_pos = transition_pos;
        }
        if !self.is_stopped {
            let (ticks_upper_bound, ticks_lower_bound, delta_time) = if self.prev_trans_us
                > self.prev_step_us
                && self.prev_trans_us - self.prev_step_us > new_data.step_time - self.prev_trans_us
            {
                let delta_time = self.prev_trans_us - self.prev_step_us;
                (
                    self.prev_trans_pos - self.prev_low,
                    self.prev_trans_pos - self.prev_high,
                    delta_time,
                )
            } else {
                let delta_time = new_data.step_time - self.prev_trans_us;
                (
                    upper_bound - self.prev_trans_pos,
                    lower_bound - self.prev_trans_pos,
                    delta_time,
                )
            };

            let speed_high = substep_calc_speed(ticks_upper_bound, delta_time);
            let speed_low = substep_calc_speed(ticks_lower_bound, delta_time);
            self.speed_2_20 = self.speed_2_20.clamp(speed_low, speed_high);
            //TODO check if this math works there is a lot of type casting going on.
            self.speed = ((self.speed_2_20 as i64 * 2500i64) >> 16) as i32;
            self.position = self.prev_trans_pos
                + self.speed_2_20 as u32
                    * (new_data.step_time - new_data.transition_time).as_micros() as u32;
            self.position = self.position.clamp(lower_bound, upper_bound);
        }
    }

    pub fn ticks(&mut self) -> i32 {
        self.sm.pull_raw_data().ticks.raw()
    }
}
