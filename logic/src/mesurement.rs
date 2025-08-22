use crate::{
    CalibrationData, Direction,
    encodeing::{DirectionDuration, Step, SubStep},
    speed::Speed,
};
use embassy_time::Instant;

/// This represents a current reading from the pio state machine.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Mesurement {
    /// The current step position the encoder is at.
    pub steps: Step,
    /// The direction the encoder is traveling.
    pub direction: Direction,
    /// The time when the step was registered
    pub step_instant: embassy_time::Instant,
    /// The time when this measurement was read from the pio.
    pub sample_instant: embassy_time::Instant,
}
impl Mesurement {
    pub fn new(
        dir_dur: DirectionDuration,
        steps: Step,
        sample_instant: Instant,
        clocks_per_us: u32,
    ) -> Self {
        let (direction, duration) = dir_dur.decode(clocks_per_us);
        Self {
            steps,
            direction,
            step_instant: sample_instant - duration,
            sample_instant,
        }
    }
    /// The last definitely known position.
    /// Calculated based on the encoder tick and encoder direction.
    pub fn mesured_position(&self, calibration: &CalibrationData) -> SubStep {
        match self.direction {
            Direction::Clockwise => self.steps.lower_bound(calibration),
            Direction::CounterClockwise => self.steps.upper_bound(calibration),
        }
    }
}
fn bound_speed(
    speed: Speed,
    previuse: Mesurement,
    current: Mesurement,
    cali: &CalibrationData,
) -> Speed {
    let mesured_position = current.mesured_position(cali);
    let first_mesurement_in_step = current.step_instant > previuse.sample_instant;

    let time_since_last_sample = current.step_instant - previuse.sample_instant;
    let time_since_current_sample = current.sample_instant - current.step_instant;
    let previuse_sample_is_farther = time_since_last_sample > time_since_current_sample;
    let (speed_lower_bound,speed_upper_bound) =
    //If this is the first measurement we want to use whichever measurement is **Farther away** for
    //our estimates.
    //Using the longer delta time gives less uncertainty in our estimates
        if first_mesurement_in_step && previuse_sample_is_farther {
            let (lower_bound, upper_bound) = previuse.steps.bounds(cali);
        //NOTE: this is (initial - final) rather than (final-initial) to compensate for the fact 
        //that we don't have negative durations.
            (
                Speed::new(mesured_position - upper_bound, time_since_last_sample),
                Speed::new(mesured_position - lower_bound, time_since_last_sample),
            )
        } else {
            let (lower_bound, upper_bound) = current.steps.bounds(cali);
            (
                Speed::new(lower_bound - mesured_position, time_since_current_sample),
                Speed::new(upper_bound - mesured_position, time_since_current_sample),
            )
        };

    speed.clamp(speed_lower_bound, speed_upper_bound)
}
#[cfg(test)]
mod tests {
    use crate::EQUAL_STEPS;

    use super::*;
    use embassy_time::Duration;

    #[test]
    fn mesurment() {
        let time = Instant::from_secs(1);
        assert_eq!(
            Mesurement::new(DirectionDuration(0 - 50), Step::new(42), time, 10),
            Mesurement {
                steps: Step::new(42),
                direction: Direction::CounterClockwise,
                step_instant: time - Duration::from_micros(65),
                sample_instant: time
            }
        );
    }
    #[test]
    fn whole_steps() {
        let speed = bound_speed(
            Speed::new(SubStep::new(64), Duration::from_millis(20)),
            Mesurement {
                steps: Step::new(0),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(0),
                sample_instant: Instant::from_millis(10),
            },
            Mesurement {
                steps: Step::new(1),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(20),
                sample_instant: Instant::from_millis(30),
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            speed,
            Speed::new(SubStep::new(64), Duration::from_millis(20))
        );
    }
    #[test]
    fn between_steps() {
        let speed = bound_speed(
            Speed::new(SubStep::new(64), Duration::from_millis(20)),
            Mesurement {
                steps: Step::new(0),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(0),
                sample_instant: Instant::from_millis(10),
            },
            Mesurement {
                steps: Step::new(0),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(10),
                sample_instant: Instant::from_millis(30),
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            speed,
            Speed::new(SubStep::new(64), Duration::from_millis(20))
        );
    }
}
