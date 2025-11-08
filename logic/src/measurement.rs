use crate::{
    CalibrationData, Direction,
    encodeing::{DirectionDuration, Step, SubStep},
    speed::Speed,
};
use embassy_time::Instant;

/// This represents a current reading from the pio state machine.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    /// The current step position the encoder is at.
    pub step: Step,
    /// The direction the encoder is traveling.
    pub direction: Direction,
    /// The time when the step was registered
    pub step_instant: embassy_time::Instant,
    /// The time when this measurement was read from the pio.
    pub sample_instant: embassy_time::Instant,
}
impl Measurement {
    pub fn new(
        dir_dur: DirectionDuration,
        steps: Step,
        sample_instant: Instant,
        clocks_per_us: u32,
    ) -> Self {
        let (direction, duration) = dir_dur.decode(clocks_per_us);
        Self {
            step: steps,
            direction,
            step_instant: sample_instant - duration,
            sample_instant,
        }
    }
    /// The last definitely known position.
    /// Calculated based on the encoder tick and encoder direction.
    pub fn measured_position(&self, calibration: &CalibrationData) -> SubStep {
        match self.direction {
            Direction::Clockwise => self.step.lower_bound(calibration),
            Direction::CounterClockwise => self.step.upper_bound(calibration),
        }
    }
}
pub fn calculate_speed(
    previous: Measurement,
    current: Measurement,
    calibration_data: &CalibrationData,
) -> Speed {
    Speed::new(
        current.measured_position(calibration_data) - previous.measured_position(calibration_data),
        current.step_instant - previous.step_instant,
    )
}

/// Calculate the lower and upper speed bounds giving the current and previous measurements
pub fn calculate_speed_bounds(
    previous: Measurement,
    current: Measurement,
    cali: &[u8; 4],
) -> (Speed, Speed) {
    let measured_position = current.measured_position(cali);
    let first_measurement_in_step = current.step_instant > previous.sample_instant;

    let time_to_last_measurement = if first_measurement_in_step {
        current.step_instant - previous.sample_instant
    } else {
        previous.sample_instant - current.step_instant
    };
    let time_to_current_measurement = current.sample_instant - current.step_instant;
    let previous_sample_is_farther_away = time_to_last_measurement > time_to_current_measurement;
    //If this is the first measurement in this encoder step we have two time frames we could chose
    //from:
    //1) Previous measurement to the step_instance.
    //2) The step_instance to the current measurement time
    //Using the longer delta time gives less uncertainty in our estimates
    if first_measurement_in_step && previous_sample_is_farther_away {
        let (lower_bound, upper_bound) = previous.step.bounds(cali);
        //NOTE: this is (initial - final) rather than (final-initial) to compensate for the fact
        //that embassy doesn't support negative durations.
        (
            Speed::new(measured_position - upper_bound, time_to_last_measurement),
            Speed::new(measured_position - lower_bound, time_to_last_measurement),
        )
    } else {
        let (lower_bound, upper_bound) = current.step.bounds(cali);
        (
            Speed::new(lower_bound - measured_position, time_to_current_measurement),
            Speed::new(upper_bound - measured_position, time_to_current_measurement),
        )
    }
}
#[cfg(test)]
mod tests {
    use crate::EQUAL_STEPS;

    use super::*;
    use embassy_time::Duration;

    #[test]
    fn construct_measurement_from_data() {
        let time = Instant::from_secs(1);
        assert_eq!(
            Measurement::new(DirectionDuration(0 - 50), Step::new(42), time, 10),
            Measurement {
                step: Step::new(42),
                direction: Direction::CounterClockwise,
                step_instant: time - Duration::from_micros(65),
                sample_instant: time
            }
        );
    }

    #[test]
    fn last_smaple_time_is_further_away_from_step_time() {
        let delta = Duration::from_millis(10);
        let last_known_position_time = Instant::from_millis(30);
        //NOTE: specificity starting at two rather than zero to avoid the issues that x + 0 = x - 0
        let speed = calculate_speed_bounds(
            Measurement {
                step: Step::new(2),
                direction: Direction::Clockwise,
                //NOTE: This step time does not matter
                step_instant: Instant::from_millis(0),
                sample_instant: last_known_position_time - delta,
            },
            Measurement {
                step: Step::new(12),
                direction: Direction::Clockwise,
                //NOTE: This is the step time we care about.
                step_instant: last_known_position_time,
                sample_instant: last_known_position_time + delta / 2,
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            speed,
            (
                Speed::new(SubStep::new(64 * 9), delta),
                Speed::new(SubStep::new(64 * 10), delta)
            )
        );
    }
    #[test]
    fn current_smaple_time_is_further_away_from_step_time() {
        //Since the larger time windows is withing the step,
        //we completely ignore the previous measurement
        let delta = Duration::from_millis(10);
        let last_known_position_time = Instant::from_millis(30);
        let speed = calculate_speed_bounds(
            Measurement {
                step: Step::new(0),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(0),
                sample_instant: last_known_position_time - delta / 2,
            },
            Measurement {
                step: Step::new(10),
                direction: Direction::Clockwise,
                step_instant: last_known_position_time,
                sample_instant: last_known_position_time + delta,
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            speed,
            (
                Speed::new(SubStep::new(0), delta),
                Speed::new(SubStep::new(64), delta)
            )
        );
    }
    #[test]
    fn speed_calculation() {
        let speed = calculate_speed(
            Measurement {
                step: Step::new(10),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(10),
                sample_instant: Instant::from_millis(10),
            },
            Measurement {
                step: Step::new(20),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(20),
                sample_instant: Instant::from_millis(20),
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            speed,
            Speed::new(SubStep::new(10 * 64), Duration::from_millis(10))
        )
    }
    #[test]
    fn testing_inter_step_bounds() {
        let speed = calculate_speed_bounds(
            Measurement {
                step: Step::new(3),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(0),
                sample_instant: Instant::from_millis(0),
            },
            Measurement {
                step: Step::new(3),
                direction: Direction::Clockwise,
                step_instant: Instant::from_millis(0),
                sample_instant: Instant::from_millis(5),
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            speed,
            (
                Speed::new(SubStep::new(0), Duration::from_millis(10)),
                Speed::new(SubStep::new(64), Duration::from_millis(5))
            )
        )
    }
}
