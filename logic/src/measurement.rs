use core::ops::Range;

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
    /// The subset where the most recent step step occurred.
    pub fn transition(&self, calibration: &CalibrationData) -> SubStep {
        match self.direction {
            Direction::Clockwise => self.step.lower_bound(calibration),
            Direction::CounterClockwise => self.step.upper_bound(calibration),
        }
    }
}

impl Measurement {
    pub fn calculate_speed(
        previous: Measurement,
        current: Measurement,
        calibration_data: &CalibrationData,
    ) -> Speed {
        Speed::new(
            current.transition(calibration_data) - previous.transition(calibration_data),
            current.step_instant - previous.step_instant,
        )
    }

    /// Calculate the lower and upper speed bounds giving the current and previous measurements
    pub fn calculate_speed_bounds(
        previous: Measurement,
        current: Measurement,
        cali: &[u8; 4],
    ) -> Range<Speed> {
        let measured_position = current.transition(cali);
        let first_measurement_in_step = current.step_instant > previous.sample_instant;

        let time_to_last_measurement = if first_measurement_in_step {
            current.step_instant - previous.sample_instant
        } else {
            previous.sample_instant - current.step_instant
        };
        let time_to_current_measurement = current.sample_instant - current.step_instant;
        let previous_sample_is_farther_away =
            time_to_last_measurement > time_to_current_measurement;
        //If this is the first measurement in this encoder step we have two time frames we could chose
        //from:
        //1) Previous measurement to the step_instance.
        //2) The step_instance to the current measurement time
        //Using the longer delta time gives less uncertainty in our estimates
        if first_measurement_in_step && previous_sample_is_farther_away {
            let range = previous.step.substep_range(cali);
            //NOTE: this is (initial - final) rather than (final-initial) to compensate for the fact
            //that embassy doesn't support negative durations.
            Speed::new(measured_position - range.end, time_to_last_measurement)
                ..Speed::new(measured_position - range.start, time_to_last_measurement)
        } else {
            let range = current.step.substep_range(cali);
            Speed::new(range.start - measured_position, time_to_current_measurement)
                ..Speed::new(range.end - measured_position, time_to_current_measurement)
        }
    }
}
#[cfg(test)]
pub mod tests {
    use crate::EQUAL_STEPS;

    use super::*;
    use embassy_time::Duration;
    pub enum Event {
        Step(i32),
        Mesurement,
    }
    /// Takes a sequence of measurement/hardware events and converts them into mesurements the pio
    /// state machine would generate.
    pub fn sequence_events(
        inital_conditions: (Step, Direction, Instant),
        events: impl IntoIterator<Item = (Instant, Event)>,
    ) -> Vec<Measurement> {
        use Direction::*;
        let mut current_step = inital_conditions.0;
        let mut current_dir = inital_conditions.1;
        let mut step_time = inital_conditions.2;
        let mut mesurements = vec![];
        for (time, event) in events {
            match event {
                Event::Step(step) => {
                    let step = Step::new(step);
                    current_dir = match step.cmp(&current_step) {
                        std::cmp::Ordering::Less => CounterClockwise,
                        std::cmp::Ordering::Equal => current_dir,
                        std::cmp::Ordering::Greater => Clockwise,
                    };
                    current_step = step;
                    step_time = time
                }
                Event::Mesurement => mesurements.push(Measurement {
                    step: current_step,
                    direction: current_dir,
                    step_instant: step_time,
                    sample_instant: time,
                }),
            }
        }
        mesurements
    }

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
        let mesurements = sequence_events(
            (Step::new(2), Direction::Clockwise, Instant::from_millis(0)),
            vec![
                (last_known_position_time - delta, Event::Mesurement),
                (last_known_position_time, Event::Step(12)),
                (last_known_position_time + delta / 2, Event::Mesurement),
            ],
        );
        let speed =
            Measurement::calculate_speed_bounds(mesurements[0], mesurements[1], &EQUAL_STEPS);
        assert_eq!(
            speed,
            Speed::new(SubStep::new(64 * 9), delta)..Speed::new(SubStep::new(64 * 10), delta)
        );
    }
    #[test]
    fn current_smaple_time_is_further_away_from_step_time() {
        let delta = Duration::from_millis(10);
        let last_known_position_time = Instant::from_millis(30);

        //NOTE: specificity starting at two rather than zero to avoid the issues that x + 0 = x - 0
        let mesurements = sequence_events(
            (Step::new(0), Direction::Clockwise, Instant::from_millis(0)),
            vec![
                //Since the larger time windows is withing the step,
                //we completely ignore the previous measurement
                (last_known_position_time - delta / 2, Event::Mesurement),
                (last_known_position_time, Event::Step(10)),
                (last_known_position_time + delta, Event::Mesurement),
            ],
        );

        let speed =
            Measurement::calculate_speed_bounds(mesurements[0], mesurements[1], &EQUAL_STEPS);
        assert_eq!(
            speed,
            Speed::new(SubStep::new(0), delta)..Speed::new(SubStep::new(64), delta)
        );
    }

    #[test]
    fn speed_calculation() {
        let mesurements = sequence_events(
            (
                Step::new(10),
                Direction::Clockwise,
                Instant::from_millis(10),
            ),
            vec![
                (Instant::from_millis(10), Event::Mesurement),
                (Instant::from_millis(20), Event::Step(20)),
                (Instant::from_millis(20), Event::Mesurement),
            ],
        );
        assert_eq!(
            Measurement::calculate_speed(mesurements[0], mesurements[1], &EQUAL_STEPS,),
            Speed::new(SubStep::new(10 * 64), Duration::from_millis(10))
        );
    }

    #[test]
    fn testing_intar_step_bounds() {
        let mesurements = sequence_events(
            (Step::new(0), Direction::Clockwise, Instant::from_millis(0)),
            vec![
                //Start moving clockwise.
                (Instant::from_millis(10), Event::Step(3)),
                //Take two mesurements without a tick between them
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Mesurement), // 20 ms since step
                (Instant::from_millis(40), Event::Mesurement), // 30 ms since step
                //Start moving clockwise counter clockwise.
                (Instant::from_millis(50), Event::Step(2)),
                //Take two mesurements without a tick between them
                (Instant::from_millis(60), Event::Mesurement),
                (Instant::from_millis(70), Event::Mesurement), // 20 ms since step
                (Instant::from_millis(80), Event::Mesurement), // 30 ms since step
            ],
        );
        //Moving clockwise
        assert_eq!(
            Measurement::calculate_speed_bounds(mesurements[0], mesurements[1], &EQUAL_STEPS),
            Speed::stopped()..Speed::new(SubStep::new(64), Duration::from_millis(20))
        );
        assert_eq!(
            Measurement::calculate_speed_bounds(mesurements[1], mesurements[2], &EQUAL_STEPS),
            Speed::stopped()..Speed::new(SubStep::new(64), Duration::from_millis(30))
        );

        //Moving counterclockwise
        assert_eq!(
            Measurement::calculate_speed_bounds(mesurements[3], mesurements[4], &EQUAL_STEPS),
            Speed::new(SubStep::new(-64), Duration::from_millis(20))..Speed::stopped()
        );
        assert_eq!(
            Measurement::calculate_speed_bounds(mesurements[4], mesurements[5], &EQUAL_STEPS),
            Speed::new(SubStep::new(-64), Duration::from_millis(30))..Speed::stopped()
        );
    }
}
