use core::ops::Range;

use crate::{
    CalibrationData, Direction,
    encodeing::DirectionDuration,
    speed::Speed,
    step::{Step, SubStep},
};
use embassy_time::{Duration, Instant};

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
    pub fn time_since_transition(&self) -> Duration {
        self.sample_instant - self.step_instant
    }
}

impl Measurement {
    pub fn calculate_speed(
        previous: Measurement,
        current: Measurement,
        calibration_data: &CalibrationData,
    ) -> Option<Speed> {
        //TODO: look back at this.
        //Should I be using step or transitions?
        //In theory we could go step 1/ measure /step 2/step 1 /measure
        //In the above case it would be valid to conclude we are moving at 0 (net) speed.( we may
        //be wobbling on the edge of a transition.)
        // TODO Think up some test cases to decide desired behavior.
        if previous.step == current.step {
            //No new transitions have occurred, we cannot provide an updated speed estimate
            None
        } else {
            Some(Speed::new(
                current.transition(calibration_data) - previous.transition(calibration_data),
                current.step_instant - previous.step_instant,
            ))
        }
    }

    /// Calculate the lower and upper speed bounds giving the current and previous measurements
    pub fn calculate_speed_bounds(
        previous: Measurement,
        current: Measurement,
        cali: &[u8; 4],
    ) -> Range<Speed> {
        let transition_point = current.transition(cali);
        // Insure duration is always positive.
        let delta_prev_to_t = duration_dif_abs(previous.sample_instant, current.step_instant);
        let delta_t_to_current = current.time_since_transition();

        // We want to always use the largest time delta possible.
        // There are a couple of scenarios that could be happening.
        // |previous| transition| current | where previous-transition > delta transition-current.
        // |previous| transition| current | where previous-transition < delta transition-current.
        // |transition| previous| current | therefore previous-transition < delta transition-current.
        // The first case uses the previous measurement sample time all the others use the current
        // measurement sample time.
        if delta_prev_to_t > delta_t_to_current {
            let range = previous.step.substep_range(cali);
            //NOTE: this is (initial - final) rather than (final-initial) to compensate for the fact
            //that embassy doesn't support negative durations.
            Speed::new(transition_point - range.end, delta_prev_to_t)
                ..Speed::new(transition_point - range.start, delta_prev_to_t)
        } else {
            let range = current.step.substep_range(cali);
            Speed::new(range.start - transition_point, delta_t_to_current)
                ..Speed::new(range.end - transition_point, delta_t_to_current)
        }
    }
    pub fn estimate_speed(
        last_known_speed: Speed,
        previous: Measurement,
        current: Measurement,
        cali: &[u8; 4],
    ) -> Speed {
        let speed_bounds = Measurement::calculate_speed_bounds(previous, current, cali);
        Measurement::calculate_speed(previous, current, cali)
            .unwrap_or(last_known_speed.clamp(speed_bounds.start, speed_bounds.end))
    }
}

/// Get the absolute value of the duration between two instances.
#[mutants::skip]
fn duration_dif_abs(
    t_0: embassy_time::Instant,
    t_1: embassy_time::Instant,
) -> embassy_time::Duration {
    if t_0 > t_1 { t_0 - t_1 } else { t_1 - t_0 }
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
                        // If we have crossed back over the last transition point the direction of
                        // travel has flipped
                        std::cmp::Ordering::Equal => current_dir.invert(),
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
    fn always_use_larger_delta_time_for_estiments() {
        let step_time = Instant::from_millis(30);
        let smaller_delta = Duration::from_millis(5);
        let larger_delta = Duration::from_millis(10);
        let start = Step::new(5);
        let end = Step::new(15);
        {
            // larger delta happens first
            let mesurements = sequence_events(
                (Step::new(5), Direction::Clockwise, Instant::from_millis(0)),
                vec![
                    (step_time - larger_delta, Event::Mesurement),
                    //Larger delta time.
                    (step_time, Event::Step(15)),
                    //Smaller delta time.
                    (step_time + smaller_delta, Event::Mesurement),
                ],
            );
            assert_eq!(
                Measurement::calculate_speed_bounds(mesurements[0], mesurements[1], &EQUAL_STEPS),
                Speed::new(
                    end.lower_bound(&EQUAL_STEPS) - start.upper_bound(&EQUAL_STEPS),
                    larger_delta
                )
                    ..Speed::new(
                        end.upper_bound(&EQUAL_STEPS) - start.upper_bound(&EQUAL_STEPS),
                        larger_delta
                    )
            );
        }
        {
            // Smaller delta happens first
            let mesurements = sequence_events(
                (Step::new(5), Direction::Clockwise, Instant::from_millis(0)),
                vec![
                    (step_time - smaller_delta, Event::Mesurement),
                    //Larger delta time.
                    (step_time, Event::Step(15)),
                    //Smaller delta time.
                    (step_time + larger_delta, Event::Mesurement),
                ],
            );

            assert_eq!(
                Measurement::calculate_speed_bounds(mesurements[0], mesurements[1], &EQUAL_STEPS),
                Speed::stopped()
                    ..Speed::new(
                        Step::new(end.raw() + 1).upper_bound(&EQUAL_STEPS)
                            - end.upper_bound(&EQUAL_STEPS),
                        larger_delta
                    )
            );
        }
        {
            // symmetric delta
            let mesurements = sequence_events(
                (Step::new(5), Direction::Clockwise, Instant::from_millis(0)),
                vec![
                    (step_time - larger_delta, Event::Mesurement),
                    (step_time, Event::Step(15)),
                    (step_time + larger_delta, Event::Mesurement),
                ],
            );

            assert_eq!(
                Measurement::calculate_speed_bounds(mesurements[0], mesurements[1], &EQUAL_STEPS),
                Speed::stopped()
                    ..Speed::new(
                        Step::new(end.raw() + 1).upper_bound(&EQUAL_STEPS)
                            - end.upper_bound(&EQUAL_STEPS),
                        larger_delta
                    )
            );
        }
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
            Some(Speed::new(SubStep::new(10 * 64), Duration::from_millis(10)))
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
