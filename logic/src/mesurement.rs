use crate::{
    CalibrationData, Direction,
    encodeing::{DirectionDuration, Step, SubStep},
    speed::Speed,
};
use embassy_time::Instant;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Mesurement {
    pub steps: Step,
    pub direction: Direction,
    pub transition_time: embassy_time::Instant,
    pub sample_time: embassy_time::Instant,
}
impl Mesurement {
    pub fn new(
        dir_dur: DirectionDuration,
        steps: Step,
        instant: Instant,
        clocks_per_us: u32,
    ) -> Self {
        let (direction, duration) = dir_dur.decode(clocks_per_us);
        Self {
            steps,
            direction,
            transition_time: instant - duration,
            sample_time: instant,
        }
    }
    ///The position in `SubSteps` given that the encoder is traveling in a specific directing.
    /// The position will always be either the upper or lower `SubStep`value.
    pub fn transition(&self, calibration: &CalibrationData) -> SubStep {
        match self.direction {
            Direction::Clockwise => self.steps.lower_bound(calibration),
            Direction::CounterClockwise => self.steps.upper_bound(calibration),
        }
    }
}
fn calculate_postion_and_speed(
    current_position: SubStep,
    previuse: Mesurement,
    current: Mesurement,
    cali: &CalibrationData,
) -> (SubStep, Speed) {
    let tick_since_prev_mesurement = current.transition_time > previuse.sample_time;
    // Time ->
    // prev_sample_time ----- transition_time ----- current_sample_time
    //                 |--a--|               |--b--|
    //
    let a = current.transition_time - previuse.sample_time;
    let b = current.sample_time - current.transition_time;
    let transition = current.transition(cali);
    let (speed_upper_bound, speed_lower_bound) = if tick_since_prev_mesurement && a > b {
        let (prev_lower, prev_upper) = previuse.steps.bounds(cali);
        (
            Speed::new(transition - prev_lower, a),
            Speed::new(transition - prev_upper, a),
        )
    } else {
        let (lower, upper) = current.steps.bounds(cali);
        (
            Speed::new(upper - transition, b),
            Speed::new(lower - transition, b),
        )
    };

    let speed = Speed::new(
        current.transition(cali) - previuse.transition(cali),
        current.transition_time - previuse.transition_time,
    );
    //.clamp(speed_lower_bound, speed_upper_bound);

    let position = (current_position + speed * b); /*.clamp(
    current.steps.lower_bound(&cali),
    current.steps.upper_bound(&cali),
    );
    */
    (position, speed)
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
                transition_time: time - Duration::from_micros(65),
                sample_time: time
            }
        );
    }
    #[test]
    fn whole_steps() {
        let (position, speed) = calculate_postion_and_speed(
            SubStep::new(0),
            Mesurement {
                steps: Step::new(0),
                direction: Direction::Clockwise,
                transition_time: Instant::from_millis(0),
                sample_time: Instant::from_millis(10),
            },
            Mesurement {
                steps: Step::new(1),
                direction: Direction::Clockwise,
                transition_time: Instant::from_millis(20),
                sample_time: Instant::from_millis(30),
            },
            &EQUAL_STEPS,
        );
        assert_eq!(
            (position, speed),
            (
                SubStep::new(20),
                Speed::new(SubStep::new(10), Duration::from_millis(50))
            )
        );
    }
}
