use core::ops::AddAssign;

use embassy_time::{Duration, Ticker};

use crate::{Direction, Measurement};

trait EncoderStateMachine {
    /// Returns whatever data is currently stored in the PIO State Machine.
    /// Since this reflects real world data, repeated calls to read ***CAN RESULT IN Different
    /// VALUES***
    fn read(&self) -> Measurement;
}

///Max value of PhaseLengths sample data before we have to rescale.
// Chosen since it is the largest a power of 16 that does not cause an overflow when passed to `normalize`
const RESCALE_THRESHOLD: Duration = Duration::from_secs(0xF_FF_FF_FF_FF);

/// Represents a measure of how long each phase takes.
// Index 0 represents the length of ticks 0,4,8... index 1 ticks 1,5,7 ext.
// Absolute values of each index is not meaningful, but their relative magnitudes is.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct PhaseLengths([Duration; 4]);

impl AddAssign for PhaseLengths {
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..self.0.len() {
            self.0[i] += rhs.0[i]
        }
        if self.inner_sum() > RESCALE_THRESHOLD {
            for entry in &mut self.0 {
                *entry /= 2
            }
        }
    }
}

impl PhaseLengths {
    /// Sum of all the phase lengths
    /// Used for resizing and normalizing data.
    fn inner_sum(&self) -> Duration {
        self.0.iter().cloned().sum::<Duration>()
    }

    pub fn to_configuration_data(&self) -> [u8; 4] {
        const {
            // if `RESCALE_THRESHOLD` does not panic when passed to normalize nothing smaller than
            // it should either. (checking at compile time)
            assert!(255 == normalize(RESCALE_THRESHOLD, RESCALE_THRESHOLD))
        }
        ///Scales a duration 0s-total_time into an u8 0-u8::Max
        const fn normalize(value: Duration, total_time: Duration) -> u8 {
            //Adding half the divisor before divide is a trick to round rather than truncate.
            let half_total = total_time.as_ticks() / 2;
            let numerator = u8::MAX as u64 * value.as_ticks() + half_total;
            let normalized_value = numerator / total_time.as_ticks();
            normalized_value as u8
        }
        let cycle_start_to_phase_start = [
            //The first phase definitionally starts immediately,
            Duration::from_millis(0),
            self.0[0],
            self.0[0] + self.0[1],
            self.0[0] + self.0[1] + self.0[3],
        ];
        cycle_start_to_phase_start.map(|duration| normalize(duration, self.inner_sum()))
    }
}

/// Repeatedly attempt to sample data until we have a duration for each step of a cycle.
async fn sample_phase_lengths(state_machine: &impl EncoderStateMachine) -> PhaseLengths {
    async fn try_sample_phase_lengths(
        state_machine: &impl EncoderStateMachine,
    ) -> Option<PhaseLengths> {
        let mut ticker = Ticker::every(Duration::from_millis(1));
        ticker.next().await;
        let inital = state_machine.read();

        // sample cycle
        let (delta_0, current) = sample_step_len(state_machine, &mut ticker, inital).await?;
        let (delta_1, currnet) = sample_step_len(state_machine, &mut ticker, current).await?;
        let (delta_2, current) = sample_step_len(state_machine, &mut ticker, currnet).await?;
        let (delta_3, _) = sample_step_len(state_machine, &mut ticker, current).await?;

        // Adjust to for starting phase
        let mut deltas = [delta_0, delta_1, delta_2, delta_3];
        deltas.rotate_left(inital.step.phase());
        Some(PhaseLengths(deltas))
    }
    loop {
        if let Some(mesurement) = try_sample_phase_lengths(state_machine).await {
            break mesurement;
        }
    }
}

async fn sample_step_len(
    state_machine: &impl EncoderStateMachine,
    ticker: &mut Ticker,
    current: Measurement,
) -> Option<(Duration, Measurement)> {
    let next_step = loop {
        ticker.next().await;
        let next = state_machine.read();
        match next.step.raw() - current.step.raw() {
            1 if current.direction == Direction::CounterClockwise => {
                break Some(next);
            }
            -1 if current.direction == Direction::Clockwise => {
                break Some(next);
            }
            0 => continue,
            _ => break None,
        }
    }?;

    let delta_t = next_step.step_instant - current.step_instant;
    let changed_direction = current.direction != next_step.direction;

    if changed_direction || delta_t > Duration::from_millis(20) {
        None
    } else {
        Some((delta_t, next_step))
    }
}

async fn calibrate_encoder(state_machine: &impl EncoderStateMachine) -> [u8; 4] {
    let mut running_total = PhaseLengths([Duration::from_millis(0); 4]);
    // Number of samples to take (just a heuristic)
    for _ in 0..32 {
        let sample = sample_phase_lengths(state_machine).await;
        running_total += sample;
    }
    running_total.to_configuration_data()
}

#[cfg(test)]
mod test {
    use std::sync::{Arc, Mutex};

    use embassy_time::{Duration, Instant, Timer};

    use crate::{
        Direction::{self, *},
        Step,
        calibration::{EncoderStateMachine, sample_phase_lengths},
        measurement::tests::MockPio,
    };

    struct MockSensor {
        mock: Arc<Mutex<MockPio>>,
    }
    struct MockSensorRunner {
        mock: Arc<Mutex<MockPio>>,
        events: Vec<(Duration, Step)>,
    }

    impl MockSensor {
        fn new(
            inital_conditions: (Step, Direction, Instant),
            events: Vec<(Duration, Step)>,
        ) -> (Self, MockSensorRunner) {
            let mock = Arc::new(Mutex::new(MockPio::new(
                inital_conditions.0,
                inital_conditions.1,
                inital_conditions.2,
            )));
            (
                MockSensor { mock: mock.clone() },
                MockSensorRunner { events, mock },
            )
        }
    }
    impl MockSensorRunner {
        async fn run(self) {
            let mut event_time = Instant::from_millis(0);
            for (delta_t, step) in self.events {
                event_time += delta_t;
                Timer::at(event_time).await;
                self.mock.lock().unwrap().position_change(step, event_time);
            }
            Timer::after_secs(1).await;
            panic!("ran out of mesurements");
        }
    }

    impl EncoderStateMachine for MockSensor {
        fn read(&self) -> crate::Measurement {
            self.mock
                .lock()
                .unwrap()
                .take_mesurement(embassy_time::Instant::now())
        }
    }

    #[tokio::test()]
    async fn balanced_mesurement() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(10), Step::new(1)),
                (Duration::from_millis(10), Step::new(2)),
                (Duration::from_millis(10), Step::new(3)),
                (Duration::from_millis(10), Step::new(4)),
                (Duration::from_millis(10), Step::new(5)),
            ],
        );
        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [10, 10, 10, 10].map(Duration::from_millis)
        );
    }
    #[tokio::test()]
    async fn balanced_mesurement_clockwise() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), Clockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(10), Step::new(-1)),
                (Duration::from_millis(10), Step::new(-2)),
                (Duration::from_millis(10), Step::new(-3)),
                (Duration::from_millis(10), Step::new(-4)),
                (Duration::from_millis(10), Step::new(-5)),
            ],
        );
        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [10, 10, 10, 10].map(Duration::from_millis)
        );
    }
    #[tokio::test]
    async fn unbalanced_mesurement() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(5), Step::new(1)),
                (Duration::from_millis(10), Step::new(2)),
                (Duration::from_millis(20), Step::new(3)),
                (Duration::from_millis(5), Step::new(4)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [5, 10, 20, 5].map(Duration::from_millis)
        );
    }
    async fn run_with_offset(i: i32, expected: [u64; 4]) {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0 + i), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(1), Step::new(1 + i)),
                (Duration::from_millis(2), Step::new(2 + i)),
                (Duration::from_millis(3), Step::new(3 + i)),
                (Duration::from_millis(4), Step::new(4 + i)),
            ],
        );
        tokio::spawn(runner.run());
        let foo = sample_phase_lengths(&sensor).await.0;
        assert_eq!(foo, expected.map(Duration::from_millis));
    }

    #[tokio::test]
    async fn mesurements_can_start_on_phase_0() {
        run_with_offset(0, [1, 2, 3, 4]).await;
    }
    #[tokio::test]
    async fn mesurements_can_start_on_phase_1() {
        run_with_offset(1, [2, 3, 4, 1]).await;
    }
    #[tokio::test]
    async fn mesurements_can_start_on_phase_2() {
        run_with_offset(2, [3, 4, 1, 2]).await;
    }
    #[tokio::test]
    async fn mesurements_can_start_on_phase_3() {
        run_with_offset(3, [4, 1, 2, 3]).await;
    }
    #[tokio::test]
    async fn exclude_long_time_deltas() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                // starting to sample
                (Duration::from_millis(10), Step::new(1)),
                // To large a time delta clear out buffer
                (Duration::from_millis(21), Step::new(2)),
                (Duration::from_millis(4), Step::new(3)),
                (Duration::from_millis(10), Step::new(4)),
                (Duration::from_millis(11), Step::new(5)),
                // Max allowed time delta
                (Duration::from_millis(20), Step::new(6)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [11, 20, 4, 10].map(Duration::from_millis)
        );
    }
    #[tokio::test]
    async fn exclude_non_adjacent_steps() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(1), Step::new(1)),
                // Step count jumps so throw out partial sample.
                (Duration::from_millis(3), Step::new(3)),
                (Duration::from_millis(4), Step::new(4)),
                (Duration::from_millis(5), Step::new(5)),
                (Duration::from_millis(6), Step::new(6)),
                (Duration::from_millis(7), Step::new(7)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [7, 4, 5, 6,].map(Duration::from_millis)
        );
    }
    #[tokio::test]
    async fn jump_forward_momentaraly() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(1), Step::new(1)),
                (Duration::from_millis(2), Step::new(2)),
                (
                    Duration::from_millis(3) - Duration::from_micros(10),
                    Step::new(3),
                ),
                //Jump forward
                (Duration::from_micros(1), Step::new(10)),
                //Jump back
                (Duration::from_micros(1), Step::new(3)),
                //Resume
                (Duration::from_millis(4), Step::new(4)),
                (Duration::from_millis(5), Step::new(5)),
                (Duration::from_millis(6), Step::new(6)),
                (Duration::from_millis(7), Step::new(7)),
                (Duration::from_millis(8), Step::new(8)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [5, 6, 7, 8].map(Duration::from_millis)
        );
    }
    #[tokio::test]
    async fn jump_back_momentaraly() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(1), Step::new(1)),
                (Duration::from_millis(2), Step::new(2)),
                (
                    // Subtracting 2 to compensate for the two jumps
                    Duration::from_millis(3) - Duration::from_micros(2),
                    Step::new(3),
                ),
                //Jump back
                (Duration::from_micros(1), Step::new(0)),
                // Jump forward (Keep sampling since there is no real way to distinguish current
                // state from the state right before the jump)
                (Duration::from_micros(1), Step::new(3)),
                (Duration::from_millis(4), Step::new(4)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(
            sample_phase_lengths(&sensor).await.0,
            [1, 2, 3, 4].map(Duration::from_millis)
        );
    }
}
