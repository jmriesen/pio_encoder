use core::ops::AddAssign;

use embassy_time::{Duration, Ticker};

use crate::{CalibrationData, EncoderStateMachine};
use crate::{Direction, Measurement, SubStep};

/// Default calibration value that assumes each encoder tick is the same size
pub const EQUAL_STEPS: CalibrationData = CalibrationData([
    SubStep::new(0),
    SubStep::new(64),
    SubStep::new(128),
    SubStep::new(192),
]);

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
}
impl From<PhaseLengths> for CalibrationData {
    fn from(phase_data: PhaseLengths) -> CalibrationData {
        const {
            // if `RESCALE_THRESHOLD` does not panic when passed to normalize nothing smaller than
            // it should either. (checking at compile time)
            assert!(255 == normalize(RESCALE_THRESHOLD, RESCALE_THRESHOLD))
        }
        ///Scales a duration 0s-total_time into an u8 0-255::Max
        const fn normalize(value: Duration, total_time: Duration) -> i32 {
            //Adding half the divisor before divide is a trick to round rather than truncate.
            let half_total = total_time.as_ticks() / 2;
            let numerator = u8::MAX as u64 * value.as_ticks() + half_total;
            let normalized_value = numerator / total_time.as_ticks();
            normalized_value as i32
        }
        let calibration_data = [
            //The first phase definitionally starts immediately,
            Duration::from_millis(0),
            phase_data.0[0],
            phase_data.0[0] + phase_data.0[1],
            phase_data.0[0] + phase_data.0[1] + phase_data.0[3],
        ];
        CalibrationData(
            calibration_data
                .map(|duration| normalize(duration, phase_data.inner_sum()))
                .map(SubStep::new),
        )
    }
}

/// Repeatedly attempt to sample data until we have a duration for each step of a cycle.
async fn sample_phase_lengths(state_machine: &impl EncoderStateMachine) -> PhaseLengths {
    async fn try_sample_phase_lengths(
        state_machine: &impl EncoderStateMachine,
    ) -> Option<PhaseLengths> {
        let mut ticker = Ticker::every(Duration::from_millis(1));
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

async fn calibrate_encoder(state_machine: &impl EncoderStateMachine) -> CalibrationData {
    let mut running_total = PhaseLengths([Duration::from_millis(0); 4]);
    // Number of samples to take (just a heuristic)
    for _ in 0..32 {
        let sample = sample_phase_lengths(state_machine).await;
        running_total += sample;
    }
    running_total.into()
}

#[cfg(test)]
pub mod test {
    use embassy_futures::select::select;
    use embassy_time::{Duration, Instant};

    use crate::{
        Direction::{self, *},
        EncoderStateMachine, Step,
        calibration::sample_phase_lengths,
        measurement::tests::MockPio,
        mock::{MockSensor, advance_embassy_clock, block_on_with_timer},
    };

    fn simulate(
        events: ((Step, crate::Direction, Instant), Vec<(Duration, Step)>),
        assert: impl AsyncFn(MockSensor),
    ) {
        let (sensor, mock_runner) = MockSensor::new(events.0, events.1);
        block_on_with_timer(select(mock_runner.run(), assert(sensor)));
    }

    #[test]
    fn balanced_mesurement() {
        let events = (
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(10), Step::new(1)),
                (Duration::from_millis(10), Step::new(2)),
                (Duration::from_millis(10), Step::new(3)),
                (Duration::from_millis(10), Step::new(4)),
                (Duration::from_millis(10), Step::new(5)),
            ],
        );
        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [10, 10, 10, 10].map(Duration::from_millis)
            );
        });
    }
    #[test]
    fn balanced_mesurement_clockwise() {
        let events = (
            (Step::new(0), Clockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(10), Step::new(-1)),
                (Duration::from_millis(10), Step::new(-2)),
                (Duration::from_millis(10), Step::new(-3)),
                (Duration::from_millis(10), Step::new(-4)),
                (Duration::from_millis(10), Step::new(-5)),
            ],
        );
        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [10, 10, 10, 10].map(Duration::from_millis)
            );
        })
    }
    #[test]
    fn unbalanced_mesurement() {
        let events = (
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(5), Step::new(1)),
                (Duration::from_millis(10), Step::new(2)),
                (Duration::from_millis(20), Step::new(3)),
                (Duration::from_millis(5), Step::new(4)),
            ],
        );

        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [5, 10, 20, 5].map(Duration::from_millis)
            );
        })
    }
    fn run_with_offset(i: i32, expected: [u64; 4]) {
        let events = (
            (Step::new(0 + i), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(1), Step::new(1 + i)),
                (Duration::from_millis(2), Step::new(2 + i)),
                (Duration::from_millis(3), Step::new(3 + i)),
                (Duration::from_millis(4), Step::new(4 + i)),
            ],
        );
        simulate(events, async |sensor| {
            let foo = sample_phase_lengths(&sensor).await.0;
            assert_eq!(foo, expected.map(Duration::from_millis));
        })
    }

    #[test]
    fn mesurements_can_start_on_phase_0() {
        run_with_offset(0, [1, 2, 3, 4]);
    }
    #[test]
    fn mesurements_can_start_on_phase_1() {
        run_with_offset(1, [2, 3, 4, 1]);
    }
    #[test]
    fn mesurements_can_start_on_phase_2() {
        run_with_offset(2, [3, 4, 1, 2]);
    }
    #[test]
    fn mesurements_can_start_on_phase_3() {
        run_with_offset(3, [4, 1, 2, 3]);
    }

    #[test]
    fn exclude_long_time_deltas() {
        let events = (
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

        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [11, 20, 4, 10].map(Duration::from_millis)
            );
        })
    }
    #[test]
    fn exclude_non_adjacent_steps() {
        let events = (
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

        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [7, 4, 5, 6,].map(Duration::from_millis)
            );
        })
    }

    #[test]
    fn jump_forward_momentaraly() {
        let events = (
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

        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [5, 6, 7, 8].map(Duration::from_millis)
            );
        })
    }

    #[test]
    fn jump_back_momentaraly() {
        let events = (
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(1), Step::new(1)),
                (Duration::from_millis(2), Step::new(2)),
                (
                    // Subtracting 2 to compensate for the two jumps
                    Duration::from_millis(3) - Duration::from_micros(20),
                    Step::new(3),
                ),
                //Jump back
                (Duration::from_micros(10), Step::new(0)),
                // Jump forward (Keep sampling since there is no real way to distinguish current
                // state from the state right before the jump)
                (Duration::from_micros(10), Step::new(3)),
                (Duration::from_millis(4), Step::new(4)),
            ],
        );

        simulate(events, async |sensor| {
            assert_eq!(
                sample_phase_lengths(&sensor).await.0,
                [1, 2, 3, 4].map(Duration::from_millis)
            );
        })
    }
}
