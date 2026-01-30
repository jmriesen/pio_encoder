use embassy_time::Duration;

use crate::Measurement;

trait EncoderStateMachine {
    /// Returns whatever data is currently stored in the PIO State Machine.
    /// Since this reflects real world data, repeated calls to read ***CAN RESULT IN Different
    /// VALUES***
    fn read(&self) -> Measurement;
}
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct CalibrationMesurement([u8; 4]);
async fn try_take_calibration_mesurement(
    state_machine: &impl EncoderStateMachine,
) -> Option<CalibrationMesurement> {
    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(1));

    ticker.next().await;
    let first = state_machine.read();
    let (delta_0, second) = wait_for_step(state_machine, &mut ticker, first).await?;
    let (delta_1, third) = wait_for_step(state_machine, &mut ticker, second).await?;
    let (delta_2, forth) = wait_for_step(state_machine, &mut ticker, third).await?;
    let (delta_3, _) = wait_for_step(state_machine, &mut ticker, forth).await?;
    let mut deltas = [delta_0, delta_1, delta_2, delta_3];
    deltas.rotate_left(first.step.phase());
    Some(CalibrationMesurement(deltas))
}

async fn take_calibration_mesurement(
    state_machine: &impl EncoderStateMachine,
) -> CalibrationMesurement {
    loop {
        if let Some(mesurement) = try_take_calibration_mesurement(state_machine).await {
            break mesurement;
        } else {
        }
    }
}

async fn wait_for_step(
    state_machine: &impl EncoderStateMachine,
    ticker: &mut embassy_time::Ticker,
    first: Measurement,
) -> Option<(u8, Measurement)> {
    loop {
        ticker.next().await;
        let mesurement = state_machine.read();

        match mesurement.step.raw() - first.step.raw() {
            0 => continue,
            1 => {
                break {
                    let delta = mesurement.step_instant - first.step_instant;
                    if delta <= Duration::from_millis(20) {
                        Some((delta.as_millis() as u8, mesurement))
                    } else {
                        None
                    }
                };
            }
            _ => break None,
        }
    }
}

#[cfg(test)]
mod test {
    use std::sync::{Arc, Mutex};

    use embassy_time::{Instant, Timer};

    use crate::{
        Direction::{self, *},
        Step,
        calibration::{EncoderStateMachine, take_calibration_mesurement},
        measurement::tests::MockPio,
    };

    struct MockSensor {
        mock: Arc<Mutex<MockPio>>,
    }
    struct MockSensorRunner {
        mock: Arc<Mutex<MockPio>>,
        events: Vec<(Instant, Step)>,
    }

    impl MockSensor {
        fn new(
            inital_conditions: (Step, Direction, Instant),
            events: Vec<(Instant, Step)>,
        ) -> (Self, MockSensorRunner) {
            assert!(
                events.iter().map(|x| x.0).is_sorted(),
                "Event timestamps must always be increatsing: {:?}",
                events
            );
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
            for (time, step) in self.events {
                Timer::at(time).await;
                self.mock.lock().unwrap().position_change(step, time);
            }
            Timer::after_secs(1).await;
            //panic!("ran out of mesurements");
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
                (Instant::from_millis(10), Step::new(1)),
                (Instant::from_millis(20), Step::new(2)),
                (Instant::from_millis(30), Step::new(3)),
                (Instant::from_millis(40), Step::new(4)),
                (Instant::from_millis(50), Step::new(5)),
            ],
        );
        tokio::spawn(runner.run());
        assert_eq!(
            take_calibration_mesurement(&sensor).await.0,
            [10, 10, 10, 10]
        );
    }
    #[tokio::test]
    async fn unbalanced_mesurement() {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(5), Step::new(1)),
                (Instant::from_millis(15), Step::new(2)),
                (Instant::from_millis(35), Step::new(3)),
                (Instant::from_millis(40), Step::new(4)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(take_calibration_mesurement(&sensor).await.0, [5, 10, 20, 5]);
    }
    async fn run_with_offset(i: i32, expected: [u8; 4]) {
        let (sensor, runner) = MockSensor::new(
            (Step::new(0 + i), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(1), Step::new(1 + i)),
                (Instant::from_millis(3), Step::new(2 + i)),
                (Instant::from_millis(6), Step::new(3 + i)),
                (Instant::from_millis(10), Step::new(4 + i)),
            ],
        );
        tokio::spawn(runner.run());
        let foo = take_calibration_mesurement(&sensor).await.0;
        assert_eq!(foo, expected);
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
                (Instant::from_millis(10), Step::new(1)),
                // To large a time delta clear out buffer
                (Instant::from_millis(31), Step::new(2)),
                //
                (Instant::from_millis(35), Step::new(3)),
                (Instant::from_millis(45), Step::new(4)),
                (Instant::from_millis(56), Step::new(5)),
                (Instant::from_millis(76), Step::new(6)),
            ],
        );

        tokio::spawn(runner.run());
        assert_eq!(
            take_calibration_mesurement(&sensor).await.0,
            [11, 20, 4, 10]
        );
    }
}
