//! This crate contains all the logic assisted with parsing pio messages and calculating speed.
//! This crate specificity does **not** depend on embassy-rs.
//! Depending on embassy-rs would prevent me from running the unit test on my base machine.
use core::cell::Cell;

use crate::{
    CalibrationData, Direction, EncoderStateMachine, Measurement, Speed, Step, SubStep,
    calibration::{EQUAL_STEPS, PhaseLengths, sample_phase_lengths},
};
use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    watch::{Receiver, Sender, Watch},
};
use embassy_time::{Duration, Ticker};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    pub speed: Speed,
    pub step: Step,
    pub position: SubStep,
    pub direction: Direction,
}

pub struct State<M: RawMutex, const SUB: usize> {
    watch: Watch<M, Status, SUB>,
}
impl<M: RawMutex, const SUB: usize> Default for State<M, SUB> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: RawMutex, const SUB: usize> State<M, SUB> {
    pub fn new() -> Self {
        Self {
            watch: Watch::new(),
        }
    }
    pub fn subscribe(&self) -> Option<Receiver<'_, M, Status, SUB>> {
        self.watch.receiver()
    }
}

/// Stores all the logical state required for the sub-step encoder.
pub struct EncoderRunner<
    's,
    const IDLE_STOPING_TIME_MS: u64,
    PIO: EncoderStateMachine,
    M: RawMutex,
    const SUB: usize,
> {
    status_sink: Sender<'s, M, Status, SUB>,
    pio_state: PIO,
    calibration_data: Cell<CalibrationData>,
}

impl<'s, const IDLE_STOPING_TIME_MS: u64, PIO: EncoderStateMachine, M: RawMutex, const SUB: usize>
    EncoderRunner<'s, IDLE_STOPING_TIME_MS, PIO, M, SUB>
{
    pub fn new(state: &'s State<M, SUB>, sensor: PIO) -> Self {
        Self {
            status_sink: state.watch.sender(),
            pio_state: sensor,
            calibration_data: Cell::new(EQUAL_STEPS),
        }
    }
    pub fn idel_stopping_time() -> Duration {
        Duration::from_millis(IDLE_STOPING_TIME_MS)
    }
    pub async fn run(self, update_rate: Duration) -> ! {
        embassy_futures::join::join(self.run_status_calc(update_rate), self.run_calibration())
            .await
            .0
    }
    pub async fn run_status_calc(&self, update_rate: Duration) -> ! {
        let mut ticker = Ticker::every(update_rate);
        let mut last_mesurement = self.pio_state.read();
        let mut speed = Speed::stopped();

        loop {
            ticker.next().await;
            let current_mesurement = self.pio_state.read();
            speed = if current_mesurement.time_since_step_start() >= Self::idel_stopping_time() {
                Speed::stopped()
            } else {
                Measurement::estimate_speed(
                    speed,
                    last_mesurement,
                    current_mesurement,
                    &self.calibration_data.get(),
                )
            };
            self.status_sink.send(Status {
                speed,
                step: current_mesurement.step,
                position: current_mesurement.step_start(&self.calibration_data.get())
                    + speed * (current_mesurement.time_since_step_start()),
                direction: current_mesurement.direction,
            });
            last_mesurement = current_mesurement;
        }
    }
    pub async fn run_calibration(&self) {
        let mut running_total = PhaseLengths::new();
        //Waiting to apply phase adjustments untill we have a decent sample size.
        for _ in 0..32 {
            running_total += sample_phase_lengths(&self.pio_state).await;
        }
        loop {
            self.calibration_data.set(running_total.into());
            running_total += sample_phase_lengths(&self.pio_state).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        Direction::CounterClockwise,
        calibration::EQUAL_STEPS,
        mock::{MockSensor, block_on_with_timer},
        runner::{State, Status},
        speed::Speed,
        step::{Step, SubStep},
    };
    use embassy_futures::{join::join, select::select};
    use embassy_sync::{blocking_mutex::raw::NoopRawMutex, watch::Receiver};
    use embassy_time::{Duration, Instant, Timer};
    fn simulate(
        events: ((Step, crate::Direction, Instant), Vec<(Instant, Step)>),
        assert: impl AsyncFn(Receiver<'static, NoopRawMutex, Status, 10>),
    ) {
        let (sensor, mock_runner) = MockSensor::new_inst(events.0, events.1);
        let state = Box::leak(Box::new(State::new()));
        let encoder_runner = super::EncoderRunner::<30, _, NoopRawMutex, 10>::new(state, sensor);
        let status = state.watch.receiver().unwrap();

        block_on_with_timer(select(
            join(
                mock_runner.run(),
                encoder_runner.run(Duration::from_millis(10)),
            ),
            assert(status),
        ));
    }

    #[test]
    fn estimate_between_ticks() {
        let events = (
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![(Instant::from_millis(35), Step::new(3))],
        );
        simulate(events, async |mut status| {
            let speeds = vec![
                Speed::stopped(),
                Speed::new(SubStep::new(64 * 3), Duration::from_millis(35)),
                //Reducing speed estimate since we need to stay in the same tick
                Speed::new(SubStep::new(64), Duration::from_millis(15)),
                Speed::new(SubStep::new(64), Duration::from_millis(25)),
                // Timed out
                Speed::stopped(),
            ];
            //Ignore first 30ms
            Timer::after_millis(30).await;
            for speed in speeds {
                assert_eq!(status.changed().await.speed, speed);
            }
        });
    }

    ///This is the example taken from the readme of the code.
    ///(https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder_substep)
    #[test]
    fn example_from_source_documentation() {
        let events = (
            (Step::new(3), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(21), Step::new(4)),
                (Instant::from_millis(34), Step::new(5)),
                (Instant::from_millis(49), Step::new(7)),
            ],
        );
        simulate(events, async |mut status| {
            let expected = vec![
                (Speed::stopped(), Step::new(3), Duration::from_millis(0)),
                (Speed::stopped(), Step::new(3), Duration::from_millis(0)),
                (
                    Speed::new(SubStep::new(64), Duration::from_millis(21)),
                    Step::new(4),
                    Duration::from_millis(9),
                ),
                (
                    Speed::new(SubStep::new(64), Duration::from_millis(13)),
                    Step::new(5),
                    Duration::from_millis(6),
                ),
                (
                    Speed::new(SubStep::new(128), Duration::from_millis(15)),
                    Step::new(7),
                    Duration::from_millis(1),
                ),
            ];
            for (speed, step, time_since_transition) in expected {
                assert_eq!(
                    status.changed().await,
                    Status {
                        speed,
                        step,
                        // In this example we are always going counterclockwise so position is based
                        // off the lower bound
                        position: step.lower_bound(&EQUAL_STEPS) + speed * time_since_transition,
                        direction: CounterClockwise
                    }
                )
            }
        });
    }

    #[test]
    fn hovering_over_a_transition_is_not_considered_movement() {
        let events = (
            (Step::new(3), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(10), Step::new(2)),
                (Instant::from_millis(30), Step::new(3)),
                (Instant::from_millis(50), Step::new(2)),
            ],
        );
        simulate(events, async |mut status| {
            for _ in 0..5 {
                let status = status.changed().await;
                assert_eq!(
                    (status.speed, status.position),
                    (Speed::stopped(), EQUAL_STEPS[3])
                )
            }
        })
    }

    #[test]
    fn use_longer_time_delta() {
        // | Time |  Event  | Note
        // |  0ms | step 0  | init
        // |  1ms | step 1  | delta between steps 1ms
        // | 10ms | reading | delta of 9ms from last transition
        let events = (
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            // Transition time to now is longer
            vec![(Instant::from_millis(1), Step::new(1))],
        );
        simulate(events, async |mut status| {
            assert_eq!(
                status.changed().await.speed,
                Speed::new(SubStep::new(64), Duration::from_millis(9))
            );
        });
        // | Time |  Event  | Note
        // |  0ms | step 0  | init
        // |  6ms | step 1  | delta between steps 6ms
        // | 10ms | reading | delta now to last step 4ms
        let events = (
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            // Last measurement to transition time is longer
            vec![(Instant::from_millis(6), Step::new(1))],
        );
        simulate(events, async |mut status| {
            assert_eq!(
                status.get().await.speed,
                Speed::new(SubStep::new(64), Duration::from_millis(6))
            );
        });
    }
}
