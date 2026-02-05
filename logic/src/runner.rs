//! This crate contains all the logic assisted with parsing pio messages and calculating speed.
//! This crate specificity does **not** depend on embassy-rs.
//! Depending on embassy-rs would prevent me from running the unit test on my base machine.
use crate::{
    Direction, EncoderStateMachine, Measurement, Speed, Step, SubStep, calibration::EQUAL_STEPS,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    watch::{Sender, Watch},
};
use embassy_time::{Duration, Ticker};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Status {
    speed: Speed,
    step: Step,
    position: SubStep,
    direction: Direction,
}

// TODO: make generic over watchers and raw mutex
/// Resources required for the encoder and runner to talk to each other
pub struct State {
    watch: Watch<CriticalSectionRawMutex, Status, 10>,
}
impl State {
    pub fn new() -> Self {
        Self {
            //TODO: consider switching to new_with
            watch: Watch::new(),
        }
    }
}

/// Stores all the logical state required for the sub-step encoder.
///
///NOTE: this intentionally does not rely on `embasy_rp` as that would prevent me from running the unit tests on my host machine.
///TODO: Make generic over mutex
pub struct EncoderRunner<'s, const IDLE_STOPING_TIME_MS: u64, PIO: EncoderStateMachine> {
    status_sink: Sender<'s, CriticalSectionRawMutex, Status, 10>,
    pio_state: PIO,
}

impl<'s, const IDLE_STOPING_TIME_MS: u64, PIO: EncoderStateMachine>
    EncoderRunner<'s, IDLE_STOPING_TIME_MS, PIO>
{
    pub fn new(state: &'s State, sensor: PIO) -> Self {
        Self {
            status_sink: state.watch.sender(),
            pio_state: sensor,
        }
    }
    pub fn idel_stopping_time() -> Duration {
        Duration::from_millis(IDLE_STOPING_TIME_MS)
    }
    pub async fn run(self, update_rate: Duration) {
        let mut ticker = Ticker::every(update_rate);
        let mut last_mesurement = self.pio_state.read();
        let mut speed = Speed::stopped();

        loop {
            ticker.next().await;
            let current_mesurement = self.pio_state.read();
            speed = if current_mesurement.time_since_transition() >= Self::idel_stopping_time() {
                Speed::stopped()
            } else {
                Measurement::estimate_speed(
                    speed,
                    last_mesurement,
                    current_mesurement,
                    &EQUAL_STEPS,
                )
            };
            self.status_sink.send(Status {
                speed,
                step: current_mesurement.step,
                position: current_mesurement.transition(&EQUAL_STEPS)
                    + speed * (current_mesurement.time_since_transition()),
                direction: current_mesurement.direction,
            });
            last_mesurement = current_mesurement;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        Direction::CounterClockwise,
        EQUAL_STEPS,
        measurement::{
            Measurement,
            tests::{Event, sequence_events},
        },
        mock::{MockSensor, advance_embassy_clock},
        runner::{EncoderRunner, State, Status},
        speed::Speed,
        step::{Step, SubStep},
    };
    use embassy_time::{Duration, Instant, Timer};

    /*
    fn simulate_assert(
        measurements: Vec<Measurement>,
        speeds: Vec<Speed>,
        positions: Vec<SubStep>,
    ) {
        // Check that we did not forget to pass anything.
        assert_eq!(measurements.len(), speeds.len());
        assert_eq!(measurements.len(), positions.len());

        //Asserts that should be run after every measurement;
        let asserts = |_measurement: Measurement,
                       speed: Speed,
                       position: SubStep,
                       encoder_state: &EncoderRunner<30>| {
            assert_eq!(speed, encoder_state.speed());
            assert_eq!(position, encoder_state.position());
        };

        let mut measurements_and_expected = measurements
            .into_iter()
            .zip(speeds.into_iter())
            .zip(positions.into_iter());

        let ((inital, speed), position) = measurements_and_expected.next().unwrap();
        let mut encoder_state = EncoderRunner::<30>::new(dbg!(inital));
        asserts(inital, speed, position, &encoder_state);

        for ((measurement, speed), position) in measurements_and_expected {
            encoder_state.update(measurement);
            asserts(measurement, speed, position, &encoder_state);
        }
    }

    #[test]
    fn estimate_between_ticks() {
        let measurements = sequence_events(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                // we start off stopped
                (Instant::from_millis(0), Event::Mesurement),
                // Start moving
                (Instant::from_millis(35), Event::Step(3)),
                // Use real speed
                (Instant::from_millis(40), Event::Mesurement),
                // Use keep using last speed estimate.
                (Instant::from_millis(45), Event::Mesurement),
                // Using last speed estimate would push position into the next step.
                // Estimate current speed is the max possible that does not push position into the
                // next step
                (Instant::from_millis(50), Event::Mesurement),
                // Last ms before time out.
                (Instant::from_millis(64), Event::Mesurement),
                // Time out and consider the encoder stopped
                (Instant::from_millis(65), Event::Mesurement),
            ],
        );

        let speeds = vec![
            Speed::stopped(),
            Speed::new(SubStep::new(64 * 3), Duration::from_millis(35)),
            Speed::new(SubStep::new(64 * 3), Duration::from_millis(35)),
            //Reducing speed estimate since we need to stay in the same tick
            Speed::new(SubStep::new(64), Duration::from_millis(15)),
            Speed::new(SubStep::new(64), Duration::from_millis(29)),
            // Timed out
            Speed::stopped(),
        ];
        let positions = vec![
            SubStep::new(0),
            Step::new(3).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(5),
            Step::new(3).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(10),
            // Clamp position at the end of the step.
            Step::new(3).upper_bound(&EQUAL_STEPS) - SubStep::new(1),
            Step::new(3).upper_bound(&EQUAL_STEPS) - SubStep::new(1),
            // Revert position back to last known transition once we are stopped.
            Step::new(3).lower_bound(&EQUAL_STEPS),
        ];
        simulate_assert(measurements, speeds, positions);
    }

    #[test]
    fn step_and_mesurement_happen_at_the_same_time() {
        let measurements = sequence_events(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(0), Event::Mesurement),
                (Instant::from_millis(10), Event::Step(1)),
                (Instant::from_millis(10), Event::Mesurement),
                (Instant::from_millis(20), Event::Step(2)),
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Step(4)),
                (Instant::from_millis(30), Event::Mesurement),
            ],
        );
        let speeds = vec![
            Speed::stopped(),
            Speed::new(SubStep::new(64), Duration::from_millis(10)),
            Speed::new(SubStep::new(64), Duration::from_millis(10)),
            Speed::new(SubStep::new(64 * 2), Duration::from_millis(10)),
        ];
        let positions = vec![
            SubStep::new(0),
            Step::new(1).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(0),
            Step::new(2).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(0),
            Step::new(4).lower_bound(&EQUAL_STEPS) + speeds[2] * Duration::from_millis(0),
        ];

        simulate_assert(measurements, speeds, positions);
    }
    */

    ///This is the example taken from the readme of the code.
    ///(https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder_substep)
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn example_from_source_documentation() {
        let (sensor, mock_runner) = MockSensor::new(
            (Step::new(3), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Duration::from_millis(21), Step::new(4)),
                (Duration::from_millis(13), Step::new(5)),
                (Duration::from_millis(15), Step::new(7)),
            ],
        );
        let expected = vec![
            (
                Instant::from_millis(1),
                Speed::stopped(),
                Step::new(3),
                Duration::from_millis(0),
            ),
            (
                Instant::from_millis(31),
                Speed::new(SubStep::new(64), Duration::from_millis(21)),
                Step::new(4),
                Duration::from_millis(9),
            ),
            (
                Instant::from_millis(41),
                Speed::new(SubStep::new(64), Duration::from_millis(13)),
                Step::new(5),
                Duration::from_millis(6),
            ),
            (
                Instant::from_millis(51),
                Speed::new(SubStep::new(128), Duration::from_millis(15)),
                Step::new(7),
                Duration::from_millis(1),
            ),
        ];
        let state = Box::leak(Box::new(State::new()));
        let encoder_runner = super::EncoderRunner::<30, _>::new(state, sensor);
        let mut status = state.watch.receiver().unwrap();

        tokio::spawn(mock_runner.run());
        tokio::spawn(encoder_runner.run(Duration::from_millis(10)));
        tokio::spawn(advance_embassy_clock());
        for (time, speed, step, time_since_transition) in expected {
            Timer::at(time).await;
            assert_eq!(
                status.get().await,
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
    }

    /*
    #[test]
    fn hovering_over_a_transition_is_not_considered_movement() {
        let measurements = sequence_events(
            (Step::new(3), CounterClockwise, Instant::from_millis(0)),
            vec![
                (Instant::from_millis(0), Event::Mesurement),
                (Instant::from_millis(10), Event::Step(2)),
                (Instant::from_millis(20), Event::Mesurement),
                (Instant::from_millis(30), Event::Step(3)),
                (Instant::from_millis(40), Event::Mesurement),
                (Instant::from_millis(50), Event::Step(2)),
                (Instant::from_millis(60), Event::Mesurement),
            ],
        );
        let speeds = vec![
            Speed::stopped(),
            Speed::stopped(),
            Speed::stopped(),
            Speed::stopped(),
        ];
        let positions = vec![
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(3).lower_bound(&EQUAL_STEPS),
            Step::new(3).lower_bound(&EQUAL_STEPS),
        ];
        simulate_assert(measurements, speeds, positions);
    }
    #[test]
    fn always_use_larger_delta_speed_for_estiments() {
        let measurements = sequence_events(
            (Step::new(0), CounterClockwise, Instant::from_millis(0)),
            vec![
                // larger delta happens first
                (Instant::from_millis(035), Event::Mesurement),
                (Instant::from_millis(050), Event::Step(10)),
                (Instant::from_millis(060), Event::Mesurement),
                //---resetting
                (Instant::from_millis(060), Event::Step(-1)),
                (Instant::from_millis(100), Event::Step(0)),
                // larger delta happens after
                (Instant::from_millis(145), Event::Mesurement),
                (Instant::from_millis(150), Event::Step(10)),
                (Instant::from_millis(160), Event::Mesurement),
                //---resetting
                (Instant::from_millis(160), Event::Step(-1)),
                (Instant::from_millis(200), Event::Step(0)),
                // Same time delta
                (Instant::from_millis(240), Event::Mesurement),
                (Instant::from_millis(250), Event::Step(10)),
                (Instant::from_millis(260), Event::Mesurement),
            ],
        );
        let speeds = vec![
            // Larger delta happens first use speed from the last two steps
            Speed::stopped(),
            Speed::new(
                Step::new(10).lower_bound(&EQUAL_STEPS) - Step::new(0).upper_bound(&EQUAL_STEPS),
                Duration::from_millis(15),
            ),
            // larger delta happens after
            Speed::stopped(),
            dbg!(Speed::new(SubStep::new(64), Duration::from_millis(10))),
            // Same time delta
            Speed::stopped(),
            Speed::new(SubStep::new(64), Duration::from_millis(10)),
        ];
        let positions = vec![
            // larger delta happens first
            Step::new(0).lower_bound(&EQUAL_STEPS),
            Step::new(10).lower_bound(&EQUAL_STEPS) + speeds[1] * Duration::from_millis(10),
            // Larger delta happens after
            Step::new(0).lower_bound(&EQUAL_STEPS),
            Step::new(10).lower_bound(&EQUAL_STEPS) + speeds[3] * Duration::from_millis(10),
            // Same time delta
            Step::new(0).lower_bound(&EQUAL_STEPS),
            Step::new(10).lower_bound(&EQUAL_STEPS) + speeds[5] * Duration::from_millis(10),
        ];
        simulate_assert(measurements, speeds, positions);
    }
    */
    #[test]
    fn update_the_remaining_tests() {
        panic!()
    }
}
