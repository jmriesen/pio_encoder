use std::sync::{Arc, Mutex};

use embassy_futures::select::select;
use embassy_time::{Duration, Instant, MockDriver, Timer};

use crate::{
    Direction::{self, *},
    EncoderStateMachine, Step,
    measurement::tests::MockPio,
};

pub struct MockSensor {
    mock: Arc<Mutex<MockPio>>,
}
pub struct MockSensorRunner {
    mock: Arc<Mutex<MockPio>>,
    events: Vec<(Duration, Step)>,
}

impl MockSensor {
    pub fn new(
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
    pub fn new_inst(
        inital_conditions: (Step, Direction, Instant),
        events_inst: Vec<(Instant, Step)>,
    ) -> (Self, MockSensorRunner) {
        let mut current = inital_conditions.2;
        let mut events = vec![];
        for (inst, step) in events_inst {
            events.push((inst.duration_since(current), step));
            current = inst;
        }
        Self::new(inital_conditions, events)
    }
}
impl MockSensorRunner {
    pub async fn run(self) {
        let mut event_time = Instant::from_millis(0);
        for (delta_t, step) in self.events {
            event_time += delta_t;
            Timer::at(event_time).await;
            dbg!(Instant::now());
            self.mock.lock().unwrap().position_change(step, event_time);
        }
        Timer::after_secs(1).await;
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

/// Blocks on a future untill completion.
/// Clock is advanced by 10 microseconds every time after every poll.
pub fn block_on_with_timer(future: impl Future) {
    let driver = MockDriver::get();
    driver.reset();
    embassy_futures::block_on(select(
        future,
        //Time updates after everything else
        async move {
            loop {
                driver.advance(embassy_time::Duration::from_micros(10));
                embassy_futures::yield_now().await;
            }
        },
    ));
}
