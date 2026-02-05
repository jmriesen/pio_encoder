use std::sync::{Arc, Mutex};

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
}
impl MockSensorRunner {
    pub async fn run(self) {
        let mut event_time = Instant::now();
        for (delta_t, step) in self.events {
            event_time += delta_t;
            Timer::at(event_time).await;
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
pub async fn advance_embassy_clock() {
    let driver = MockDriver::get();
    let mut interval = tokio::time::interval(std::time::Duration::from_micros(10));
    loop {
        interval.tick().await;
        driver.advance(embassy_time::Duration::from_micros(10));
    }
}
