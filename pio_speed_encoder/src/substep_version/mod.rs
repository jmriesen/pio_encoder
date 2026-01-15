#![allow(dead_code)]
use embassy_rp::{
    Peri,
    pio::{Common, Instance, PioPin, StateMachine},
};
/// Contains logic for parsing the pio messages into logical values
mod pio;

use pio::EncoderStateMachine;
pub use pio::PioEncoderProgram;
use pio_speed_encoder_logic::{
    Encoder, EncoderState, Speed,
    encodeing::{Step, SubStep},
};
type CalibrationData = [u32; 4];

/// Pio Backed quadrature encoder reader
pub struct PioEncoder<'d, T: Instance, const SM: usize, const IDLE_STOPING_TIME_MS: u64> {
    sm: EncoderStateMachine<'d, T, SM>,
    state: EncoderState<IDLE_STOPING_TIME_MS>,
}

impl<'d, T: Instance, const SM: usize, const IDLE_STOPING_TIME_MS: u64>
    PioEncoder<'d, T, SM, IDLE_STOPING_TIME_MS>
{
    pub fn new(
        pio: &mut Common<'d, T>,
        sm: StateMachine<'d, T, SM>,
        pin_a: Peri<'d, impl PioPin + 'd>,
        pin_b: Peri<'d, impl PioPin + 'd>,
        program: &PioEncoderProgram<'d, T>,
    ) -> Self {
        let mut sm = EncoderStateMachine::new(pio, sm, pin_a, pin_b, program);
        let inial_data = sm.pull_data();
        Self {
            sm: sm,
            state: EncoderState::new(inial_data),
        }
    }
}

impl<'d, T: Instance, const SM: usize, const IDLE_STOPING_TIME_MS: u64> Encoder
    for PioEncoder<'d, T, SM, IDLE_STOPING_TIME_MS>
{
    fn update(&mut self) {
        let measurement = self.sm.pull_data();
        self.state.update(measurement);
    }

    fn ticks(&self) -> Step {
        self.state.steps()
    }
    fn position(&self) -> SubStep {
        self.state.position()
    }
    fn speed(&self) -> Speed {
        self.state.speed()
    }
}
