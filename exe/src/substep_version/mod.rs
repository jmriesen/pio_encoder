#![allow(dead_code)]
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
/// Contains logic for parsing the pio messages into logical values
mod pio;

use logic::{
    EncoderState, Speed,
    encodeing::{Step, SubStep},
};
use pio::EncoderStateMachine;
pub use pio::PioEncoderProgram;
type CalibrationData = [u32; 4];

/// Pio Backed quadrature encoder reader
pub struct PioEncoder<'d, T: Instance, const SM: usize> {
    sm: EncoderStateMachine<'d, T, SM>,
    state: logic::EncoderState,
}

impl<'d, T: Instance, const SM: usize> PioEncoder<'d, T, SM> {
    pub fn new(
        pio: &mut Common<'d, T>,
        sm: StateMachine<'d, T, SM>,
        pin_a: impl PioPin,
        pin_b: impl PioPin,
        program: &PioEncoderProgram<'d, T>,
    ) -> Self {
        let mut sm = EncoderStateMachine::new(pio, sm, pin_a, pin_b, program);
        let inial_data = sm.pull_data();
        Self {
            sm: sm,
            state: EncoderState::new(inial_data),
        }
    }
    pub fn update(&mut self) {
        let measurement = self.sm.pull_data();
        self.state.update_state(measurement);
    }

    pub fn ticks(&self) -> Step {
        self.state.steps()
    }
    pub fn position(&self) -> SubStep {
        self.state.position()
    }
    pub fn speed(&self) -> Speed {
        self.state.speed()
    }
}
