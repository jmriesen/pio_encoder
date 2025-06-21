#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::{
    PeripheralRef, bind_interrupts,
    gpio::Pull,
    peripherals::PIO0,
    pio::{
        Common, Config, Direction, FifoJoin, Instance, InterruptHandler, LoadedProgram, Pio,
        PioPin, ShiftDirection, StateMachine,
    },
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

pub struct PioEncoderProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioEncoderProgram<'a, PIO> {
    /// Load the program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = embassy_rp::pio::program::pio_file!("src/quadrature_encoder.pio");

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// Pio Backed quadrature encoder reader
pub struct PioEncoder<'d, T: Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
}

impl<'d, T: Instance, const SM: usize> PioEncoder<'d, T, SM> {
    /// Configure a state machine with the loaded [PioEncoderProgram]
    pub fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        pin_a: impl PioPin,
        pin_b: impl PioPin,
        program: &PioEncoderProgram<'d, T>,
    ) -> Self {
        let mut pin_a = pio.make_pio_pin(pin_a);
        let mut pin_b = pio.make_pio_pin(pin_b);
        pin_a.set_pull(Pull::Up);
        pin_b.set_pull(Pull::Up);
        sm.set_pin_dirs(Direction::In, &[&pin_a, &pin_b]);

        let mut cfg = Config::default();
        cfg.set_in_pins(&[&pin_a, &pin_b]);
        cfg.fifo_join = FifoJoin::Duplex;
        cfg.shift_in.direction = ShiftDirection::Left;

        cfg.use_program(&program.prg, &[]);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm }
    }

    pub fn ticks(&mut self) -> i32 {
        let rx = self.sm.rx();

        //Purging buffer of stale data
        let num_stale_data = rx.level();
        for _ in 0..num_stale_data {
            rx.try_pull();
        }
        //NOTE: Note a new value is pushed into rx in at most 13 clock cycles.
        // At 125Mhz this is about 0.1 micro second.
        embassy_futures::block_on(rx.wait_pull()) as i32
    }
    pub async fn read(&mut self) -> embassy_rp::pio_programs::rotary_encoder::Direction {
        use embassy_rp::pio_programs::rotary_encoder::Direction;
        let current = self.ticks();

        loop {
            return match current.cmp(&self.ticks()) {
                core::cmp::Ordering::Less => Direction::Clockwise,
                core::cmp::Ordering::Greater => Direction::CounterClockwise,
                core::cmp::Ordering::Equal => {
                    yield_now().await;
                    continue;
                }
            };
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let pio = p.PIO0;
    let Pio {
        mut common, sm0, ..
    } = Pio::new(pio, Irqs);
    let prg = PioEncoderProgram::new(&mut common);
    let mut encoder = PioEncoder::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);
    loop {
        info!("read {}", encoder.ticks());
        Timer::after_millis(10).await;
    }
}
