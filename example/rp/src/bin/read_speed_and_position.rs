#![no_std]
#![no_main]
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Duration;
use pio_speed_encoder::{EncoderRunner, substep_version::EncoderStateMachine};
use pio_speed_encoder::{State, substep_version::PioEncoderProgram};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let pio = p.PIO0;
    let Pio {
        mut common, sm0, ..
    } = Pio::new(pio, Irqs);

    let prg = PioEncoderProgram::new(&mut common);
    let sm = EncoderStateMachine::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);
    let state = State::<NoopRawMutex, 1>::new();
    let runner = EncoderRunner::<30, _, _, 1>::new(&state, sm);

    join(runner.run(Duration::from_millis(10)), async {
        let mut encoder = state.subscribe().unwrap();
        loop {
            let status = encoder.changed().await;
            info!("speed{}", status.speed);
            info!("sub steps:{}", status.position);
        }
    })
    .await;
}
