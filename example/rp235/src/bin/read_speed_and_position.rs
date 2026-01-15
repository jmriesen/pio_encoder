#![no_std]
#![no_main]
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
};
use embassy_time::Timer;
use pio_speed_encoder::Encoder;
use pio_speed_encoder::substep_version::{PioEncoder, PioEncoderProgram};
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
    let mut encoder = PioEncoder::<_, 0, 30>::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);

    loop {
        encoder.update();
        Timer::after_millis(10).await;
        info!("speed{}", encoder.speed());
        info!("sub steps:{}", encoder.position());
    }
}
