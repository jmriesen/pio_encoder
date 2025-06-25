#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
};
//mod step_verstion;
mod substep_version;
use embassy_time::Timer;
use substep_version::{PioEncoder, PioEncoderProgram};
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
    let mut encoder = PioEncoder::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);

    let last_position = 0;
    let last_speed = 0;
    let last_raw_step = 0;
    loop {
        encoder.update();
        if last_position == encoder.position
            && last_speed == encoder.speed
            && last_raw_step == encoder.ticks()
        {
        } else {
            info!("ticks {}", encoder.ticks());
            info!("position{}", encoder.position);
            info!("speed{}", encoder.speed);
        }

        Timer::after_millis(10).await;
    }
}
