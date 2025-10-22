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
    let mut encoder = PioEncoder::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);

    loop {
        encoder.update();
        Timer::after_millis(10).await;
        info!("speed{}", encoder.speed());
        info!("sub steps:{}", encoder.position());
        /*
                        info!("// 100% duty cycle, fully on");
                        pwm.set_duty_cycle_fully_on().unwrap();
                        Timer::after_secs(1).await;

                        info!("// 66% duty cycle. Expressed as simple percentage.");
                        pwm.set_duty_cycle_percent(66).unwrap();
                        Timer::after_secs(1).await;

        info!("// 25% duty cycle. Expressed as 32768/4 = 8192.");
        pwm.set_duty_cycle(config.top / 2).unwrap();
        Timer::after_secs(1).await;

        info!("// 0% duty cycle, fully off.");
        pwm.set_duty_cycle_fully_off().unwrap();
        Timer::after_secs(1).await;
        */
    }
}
