#![no_std]
#![no_main]
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
    pwm::{Config, Pwm, SetDutyCycle},
};
use embassy_time::{Duration, Timer};
use pid::Pid;
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
    let mut encoder = PioEncoder::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);

    let desired_freq_hz = 20_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut config = Config::default();
    config.top = period;
    config.divider = divider.into();

    let mut pwm = Pwm::new_output_b(p.PWM_SLICE2, p.PIN_5, config.clone());

    //NOTE: Change set_point p and i value to suit your motor.
    let mut pid: Pid<f32> = Pid::new(222_088.0 / 2.0, config.top as f32);
    pid.p(0.0001, config.top);
    pid.i(0.0001, config.top);

    loop {
        info!("ticks {}", encoder.ticks());
        info!("speed{}", encoder.speed());
        encoder.update();
        let output =
            pid.next_control_output((encoder.speed() * Duration::from_secs(1)).val() as f32);
        pwm.set_duty_cycle(output.output as u16).unwrap();
        Timer::after_millis(10).await;
    }
}
