#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
    pwm::{Config, Pwm, SetDutyCycle},
};
use embassy_time::Timer;
use pid::Pid;
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
    /*let program = step_verstion::PioEncoderProgram::new(&mut common);
        let mut encoder =
            step_verstion::PioEncoder::new(&mut common, sm0, p.PIN_16, p.PIN_17, &program);
    */
    let prg = PioEncoderProgram::new(&mut common);
    let mut encoder = PioEncoder::new(&mut common, sm0, p.PIN_16, p.PIN_17, &prg);

    let desired_freq_hz = 20_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut config = Config::default();
    config.top = period;
    config.divider = divider.into();

    let mut pwm = Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, config.clone());
    let mut pid: Pid<f32> = Pid::new(222_088.0 / 1.0, config.top as f32);
    pid.p(0.0001, config.top);
    pid.i(0.0001, config.top);

    embassy_futures::join::join(
        async {
            loop {
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
                yield_now().await;
            }
        },
        async {
            loop {
                //info!("ticks {}", encoder.ticks());
                //info!("speed{}", encoder.speed());
                encoder.update();
                let output = pid.next_control_output(encoder.speed().ticks_per_second() as f32);
                //info!("o%{}", output.output as f32 / config.top as f32);
                //info!("o%{}", output.output as u16);
                //info!("T%{}", config.top as f32);
                //info!("p%{}", output.p as f32 / config.top as f32);
                //info!("i%{}", output.i as f32 / config.top as f32);
                //info!("d%{}", output.d as f32 / config.top as f32);
                pwm.set_duty_cycle(output.output as u16).unwrap();

                Timer::after_millis(10).await;
            }
        },
    )
    .await;
}
