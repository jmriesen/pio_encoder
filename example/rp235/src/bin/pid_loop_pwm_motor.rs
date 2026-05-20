#![no_std]
#![no_main]
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join;
use embassy_rp::{
    bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
    pwm::{Config, Pwm, SetDutyCycle},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Ticker};
use pid::Pid;
use pio_speed_encoder::substep_version::EncoderStateMachine;
use pio_speed_encoder::substep_version::PioEncoderProgram;
use pio_speed_encoder::{EncoderRunner, State};
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

    join::join(runner.run(Duration::from_millis(10)), async {
        let mut encoder = state
            .subscribe()
            .expect("recivers are preallocated in state");
        let mut ticker = Ticker::every(Duration::from_millis(10));
        loop {
            ticker.next().await;
            let status = encoder.get().await;

            info!("ticks :{}", status.step);
            info!("speed :{}", status.speed);
            let output =
                pid.next_control_output((status.speed * Duration::from_secs(1)).raw() as f32);
            pwm.set_duty_cycle(output.output as u16).unwrap();
        }
    })
    .await;
}
