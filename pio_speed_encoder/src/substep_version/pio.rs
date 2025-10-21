//This modal stores deals with interacting with the pio hardware.
//This includes interpreting the rx output.
//
use embassy_futures::block_on;
#[cfg(feature = "rp235x")]
use embassy_rp::pio::StatusN;
use embassy_rp::{
    Peri,
    gpio::{Output, Pull},
    pio::{
        Common, Config, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection,
        StateMachine, StatusSource,
        program::{InstructionOperands, MovDestination, MovOperation, MovSource, pio_file},
    },
};
use embassy_time::Instant;
use fixed::traits::ToFixed;
use logic::{DirectionDuration, Measurement, encodeing::Step};

pub struct PioEncoderProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}
impl<'a, PIO: Instance> PioEncoderProgram<'a, PIO> {
    /// Load the program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_file!("src/quadrature_encoder_substep.pio");
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

pub struct EncoderStateMachine<'d, T: Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
    clocks_per_us: u32,
}

impl<'d, T: Instance, const SM: usize> EncoderStateMachine<'d, T, SM> {
    /// Configure a state machine with the loaded [PioEncoderProgram]
    pub fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        pin_a: Peri<'d, impl PioPin + 'd>,
        pin_b: Peri<'d, impl PioPin + 'd>,
        program: &PioEncoderProgram<'d, T>,
    ) -> Self {
        use embassy_rp::pio::Direction;
        let mut pin_a = pio.make_pio_pin(pin_a);
        let mut pin_b = pio.make_pio_pin(pin_b);
        pin_a.set_pull(Pull::Up);
        pin_b.set_pull(Pull::Up);
        sm.set_pin_dirs(Direction::In, &[&pin_a, &pin_b]);

        let mut cfg = Config::default();
        cfg.set_in_pins(&[&pin_a, &pin_b]);
        cfg.shift_in = ShiftConfig {
            direction: ShiftDirection::Left,
            auto_fill: true,
            threshold: 32,
        };
        cfg.shift_out = ShiftConfig {
            direction: ShiftDirection::Right,
            auto_fill: false,
            threshold: 32,
        };
        cfg.fifo_join = FifoJoin::Duplex;
        cfg.clock_divider = 1.to_fixed();

        cfg.status_sel = StatusSource::RxFifoLevel;
        #[cfg(feature = "rp2040")]
        {
            cfg.status_n = 0x12;
        }
        #[cfg(feature = "rp235x")]
        {
            cfg.status_n = StatusN::This(2);
        }
        cfg.use_program(&program.prg, &[]);
        sm.set_config(&cfg);
        //Raw reading the pins this is fine since we already own the pins.
        let pin_state = 0i32; //TODO actually read the value

        critical_section::with(|_| {
            unsafe {
                sm.set_y((-pin_state) as u32);
                sm.exec_instr(
                    InstructionOperands::MOV {
                        destination: MovDestination::OSR,
                        op: MovOperation::None,
                        source: MovSource::Y,
                    }
                    .encode(),
                );
                sm.set_y(match pin_state {
                    0 => 0,
                    1 => 3,
                    2 => 1,
                    3 => 2,
                    _ => 0, /*unreachable*/
                });
            }
        });

        sm.set_enable(true);
        Self {
            sm,
            clocks_per_us: (embassy_rp::clocks::clk_sys_freq() + 500_000) / 1_000_000,
        }
    }

    pub fn pull_raw_data(&mut self) -> (u32, u32, Instant) {
        let rx = self.sm.rx();

        //Purging buffer of stale data
        let num_stale_data = rx.level() / 2;
        critical_section::with(|_| {
            for _ in 0..num_stale_data {
                block_on(rx.wait_pull());
                block_on(rx.wait_pull());
            }
            //NOTE: Note a new value is pushed into rx in at most 13 clock cycles.
            // At 125Mhz this is about 0.1 micro second.
            (
                block_on(rx.wait_pull()),
                block_on(rx.wait_pull()),
                Instant::now(),
            )
        })
    }
    pub fn pull_data(&mut self) -> Measurement {
        let raw = self.pull_raw_data();
        Measurement::new(
            DirectionDuration::new(raw.0 as i32),
            Step::new(raw.1 as i32),
            raw.2,
            self.clocks_per_us,
        )
    }
}
