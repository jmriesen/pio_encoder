use core::cell::RefCell;

//This modal stores deals with interacting with the pio hardware.
//This includes interpreting the rx output.
//
use embassy_futures::block_on;
#[cfg(feature = "rp235x")]
use embassy_rp::pio::StatusN;
use embassy_rp::{
    Peri,
    gpio::{self, Input, Pull},
    pio::{
        Common, Config, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection,
        StateMachine, StatusSource,
        program::{InstructionOperands, MovDestination, MovOperation, MovSource, pio_file},
    },
};
use embassy_time::Instant;
use fixed::traits::ToFixed;
use pio_speed_encoder_logic::{DirectionDuration, Measurement, Step};

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
    sm: RefCell<StateMachine<'d, T, SM>>,
    clocks_per_us: u32,
}

impl<'d, T: Instance, const SM: usize> EncoderStateMachine<'d, T, SM> {
    /// Configure a state machine with the loaded [PioEncoderProgram]
    pub fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        mut pin_a: Peri<'d, impl PioPin + 'd>,
        mut pin_b: Peri<'d, impl PioPin + 'd>,
        program: &PioEncoderProgram<'d, T>,
    ) -> Self {
        use embassy_rp::pio::Direction;
        let inital_pin_state = {
            [
                Input::new(pin_a.reborrow(), Pull::Up).get_level(),
                Input::new(pin_b.reborrow(), Pull::Up).get_level(),
            ]
        };
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

        let [a, b] = inital_pin_state.map(|x| x == gpio::Level::High);
        let inital_pin_state_int = (a as u8) << 1 | (b as u8) << 0;

        critical_section::with(|_| {
            use gpio::Level as E;
            unsafe {
                // The output shift register is used to hold the current + previous state.
                // This is combined with a jump table to figure out how each transition should be
                // handled.
                //
                // Here we are setting the ~current position as a dummy previous position
                sm.set_y((!inital_pin_state_int) as u32);
                sm.exec_instr(
                    InstructionOperands::MOV {
                        destination: MovDestination::OSR,
                        op: MovOperation::None,
                        source: MovSource::Y,
                    }
                    .encode(),
                );
                // Phase calibration corresponds to physical differences in an encoder.
                // If you use precomputed calibration date (not supported at time of writing, but
                // planned) it is important to make sure step 0 (the int) always corresponds to phase zero (The low/low signal)
                sm.set_y(match inital_pin_state {
                    [E::Low, E::Low] => 0,
                    [E::High, E::Low] => 1,
                    [E::High, E::High] => 2,
                    [E::Low, E::High] => 3,
                });
            }
        });

        sm.set_enable(true);
        Self {
            sm: RefCell::new(sm),
            clocks_per_us: (embassy_rp::clocks::clk_sys_freq() + 500_000) / 1_000_000,
        }
    }

    fn pull_raw_data(&self) -> (u32, u32, Instant) {
        //Reading data is idempotent since the PIO code will refill the rx buffer.
        let mut inner_ref = self.sm.borrow_mut();
        let rx = inner_ref.rx();

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
}
use pio_speed_encoder_logic::EncoderStateMachine as ExtTrait;
impl<'d, T: Instance, const SM: usize> ExtTrait for EncoderStateMachine<'d, T, SM> {
    fn read(&self) -> pio_speed_encoder_logic::Measurement {
        let (dir_dur, step, now) = self.pull_raw_data();
        let (direction, time_since_transition) =
            DirectionDuration::new(dir_dur as i32).decode(self.clocks_per_us);
        Measurement::new(
            direction,
            Step::new(step as i32),
            now,
            time_since_transition,
        )
    }
}
