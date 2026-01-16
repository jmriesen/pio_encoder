use super::Direction;
use embassy_time::Duration;

/// The number of clock cycles it takes for the pio loop to for one iteration.
///
/// The pio program always takes 13 clock cycles for each loop.
const LOOP_DURATION: u32 = 13;

/// Contains the direction of the last encoder tick and how long ago that happened.
///
/// This encoding works by splitting the i32 in half.
/// - Counterclockwise range = [0, `i32::MIN`)
/// - Clockwise range = [`i32::MIN`,0)
/// When a tick is register the counter is reset to the top of its respective range (Clockwise or
/// Counterclockwise)
/// After every subsequent loop if a step was not detected we decrement the counter.
///
/// NOTE: the cycles counter **can** overflow (i32 are not infinite).
/// In that case the direction will flip and the duration will reset to zero.
/// This happens after about 3.5 minutes (assuming 125Mhz clock speed)
///
/// However, overflows will not cause any faulty readings.
/// The duration is not used in speed calculations if the encoder is in a stopped state.
/// The caller of the crate is responsible for repeatedly calling the update function,
/// (10hz at least)
/// so we will always be in a stopped state before an overflow could occur.
/// Represents the encoders current direction and how many PIO loop has run since the last step.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DirectionDuration(pub i32);

// The value that means 0 PIO loops since last step.
fn loop_count_start(direction: Direction) -> i32 {
    match direction {
        Direction::CounterClockwise => 0,
        Direction::Clockwise => i32::MIN,
    }
}

impl DirectionDuration {
    pub fn new(val: i32) -> Self {
        Self(val)
    }
    /// Split into direction and duration.
    pub fn decode(self, clock_ticks_per_us: u32) -> (Direction, Duration) {
        let direction = if self.0 < 0 {
            Direction::CounterClockwise
        } else {
            Direction::Clockwise
        };
        let iterations = loop_count_start(direction).wrapping_sub(self.0);
        let iterations = {
            #[expect(
                clippy::cast_sign_loss,
                reason = "sign infomation is used to store direction and has already been extracted"
            )]
            {
                iterations as u32
            }
        };

        // By the time we have hit u32::Max cycles the encoder should be in a stopped state.
        // So saturating here should not affect anything (aside from preventing an overflow).
        let cycles = (iterations).saturating_mul(LOOP_DURATION);
        let duration = Duration::from_micros((cycles / clock_ticks_per_us).into());
        (direction, duration)
    }
}

#[cfg(test)]
mod tests {
    use super::{Direction, DirectionDuration};
    use crate::encodeing::{LOOP_DURATION, loop_count_start};
    use embassy_time::Duration;

    #[test]
    fn verify_start_positions() {
        assert_eq!(loop_count_start(Direction::CounterClockwise), 0);
        assert_eq!(loop_count_start(Direction::Clockwise), i32::MIN);
    }

    /// Due to how the encoding is defined cycles is always a nonzero value.
    /// Attempting to construct a DirectionDuration with zero cycle results in an underflow.
    ///
    /// The first thing the PIO code does after setting the register to the start value is
    /// subtract 1. So the PIO code will never return a zero cycles.
    /// (It may overflow the number of cycles which will flip the direction and restart the cycle
    /// count)
    ///
    #[test]
    fn zero_cycles_underflow() {
        for direction in [Direction::Clockwise, Direction::CounterClockwise] {
            // Lowest values in x direction.
            assert_eq!(
                DirectionDuration(loop_count_start(direction).wrapping_sub(1)).decode(1),
                (direction, Duration::from_micros(u64::from(LOOP_DURATION)))
            );
            // under/overflow
            assert_eq!(
                DirectionDuration(loop_count_start(direction)).decode(1),
                (direction.invert(), Duration::from_micros(u32::MAX as u64))
            );
        }
    }

    #[test]
    fn decode() {
        for direction in [Direction::Clockwise, Direction::CounterClockwise] {
            for ticks_per_ms in [1, 5, 10] {
                for cycles in [1, 5, 10] {
                    println!("direction:{direction:?},cycle:{cycles},ticks_per_ms:{ticks_per_ms}");
                    assert_eq!(
                        DirectionDuration(
                            loop_count_start(direction).wrapping_sub((cycles) as i32)
                        )
                        .decode(ticks_per_ms),
                        (
                            direction,
                            Duration::from_micros(u64::from(cycles * LOOP_DURATION / ticks_per_ms))
                        )
                    );
                }
            }
        }
    }
}
