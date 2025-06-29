use super::Direction;
use embassy_time::Duration;

/// Contains the direction of the last encoder tick and how long ago that happened.
///
/// let C = cycles since last encoder tick;
/// If moving clockwise value = 0 - C.
/// If moving counterclockwise value = i32::max - C +1.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DirectionDuration(i32);
impl DirectionDuration {
    pub fn new(val: i32) -> Self {
        Self(val)
    }
    pub fn decode(self, clocks_per_us: u32) -> (Direction, Duration) {
        let (cycles, direction) = if self.0 < 0 {
            (-self.0, Direction::Clockwise)
        } else {
            (i32::MAX - self.0 + 1, Direction::Clockwise)
        };
        let duration = Duration::from_micros(((cycles * 13) as u32 / clocks_per_us).into());
        (direction, duration)
    }
}

#[cfg(test)]
mod tests {
    use super::Direction;
    use embassy_time::Duration;

    use super::DirectionDuration;

    fn decrimenting() {
        let raw = DirectionDuration(0 - 50);
        assert_eq!(
            raw.decode(10),
            (Direction::Clockwise, Duration::from_micros(500))
        );
    }
}
