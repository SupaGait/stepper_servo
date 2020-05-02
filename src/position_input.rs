use embedded_hal as hal;
use hal::digital::v2::InputPin;
use rotary_encoder_hal::{Direction, Rotary};

pub struct PositionInput<A, B> {
    rotary: Rotary<A, B>,
    position: isize,
}

impl<A, B> PositionInput<A, B>
where
    A: InputPin,
    B: InputPin,
{
    pub fn new(pin_a: A, pin_b: B) -> Self {
        Self {
            rotary: Rotary::new(pin_a, pin_b),
            position: 0,
        }
    }
    pub fn update(&mut self) {
        match self.rotary.update() {
            Ok(direction) => match direction {
                Direction::Clockwise => {
                    self.position += 1;
                }
                Direction::CounterClockwise => {
                    self.position -= 1;
                }
                Direction::None => {}
            },
            Err(_) => {}
        }
        // match self.rotary.update().unwrap() {
        //     Direction::Clockwise => {
        //         self.position += 1;
        //     }
        //     Direction::CounterClockwise => {
        //         self.position -= 1;
        //     }
        //     Direction::None => {}
        // }
    }
}
