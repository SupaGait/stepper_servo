use embedded_hal as hal;
use hal::digital::v2::InputPin;
use stm32f1xx_hal::gpio::*;

use stepper_servo_lib::motor_control;

const MAX_VALUE: i32 = 10_000;
const MIN_VALUE: i32 = -10_000;

pub struct PositionInput<A, B> {
    pin_a: A,
    pin_b: B,
    pin_a_state: bool,
    pin_b_state: bool,
    position: i32,
}
impl<A, B> motor_control::PositionInput for PositionInput<A, B> {
    fn get_position(&self) -> i32 {
        self.position
    }
}

impl<A, B> PositionInput<A, B>
where
    A: InputPin + ExtiPin,
    B: InputPin + ExtiPin,
{
    pub fn new(pin_a: A, pin_b: B) -> Self {
        Self {
            pin_a,
            pin_b,
            pin_a_state: false,
            pin_b_state: false,
            position: 0,
        }
    }
    pub fn get_position(&self) -> i32 {
        self.position
    }

    //        | A_up | A_down |  B_up | B_down |
    //        ---------------------------------
    // A_up   |  X   |   X    |   I   |   D    |
    // A_down |  X   |   X    |   D   |   I    |
    // B_up   |  D   |   I    |   X   |   X    |
    // B_down |  I   |   D    |   X   |   X    |
    //        ----------------------------------
    pub fn update(&mut self) {
        if self.pin_a.check_interrupt() {
            self.pin_a.clear_interrupt_pending_bit();

            self.pin_a_state = self.pin_a.is_high().ok().unwrap();
            if self.pin_a_state == self.pin_b_state {
                self.increase();
            } else {
                self.decrement();
            }
        }
        if self.pin_b.check_interrupt() {
            self.pin_b.clear_interrupt_pending_bit();

            self.pin_b_state = self.pin_b.is_high().ok().unwrap();
            if self.pin_a_state == self.pin_b_state {
                self.decrement();
            } else {
                self.increase();
            }
        }
    }

    fn increase(&mut self) {
        self.position = if self.position < MAX_VALUE {
            self.position + 1
        } else {
            MIN_VALUE
        }
    }
    fn decrement(&mut self) {
        self.position = if self.position > MIN_VALUE {
            self.position - 1
        } else {
            MAX_VALUE
        }
    }
}
