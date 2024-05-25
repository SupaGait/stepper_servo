use crate::AS5045::AS5045;
use embedded_hal as hal;
use hal::blocking::spi::Transfer;
use hal::digital::v2::OutputPin;

use stepper_servo_lib::position_control::PositionInput;

const MAX_POSTION_VALUE: i32 = 2048;

pub struct AS504Position<SPI, CS> {
    driver: AS5045<SPI, CS>,
    position: i32,
    prev_position: i32,
    relative_position: i32,
}

fn calculate_position_diff(old_position: i32, new_position: i32) -> i32 {
    let absolute_diff = new_position - old_position;
    // Test for wrapparround
    let diff = if absolute_diff.abs() > MAX_POSTION_VALUE / 2 {
        let diff = MAX_POSTION_VALUE - old_position + new_position;
        if diff < 0 {
            // overflow
            diff
        } else {
            // underflow
            -diff
        }
    } else {
        absolute_diff
    };
    diff
}

impl<SPI, CS, Error> PositionInput for AS504Position<SPI, CS>
where
    SPI: Transfer<u8, Error = Error>,
    CS: OutputPin,
{
    fn get_position(&self) -> i32 {
        self.relative_position
    }
    fn update(&mut self) {
        match self.driver.read_angle() {
            Ok(value) => {
                self.position = value;
                let absolute_diff = self.position - self.prev_position;
                // Test for wrapparround
                let diff = if absolute_diff.abs() > MAX_POSTION_VALUE / 2 {
                    if absolute_diff < 0 {
                        // overflow
                        MAX_POSTION_VALUE - self.prev_position + self.position
                    } else {
                        // underflow
                        -self.prev_position - (MAX_POSTION_VALUE - self.position)
                    }
                } else {
                    absolute_diff
                };

                self.relative_position += diff;
                self.prev_position = self.position;
            }
            _ => (),
        }
    }
    fn reset(&mut self) {
        self.relative_position = 0;
    }
}

impl<SPI, CS> AS504Position<SPI, CS> {
    pub fn new(as5045: AS5045<SPI, CS>) -> Self {
        Self {
            driver: as5045,
            position: 0,
            prev_position: 0,
            relative_position: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn position_positive_diff() {
        assert_eq!(10, calculate_position_diff(0, 10));
    }
}
