use embedded_hal::blocking::i2c::Write;

// use embedded_graphics::{
//     image::{Image, ImageRaw},
//     pixelcolor::BinaryColor,
//     prelude::*,
// };
use core::fmt::Write as _;
use ssd1306::{interface::i2c::I2cInterface, mode::TerminalMode, Builder};

pub struct Display<DI> {
    display: TerminalMode<DI>,
}

impl<I2C> Display<I2cInterface<I2C>>
where
    I2C: Write,
{
    pub fn new(i2c: I2C) -> Self {
        let mut display: TerminalMode<I2cInterface<I2C>> = Builder::new().connect_i2c(i2c).into();
        display.init().ok();
        display.clear().ok();

        Self { display }
    }

    pub fn update(&mut self, value: u32) {
        //self.display.clear().ok();
        self.display.set_position(0, 0).ok();
        write!(self.display, "{}", value).expect("Can't write");
        //self.display.write_str(10_u32.into()).ok();
    }
}
