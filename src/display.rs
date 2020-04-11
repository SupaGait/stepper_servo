use embedded_hal::blocking::i2c::{Write};

// use embedded_graphics::{
//     image::{Image, ImageRaw},
//     pixelcolor::BinaryColor,
//     prelude::*,
// };
use ssd1306::{mode::TerminalMode, Builder, interface::i2c::I2cInterface};
use core::fmt::Write as _;

pub struct Display<DI>
{
    display: TerminalMode<DI>,
}

impl<I2C> Display<I2cInterface<I2C>>
where 
    I2C: Write,
{
    pub fn new(i2c : I2C) -> Self
    {
        let mut display : TerminalMode<I2cInterface<I2C>> = Builder::new().connect_i2c(i2c).into();
        display.init().ok();
        display.clear().ok();

        Self
        {
            display,
        }
    }

    pub fn update( &mut self)
    {
        //self.display.clear().ok();
        self.display.set_position(0, 0).ok();
        self.display.write_str("Ghello").ok();
    }
}