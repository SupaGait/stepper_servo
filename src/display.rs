use embedded_hal::blocking::i2c::Write;
use ssd1306::{interface::i2c::I2cInterface, mode::TerminalMode, Builder};
use core::fmt::Write as _;

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

    pub fn update(&mut self, label: &str, row: u8, value: u32) {
        //self.display.clear().ok();
        self.display.set_position(0, row).ok();
        write!(self.display, "{}:{}", label, value).expect("Can't write");
    }
}
