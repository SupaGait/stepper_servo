use core::fmt::Write as _;
use embedded_hal::blocking::i2c::Write;
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

    pub fn update(&mut self, label: &str, row: u8, value: u32) {
        self.update_row_column(label, row, 0, value);
    }

    pub fn update_row_column(&mut self, label: &str, row: u8, column: u8, value: u32) {
        self.display.set_position(column, row).ok();
        write!(self.display, "{}:{:<4}", label, value).expect("Can't write");
    }
}
