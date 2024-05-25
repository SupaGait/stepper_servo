use embedded_hal as hal;
use hal::blocking::spi::Transfer;
use hal::digital::v2::OutputPin;

pub struct AS5045<SPI, CS> {
    spi: SPI,
    cs: CS,
}

//pub enum Error {}

impl<SPI, CS, Error> AS5045<SPI, CS>
where
    SPI: Transfer<u8, Error = Error>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        AS5045 { spi, cs }
    }

    pub fn read_angle(&mut self) -> Result<i32, Error> {
        let raw_data = self.read()?;

        // Byte 0 = data 11 - 4
        // byte 1 = data 3 - 0 + OCF, COF, LIN, MAG inc
        // Byte 3 = Mag dec, Par, + 6 spare bits.
        let data = ((raw_data[0] as u32) << 4) + ((raw_data[1] as u32) >> 4);
        Ok(data as i32)
    }

    // 18bits information
    // 3*8 bits = 6spare + 12data + 6status
    pub fn read(&mut self) -> Result<[u8; 3], Error> {
        // Ignoring CS errors for now
        let mut data = [0x00, 0x00, 0x00];
        self.cs.set_low();
        let error = self.spi.transfer(&mut data).err();
        self.cs.set_high();

        if error.is_some() {
            return Err(error.unwrap());
        }
        Ok(data)
    }
}
