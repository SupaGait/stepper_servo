#![no_std]
#![no_main]

use cortex_m_rt::entry;
//use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use panic_halt as _;
use stm32f1xx_hal::{adc, pac, prelude::*};

mod current_control;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();

    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);
    let adc1 = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);

    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

    let mut current_control = current_control::CurrentControl
        ::new(adc1, ch0, 4.2);

    current_control.set_current(0.5);

    loop {
        current_control.update();
    }
}
