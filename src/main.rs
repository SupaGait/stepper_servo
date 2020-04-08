#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use panic_halt as _;
use stm32f1xx_hal::{adc, gpio, pac, prelude::*};

//mod currentControl;

fn toggle_led<T>(pin: &mut T)
where
    T: ToggleableOutputPin,
{
    //pin.set_high().ok();
    pin.toggle().ok();
}

// use `main` as the entry point of this application
#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();

    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);
    let adc1 = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);

    let mut _gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    loop {
        cortex_m::asm::delay(2000000);
        toggle_led(&mut led);
    }
}
