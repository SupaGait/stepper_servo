use stm32f1xx_hal::{adc, flash, pac, prelude::*, rcc};

pub struct CurrentControl<ADC> {
    adc: adc::Adc<ADC>,
}

impl<ADC> CurrentControl<ADC> {
    fn new(rcc: &mut rcc::Rcc, acr: &mut flash::ACR, adc: pac::ADC1) -> CurrentControl<ADC> {
        let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut acr);
        let adc1 = adc::Adc::adc1(adc, &mut rcc.apb2, clocks);

        CurrentControl {
            adc: adc::Adc::<ADC>(adc, &mut rcc.apb2, clocks),
        }
    }

    fn setCurrent(&self, current: f32) {}

    fn setShuntResistance(&self, resistance: f32) {}
}
