use embedded_hal::adc::Channel;
use embedded_hal::adc::OneShot;
use stm32f1xx_hal as hal;

/// For now hard bound to ADC1
pub struct CurrentControl<PIN>
{
    adc: hal::adc::Adc<hal::stm32::ADC1>,
    pins: PIN,
    shunt_resistance: f32,
    req_current: f32,
}

impl<PIN> CurrentControl<PIN>
{
    pub fn new(adc: hal::adc::Adc<hal::stm32::ADC1>, pins: PIN, shunt_resistance: f32 ) -> Self
    where 
        PIN: Channel<hal::stm32::ADC1, ID = u8>,
    {
        CurrentControl 
        {
            adc,
            pins,
            shunt_resistance,
            req_current : 0.0,
        }
    }

    pub fn set_current(& mut self, amps: f32)
    {
        self.req_current = amps;
    }

    pub fn update(& mut self)
    where 
        PIN: Channel<hal::stm32::ADC1, ID = u8>,
    {
        let adc_value: u16 = self.adc.read(&mut self.pins).unwrap();
        let voltage = adc_value as f32 / 255.0;
        let _current =  voltage / self.shunt_resistance;
    }
}

// pub struct CurrentControl<ADC, Word, Pin, Error, Adc>
// where   Pin: embedded_hal::adc::Channel<ADC>,
//         Adc : hal::adc::OneShot<ADC, Word, Pin, Error=Error>
// {
//     adc: Adc,
//     shunt_resistance : f32,

//     _adc: PhantomData<ADC>,
//     _word: PhantomData<Word>,
//     _pin: PhantomData<Pin>,
//     _error: PhantomData<Error>,
// }

// impl<ADC, Word, Pin, Error, Adc> CurrentControl<ADC, Word, Pin, Error, Adc>
// where   Pin: embedded_hal::adc::Channel<ADC>,
//         Adc : hal::adc::OneShot<ADC, Word, Pin, Error=Error>
// {
//     pub fn new(adc: Adc, shunt_resistance: f32) -> Self
//     where   Pin: embedded_hal::adc::Channel<ADC>,
//             Adc : hal::adc::OneShot<ADC, Word, Pin, Error=Error>
//     {
//         Self {
//             adc, 
//             shunt_resistance, 
//             _adc: PhantomData,
//             _word: PhantomData,
//             _pin: PhantomData,
//             _error: PhantomData,
//         }
//     }

//     pub fn set_current(& mut self, current: f32) {}

//     pub fn set_shunt_resistance(& mut self, resistance: f32) {}
// }