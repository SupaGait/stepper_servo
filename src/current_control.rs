use embedded_hal::PwmPin;
use stm32f1xx_hal as hal;

type AdcType = hal::adc::AdcInt<hal::stm32::ADC1>;

/// For now hard bound to ADC1
pub struct CurrentControl<PWM> {
    adc: AdcType,
    shunt_resistance: f32,
    current_setpoint: f32,
    pwm: PWM,
    adc_value: u16,
}

impl<PWM> CurrentControl<PWM>
where
    PWM: PwmPin<Duty = u16>,
{
    pub fn new(
        adc: AdcType,
        shunt_resistance: f32,
        pwm: PWM,
    ) -> Self {
        Self {
            adc,
            shunt_resistance,
            current_setpoint: 0.0,
            pwm,
            adc_value: 0,
        }
    }

    pub fn set_current(&mut self, amps: f32) {
        self.current_setpoint = amps;
    }

    pub fn handle_adc_interrupt(&mut self)
    {
        if self.adc.is_ready() {
            self.adc_value = self.adc.read_value();
        }
    }

    pub fn adc_value(&self) -> u16 {
        self.adc_value
    }

    pub fn update(&mut self)
    {
        let voltage_measured = self.adc_value as f32 / 255.0;
        let current_measured = voltage_measured / self.shunt_resistance;
        self.calc_pwm(current_measured);
    }

    fn calc_pwm(&mut self, current_measured: f32) {
        let current_delta = self.current_setpoint - current_measured;
        if current_delta > 0.01 {
            let duty_cyle = self.pwm.get_duty();
            if duty_cyle != u16::min_value() {
                self.pwm.set_duty(duty_cyle - 1);
            }
        }
        if current_delta < 0.01 {
            let duty_cyle = self.pwm.get_duty();
            if duty_cyle != u16::max_value() {
                self.pwm.set_duty(duty_cyle + 1);
            }
        }
    }
}