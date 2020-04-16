use embedded_hal::PwmPin;
use stm32f1xx_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use motor_driver as motor;
use motor::ic::L298;

type AdcType = hal::adc::AdcInt<hal::stm32::ADC1>;

#[derive(PartialEq)]
enum Direction { CCW, CW, }

/// For now hard bound to ADC1
pub struct CurrentControl<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    adc: AdcType,
    shunt_resistance: f32,
    current_setpoint: f32,
    adc_value: u16,
    voltage: f32,
    duty_cyle: u16,
    max_duty_cyle: u16,
    motor: motor::Motor<IN1, IN2, PWM, L298>,
    direction: Direction,
}

impl<PWM, IN1, IN2> CurrentControl<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    pub fn new(
        adc: AdcType,
        shunt_resistance: f32,
        pwm: PWM,
        in1: IN1,
        in2: IN2,
    ) -> Self {


        let mut s = Self {
            adc,
            shunt_resistance,
            current_setpoint: 0.0,
            adc_value: 0,
            voltage: 0.0,
            duty_cyle: 0,
            max_duty_cyle: pwm.get_max_duty(),
            motor: motor::Motor::l298(in1, in2, pwm),
            direction: Direction::CW,
        };

        s.motor.cw();
        s
    }

    /// Can be positive and negative
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

    pub fn duty_cycle(&self) -> u16 {
        self.duty_cyle
    }

    pub fn voltage(&self) -> f32 {
        self.voltage
    }

    pub fn update(&mut self)
    {
        self.set_direction();
        let mut current_measured = 0.0;
        if self.adc_value > 0
        {
            self.voltage = self.adc_value as f32 / 255.0;
            current_measured = self.voltage / self.shunt_resistance;
        }
        self.calc_pwm(current_measured);
    }

    fn set_direction(&mut self)
    {
        if self.current_setpoint >= 0.0
        { 
            if self.direction  != Direction::CW {
                self.motor.cw();
                self.direction = Direction::CW;
            }
        }
        else
        {
            if self.direction != Direction::CCW {
                self.motor.ccw();
                self.direction = Direction::CCW;
            }
        }
    }

    fn calc_pwm(&mut self, current_measured: f32) 
    {
        const THRESHOLD: f32 = 0.01;
        let current_delta = self.current_setpoint - current_measured;
        if current_delta > THRESHOLD || current_delta < THRESHOLD
        {
            if current_delta > 0.0
            {
                if self.duty_cyle != self.max_duty_cyle 
                {
                    self.duty_cyle += 1;
                    self.motor.duty(self.duty_cyle);
                }
            }
            else
            {
                if self.duty_cyle != u16::min_value() 
                {
                    self.duty_cyle -= 1;
                    self.motor.duty(self.duty_cyle);
                }
            }
        }
    }
}