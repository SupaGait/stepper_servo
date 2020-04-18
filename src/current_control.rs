use embedded_hal::PwmPin;
use stm32f1xx_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use motor_driver as motor;
use motor::ic::L298;

use crate::pid::{PIDController, Controller};

#[derive(PartialEq)]
enum Direction { CCW, CW, }

/// For now hard bound to ADC1
pub struct CurrentControl<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    shunt_resistance: f32,
    current_setpoint: f32,
    adc_value: u16,
    voltage: f32,
    current: f32,
    duty_cycle: u16,
    motor: motor::Motor<IN1, IN2, PWM, L298>,
    direction: Direction,
    pid: PIDController,
}

impl<PWM, IN1, IN2> CurrentControl<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    pub fn new(
        shunt_resistance: f32,
        pwm: PWM,
        in1: IN1,
        in2: IN2,
    ) -> Self {
        let mut s = Self {
            shunt_resistance,
            current_setpoint: 0.0,
            adc_value: 0,
            voltage: 0.0,
            current: 0.0,
            duty_cycle: 0,
            motor: motor::Motor::l298(in1, in2, pwm),
            direction: Direction::CW,
            pid: PIDController::new(1.0, 0.0, 0.0),  // PID
        };

        s.set_duty_cycle(220);
        s.pid.set_limits(0.0, s.motor.get_max_duty() as f32);
        s.motor.cw();
        s
    }

    /// Can be positive and negative
    pub fn set_current(&mut self, amps: f32) {
        self.current_setpoint = amps;
        self.pid.set_target(amps);
    }

    pub fn adc_value(&self) -> u16 {
        self.adc_value
    }

    pub fn duty_cycle(&self) -> u16 {
        self.duty_cycle
    }

    pub fn voltage(&self) -> f32 {
        self.voltage
    }

    pub fn current(&self) -> f32 {
        self.current
    }

    pub fn update(&mut self, adc_value: u16, adc_voltage: f32)
    {
        self.adc_value = adc_value;
        self.voltage = adc_voltage;

        self.set_direction();
        if self.adc_value > 0
        {
            self.current = self.voltage / self.shunt_resistance;
        }
        else
        {
            self.current = 0.0;
        }
        //self.calc_pwm();
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

    fn set_duty_cycle(&mut self, duty_cycle : u16) {
        self.motor.duty(duty_cycle);
        self.duty_cycle = duty_cycle;
    }

    fn calc_pwm(&mut self) 
    {
        const DT: f32 = (1/(8_000_000/1_00)) as f32;
        let pwm_value = self.pid.update(self.current, DT);
        self.set_duty_cycle(pwm_value as u16);
    }
}