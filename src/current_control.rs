use embedded_hal::PwmPin;
use embedded_hal::digital::v2::OutputPin;
use motor_driver as motor;
use motor::ic::L298;
use crate::pid::{PIDController, Controller, util};

#[derive(PartialEq)]
enum Direction { CCW, CW, }
const CURRENT_BUFFER_SIZE: usize = 5;
const PID_SCALING_FACTOR: i32 = 1000;

/// For now hard bound to ADC1
pub struct CurrentControl<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    shunt_resistance: u32,
    current_setpoint: i32,
    adc_value: u32,
    voltage: u32,
    current: u32,
    duty_cycle: u32,
    motor: motor::Motor<IN1, IN2, PWM, L298>,
    direction: Direction,
    pid: PIDController<i32>,

    current_buffer: [u32; CURRENT_BUFFER_SIZE],
    buffer_index: usize,
}

impl<PWM, IN1, IN2> CurrentControl<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    pub fn new(
        shunt_resistance: u32,
        pwm: PWM,
        in1: IN1,
        in2: IN2,
    ) -> Self {
        let mut s = Self {
            shunt_resistance,
            current_setpoint: 0,
            adc_value: 0,
            voltage: 0,
            current: 0,
            duty_cycle: 0,
            motor: motor::Motor::l298(in1, in2, pwm),
            direction: Direction::CW,
            pid: PIDController::new(80, 0, 0),  // PID

            current_buffer: [0; CURRENT_BUFFER_SIZE],
            buffer_index: 0,
        };

        //s.set_duty_cycle(220);
        //s.pid.set_limits(0.0, s.motor.get_max_duty() as f32);
        s.pid.set_limits(0, 230);
        s.motor.cw();
        s
    }

    /// Can be positive and negative
    pub fn set_current(&mut self, milli_amps: i32) {
        self.current_setpoint = milli_amps;
        self.pid.set_target(milli_amps);
    }

    pub fn adc_value(&self) -> u32 {
        self.adc_value
    }

    pub fn duty_cycle(&self) -> u32 {
        self.duty_cycle
    }

    pub fn voltage(&self) -> u32 {
        self.voltage
    }

    pub fn current(&self) -> u32 {
        self.current
    }

    pub fn update(&mut self, dt:u32, adc_value: u32, adc_voltage: u32)
    {
        self.adc_value = adc_value;
        self.voltage = adc_voltage;

        self.set_direction();

        let mut current = 0;
        if self.voltage > 0
        {
            current = (self.voltage * 1000) / self.shunt_resistance; // uV / mOhm = mA
        }
        self.average_current(current, dt);
        //self.current = current;

        //self.calc_pwm(dt);
    }

    fn average_current(&mut self, current: u32, dt:u32) {
        self.current_buffer[self.buffer_index] = current;
        if self.buffer_index < (CURRENT_BUFFER_SIZE - 1) {
            self.buffer_index += 1;
        }
        else {
            self.buffer_index = 0;
            self.calc_pwm(dt);
        }

        // @5khz
        self.current = self.current_buffer.iter().sum::<u32>() / CURRENT_BUFFER_SIZE as u32;
    }

    fn set_direction(&mut self)
    {
        if self.current_setpoint >= 0
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

    fn set_duty_cycle(&mut self, duty_cycle : u32) {
        self.duty_cycle = duty_cycle;
        self.motor.duty(self.duty_cycle as u16);
    }

    fn calc_pwm(&mut self, dt: u32) 
    {
        let mut duty_cycle = self.duty_cycle as i32;
        duty_cycle += self.pid.update(self.current as i32, dt as i32) / PID_SCALING_FACTOR;

        let duty_cycle = util::limit_range(0, 230, duty_cycle) as u32;
        self.set_duty_cycle(duty_cycle);
    }
}