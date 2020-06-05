use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use l298n;
use stepper_servo_lib::current_control::CurrentOutput;

#[derive(PartialEq)]
enum Direction {
    Brake,
    CCW,
    CW,
}

/// For now hard bound to ADC1
pub struct CurrentOuput<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    duty_cycle: u32,
    motor: l298n::Motor<IN1, IN2, PWM>,
    direction: Direction,
    enabled: bool,
}

impl<PWM, IN1, IN2> CurrentOuput<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    pub fn new(pwm: PWM, in1: IN1, in2: IN2) -> Self {
        let mut s = Self {
            duty_cycle: 0,
            motor: l298n::Motor::new(in1, in2, pwm),
            direction: Direction::CW,
            enabled: false,
        };
        s.motor.forward();
        s
    }

    pub fn duty_cycle(&self) -> u32 {
        self.duty_cycle
    }

    fn set_direction(&mut self, value: i32) {
        if self.enabled {
            if value >= 0 {
                if self.direction != Direction::CW {
                    self.motor.forward();
                    self.direction = Direction::CW;
                }
            } else {
                if self.direction != Direction::CCW {
                    self.motor.reverse();
                    self.direction = Direction::CCW;
                }
            }
        }
    }

    fn set_duty_cycle(&mut self, value: i32) {
        if self.enabled {
            self.duty_cycle = if value >= 0 {
                value as u32
            } else {
                (value * -1) as u32
            };

            self.motor.set_duty(self.duty_cycle as u16);
        }
    }
}

impl<PWM, IN1, IN2> CurrentOutput for CurrentOuput<PWM, IN1, IN2>
where
    PWM: PwmPin<Duty = u16>,
    IN1: OutputPin,
    IN2: OutputPin,
{
    fn set_output_value(&mut self, value: i32) {
        self.set_direction(value);
        self.set_duty_cycle(value);
    }

    fn enable(&mut self, enable: bool) {
        match enable {
            true => {
                self.enabled = true;
            }
            false => {
                self.duty_cycle = 0;
                self.motor.set_duty(0);
                self.motor.brake();
                self.enabled = false;
                self.direction = Direction::Brake;
            }
        }
    }

    fn get_max_output_value(&mut self) -> i32 {
        self.motor.get_max_duty() as i32
    }
}
