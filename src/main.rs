#![no_std]
#![no_main]

// Local modules
mod current_output;
mod display;
mod position_control;

// Imports
use cortex_m::asm::nop;
use embedded_hal::digital::v2::OutputPin;
use motor_control::{CurrentDevice, MotorControl, PositionControlled};
use panic_halt as _;
use rtfm::cyccnt::{Duration, Instant, U32Ext, CYCCNT};
use rtfm::Monotonic;
use stepper_servo_lib::{
    current_control::CurrentControl,
    motor_control,
    serial_commands::{Command, SerialCommands},
};
use stm32f1xx_hal::{
    adc, device,
    gpio::{gpioa, gpiob, gpioc},
    gpio::{Alternate, Input, OpenDrain, Output, PullUp, PushPull},
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    pwm,
    pwm::Channel,
    serial::{Config, Rx, Serial, Tx},
    timer::{Tim2NoRemap, Timer},
};

const DWT_FREQ: u32 = 8_000_000;
const BLINKING_LED_PERIOD: u32 = DWT_FREQ / 1;
const DISPLAY_REFRESH_PERIOD: u32 = DWT_FREQ / 10;
const SHUNT_RESISTANCE: u32 = 400; //mOhms

// Types
type AdcType = adc::AdcInt<device::ADC1>;
type CurrentOutputCoilAType = current_output::CurrentOuput<
    pwm::PwmChannel<device::TIM2, stm32f1xx_hal::pwm::C1>,
    gpioa::PA2<Output<PushPull>>,
    gpioa::PA3<Output<PushPull>>,
>;
type CurrentOutputCoilBType = current_output::CurrentOuput<
    pwm::PwmChannel<device::TIM2, stm32f1xx_hal::pwm::C1>,
    gpioa::PA4<Output<PushPull>>,
    gpioa::PA5<Output<PushPull>>,
>;
type MotorControlType = MotorControl<CurrentControl<CurrentOutputCoilAType>>;
type PositionControlType =
    position_control::PositionControl<gpiob::PB10<Input<PullUp>>, gpiob::PB11<Input<PullUp>>>;
type DisplayType = display::Display<
    ssd1306::interface::i2c::I2cInterface<
        BlockingI2c<
            device::I2C1,
            (
                gpiob::PB6<Alternate<OpenDrain>>,
                gpiob::PB7<Alternate<OpenDrain>>,
            ),
        >,
    >,
>;
type Usart1Type = (Tx<device::USART1>, Rx<device::USART1>);

#[rtfm::app(device = stm32f1xx_hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        adc: AdcType,
        motor_control: MotorControlType,
        position_control: PositionControlType,
        onboard_led: gpioc::PC13<Output<PushPull>>,
        display: DisplayType,
        usart1: Usart1Type,
        serial_commands: SerialCommands,
        prev_time: Instant,
        trigger_pin: gpioa::PA4<Output<PushPull>>,
        total_sleep_time: Duration,
        previous_time: Instant,
        processor_usage: u32,
    }

    #[init(schedule = [blink_led, update_display])]
    fn init(cx: init::Context) -> init::LateResources {
        let peripherals = cx.device;
        let mut core = cx.core;
        let mut rcc = peripherals.RCC.constrain();
        let mut flash = peripherals.FLASH.constrain();

        let mut gpioa: gpioa::Parts = peripherals.GPIOA.split(&mut rcc.apb2);
        let mut gpiob: gpiob::Parts = peripherals.GPIOB.split(&mut rcc.apb2);
        let mut gpioc: gpioc::Parts = peripherals.GPIOC.split(&mut rcc.apb2);

        let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

        // Enable the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        // Current Control - Coil A - ADC
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let mut adc = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);
        adc.set_sample_time(adc::SampleTime::T_239);
        let mut adc_coil_a = adc.into_interrupt(ch0);
        adc_coil_a.enable();

        // Current Control - Coil A - PWM
        let pwm_pin = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
        let timer2 = Timer::tim2(peripherals.TIM2, &clocks, &mut rcc.apb1);
        let mut pwm_timer2 = timer2.pwm::<Tim2NoRemap, _, _, _>(pwm_pin, &mut afio.mapr, 20.khz());
        pwm_timer2.enable(Channel::C1);

        // Current Control - Coil A - DIR
        let in1 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        let in2 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let pwm = pwm_timer2.split();
        let current_output = CurrentOutputCoilAType::new(pwm, in1, in2);
        let mut current_control_coil_a = CurrentControl::new(SHUNT_RESISTANCE, current_output);
        current_control_coil_a.set_current(100);

        // Current Control - Coil B - ADC
        let ch0 = gpiob.pb1.into_analog(&mut gpiob.crl);
        let mut adc = adc::Adc::adc2(peripherals.ADC2, &mut rcc.apb2, clocks);
        adc.set_sample_time(adc::SampleTime::T_239);
        let mut adc_coil_b = adc.into_interrupt(ch0);
        adc_coil_b.enable();

        // Current Control - Coil B - PWM
        let pwm_pin = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
        let timer3 = Timer::tim3(peripherals.TIM3, &clocks, &mut rcc.apb1);
        let mut pwm_timer3 = timer3.pwm::<Tim2NoRemap, _, _, _>(pwm_pin, &mut afio.mapr, 20.khz());
        pwm_timer3.enable(Channel::C1);
        
        // Current Control - Coil B - DIR
        let in1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let in2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let pwm = pwm_timer2.split();
        let current_output = CurrentOutputCoilBType::new(pwm, in1, in2);
        let mut current_control_coil_b = CurrentControl::new(SHUNT_RESISTANCE, current_output);
        current_control_coil_b.set_current(100);
        



        // Position control
        let pin_a = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
        let pin_b = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);
        let position_control = position_control::PositionControl::new(pin_a, pin_b);

        // Led
        let onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        cx.schedule
            .blink_led(cx.start + BLINKING_LED_PERIOD.cycles())
            .unwrap();

        // Display
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c = BlockingI2c::i2c1(
            peripherals.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );
        let display = display::Display::new(i2c);
        cx.schedule
            .update_display(cx.start + DISPLAY_REFRESH_PERIOD.cycles())
            .unwrap();

        // USART1
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10;
        let mut usart1 = Serial::usart1(
            peripherals.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb2,
        )
        .split();
        usart1.1.listen();

        // DEBUG
        let trigger_pin = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

        init::LateResources {
            adc: adc_coil_a,
            motor_control: MotorControl::new(current_control_coil_a),
            position_control,
            onboard_led,
            display,
            usart1,
            serial_commands: SerialCommands::default(),
            prev_time: cx.start,
            trigger_pin,
            total_sleep_time: Duration::from_cycles(0),
            previous_time: CYCCNT::zero(),
            processor_usage: 0,
        }
    }

    #[task(schedule = [blink_led], resources = [onboard_led, total_sleep_time, previous_time, processor_usage])]
    fn blink_led(cx: blink_led::Context) {
        // Calc elapsed time
        let current_time = Instant::now();
        let elapsed_time = current_time
            .duration_since(*cx.resources.previous_time)
            .as_cycles();

        *cx.resources.processor_usage =
            100 - (100 * cx.resources.total_sleep_time.as_cycles()) / elapsed_time;

        // Prepare for next iteration.
        *cx.resources.total_sleep_time = Duration::from_cycles(0);
        *cx.resources.previous_time = current_time;

        // Blink
        cx.resources.onboard_led.toggle().unwrap();

        cx.schedule
            .blink_led(cx.scheduled + BLINKING_LED_PERIOD.cycles())
            .unwrap();
    }

    #[task(schedule = [update_display], resources = [display, motor_control, processor_usage])]
    fn update_display(cx: update_display::Context) {
        let motor_control: &mut MotorControlType = cx.resources.motor_control;
        let motor = motor_control.get_current_control();

        cx.resources
            .display
            .update("Cyc", 0, cx.scheduled.elapsed().as_cycles() as i32);
        cx.resources
            .display
            .update("ADC", 2, motor.adc_value() as i32);
        cx.resources
            .display
            .update_row_column("mV", 3, 0, motor.voltage() as i32);
        cx.resources
            .display
            .update_row_column("mA", 3, 8, motor.current());
        cx.resources
            .display
            .update("Dut", 4, motor.get_current_output().duty_cycle() as i32);
        cx.resources
            .display
            .update("Cpu", 7, *cx.resources.processor_usage as i32);

        cx.schedule
            .update_display((cx.scheduled + DISPLAY_REFRESH_PERIOD.cycles()).into())
            .unwrap();
    }

    #[task(binds = ADC1_2, resources = [adc, motor_control, prev_time, trigger_pin])]
    fn handle_adc(cx: handle_adc::Context) {
        let adc: &AdcType = &cx.resources.adc;
        if adc.is_ready() {
            let adc_value = adc.read_value() as u32;
            cx.resources.trigger_pin.set_high().unwrap();

            let adc_voltage = if adc_value > 0 {
                ((3300 * adc_value) / adc.max_sample() as u32) as i32
            } else {
                0
            };

            let delta_time = cx.start.duration_since(*cx.resources.prev_time).as_cycles();

            // Set new angle == current setpoint
            // 1Hz = DWT_FREQ(8_000_000) cycles
            let motor_control: &mut MotorControlType = cx.resources.motor_control;
            // static mut ELAPSED: u32 = 0;
            // unsafe {
            //     ELAPSED += delta_time;
            //     if ELAPSED >= DWT_FREQ / 360 {
            //         ELAPSED -= DWT_FREQ / 360;
            //         motor_control.set_angle(motor_control.get_angle() + 1);
            //     }
            // };

            // Update the current loop
            let motor = motor_control.get_current_control();
            motor.update(delta_time, adc_value, adc_voltage);

            // used for delta time.
            *cx.resources.prev_time = cx.start;
            cx.resources.trigger_pin.set_low().unwrap();
        }
    }

    #[task(binds = USART1, resources = [usart1, serial_commands, motor_control])]
    fn handle_usart1(cx: handle_usart1::Context) {
        let (_tx, rx): &mut Usart1Type = cx.resources.usart1;
        let serial_commands: &mut SerialCommands = cx.resources.serial_commands;
        let motor_control: &mut MotorControlType = cx.resources.motor_control;
        let motor = motor_control.get_current_control();

        let data = rx.read().unwrap();
        serial_commands.add_character(data);

        match serial_commands.get_command() {
            Some(Command::Enable) => motor.enable(true),
            Some(Command::Disable) => motor.enable(false),
            Some(Command::Left { speed: _ }) => (),
            Some(Command::Right { speed: _ }) => (),
            Some(Command::P(value)) => motor.set_p_value(value),
            Some(Command::I(value)) => motor.set_i_value(value),
            Some(Command::D(value)) => motor.set_d_value(value),
            None => (),
        }
    }

    #[idle(resources = [total_sleep_time])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            nop();

            cortex_m::interrupt::free(|_| {
                let sleep = Instant::now();
                rtfm::export::wfi();

                cx.resources.total_sleep_time.lock(|total_time| {
                    *total_time += Instant::now().duration_since(sleep);
                });
            })
        }
    }

    extern "C" {
        fn EXTI0();
    }
};
