#![no_std]
#![no_main]

// Local modules
mod current_output;
mod display;
mod position_input;

// Imports
//use core::sync::atomic::{AtomicUsize, Ordering};
//use cortex_m::asm::nop;
//use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rtfm::cyccnt::{Duration, Instant, U32Ext /*CYCCNT*/};
//use rtfm::Monotonic;
use stepper_servo_lib::{
    current_control::{CurrentControl, CurrentDevice, PIDControl},
    motor_control::MotorControl,
    serial_commands::{Command, SerialCommands},
};
use stm32f1xx_hal::{
    adc, device,
    gpio::{gpioa, gpiob, gpioc},
    gpio::{Alternate, Input, OpenDrain, Output, PullUp, PushPull},
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    pwm,
    serial::{Config, Rx, Serial, Tx},
    timer::{Tim2NoRemap, Timer},
};

const DWT_FREQ: u32 = 72_000_000;
const BLINKING_LED_PERIOD: u32 = DWT_FREQ / 2;
const DISPLAY_REFRESH_PERIOD: u32 = DWT_FREQ / 10;
const MOTOR_CONTROL_PERIOD: u32 = DWT_FREQ / 1_000;
const CONTROL_LOOP_PERIOD: u32 = DWT_FREQ / 20_000;
const SHUNT_RESISTANCE: u32 = 400; //mOhms

// Types
type AdcCoilAType = adc::AdcInt<device::ADC1>;
type AdcCoilBType = adc::AdcInt<device::ADC2>;
type CurrentOutputCoilAType = current_output::CurrentOuput<
    pwm::PwmChannel<device::TIM2, stm32f1xx_hal::pwm::C2>,
    gpioa::PA2<Output<PushPull>>,
    gpioa::PA3<Output<PushPull>>,
>;
type CurrentOutputCoilBType = current_output::CurrentOuput<
    pwm::PwmChannel<device::TIM2, stm32f1xx_hal::pwm::C1>,
    gpioa::PA4<Output<PushPull>>,
    gpioa::PA5<Output<PushPull>>,
>;
type MotorControlType =
    MotorControl<CurrentControl<CurrentOutputCoilAType>, CurrentControl<CurrentOutputCoilBType>>;
type PositionControlType =
    position_input::PositionInput<gpiob::PB10<Input<PullUp>>, gpiob::PB11<Input<PullUp>>>;
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
        adc_coil_a: AdcCoilAType,
        adc_coil_b: AdcCoilBType,
        motor_control: MotorControlType,
        position_control: PositionControlType,
        onboard_led: gpioc::PC13<Output<PushPull>>,
        display: DisplayType,
        usart1: Usart1Type,
        serial_commands: SerialCommands,
        debug_pin: gpiob::PB12<Output<PushPull>>,
        total_sleep_time: Duration,
        processor_usage: i32,
    }

    #[init(schedule = [blink_led, update_motor, control_loop, update_display, cpu_usage])]
    fn init(cx: init::Context) -> init::LateResources {
        let peripherals = cx.device;
        let mut core = cx.core;
        let mut rcc = peripherals.RCC.constrain();
        let mut flash = peripherals.FLASH.constrain();

        let mut gpioa: gpioa::Parts = peripherals.GPIOA.split(&mut rcc.apb2);
        let mut gpiob: gpiob::Parts = peripherals.GPIOB.split(&mut rcc.apb2);
        let mut gpioc: gpioc::Parts = peripherals.GPIOC.split(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .hclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(36.mhz())
            .adcclk(8.mhz()) // 8.000.000 / ADC:~250cycles = 36khz.
            .freeze(&mut flash.acr);

        // Enable the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        // Current Control - Coil A & B - PWM
        let pwm_pin_coil_a = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
        let pwm_pin_coli_b = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
        let timer2 = Timer::tim2(peripherals.TIM2, &clocks, &mut rcc.apb1);
        let pwm_timer2 = timer2.pwm::<Tim2NoRemap, _, _, _>(
            (pwm_pin_coli_b, pwm_pin_coil_a),
            &mut afio.mapr,
            20.khz(),
        );
        let pwm_channels = pwm_timer2.split();
        let mut pwm_coil_a = pwm_channels.1;
        let mut pwm_coil_b = pwm_channels.0;

        pwm_coil_a.enable();
        pwm_coil_b.enable();

        // Current Control - Coil A - ADC
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let mut adc = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);
        adc.set_sample_time(adc::SampleTime::T_239);
        let mut adc_coil_a = adc.into_interrupt(ch0);
        adc_coil_a.enable();

        // Current Control - Coil A - DIR
        let in1 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        let in2 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let current_output = CurrentOutputCoilAType::new(pwm_coil_a, in1, in2);
        let current_control_coil_a = CurrentControl::new(
            SHUNT_RESISTANCE,
            current_output,
            adc_coil_a.max_sample() as u32,
        );

        // Current Control - Coil B - ADC
        let ch0 = gpiob.pb1.into_analog(&mut gpiob.crl);
        let mut adc = adc::Adc::adc2(peripherals.ADC2, &mut rcc.apb2, clocks);
        adc.set_sample_time(adc::SampleTime::T_239);
        let mut adc_coil_b = adc.into_interrupt(ch0);
        adc_coil_b.enable();

        // Current Control - Coil B - DIR
        let in1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let in2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let current_output = CurrentOutputCoilBType::new(pwm_coil_b, in1, in2);
        let current_control_coil_b = CurrentControl::new(
            SHUNT_RESISTANCE,
            current_output,
            adc_coil_b.max_sample() as u32,
        );

        // Position control
        let pin_a = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
        let pin_b = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);
        let position_control = position_input::PositionInput::new(pin_a, pin_b);

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

        let mut motor_control = MotorControl::new(current_control_coil_a, current_control_coil_b);
        motor_control.set_controller_p(0);
        motor_control.set_controller_i(7);
        motor_control.set_controller_d(0);

        // CPU usage
        cx.schedule.cpu_usage(cx.start + DWT_FREQ.cycles()).unwrap();

        // Motor update
        cx.schedule
            .update_motor(cx.start + MOTOR_CONTROL_PERIOD.cycles())
            .unwrap();

        // control_loop update
        cx.schedule
            .control_loop(cx.start + CONTROL_LOOP_PERIOD.cycles())
            .unwrap();

        // DEBUG
        let debug_pin = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

        init::LateResources {
            adc_coil_a,
            adc_coil_b,
            motor_control,
            position_control,
            onboard_led,
            display,
            usart1,
            serial_commands: SerialCommands::default(),
            debug_pin,
            total_sleep_time: Duration::from_cycles(0),
            processor_usage: 0,
        }
    }

    #[task(schedule = [cpu_usage],  priority = 5,
        resources = [total_sleep_time, processor_usage])]
    fn cpu_usage(cx: cpu_usage::Context) {
        // Calc elapsed time
        *cx.resources.processor_usage = (100.0
            - (100.0 * cx.resources.total_sleep_time.as_cycles() as f32) / DWT_FREQ as f32)
            as i32;

        *cx.resources.total_sleep_time = Duration::from_cycles(0);

        cx.schedule
            .cpu_usage(cx.scheduled + DWT_FREQ.cycles())
            .unwrap();
    }

    #[task(schedule = [blink_led], resources = [onboard_led])]
    fn blink_led(cx: blink_led::Context) {
        // Blink
        cx.resources.onboard_led.toggle().unwrap();
        cx.schedule
            .blink_led(cx.scheduled + BLINKING_LED_PERIOD.cycles())
            .unwrap();
    }

    #[task(schedule = [update_motor],  priority = 5,
        resources = [motor_control])]
    fn update_motor(mut cx: update_motor::Context) {
        // Returns nex required update in uS
        let mut next_call_us = 0;
        cx.resources
            .motor_control
            .lock(|m| next_call_us = m.update());
        let next_call_cycles = next_call_us; // @ 72_000_000 hz

        cx.schedule
            .update_motor(cx.scheduled + Duration::from_cycles(next_call_cycles))
            .unwrap();
    }

    #[task(schedule = [update_display], resources = [display, motor_control, processor_usage])]
    fn update_display(mut cx: update_display::Context) {
        let (mut adc_value, mut voltage, mut current, mut duty_cycle): (i32, i32, i32, i32) =
            Default::default();

        // COIL A
        cx.resources.motor_control.lock(|m| {
            let current_control_coil_a = m.coil_a().current_control();
            adc_value = current_control_coil_a.adc_value() as i32;
            voltage = current_control_coil_a.voltage() as i32;
            current = current_control_coil_a.current() as i32;
            duty_cycle = current_control_coil_a.get_current_output().duty_cycle() as i32;
        });

        cx.resources
            .display
            .update_row_column("ADC", 2, 0, adc_value);
        cx.resources.display.update_row_column("mV", 3, 0, voltage);
        cx.resources.display.update_row_column("mA", 4, 0, current);
        cx.resources
            .display
            .update_row_column("Dut", 5, 0, duty_cycle);

        // COIL B
        cx.resources.motor_control.lock(|m| {
            let current_control_coil_a = m.coil_b().current_control();
            adc_value = current_control_coil_a.adc_value() as i32;
            voltage = current_control_coil_a.voltage() as i32;
            current = current_control_coil_a.current() as i32;
            duty_cycle = current_control_coil_a.get_current_output().duty_cycle() as i32;
        });

        cx.resources
            .display
            .update_row_column("ADC", 2, 8, adc_value);
        cx.resources.display.update_row_column("mV", 3, 8, voltage);
        cx.resources.display.update_row_column("mA", 4, 8, current);
        cx.resources
            .display
            .update_row_column("Dut", 5, 8, duty_cycle);

        // CPU
        let mut processor_usage = 0;
        cx.resources.processor_usage.lock(|p| processor_usage = *p);
        cx.resources
            .display
            .update_row_column("Cpu", 7, 0, processor_usage);

        // Re-shedule
        cx.schedule
            .update_display((cx.scheduled + DISPLAY_REFRESH_PERIOD.cycles()).into())
            .unwrap();
    }

    #[task(schedule = [control_loop], priority = 9,
        resources = [motor_control])]
    fn control_loop(mut cx: control_loop::Context) {
        // Update the current loop
        cx.resources
            .motor_control
            .lock(|m| m.update_control_loop(CONTROL_LOOP_PERIOD));

        // Re-shedule
        cx.schedule
            .control_loop((cx.scheduled + CONTROL_LOOP_PERIOD.cycles()).into())
            .unwrap();
    }

    #[task(binds = ADC1_2, priority = 10,
        resources = [ adc_coil_a, adc_coil_b, motor_control, debug_pin])]
    fn handle_adc(cx: handle_adc::Context) {
        let adc_coil_a: &AdcCoilAType = &cx.resources.adc_coil_a;
        if adc_coil_a.is_ready() {
            // END mark
            //cx.resources.debug_pin.set_low().unwrap();

            let adc_value = adc_coil_a.read_value() as u32;

            // Remove offset
            let ADC_Offset = 15;
            let adc_value = (adc_value as i32 - ADC_Offset).max(0) as u32;

            let motor_control: &mut MotorControlType = cx.resources.motor_control;
            let current_control = motor_control.coil_a().current_control();
            current_control.add_sample(adc_value);

            // START mark
            //cx.resources.debug_pin.set_high().unwrap();
        }

        let adc_coil_b: &AdcCoilBType = &cx.resources.adc_coil_b;
        if adc_coil_b.is_ready() {
            let adc_value = adc_coil_b.read_value() as u32;

            // Remove offset
            let ADC_Offset = 15;
            let adc_value = (adc_value as i32 - ADC_Offset).max(0) as u32;

            let motor_control: &mut MotorControlType = cx.resources.motor_control;
            let current_control = motor_control.coil_b().current_control();
            current_control.add_sample(adc_value);
        }
    }

    #[task(binds = USART1, resources = [usart1, serial_commands, motor_control])]
    fn handle_usart1(mut cx: handle_usart1::Context) {
        let (_tx, rx): &mut Usart1Type = cx.resources.usart1;
        let serial_commands: &mut SerialCommands = cx.resources.serial_commands;

        let data = rx.read().unwrap();
        serial_commands.add_character(data);

        match serial_commands.get_command() {
            Some(Command::Enable) => cx.resources.motor_control.lock(|m| m.enable(true)),
            Some(Command::Disable) => cx.resources.motor_control.lock(|m| m.enable(false)),
            Some(Command::Run { speed }) => cx.resources.motor_control.lock(|m| m.rotate(speed)),
            Some(Command::Hold) => cx.resources.motor_control.lock(|m| m.hold(true)),
            Some(Command::Cur { current }) => {
                cx.resources.motor_control.lock(|m| m.set_current(current))
            }
            Some(Command::P(value)) => cx
                .resources
                .motor_control
                .lock(|m| m.set_controller_p(value)),
            Some(Command::I(value)) => cx
                .resources
                .motor_control
                .lock(|m| m.set_controller_i(value)),
            Some(Command::D(value)) => cx
                .resources
                .motor_control
                .lock(|m| m.set_controller_d(value)),
            Some(Command::ForceDuty(duty)) => {
                cx.resources.motor_control.lock(|m| m.force_duty(duty))
            }
            None => (),
        }
    }

    #[idle(resources = [total_sleep_time, debug_pin])] // PRIO = 0
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::interrupt::free(|_| {
                // END mark
                //cx.resources.debug_pin.set_low().unwrap();

                let sleep = Instant::now();
                rtfm::export::wfi();

                // START mark
                //cx.resources.debug_pin.set_high().unwrap();

                cx.resources.total_sleep_time.lock(|total_time| {
                    *total_time += Instant::now().duration_since(sleep);
                });
            })
        }
    }

    extern "C" {
        fn EXTI0();
        fn EXTI1();
        fn EXTI2();
    }
};
