#![no_std]
#![no_main]

// Local modules
mod current_output;
mod display;
mod encoder_input;

// Imports
//use core::sync::atomic::{AtomicUsize, Ordering};
//use cortex_m::asm::nop;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
//use panic_halt as _;
use rtic::cyccnt::{Duration, Instant, U32Ext /*CYCCNT*/};
//use rtfm::Monotonic;
use core::fmt::Write;
use nb;
use stepper_servo_lib::{
    current_control::{CurrentControl, CurrentDevice, PIDControl},
    motor_control::MotorControl,
    serial_commands::{Command, SerialCommands},
};
use stm32f1xx_hal::{
    adc,
    gpio::*,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
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

const SHUNT_RESISTANCE: u32 = 220; //mOhms
const ADC_1_OFFSET: u32 = 125; //bits
const ADC_2_OFFSET: u32 = 125; //bits

// Types
type AdcCoilAType = adc::AdcInt<pac::ADC1>;
//type AdcCoilBType = adc::AdcInt<pac::ADC2>;
type AdcCoilBType = adc::AdcInjInt<pac::ADC2>;
type CurrentOutputCoilAType = current_output::CurrentOuput<
    pwm::PwmChannel<pac::TIM2, stm32f1xx_hal::pwm::C3>,
    pwm::PwmChannel<pac::TIM2, stm32f1xx_hal::pwm::C1>,
    gpioa::PA0<Output<PushPull>>,
    gpioa::PA1<Output<PushPull>>,
>;
type CurrentOutputCoilBType = current_output::CurrentOuput<
    pwm::PwmChannel<pac::TIM2, stm32f1xx_hal::pwm::C4>,
    pwm::PwmChannel<pac::TIM2, stm32f1xx_hal::pwm::C2>,
    gpioa::PA4<Output<PushPull>>,
    gpioa::PA5<Output<PushPull>>,
>;
type PositionControlType =
    encoder_input::EncoderInput<gpiob::PB10<Input<PullUp>>, gpiob::PB11<Input<PullUp>>>;
type MotorControlType = MotorControl<
    CurrentControl<CurrentOutputCoilAType>,
    CurrentControl<CurrentOutputCoilBType>,
    PositionControlType,
>;
type DisplayType = display::Display<
    ssd1306::interface::i2c::I2cInterface<
        BlockingI2c<
            pac::I2C1,
            (
                gpiob::PB6<Alternate<OpenDrain>>,
                gpiob::PB7<Alternate<OpenDrain>>,
            ),
        >,
    >,
>;
type Usart1Type = (Tx<pac::USART1>, Rx<pac::USART1>);

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        adc_coil_a: AdcCoilAType,
        adc_coil_b: AdcCoilBType,
        motor_control: MotorControlType,
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
        let pwm_pin_coil_a = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let pwm_pin_coli_b = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);
        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
        let timer2 = Timer::tim2(peripherals.TIM2, &clocks, &mut rcc.apb1);
        let mut pwm_timer2 = timer2.pwm::<Tim2NoRemap, _, _, _>(
            (pwm_pin_coil_a, pwm_pin_coli_b),
            &mut afio.mapr,
            40.khz(),
        );
        let (mut coil_a_trigger_adc, mut coil_b_trigger_adc) = pwm_timer2.config_ch1_ch2();
        pwm_timer2.change_pwm_mode(pac::tim2::cr1::CMS_A::CENTERALIGNED2);
        let (mut pwm_coil_a, mut pwm_coil_b) = pwm_timer2.split();

        coil_a_trigger_adc.enable();
        coil_b_trigger_adc.enable();
        pwm_coil_a.enable();
        pwm_coil_b.enable();

        // Current Control - Coil A - ADC
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let mut adc = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);
        adc.set_sample_time(adc::SampleTime::T_41);
        let mut adc_coil_a = adc.into_reg_interrupt(ch0);
        adc_coil_a.enable_ext_trigger(pac::adc1::cr2::EXTSEL_A::TIM2CC2);

        // Current Control - Coil A - DIR
        let in1 = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);
        let in2 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
        let current_output = CurrentOutputCoilAType::new(pwm_coil_a, coil_a_trigger_adc, in1, in2);
        let current_control_coil_a = CurrentControl::new(
            SHUNT_RESISTANCE,
            current_output,
            ADC_1_OFFSET,
            adc_coil_a.max_sample() as u32,
        );

        // Current Control - Coil B - ADC
        let ch0 = gpiob.pb1.into_analog(&mut gpiob.crl);
        let mut adc = adc::Adc::adc2(peripherals.ADC2, &mut rcc.apb2, clocks);
        adc.set_sample_time(adc::SampleTime::T_41);
        let mut adc_coil_b = adc.into_inj_interrupt(ch0);
        //let mut adc_coil_b = adc.into_reg_interrupt(ch0);
        adc_coil_b.enable_ext_trigger(pac::adc2::cr2::JEXTSEL_A::TIM2CC1);
        //adc_coil_b.enable_ext_trigger(pac::adc2::cr2::EXTSEL_A::TIM2CC2);

        // Current Control - Coil B - DIR
        let in1 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let in2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let current_output = CurrentOutputCoilBType::new(pwm_coil_b, coil_b_trigger_adc, in1, in2);
        let current_control_coil_b = CurrentControl::new(
            SHUNT_RESISTANCE,
            current_output,
            ADC_2_OFFSET,
            adc_coil_b.max_sample() as u32,
        );

        // Position control
        let mut position_pin_a = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
        position_pin_a.make_interrupt_source(&mut afio);
        position_pin_a.trigger_on_edge(&peripherals.EXTI, Edge::RISING_FALLING);
        position_pin_a.enable_interrupt(&peripherals.EXTI);
        let mut position_pin_b = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);
        position_pin_b.make_interrupt_source(&mut afio);
        position_pin_b.trigger_on_edge(&peripherals.EXTI, Edge::RISING_FALLING);
        position_pin_b.enable_interrupt(&peripherals.EXTI);
        let position_control = encoder_input::EncoderInput::new(position_pin_a, position_pin_b);

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
            Config::default().baudrate(115200.bps()),
            clocks,
            &mut rcc.apb2,
        )
        .split();
        usart1.1.listen();

        let mut motor_control = MotorControl::new(
            current_control_coil_a,
            current_control_coil_b,
            position_control,
        );
        motor_control.set_controller_p(2);
        motor_control.set_controller_i(30);
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
        // Returns nex required update in cycles
        let mut next_call_us = 0;
        cx.resources
            .motor_control
            .lock(|m| next_call_us = m.update());

        cx.schedule
            .update_motor(cx.scheduled + Duration::from_cycles(next_call_us))
            .unwrap();
    }

    #[task(schedule = [update_display], resources = [display, motor_control, processor_usage])]
    fn update_display(mut cx: update_display::Context) {
        let (mut adc_value, mut voltage, mut current, mut duty_cycle): (i32, i32, i32, i32) =
            Default::default();

        // COIL A
        cx.resources.motor_control.lock(|m| {
            let current_control = m.coil_a().current_control();
            adc_value = current_control.adc_value() as i32;
            voltage = current_control.voltage() as i32;
            current = current_control.current() as i32;
            duty_cycle = current_control.get_current_output().duty_cycle() as i32;
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
            let current_control = m.coil_b().current_control();
            adc_value = current_control.adc_value() as i32;
            voltage = current_control.voltage() as i32;
            current = current_control.current() as i32;
            duty_cycle = current_control.get_current_output().duty_cycle() as i32;
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

        // Pos
        let mut position = 0;
        cx.resources
            .motor_control
            .lock(|m| position = m.position_control().get_current_position());
        cx.resources.display.update_row_column("P", 7, 8, position);

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
            let adc_value = adc_coil_a.read_value() as u32;
            let motor_control: &mut MotorControlType = cx.resources.motor_control;
            let current_control = motor_control.coil_a().current_control();
            current_control.add_sample(adc_value);
        }

        let adc_coil_b: &AdcCoilBType = &cx.resources.adc_coil_b;
        if adc_coil_b.is_ready() {
            // START mark
            cx.resources.debug_pin.set_high().unwrap();

            let adc_value = adc_coil_b.read_value() as u32;
            let motor_control: &mut MotorControlType = cx.resources.motor_control;
            let current_control = motor_control.coil_b().current_control();
            current_control.add_sample(adc_value);

            // END mark
            cx.resources.debug_pin.set_low().unwrap();
        }
    }

    #[task(binds = USART1, resources = [usart1, serial_commands, motor_control])]
    fn handle_usart1(mut cx: handle_usart1::Context) {
        let (tx, rx): &mut Usart1Type = cx.resources.usart1;
        let serial_commands: &mut SerialCommands = cx.resources.serial_commands;

        let data = rx.read().unwrap();
        serial_commands.add_character(data);

        let command = serial_commands.get_command();
        match command {
            Some(Command::Enable) => cx.resources.motor_control.lock(|m| m.enable(true)),
            Some(Command::Disable) => cx.resources.motor_control.lock(|m| m.enable(false)),
            Some(Command::Rotate { speed }) => cx.resources.motor_control.lock(|m| m.rotate(speed)),
            Some(Command::Hold) => cx.resources.motor_control.lock(|m| m.hold()),
            Some(Command::Cur { current }) => {
                cx.resources.motor_control.lock(|m| m.set_current(current))
            }
            Some(Command::Position { position }) => cx.resources.motor_control.lock(|m| {
                m.set_position(position);
            }),
            Some(Command::Speed { speed }) => cx.resources.motor_control.lock(|m| {
                m.set_speed(speed);
            }),
            Some(Command::PositionAndSpeed { position, speed }) => {
                cx.resources.motor_control.lock(|m| {
                    m.set_position(position);
                    m.set_speed(speed);
                })
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
            Some(Command::Calibrate) => cx.resources.motor_control.lock(|m| m.calibrate()),
            Some(Command::ShowCalData) => cx.resources.motor_control.lock(|m| {
                let cal_data = m.position_control().get_calibration_data();
                for data in &mut cal_data.pulse_at_angle.iter() {
                    write!(tx, "{}\n", data).ok();
                    nb::block!(tx.flush()).ok();
                }
                write!(tx, "\n---------\n").ok();
                nb::block!(tx.flush()).ok();
            }),
            Some(Command::ForceDuty(duty)) => {
                cx.resources.motor_control.lock(|m| m.force_duty(duty))
            }
            None => (),
        }

        // Feedback
        if command.is_some() {
            write!(tx, " > OK").ok();
        }
        nb::block!(tx.flush()).ok();
        if data == b'\r' {
            write!(tx, "\n").ok();
        } else {
            nb::block!(tx.write(data)).ok();
        }
        nb::block!(tx.flush()).ok();
    }

    #[idle(resources = [total_sleep_time, /*debug_pin*/])] // PRIO = 0
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::interrupt::free(|_| {
                // END mark
                //cx.resources.debug_pin.set_low().unwrap();

                let sleep = Instant::now();
                rtic::export::wfi();

                // START mark
                //cx.resources.debug_pin.set_high().unwrap();

                cx.resources.total_sleep_time.lock(|total_time| {
                    *total_time += Instant::now().duration_since(sleep);
                });
            })
        }
    }

    #[task(binds = EXTI15_10, priority = 10,
        resources = [ motor_control, debug_pin])]
    fn position_control_input(cx: position_control_input::Context) {
        // END mark
        //cx.resources.debug_pin.toggle().unwrap();

        cx.resources.motor_control.handle_new_position();

        // START mark
        //cx.resources.debug_pin.set_high().unwrap();
    }

    extern "C" {
        // used interrupts for SW sheduling.
        fn DMA1_CHANNEL1();
        fn DMA1_CHANNEL2();
        fn DMA1_CHANNEL3();
    }
};
