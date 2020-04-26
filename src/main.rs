#![no_std]
#![no_main]

// Local modules
mod current_control;
mod display;
mod pid;
mod position_control;

// Imports
use cortex_m::asm::nop;
use panic_halt as _;
use rtfm::cyccnt::{Duration, Instant, U32Ext, CYCCNT};
use rtfm::Monotonic;
use stepper_servo_lib::serial_commands::{Command, SerialCommands};
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
type CurrentControlType = current_control::CurrentControl<
    pwm::PwmChannel<device::TIM2, stm32f1xx_hal::pwm::C1>,
    gpiob::PB12<Output<PushPull>>,
    gpiob::PB13<Output<PushPull>>,
>;
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
        current_control: CurrentControlType,
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

        //ADC
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let mut adc1 = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);
        adc1.set_sample_time(adc::SampleTime::T_239);
        let mut adc1 = adc1.into_interrupt(ch0);
        adc1.enable();
        // Enable the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        // Current control
        //PWM
        let pwm_pin = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
        let timer2 = Timer::tim2(peripherals.TIM2, &clocks, &mut rcc.apb1);
        let mut pwm_timer2 = timer2.pwm::<Tim2NoRemap, _, _, _>(pwm_pin, &mut afio.mapr, 20.khz());
        pwm_timer2.enable(Channel::C1);

        // Current Control
        let in1 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let in2 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        let pwm = pwm_timer2.split();
        let mut current_control =
            current_control::CurrentControl::new(SHUNT_RESISTANCE, pwm, in1, in2);
        current_control.set_current(100);

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
            adc: adc1,
            current_control,
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

    #[task(schedule = [update_display], resources = [display, current_control, processor_usage])]
    fn update_display(cx: update_display::Context) {
        cx.resources
            .display
            .update("Cyc", 0, cx.scheduled.elapsed().as_cycles());
        cx.resources
            .display
            .update("ADC", 2, cx.resources.current_control.adc_value() as u32);
        cx.resources
            .display
            .update_row_column("mV", 3, 0, cx.resources.current_control.voltage());
        cx.resources
            .display
            .update_row_column("mA", 3, 8, cx.resources.current_control.current());
        cx.resources
            .display
            .update("Dut", 4, cx.resources.current_control.duty_cycle() as u32);
        cx.resources
            .display
            .update("Cpu", 7, *cx.resources.processor_usage);

        cx.schedule
            .update_display(cx.scheduled + DISPLAY_REFRESH_PERIOD.cycles())
            .unwrap();
    }

    #[task(binds = ADC1_2, resources = [adc, current_control, prev_time, trigger_pin])]
    fn handle_adc(cx: handle_adc::Context) {
        //cx.resources.trigger_pin.toggle().unwrap();
        let adc: &AdcType = &cx.resources.adc;
        if adc.is_ready() {
            let adc_value = adc.read_value() as u32;
            let mut adc_voltage = 0;
            if adc_value > 0 {
                adc_voltage = (3300 * adc_value) / adc.max_sample() as u32;
            }

            let duration = cx.start.duration_since(*cx.resources.prev_time).as_cycles();
            cx.resources
                .current_control
                .update(duration, adc_value, adc_voltage);
            *cx.resources.prev_time = cx.start;
        }
    }

    #[task(binds = USART1, resources = [usart1, serial_commands, current_control])]
    fn handle_usart1(cx: handle_usart1::Context) {
        let (_tx, rx): &mut Usart1Type = cx.resources.usart1;
        let serial_commands: &mut SerialCommands = cx.resources.serial_commands;

        let data = rx.read().unwrap();
        serial_commands.add_character(data);

        let current_control = cx.resources.current_control;
        match serial_commands.get_command() {
            Some(Command::Stop) => (),
            Some(Command::Left { speed: _ }) => (),
            Some(Command::Right { speed: _ }) => (),
            Some(Command::P(value)) => current_control.set_p_value(value),
            Some(Command::I(value)) => current_control.set_i_value(value),
            Some(Command::D(value)) => current_control.set_d_value(value),
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
