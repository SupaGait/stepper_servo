#![no_std]
#![no_main]

use panic_halt as _;
use stm32f1xx_hal as hal;
use hal::{
    prelude::*, 
    adc,
    gpio::{Output, PushPull, gpioa, gpiob, gpioc,}, 
    timer::{Tim2NoRemap, Timer},
    i2c::{BlockingI2c, DutyCycle, Mode},
};
use rtfm::cyccnt::{Instant, U32Ext};
use cortex_m::asm::nop;
//use embedded_hal::digital::v2::OutputPin;

// Local modules
mod current_control;
mod position_control;
mod display;
mod pid;

const DWT_FREQ: u32 =  8_000_000;
const BLINKING_LED_PERIOD: u32 = DWT_FREQ/1;
const DISPLAY_REFRESH_PERIOD: u32 = DWT_FREQ/10;
const SHUNT_RESISTANCE: u32 = 400; //mOhms

#[rtfm::app(device = stm32f1xx_hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources<I2C> {
        adc: hal::adc::AdcInt<hal::stm32::ADC1>,
        current_control: current_control::CurrentControl<
            stm32f1xx_hal::pwm::Pwm<hal::device::TIM2, stm32f1xx_hal::pwm::C1>,
            stm32f1xx_hal::gpio::gpiob::PB12<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,         
            stm32f1xx_hal::gpio::gpiob::PB13<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>
            >,
        position_control: position_control::PositionControl<stm32f1xx_hal::gpio::gpiob::PB10<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>, stm32f1xx_hal::gpio::gpiob::PB11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>>,
        onboard_led: gpioc::PC13<Output<PushPull>>,
        display: display::Display<ssd1306::interface::i2c::I2cInterface<stm32f1xx_hal::i2c::BlockingI2c<hal::device::I2C1, (stm32f1xx_hal::gpio::gpiob::PB6<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>, stm32f1xx_hal::gpio::gpiob::PB7<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>)>>>,
        prev_time: Instant,
        trigger_pin: gpioa::PA4<Output<PushPull>>,
    }

    #[init(schedule = [blink_led, update_display, /*control_loop*/])]
    fn init(cx: init::Context)  -> init::LateResources {
        let peripherals = cx.device;
        let mut core = cx.core;
        let mut rcc = peripherals.RCC.constrain();  
        let mut flash = peripherals.FLASH.constrain();

        let mut gpioa: gpioa::Parts = peripherals.GPIOA.split(&mut rcc.apb2);
        let mut gpiob: gpiob::Parts = peripherals.GPIOB.split(&mut rcc.apb2);
        let mut gpioc: gpioc::Parts = peripherals.GPIOC.split(&mut rcc.apb2);

        let clocks = rcc.cfgr
            .adcclk(2.mhz())
            .freeze(&mut flash.acr);

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
        let mut pwm_timer2 = timer2.pwm::<Tim2NoRemap, _, _, _>(
            pwm_pin, 
            &mut afio.mapr,
            20.khz());
        pwm_timer2.enable();

        // Current Control
        let in1 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let in2 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        let mut current_control = current_control::CurrentControl::new(
            SHUNT_RESISTANCE,
            pwm_timer2,
            in1,
            in2);
        current_control.set_current(100);

        // Position control
        let pin_a = gpiob.pb10.into_pull_up_input( &mut gpiob.crh);
        let pin_b = gpiob.pb11.into_pull_up_input( &mut gpiob.crh);
        let position_control = position_control::PositionControl::new(pin_a, pin_b);

        // Led
        let onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        cx.schedule.blink_led(cx.start + BLINKING_LED_PERIOD.cycles()).unwrap();

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
        cx.schedule.update_display(cx.start + DISPLAY_REFRESH_PERIOD.cycles()).unwrap();

        // DEBUG
        let trigger_pin = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        
        init::LateResources {
            adc: adc1,
            current_control,
            position_control,
            onboard_led,
            display,
            prev_time: cx.start,
            trigger_pin,
        }
    }

    #[task(schedule = [blink_led], resources = [onboard_led])]
    fn blink_led(cx: blink_led::Context) {
        cx.resources.onboard_led.toggle().unwrap();

        cx.schedule.blink_led(cx.scheduled + BLINKING_LED_PERIOD.cycles()).unwrap();
    }

    #[task(schedule = [update_display], resources = [display, current_control])]
    fn update_display(cx: update_display::Context) {
        cx.resources.display.update("Cyc", 0, cx.scheduled.elapsed().as_cycles());
        cx.resources.display.update("ADC", 2, cx.resources.current_control.adc_value() as u32);
        cx.resources.display.update_row_column("mV", 3, 0, cx.resources.current_control.voltage());
        cx.resources.display.update_row_column("mA", 3, 8, cx.resources.current_control.current());
        cx.resources.display.update("Dut", 4, cx.resources.current_control.duty_cycle() as u32);

        cx.schedule.update_display(cx.scheduled + DISPLAY_REFRESH_PERIOD.cycles()).unwrap();
    }

    #[task(binds = ADC1_2, resources = [adc, current_control, prev_time, trigger_pin])]
    fn handle_adc(cx: handle_adc::Context) {
        cx.resources.trigger_pin.toggle().unwrap();
        let adc = &cx.resources.adc;
        if adc.is_ready() {
            let adc_value = adc.read_value() as u32;
            let mut adc_voltage = 0;
            if adc_value > 0
            {
                adc_voltage = (3300 * adc_value) / adc.max_sample() as u32;
            }

            let duration = cx.start.duration_since(*cx.resources.prev_time).as_cycles();
            cx.resources.current_control.update(duration, adc_value, adc_voltage);
            *cx.resources.prev_time = cx.start;
        }
    }

    #[idle(resources = [])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            nop();
        }
    }

    extern "C" {
        fn EXTI0();
    }
};
