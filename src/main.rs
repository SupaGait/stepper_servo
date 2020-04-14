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
use rtfm::cyccnt::{U32Ext};

// Local modules
mod current_control;
mod position_control;
mod display;

const BLINKING_LED_PERIOD: u32 = 8_000_000;
const DISPLAY_REFRESH_PARIOD: u32 = 8_000_000/10;

#[rtfm::app(device = stm32f1xx_hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources<I2C> {
        current_control: current_control::CurrentControl<stm32f1xx_hal::gpio::gpiob::PB0<stm32f1xx_hal::gpio::Analog>, stm32f1xx_hal::pwm::Pwm<hal::device::TIM2, stm32f1xx_hal::pwm::C1>>,
        position_control: position_control::PositionControl<stm32f1xx_hal::gpio::gpiob::PB10<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>, stm32f1xx_hal::gpio::gpiob::PB11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>>,
        onboard_led: gpioc::PC13<Output<PushPull>>,
        display: display::Display<ssd1306::interface::i2c::I2cInterface<stm32f1xx_hal::i2c::BlockingI2c<hal::device::I2C1, (stm32f1xx_hal::gpio::gpiob::PB6<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>, stm32f1xx_hal::gpio::gpiob::PB7<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>)>>>,
    }

    #[init(schedule = [blink_led, update_display])]
    fn init(cx: init::Context)  -> init::LateResources {
        let peripherals = cx.device;
        let mut core = cx.core;
        let mut rcc = peripherals.RCC.constrain();  
        let mut flash = peripherals.FLASH.constrain();

        let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);
        let adc1 = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);

        let mut gpioa: gpioa::Parts = peripherals.GPIOA.split(&mut rcc.apb2);
        let mut gpiob: gpiob::Parts = peripherals.GPIOB.split(&mut rcc.apb2);
        let mut gpioc: gpioc::Parts = peripherals.GPIOC.split(&mut rcc.apb2);

        // Enable the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        // Current control
        //ADC
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
        //PWM
        let pwm_pin = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);
        let timer2 = Timer::tim2(peripherals.TIM2, &clocks, &mut rcc.apb1);
        let mut pwm_timer2 = timer2.pwm::<Tim2NoRemap, _, _, _>(
            pwm_pin, 
            &mut afio.mapr,
            1.khz());
        pwm_timer2.enable();

        // Control
        let mut current_control = current_control::CurrentControl::new(
            adc1, 
            ch0, 
            4.2,
            pwm_timer2);
        current_control.set_current(0.5);

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
        cx.schedule.update_display(cx.start + DISPLAY_REFRESH_PARIOD.cycles()).unwrap();
        
        init::LateResources {
            current_control,
            position_control,
            onboard_led,
            display,
        }
    }

    #[task(schedule = [blink_led], resources = [onboard_led])]
    fn blink_led(cx: blink_led::Context) {
        cx.resources.onboard_led.toggle().unwrap();

        cx.schedule.blink_led(cx.scheduled + BLINKING_LED_PERIOD.cycles()).unwrap();
    }

    #[task(schedule = [update_display], resources = [display])]
    fn update_display(cx: update_display::Context) {
        cx.resources.display.update(cx.scheduled.elapsed().as_cycles());

        cx.schedule.update_display(cx.scheduled + DISPLAY_REFRESH_PARIOD.cycles()).unwrap();
    }

    #[idle(resources = [current_control, position_control])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.resources.current_control.update();
            cx.resources.position_control.update();
        }
    }

    extern "C" {
        fn EXTI0();
    }
};
