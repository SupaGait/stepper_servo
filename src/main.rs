#![no_std]
#![no_main]

use cortex_m_rt::entry;
//use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use panic_halt as _;
use stm32f1xx_hal as hal;
use hal::{
    prelude::*, 
    adc, pac, gpio, 
    timer::{Tim2NoRemap, Timer},
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},};

mod current_control;
mod position_control;
mod display;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC.constrain();  
    let mut flash = peripherals.FLASH.constrain();

    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);
    let adc1 = adc::Adc::adc1(peripherals.ADC1, &mut rcc.apb2, clocks);

    let mut gpioa: gpio::gpioa::Parts = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob: gpio::gpiob::Parts = peripherals.GPIOB.split(&mut rcc.apb2);
    let mut gpioc: gpio::gpioc::Parts = peripherals.GPIOC.split(&mut rcc.apb2);

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
    let mut position_control = position_control::PositionControl
        ::new(pin_a, pin_b);

    // Led
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, clocks);
    let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

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
    let mut display = display::Display::new(i2c);

    // Update
    loop {
        current_control.update();
        position_control.update();
        display.update();
        
        delay.delay_ms(1000_u16);
        onboard_led.toggle().ok();
    }
}
