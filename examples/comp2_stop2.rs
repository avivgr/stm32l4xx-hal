#![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use panic_halt as _;
use stm32l4xx_hal as hal;

use hal::comp::{CompExt, Config, Edge, EnabledComp2, Hysteresis, Polarity, PowerMode};
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::rcc::RccExt;
use hal::stm32;

static EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));
static WOKE: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clock_config = rcc.cfgr.sysclk(80.MHz());
    clock_config.freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let det_in = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let det_th = gpioa.pa5.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let comparators = dp.COMP.split(&mut rcc.apb2);
    let mut comp2 = comparators.comp2.enable_pa3_pa5(
        det_in,
        det_th,
        Config::default()
            .power_mode(PowerMode::UltraLowPower)
            .hysteresis(Hysteresis::Medium)
            .polarity(Polarity::Inverted),
    );

    let mut exti = dp.EXTI;
    comp2.listen(Edge::Rising, &mut exti, &mut cp.NVIC);
    free(|cs| EXTI.borrow(cs).replace(Some(exti)));

    loop {
        WOKE.store(false, Ordering::Release);
        pwr.stop2(&mut cp.SCB);

        // STOP2 does not restore the PLL/high-speed clock tree. Reapply the
        // retained configuration before touching timing-sensitive peripherals.
        clock_config.freeze(&mut flash.acr, &mut pwr);

        // The ISR executes before stop2() returns. In a real application this
        // flag starts ADC qualification of the wake candidate.
        if WOKE.swap(false, Ordering::AcqRel) {
            cortex_m::asm::nop();
        }
    }
}

#[interrupt]
fn COMP() {
    free(|cs| {
        if let Some(exti) = EXTI.borrow(cs).borrow_mut().as_mut() {
            if EnabledComp2::is_pending(exti) {
                EnabledComp2::clear_pending(exti);
                WOKE.store(true, Ordering::Release);
            }
        }
    });
}
