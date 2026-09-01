#![no_main]
#![no_std]

use panic_rtt_target as _;

use cortex_m_rt::entry;
use rtt_target::rprintln;
use stm32l4xx_hal::{
    adc::{Adc, AdcCommon, DmaMode, ExternalTrigger, SampleTime, Sequence, TriggerEdge},
    dma::Transfer,
    pac,
    prelude::*,
    timer::{MasterMode, Timer},
};

const SAMPLE_RATE: u32 = 8_000;
const SAMPLE_COUNT: usize = 256;

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);
    let mut delay = stm32l4xx_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut microphone = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let adc_common = AdcCommon::new(dp.ADC_COMMON, &mut rcc.ahb2);
    let mut adc = Adc::adc1(dp.ADC1, adc_common, &mut rcc.ccipr, &mut delay);
    adc.configure_sequence(&mut microphone, Sequence::One, SampleTime::Cycles47_5);
    adc.configure_external_trigger(ExternalTrigger::Tim6Trgo, TriggerEdge::Rising);

    let mut timer = Timer::tim6(dp.TIM6, SAMPLE_RATE.Hz(), clocks, &mut rcc.apb1r1);
    timer.set_master_mode(MasterMode::Update);

    let dma_channels = dp.DMA1.split(&mut rcc.ahb1);
    let samples = {
        static mut SAMPLES: [u16; SAMPLE_COUNT] = [0; SAMPLE_COUNT];
        // SAFETY: `main` takes this reference once and DMA owns it thereafter.
        #[allow(static_mut_refs)]
        unsafe {
            &mut SAMPLES
        }
    };

    let transfer = Transfer::from_adc(adc, dma_channels.1, samples, DmaMode::Oneshot, false);
    let (samples, rx_dma) = transfer.wait();
    let (mut adc, _) = rx_dma.split();
    adc.stop_conversion();

    let mut min = u16::MAX;
    let mut max = u16::MIN;
    let mut sum = 0_u32;
    for &sample in samples.iter() {
        min = min.min(sample);
        max = max.max(sample);
        sum += u32::from(sample);
    }

    rprintln!(
        "captured {} samples at {} Hz: min={}, max={}, mean={}",
        SAMPLE_COUNT,
        SAMPLE_RATE,
        min,
        max,
        sum / SAMPLE_COUNT as u32
    );
    rprintln!("first 16 samples: {:?}", &samples[..16]);

    loop {
        cortex_m::asm::wfi();
    }
}
