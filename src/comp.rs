//! Analog comparators.
//!
//! This module currently supports the L4x2 comparator register layout.  It
//! owns the internal EXTI lines used by COMP1 and COMP2, rather than treating
//! them as GPIO EXTI pins.

use cortex_m::peripheral::NVIC;

use crate::gpio::{Analog, PA0, PA1, PA3, PA5};
use crate::pac::{Interrupt, COMP, EXTI, SYSCFG};
use crate::rcc::{Enable, APB2};

/// Comparator speed/current selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    HighSpeed = 0b00,
    MediumSpeed = 0b01,
    LowPower = 0b10,
    UltraLowPower = 0b11,
}

/// Internal comparator hysteresis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Hysteresis {
    None = 0b00,
    Low = 0b01,
    Medium = 0b10,
    High = 0b11,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polarity {
    Normal,
    Inverted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub power_mode: PowerMode,
    pub hysteresis: Hysteresis,
    pub polarity: Polarity,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            power_mode: PowerMode::MediumSpeed,
            hysteresis: Hysteresis::None,
            polarity: Polarity::Normal,
        }
    }
}

impl Config {
    pub fn power_mode(mut self, power_mode: PowerMode) -> Self {
        self.power_mode = power_mode;
        self
    }

    pub fn hysteresis(mut self, hysteresis: Hysteresis) -> Self {
        self.hysteresis = hysteresis;
        self
    }

    pub fn polarity(mut self, polarity: Polarity) -> Self {
        self.polarity = polarity;
        self
    }
}

/// Extension trait for the device comparator register block.
pub trait CompExt {
    fn split(self, apb2: &mut APB2) -> Parts;
}

impl CompExt for COMP {
    fn split(self, apb2: &mut APB2) -> Parts {
        // COMP shares the SYSCFG peripheral clock gate on STM32L4x2.
        SYSCFG::enable(apb2);
        let _ = self;
        Parts {
            comp1: Comp1 { _private: () },
            comp2: Comp2 { _private: () },
        }
    }
}

pub struct Parts {
    pub comp1: Comp1,
    pub comp2: Comp2,
}

pub struct Comp1 {
    _private: (),
}

pub struct Comp2 {
    _private: (),
}

/// Enabled COMP1 with its PA1 non-inverting and PA0 inverting input routes.
pub struct EnabledComp1 {
    comp: Comp1,
    _positive: PA1<Analog>,
    _negative: PA0<Analog>,
}

/// Enabled COMP2 with the PA3 non-inverting and PA5 inverting routes.
pub struct EnabledComp2 {
    comp: Comp2,
    _positive: PA3<Analog>,
    _negative: PA5<Analog>,
}

impl Comp1 {
    /// Configure the STM32L4x2 PA1/PA0 comparator route.
    pub fn enable_pa1_pa0(
        self,
        positive: PA1<Analog>,
        negative: PA0<Analog>,
        config: Config,
    ) -> EnabledComp1 {
        let regs = unsafe { &*COMP::ptr() };
        regs.comp1_csr.modify(|_, w| unsafe {
            w.comp1_pwrmode()
                .bits(config.power_mode as u8)
                .comp1_inpsel()
                .bits(0b01)
                .comp1_inmsel()
                .bits(0b110)
                .comp1_polarity()
                .bit(config.polarity == Polarity::Inverted)
                .comp1_hyst()
                .bits(config.hysteresis as u8)
                .comp1_en()
                .set_bit()
        });
        EnabledComp1 {
            comp: self,
            _positive: positive,
            _negative: negative,
        }
    }
}

impl Comp2 {
    /// Configure the STM32L4x2 PA3/PA5 comparator route.
    ///
    /// PA5 requires both INMSEL=extended GPIO and INMESEL=PA5. Omitting the
    /// latter silently selects PB7.
    pub fn enable_pa3_pa5(
        self,
        positive: PA3<Analog>,
        negative: PA5<Analog>,
        config: Config,
    ) -> EnabledComp2 {
        let regs = unsafe { &*COMP::ptr() };
        regs.comp2_csr.modify(|_, w| unsafe {
            w.comp2_pwrmode()
                .bits(config.power_mode as u8)
                .comp2_inpsel()
                .bits(0b10)
                .comp2_inmsel()
                .bits(0b111)
                .comp2_inmesel()
                .bits(0b11)
                .comp2_polarity()
                .bit(config.polarity == Polarity::Inverted)
                .comp2_hyst()
                .bits(config.hysteresis as u8)
                .comp2_en()
                .set_bit()
        });
        EnabledComp2 {
            comp: self,
            _positive: positive,
            _negative: negative,
        }
    }
}

macro_rules! enabled_comp {
    ($Type:ident, $csr:ident, $value:ident, $mr:ident, $tr:ident, $pr:ident) => {
        impl $Type {
            pub fn output(&self) -> bool {
                let regs = unsafe { &*COMP::ptr() };
                regs.$csr.read().$value().bit_is_set()
            }

            pub fn listen(&mut self, edge: Edge, exti: &mut EXTI, _nvic: &mut NVIC) {
                match edge {
                    Edge::Rising => {
                        exti.rtsr1.modify(|_, w| w.$tr().set_bit());
                        exti.ftsr1.modify(|_, w| w.$tr().clear_bit());
                    }
                    Edge::Falling => {
                        exti.rtsr1.modify(|_, w| w.$tr().clear_bit());
                        exti.ftsr1.modify(|_, w| w.$tr().set_bit());
                    }
                    Edge::RisingFalling => {
                        exti.rtsr1.modify(|_, w| w.$tr().set_bit());
                        exti.ftsr1.modify(|_, w| w.$tr().set_bit());
                    }
                }
                Self::clear_pending(exti);
                NVIC::unpend(Interrupt::COMP);
                exti.imr1.modify(|_, w| w.$mr().set_bit());
                // Safe at this API boundary: the caller supplies exclusive
                // NVIC access and has explicitly requested interrupt delivery.
                unsafe { NVIC::unmask(Interrupt::COMP) };
            }

            pub fn unlisten(&mut self, exti: &mut EXTI) {
                exti.imr1.modify(|_, w| w.$mr().clear_bit());
            }

            pub fn is_pending(exti: &EXTI) -> bool {
                exti.pr1.read().$pr().bit_is_set()
            }

            pub fn clear_pending(exti: &mut EXTI) {
                exti.pr1.write(|w| w.$pr().set_bit());
            }
        }
    };
}

enabled_comp!(EnabledComp1, comp1_csr, comp1_value, mr21, tr21, pr21);
enabled_comp!(EnabledComp2, comp2_csr, comp2_value, mr22, tr22, pr22);

impl EnabledComp1 {
    pub fn disable(self) -> (Comp1, PA1<Analog>, PA0<Analog>) {
        let regs = unsafe { &*COMP::ptr() };
        regs.comp1_csr.modify(|_, w| w.comp1_en().clear_bit());
        (self.comp, self._positive, self._negative)
    }
}

impl EnabledComp2 {
    pub fn disable(self) -> (Comp2, PA3<Analog>, PA5<Analog>) {
        let regs = unsafe { &*COMP::ptr() };
        regs.comp2_csr.modify(|_, w| w.comp2_en().clear_bit());
        (self.comp, self._positive, self._negative)
    }
}
