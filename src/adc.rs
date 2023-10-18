use core::ptr;

use crate::gpio::AnyPin;
use crate::peripherals::{self, ADC_TEMP_SENSOR, ADC_VBAT_SENSOR};
use crate::{into_ref, pac, Peripheral, PeripheralRef};

const ROM_CFG_TMP_25C: *const u32 = 0x7F014 as *const u32;

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub enum ClkSel {
    _3_2MHz = 0b00, // CK32M div 10
    _8MHz = 0b01,   // CK32M div 4
    #[default]
    _5_33MHz = 0b10, // CK32M div 6
    _4MHz = 0b11,   // CK32M div 8
}

#[non_exhaustive]
#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub struct Config {
    pub clksel: ClkSel,
}

pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: crate::PeripheralRef<'d, T>,
}

pub(crate) mod sealed {

    pub trait Instance {
        type Interrupt: crate::interrupt::Interrupt;

        fn regs() -> &'static crate::pac::adc::RegisterBlock;
    }

    pub trait AdcPin<T: Instance> {
        fn set_as_analog(&mut self) {}

        fn channel(&self) -> u8;
    }

    pub trait InternalChannel<T> {
        fn channel(&self) -> u8;
    }
}

pub trait Instance: sealed::Instance + crate::Peripheral<P = Self> {}
pub trait AdcPin<T: Instance>: sealed::AdcPin<T> {}
pub trait InternalChannel<T>: sealed::InternalChannel<T> {}

impl sealed::Instance for peripherals::ADC {
    type Interrupt = crate::interrupt::ADC;

    fn regs() -> &'static crate::pac::adc::RegisterBlock {
        unsafe { &*pac::ADC::PTR }
    }
}
impl Instance for peripherals::ADC {}

macro_rules! impl_adc_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::adc::AdcPin<peripherals::$inst> for crate::peripherals::$pin {}

        impl crate::adc::sealed::AdcPin<peripherals::$inst> for crate::peripherals::$pin {
            fn set_as_analog(&mut self) {
                <Self as crate::gpio::sealed::Pin>::set_as_analog(self);
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

impl_adc_pin!(ADC, PA7, 11);
impl_adc_pin!(ADC, PA8, 12);
impl_adc_pin!(ADC, PA9, 13);
impl_adc_pin!(ADC, PA4, 0);
impl_adc_pin!(ADC, PA5, 1);
impl_adc_pin!(ADC, PA6, 2);
impl_adc_pin!(ADC, PA0, 9);
impl_adc_pin!(ADC, PA1, 8);
impl_adc_pin!(ADC, PA2, 7);
impl_adc_pin!(ADC, PA3, 6);
impl_adc_pin!(ADC, PA15, 5);
impl_adc_pin!(ADC, PA14, 4);
impl_adc_pin!(ADC, PA13, 3);
impl_adc_pin!(ADC, PA12, 2);

pub struct Temperature;
impl AdcPin<peripherals::ADC> for Temperature {}
impl sealed::AdcPin<peripherals::ADC> for Temperature {
    fn channel(&self) -> u8 {
        15
    }
}

impl Temperature {
    /// Time needed for temperature sensor readings to stabilize
    pub fn start_time_us() -> u32 {
        10
    }
}

pub struct Vbat;
impl AdcPin<peripherals::ADC> for Vbat {}
impl sealed::AdcPin<peripherals::ADC> for Vbat {
    fn channel(&self) -> u8 {
        15
    }
}

impl<'d, T> Adc<'d, T>
where
    T: Instance,
{
    pub fn new(adc: impl Peripheral<P = T> + 'd, config: Config) -> Self {
        into_ref!(adc);

        let rb = T::regs();
        rb.cfg.modify(|_, w| {
            w.power_on()
                .set_bit()
                .diff_en()
                .set_bit() // must for temp
                .clk_div()
                .variant(config.clksel as u8)
                .buf_en()
                .clear_bit()
                .pga_gain()
                .variant(0b11)
        });

        Self { adc }
    }

    /// Enable Temperature sensor
    pub fn enable_temperature(&self) -> Temperature {
        let rb = T::regs();

        rb.tem_sensor.modify(|_, w| w.tem_sen_pwr_on().set_bit());

        Temperature {}
    }

    pub fn enable_vbat(&self) -> Vbat {
        Vbat {}
    }

    /// Perform a single conversion.
    fn convert(&mut self) -> u16 {
        let rb = T::regs();

        // start adc convert
        rb.convert.modify(|_, w| w.start().set_bit());
        // wait for convert
        while rb.convert.read().start().bit_is_set() {}

        rb.data.read().data().bits()
    }

    pub fn read(&mut self, pin: &mut impl AdcPin<T>) -> u16 {
        let rb = T::regs();

        pin.set_as_analog();

        // Configure ADC
        let channel = pin.channel();

        // Select channel
        rb.channel.modify(|_, w| w.ch_inx().variant(channel));

        self.convert()
    }
}

impl<'d, T: Instance> Drop for Adc<'d, T> {
    fn drop(&mut self) {
        let rb = T::regs();

        rb.cfg.modify(|_, w| w.power_on().clear_bit());
    }
}

pub fn adc_to_temperature_celsius(data: u16) -> i32 {
    let c25 = unsafe { ptr::read_volatile(ROM_CFG_TMP_25C) };

    // current temperature = standard temperature + (adc deviation * adc linearity coefficient)
    //  temp = (((C25 >> 16) & 0xFFFF) ? ((C25 >> 16) & 0xFFFF) : 25) + \
    // (adc_val - ((int)(C25 & 0xFFFF))) * 10 / 27;
    let c25_ = ((c25 >> 16) & 0xFFFF) as i32;
    let c25_ = if c25 != 0 { c25_ } else { 25 };
    c25_ + ((data as i32) - ((c25 & 0xFFFF) as i32)) * 10 / 27
}

/// Convert ADC data to temperature in milli celsius.
pub fn adc_to_temperature_milli_celsius(data: u16) -> i32 {
    let c25 = unsafe { ptr::read_volatile(ROM_CFG_TMP_25C) };

    let c25_ = ((c25 >> 16) & 0xFFFF) as i32;
    let c25_ = if c25 != 0 { c25_ } else { 25 };
    c25_ * 1000 + ((data as i32) - ((c25 & 0xFFFF) as i32)) * 10_000 / 27
}
