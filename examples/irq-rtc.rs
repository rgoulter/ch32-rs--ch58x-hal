#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_1::delay::DelayNs;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use hal::isp::EEPROM_BLOCK_SIZE;
use hal::rtc::{DateTime, Rtc};
use hal::sysctl::Config;
use hal::uart::UartTx;
use hal::{pac, peripherals, Peripherals};
use qingke::riscv;
use {ch58x_hal as hal};

static mut SERIAL: Option<UartTx<peripherals::UART1>> = None;
macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            use core::fmt::Write;
            use core::writeln;

            if let Some(uart) = SERIAL.as_mut() {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

#[qingke_rt::interrupt]
#[qingke_rt::highcode]
fn RTC() {
    println!("RTC\r");
    let mut rtc = Rtc;

    rtc.ack_timing();

    let now = rtc.now();
    println!("RTC: Current time: {} weekday={}\r", now, now.isoweekday());
    //  writeln!(uart, "mepc: {:08x}", riscv::register::mepc::read()).unwrap();
}

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    let mut pa8 = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe { SERIAL.replace(uart) };

    println!("\r\nMCU init ok!\r");

    let mut rtc = Rtc;
    rtc.enable_timing(hal::rtc::TimingMode::_2S);
    unsafe {
        qingke::pfic::enable_interrupt(pac::Interrupt::RTC as u8);
    }

    loop {
        unsafe {
            pa8.toggle();

            hal::delay_ms(1000);
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    let pa9 = unsafe { peripherals::PA9::steal() };
    let uart1 = unsafe { peripherals::UART1::steal() };
    let mut serial = UartTx::new(uart1, pa9, Default::default()).unwrap();

    let _ = writeln!(&mut serial, "\r\n\n\n{}", info);

    loop {}
}
