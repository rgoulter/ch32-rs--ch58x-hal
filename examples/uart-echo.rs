#![no_std]
#![no_main]

use ch58x_hal as hal;
use hal::gpio::{Level, Output, OutputDrive};
use hal::peripherals;
use hal::rtc::Rtc;
use hal::uart::Uart;
use qingke_rt::highcode;

static mut SERIAL: Option<Uart<peripherals::UART1>> = None;

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

macro_rules! print {
    ($($arg:tt)*) => {
        unsafe {
            use core::fmt::Write;
            use core::write;

            if let Some(uart) = SERIAL.as_mut() {
                write!(uart, $($arg)*).unwrap();
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    let pa8 = unsafe { peripherals::PA8::steal() };
    let pa9 = unsafe { peripherals::PA9::steal() };
    let uart1 = unsafe { peripherals::UART1::steal() };
    let mut serial = Uart::new(uart1, pa9, pa8, Default::default()).unwrap();

    let _ = writeln!(&mut serial, "\r\n\r\n\r\n{}", info);

    loop {}
}

#[qingke_rt::entry]
#[highcode]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();
    let p = hal::init(config);

    let uart = Uart::new(p.UART1, p.PA9, p.PA8, Default::default()).unwrap();
    unsafe {
        SERIAL.replace(uart);
    }

    let rtc = Rtc::new(p.RTC);

    println!("\r\n\r\nHello World!\r");
    println!("System Clocks: {}\r", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}\r", hal::signature::get_chip_id());
    println!("RTC datetime: {}\r", rtc.now());
    println!("Now, type something to echo:\r");

    loop {
        unsafe {
            if let Some(uart) = SERIAL.as_mut() {
                let b = nb::block!(uart.nb_read()).unwrap();
                print!("{}", b as char);
            }
        }
    }
}
