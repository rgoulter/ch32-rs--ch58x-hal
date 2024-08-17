#![no_std]
#![no_main]

use core::fmt::Write;
use core::writeln;

use ch58x_hal as hal;
use hal::peripherals;
use hal::rtc::Rtc;
use hal::uart::Uart;

use qingke_rt::highcode;

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

    let mut uart = Uart::new(p.UART1, p.PA9, p.PA8, Default::default()).unwrap();

    let rtc = Rtc::new(p.RTC);

    writeln!(uart, "\r\n\r\nHello World!\r").unwrap();
    writeln!(uart, "System Clocks: {}\r", hal::sysctl::clocks().hclk).unwrap();
    writeln!(uart, "ChipID: 0x{:02x}\r", hal::signature::get_chip_id()).unwrap();
    writeln!(uart, "RTC datetime: {}\r", rtc.now()).unwrap();
    writeln!(uart, "Now, type something to echo:\r").unwrap();

    loop {
        let b = nb::block!(uart.nb_read()).unwrap();
        write!(uart, "{}", b as char).unwrap();
    }
}
