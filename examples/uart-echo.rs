#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;

use ch58x_hal as hal;
use hal::peripherals;
use hal::rtc::Rtc;
use hal::uart::{UartTx, Uart};
use qingke_rt::highcode;

static G_SERIAL: Mutex<RefCell<Option<UartTx<peripherals::UART1>>>> = Mutex::new(RefCell::new(None));

macro_rules! println {
    ($($arg:tt)*) => {
        critical_section::with(|cs| {
            use core::fmt::Write;
            use core::writeln;

            if let Some(uart) = G_SERIAL.borrow_ref_mut(cs).as_mut() {
                writeln!(uart, $($arg)*).unwrap();
            }
        });
    }
}

macro_rules! print {
    ($($arg:tt)*) => {
        critical_section::with(|cs| {
            use core::fmt::Write;
            use core::write;

            if let Some(uart) = G_SERIAL.borrow_ref_mut(cs).as_mut() {
                write!(uart, $($arg)*).unwrap();
            }
        });
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
    let (tx, mut rx) = uart.split();

    critical_section::with(|cs| {
        G_SERIAL.replace(cs, Some(tx));
    });

    let rtc = Rtc::new(p.RTC);

    println!("\r\n\r\nHello World!\r");
    println!("System Clocks: {}\r", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}\r", hal::signature::get_chip_id());
    println!("RTC datetime: {}\r", rtc.now());
    println!("Now, type something to echo:\r");

    loop {
        let b = nb::block!(rx.nb_read()).unwrap();
        print!("{}", b as char);
    }
}
