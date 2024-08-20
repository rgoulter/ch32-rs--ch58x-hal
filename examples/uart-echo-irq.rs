#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;

use ch58x_hal as hal;
use hal::peripherals;
use hal::uart::{UartRx, UartTx, Uart};
use qingke_rt::highcode;

static G_SERIAL: Mutex<RefCell<Option<UartTx<peripherals::UART1>>>> = Mutex::new(RefCell::new(None));
static G_RX: Mutex<RefCell<Option<UartRx<peripherals::UART1>>>> = Mutex::new(RefCell::new(None));

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

    // TODO: this code uses PAC; wrapping the following into HAL would be more idiomatic.
    let uart1 = p.UART1;

    // 7-byte trigger
    unsafe {
        uart1.fcr().modify(|_, w| w.fifo_trig().bits(3));
    }
    // Enable UART interrupt flags
    uart1.ier().modify(|_, w| w.recv_rdy().bit(true).line_stat().bit(true));
    uart1.mcr().modify(|_, w| w.out2__rb_mcr_int_oe().bit(true));

    let uart = Uart::new(uart1, p.PA9, p.PA8, Default::default()).unwrap();

    let (tx, rx) = uart.split();

    critical_section::with(|cs| {
        G_SERIAL.replace(cs, Some(tx));
        G_RX.replace(cs, Some(rx));
    });

    println!("\r\nThis is a serial example, using interrupts:\r");

    unsafe {
        // Enable UART1 interrupt
        qingke::pfic::enable_interrupt(hal::pac::Interrupt::UART1 as u8);
    }

    loop {}
}

#[qingke_rt::interrupt]
#[highcode]
fn UART1() {
    // TODO: this code uses PAC; wrapping the following into HAL would be more idiomatic.
    let p: hal::peripherals::UART1 = unsafe {
        hal::peripherals::UART1::steal()
    };
    let iir = p.iir().read();
    let iir_int = iir.bits() & iir.int_mask().bits();
    match iir_int {
        0x06 => {
            // Line Status error
            p.lsr().read().bits();
        }

        0x04 => {
            // Recv ready
            critical_section::with(|cs| {

                if let Some(tx) = G_SERIAL.borrow_ref_mut(cs).as_mut() {
                    if let Some(rx) = G_RX.borrow_ref_mut(cs).as_mut() {
                        let buf = &mut [0u8; 7];
                        for i in 0..7 {
                            buf[i] = rx.nb_read().unwrap();
                        }
                        tx.blocking_write(buf).unwrap();
                    }
                }
            });
        }

        0x0C => {
            critical_section::with(|cs| {
                if let Some(tx) = G_SERIAL.borrow_ref_mut(cs).as_mut() {
                    if let Some(rx) = G_RX.borrow_ref_mut(cs).as_mut() {
                        let buf = &mut [0u8; 7];
                        let mut i = 0;
                        while let Ok(b) = rx.nb_read() {
                            buf[i] = b;
                            i += 1;
                        }
                        tx.blocking_write(buf).unwrap();
                    }
                }
            });
        }

        _ => {}
    }
}
