// Transliteration of the USB/Device/CompoundDev example
//  from the CH583 EVT Examples.
//
// The transliteration aims to be close to the C code,
//  and doesn't prioritise idiomatic Rust.

#![no_std]
#![no_main]

use core::ptr::addr_of;
use core::cell::RefCell;
use critical_section::Mutex;

use ch58x_hal as hal;
use hal::peripherals;
use hal::uart::{UartTx, Uart};
use qingke_rt::highcode;

static G_SERIAL: Mutex<RefCell<Option<UartTx<peripherals::UART1>>>> = Mutex::new(RefCell::new(None));

const DevEP0SIZE: u8 = 0x40;

const USB_INTERFACE_MAX_NUM: u8 = 2;
const USB_INTERFACE_MAX_INDEX: u8 = 1;

// Device Descriptor
const MyDevDescr: [u8; 18] = [0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, DevEP0SIZE, 0x3d, 0x41, 0x07, 0x21, 0x00, 0x00, 0x01, 0x02, 0x00, 0x01];

// Configuration Descriptor
const MyCfgDescr: [u8; 59] = [
    0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32, //Configuration Descriptor
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00, //Interface descriptor, keyboard
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3e, 0x00, //HID class descriptor
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a,             //Endpoint descriptor
    0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00, //Interface descriptor, mouse
    0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00, //HID class descriptor
    0x07, 0x05, 0x82, 0x03, 0x04, 0x00, 0x0a              //Endpoint descriptor
];

/* USB speed match descriptor */
const My_QueDescr: [u8; 10] = [0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00];

/* USB full speed mode, other speed configuration descriptor */
static mut USB_FS_OSC_DESC: [u8; MyCfgDescr.len()] = [
    0x09, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
];

const MyLangDescr: [u8; 4] = [0x04, 0x03, 0x09, 0x04];
const MyManuInfo: [u8; 14] = [0x0E, 0x03, b'w', 0, b'c', 0, b'h', 0, b'.', 0, b'c', 0, b'n', 0];
const MyProdInfo: [u8; 12] = [0x0C, 0x03, b'C', 0, b'H', 0, b'5', 0, b'8', 0, b'x', 0];

/*HID class report descriptor*/
const KeyRepDesc: [u8; 62] = [
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25,
    0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x03,
    0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x03, 0x91, 0x02, 0x95, 0x05, 0x75, 0x01, 0x91,
    0x01, 0x95, 0x06, 0x75, 0x08, 0x26, 0xff, 0x00, 0x05, 0x07, 0x19, 0x00, 0x29, 0x91, 0x81,
    0x00, 0xC0
];

const MouseRepDesc: [u8; 52] = [
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29,
    0x03, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03, 0x81, 0x02, 0x75, 0x05, 0x95, 0x01,
    0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7f, 0x75,
    0x08, 0x95, 0x03, 0x81, 0x06, 0xC0, 0xC0
];


/**********************************************************/
static mut DevConfig: u8 = 0;
static mut Ready: u8 = 0;
static mut SetupReqCode: u8 = 0;
static mut SetupReqLen: u16 = 0;
static mut pDescr: &[u8] = &[];
static mut Report_Value: [u8; USB_INTERFACE_MAX_INDEX as usize + 1] = [0x00; USB_INTERFACE_MAX_INDEX as usize + 1];
static mut Idle_Value: [u8; USB_INTERFACE_MAX_INDEX as usize + 1] = [0x00; USB_INTERFACE_MAX_INDEX as usize + 1];
static mut USB_SleepStatus: u8 = 0x00;

/*Mouse and keyboard data*/
static mut HIDMouse: [u8; 4] = [0x0; 4];
static mut HIDKey: [u8; 8] = [0x0; 8];

/******** User-defined allocation of endpoint RAM ****************************************/
#[repr(C, align(4))]
struct EPBuffer<const N: usize>([u8; N]);

static mut EP0_Databuf: EPBuffer<{64 + 64 + 64}> = EPBuffer([0; 64 + 64 + 64]); //ep0_out(64)+ep0_in(64)+ep0_setup(64)
static mut EP1_Databuf: EPBuffer<{64 + 64}> = EPBuffer([0; 64 + 64]); //ep1_out(64)+ep1_in(64)
static mut EP2_Databuf: EPBuffer<{64 + 64}> = EPBuffer([0; 64 + 64]); //ep2_out(64)+ep2_in(64)
static EP3_Databuf: EPBuffer<{64 + 64}> = EPBuffer([0; 64 + 64]); //ep3_out(64)+ep3_in(64)

#[repr(packed, C)]
struct USB_SETUP_REQ {
    bRequestType: u8,
    bRequest: u8,
    wValue: u16,
    wIndex: u16,
    wLength: u16,
}

const RB_UEP_AUTO_TOG: u8 = 0x10;      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle

// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
const UEP_R_RES_ACK: u8 = 0x00;
const UEP_R_RES_TOUT: u8 = 0x04;
const UEP_R_RES_NAK: u8 = 0x08;
const UEP_R_RES_STALL: u8 = 0x0C;


const RB_UEP_R_TOG: u8 =       0x80;     // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
const RB_UEP_T_TOG: u8 =       0x40;     // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1

const RB_UEP_T_RES1: u8 =      0x02;     // handshake response type high bit for USB endpoint X transmittal (IN)
const RB_UEP_T_RES0: u8 =      0x01;     // handshake response type low bit for USB endpoint X transmittal (IN)
const MASK_UEP_T_RES: u8 =     0x03;     // bit mask of handshake response type for USB endpoint X transmittal (IN)

const MASK_UEP_R_RES: u8 =     0x0C;     // bit mask of handshake response type for USB endpoint X receiving (OUT)

// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
const UEP_T_RES_ACK: u8 = 0x00;
const UEP_T_RES_TOUT: u8 = 0x01;
const UEP_T_RES_NAK: u8 = 0x02;
const UEP_T_RES_STALL: u8 = 0x03;

const RB_UD_PD_DIS: u8 = 0x80;

const RB_UIS_SETUP_ACT: u8 = 0x80;   // RO, indicate SETUP token & 8 bytes setup request received for USB device mode
const RB_UIS_TOG_OK: u8 = 0x40;      // RO, indicate current USB transfer toggle is OK
const RB_UIS_TOKEN1: u8 = 0x20;      // RO, current token PID code bit 1 received for USB device mode
const RB_UIS_TOKEN0: u8 = 0x10;      // RO, current token PID code bit 0 received for USB device mode
const MASK_UIS_TOKEN: u8 = 0x30;     // RO, bit mask of current token PID code received for USB device mode
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode, keep last status during SETUP token, clear RB_UIF_TRANSFER ( RB_UIF_TRANSFER from 1 to 0 ) to set free
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: free
const UIS_TOKEN_OUT: u8 = 0x00;
const UIS_TOKEN_IN: u8 = 0x20;
const UIS_TOKEN_SETUP: u8 = 0x30;

const MASK_UIS_ENDP: u8 = 0x0F;      // RO, bit mask of current transfer endpoint number for USB device mode


// USB standard device request code/
const USB_GET_STATUS: u8 =        0x00;
const USB_CLEAR_FEATURE: u8 =     0x01;
const USB_SET_FEATURE: u8 =       0x03;
const USB_SET_ADDRESS: u8 =       0x05;
const USB_GET_DESCRIPTOR: u8 =    0x06;
const USB_SET_DESCRIPTOR: u8 =    0x07;
const USB_GET_CONFIGURATION: u8 = 0x08;
const USB_SET_CONFIGURATION: u8 = 0x09;
const USB_GET_INTERFACE: u8 =     0x0A;
const USB_SET_INTERFACE: u8 =     0x0B;
const USB_SYNCH_FRAME: u8 =       0x0C;

/* Bit define for USB request type */
const USB_REQ_TYP_IN: u8        = 0x80;           /* control IN, device to host */
const USB_REQ_TYP_OUT: u8       = 0x00;           /* control OUT, host to device */
const USB_REQ_TYP_READ: u8      = 0x80;           /* control read, device to host */
const USB_REQ_TYP_WRITE: u8     = 0x00;           /* control write, host to device */
const USB_REQ_TYP_MASK: u8      = 0x60;           /* bit mask of request type */
const USB_REQ_TYP_STANDARD: u8  = 0x00;
const USB_REQ_TYP_CLASS: u8     = 0x20;
const USB_REQ_TYP_VENDOR: u8    = 0x40;
const USB_REQ_TYP_RESERVED: u8  = 0x60;
const USB_REQ_RECIP_MASK: u8    = 0x1F;           /* bit mask of request recipient */
const USB_REQ_RECIP_DEVICE: u8  = 0x00;
const USB_REQ_RECIP_INTERF: u8  = 0x01;
const USB_REQ_RECIP_ENDP: u8    = 0x02;
const USB_REQ_RECIP_OTHER: u8   = 0x03;

/* USB descriptor type */
const USB_DESCR_TYP_DEVICE: u8 =    0x01;
const USB_DESCR_TYP_CONFIG: u8 =    0x02;
const USB_DESCR_TYP_STRING: u8 =    0x03;
const USB_DESCR_TYP_INTERF: u8 =    0x04;
const USB_DESCR_TYP_ENDP: u8 =      0x05;
const USB_DESCR_TYP_QUALIF: u8 =    0x06;
const USB_DESCR_TYP_SPEED: u8 =     0x07;
const USB_DESCR_TYP_OTG: u8 =       0x09;
const USB_DESCR_TYP_HID: u8 =       0x21;
const USB_DESCR_TYP_REPORT: u8 =    0x22;
const USB_DESCR_TYP_PHYSIC: u8 =    0x23;
const USB_DESCR_TYP_CS_INTF: u8 =   0x24;
const USB_DESCR_TYP_CS_ENDP: u8 =   0x25;
const USB_DESCR_TYP_HUB: u8 =       0x29;

/* HID Class Request */
const DEF_USB_GET_IDLE: u8 =           0x02;                                        /* get idle for key or mouse */
const DEF_USB_GET_PROTOCOL: u8 =       0x03;                                        /* get protocol for bios type */
const DEF_USB_SET_REPORT: u8 =         0x09;                                        /* set report for key */
const DEF_USB_SET_IDLE: u8 =           0x0A;                                        /* set idle for key or mouse */
const DEF_USB_SET_PROTOCOL: u8 =       0x0B;                                        /* set protocol for bios type */

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

/*********************************************************************
 * @fn      USB_DeviceInit
 *
 * @brief   
 *
 * @param   none
 *
 * @return  none
 */
fn USB_DeviceInit() {
    unsafe {
        let usb = peripherals::USB::steal();

        // Set mode, cancel RB_UC_CLR_ALL
        usb.ctrl().write_with_zero(|w| w.bits(0x00));

        // Endpoint 4 OUT+IN, Endpoint 1 OUT+IN
        usb.uep4_1_mod().write_with_zero(|w| {
            w.uep4_rx_en().set_bit();
            w.uep4_tx_en().set_bit();
            w.uep1_rx_en().set_bit();
            w.uep1_tx_en().set_bit()
        });
        // Endpoint 2 OUT+IN, Endpoint 3 OUT+IN
        usb.uep2_3_mod__r8_uh_ep_mod().write_with_zero(|w| {
            w.uep2_rx_en__rb_uh_ep_rx_en().set_bit();
            w.uep2_tx_en().set_bit();
            w.uep3_rx_en().set_bit();
            w.uep3_tx_en__rb_uh_ep_tx_en().set_bit()
        });

        usb.uep0_dma().write_with_zero(|w| w.uep0_dma().bits((addr_of!(EP0_Databuf) as *const _ as u32) as u16));
        usb.uep1_dma().write_with_zero(|w| w.uep1_dma().bits((addr_of!(EP1_Databuf) as *const _ as u32) as u16));
        usb.uep2_dma__r16_uh_rx_dma().write_with_zero(|w| w.uep2_dma().bits((addr_of!(EP2_Databuf) as *const _ as u32) as u16));
        usb.uep3_dma__r16_uh_tx_dma().write_with_zero(|w| w.uep3_dma().bits((addr_of!(EP3_Databuf) as *const _ as u32) as u16));

        usb.uep0_ctrl().write_with_zero(|w| {
            w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
        });
        usb.uep1_ctrl__r8_uh_setup().write_with_zero(|w| {
            w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK);
            w.uep_auto_tog().set_bit()
        });
        usb.uep2_ctrl_r8_uh_rx_ctrl().write_with_zero(|w| {
            w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK);
            w.uep_auto_tog__rb_uh_r_auto_tog().set_bit()
        });
        usb.uep3_ctrl__r8_uh_tx_ctrl().write_with_zero(|w| {
            w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK);
            w.uep_auto_tog_rb_uh_t_auto_tog().set_bit()
        });
        usb.uep4_ctrl().write_with_zero(|w| {
            w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
        });

        usb.dev_ad().write_with_zero(|w| w.bits(0x00));

        // Start the USB device and DMA, and automatically return NAK before the interrupt flag is cleared during the interrupt
        usb.ctrl().write_with_zero(|w| {
            w.uc_dev_pu_en().set_bit();
            w.uc_int_busy().set_bit();
            w.uc_dma_en().set_bit()
        });

        // Prevent USB ports from floating and pull-up resistors
        let sys = hal::pac::SYS::steal();
        sys.pin_analog_ie().modify(|_, w| {
            w.pin_usb_ie().set_bit();
            w.pin_usb_dp_pu().set_bit()
        });

        usb.int_fg().write_with_zero(|w| w.bits(0xFF)); // Clear interrupt flag

        // SVD BUG?: RB_UD_PD_DIS is RW in Datasheet, but RO in SVD
        usb.udev_ctrl__r8_uhost_ctrl().write_with_zero(|w| {
            w.bits(RB_UD_PD_DIS);
            w.ud_port_en__rb_uh_port_en().set_bit()
        }); // Allow USB ports

        usb.int_en().write_with_zero(|w| {
            w.uie_suspend().set_bit();
            w.uie_bus_rst__rb_uie_detect().set_bit();
            w.uie_transfer().set_bit()
        });
    }
}

fn USB_DevTransProcess() {
    unsafe {
        let mut len: u8 = 0;
        let mut chtype: u8;
        let mut intflag: u8;
        let mut errflag: u8 = 0;

        let usb = peripherals::USB::steal();

        let int_fg = usb.int_fg().read();
        intflag = int_fg.bits();

        if int_fg.uif_transfer().bit_is_set() {
            let int_st = usb.int_st().read();

            if (int_st.bits() & int_st.mask_uis_token().bits()) != MASK_UIS_TOKEN { // Non-idle
                let token = int_st.bits() & MASK_UIS_TOKEN;
                let endp = int_st.bits() & MASK_UIS_ENDP;
                match (token, endp) {
                    (UIS_TOKEN_IN, 0) => {
                        match SetupReqCode as u8 {
                            USB_GET_DESCRIPTOR => {
                                len = if (SetupReqLen as u8) >= DevEP0SIZE { DevEP0SIZE } else { SetupReqLen as u8 };
                                EP0_Databuf.0[..len as usize].copy_from_slice(&pDescr[..len as usize]);
                                SetupReqLen -= len as u16;
                                pDescr = &pDescr[len as usize..];
                                usb.uep0_t_len().write(|w| w.bits(len as u8));
                                usb.uep0_ctrl().modify(|r, w| {
                                    w.uep_t_tog().bit(true ^ r.uep_t_tog().bit())
                                });
                            }

                            USB_SET_ADDRESS => {
                                usb.dev_ad().modify(|r, w| {
                                    w.bits(SetupReqLen as u8);
                                    w.uda_gp_bit().bit(r.uda_gp_bit().bit() & true)
                                });
                                usb.uep0_ctrl().write(|w| w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK));
                            }

                            USB_SET_FEATURE => {}

                            _ => {
                                // The status stage is completed and the
                                //    interruption is completed or the zero-length data
                                //    packet is forced to be uploaded to end the control
                                //    transmission.
                                usb.uep0_t_len().write(|w| w.bits(0));
                                usb.uep0_ctrl().write(|w| w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK));
                            }
                        }
                    }

                    (UIS_TOKEN_OUT, 0) => {
                        len = usb.rx_len().read().bits();
                        if SetupReqCode == 0x09 {
                            println!("[{}] Num Lock\r", if EP0_Databuf.0[0] & (1 << 0) != 0 { "*" } else { " " });
                            println!("[{}] Caps Lock\r", if EP0_Databuf.0[0] & (1 << 1) != 0 { "*" } else { " " });
                            println!("[{}] Scroll Lock\r\n", if EP0_Databuf.0[0] & (1 << 2) != 0 { "*" } else { " " });
                        }
                    }

                    (UIS_TOKEN_OUT, 1) => {
                        if int_st.uis_tog_ok().bit_is_set() {
                            // Discard out-of-sync packets
                            usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                                w.uep_r_tog__rb_uh_pre_pid_en().bit(r.uep_r_tog__rb_uh_pre_pid_en().bit() ^ true)
                            });
                            len = usb.rx_len().read().bits();
                            // DevEP1_OUT_Deal(len); // TODO
                        }
                    }

                    (UIS_TOKEN_IN, 1) => {
                        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                            w.uep_t_tog__rb_uh_sof_en().bit(r.uep_t_tog__rb_uh_sof_en().bit() ^ true)
                        });
                        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                            w.bits((r.bits() & !MASK_UEP_T_RES) | UEP_T_RES_NAK)
                        });
                    }

                    (UIS_TOKEN_OUT, 2) => {
                        if int_st.uis_tog_ok().bit_is_set() {
                            // Discard out-of-sync packets
                            usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {  
                                w.uep_r_tog__rb_uh_r_tog().bit(r.uep_r_tog__rb_uh_r_tog().bit() ^ true)
                            });
                            len = usb.rx_len().read().bits();
                            // DevEP2_OUT_Deal(len); // TODO
                        }
                    }

                    (UIS_TOKEN_IN, 2) => {
                        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
                            w.uep_t_tog().bit(r.uep_t_tog().bit() ^ true)
                        });
                        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
                            w.bits((r.bits() & !MASK_UEP_T_RES) | UEP_T_RES_NAK)
                        });
                    }

                    (UIS_TOKEN_OUT, 3) => {
                        if int_st.uis_tog_ok().bit_is_set() {
                            // Discard out-of-sync packets
                            usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                                w.uep_r_tog().bit(r.uep_r_tog().bit() ^ true)
                            });
                            len = usb.rx_len().read().bits();
                            // DevEP3_OUT_Deal(len); // TODO
                        }
                    }

                    (UIS_TOKEN_IN, 3) => {
                        usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                            w.uep_t_tog_rb_uh_t_tog().bit(r.uep_t_tog_rb_uh_t_tog().bit() ^ true)
                        });
                        usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                            w.bits((r.bits() & !MASK_UEP_T_RES) | UEP_T_RES_NAK)
                        });
                    }

                    (UIS_TOKEN_OUT, 4) => {
                        if int_st.uis_tog_ok().bit_is_set() {
                            // Discard out-of-sync packets
                            usb.uep4_ctrl().modify(|r, w| {
                                w.uep_r_tog().bit(r.uep_r_tog().bit() ^ true)
                            });
                            len = usb.rx_len().read().bits();
                            // DevEP4_OUT_Deal(len); // TODO
                        }
                    }

                    (UIS_TOKEN_IN, 4) => {
                        usb.uep4_ctrl().modify(|r, w| {
                            w.uep_t_tog().bit(r.uep_t_tog().bit() ^ true)
                        });
                        usb.uep4_ctrl().modify(|r, w| {
                            w.bits((r.bits() & !MASK_UEP_T_RES) | UEP_T_RES_NAK)
                        });
                    }

                    _ => {}
                }

                usb.int_fg().write_with_zero(|w| w.uif_transfer().set_bit());
            }

            if int_st.uis_setup_act().bit_is_set() { // Setup processing
                usb.uep0_ctrl().write_with_zero(|w| {
                    w.uep_r_tog().set_bit();
                    w.uep_t_tog().set_bit();
                    w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
                });

                // read raw bytes from EP0 databuf into setup req struct

                let pSetupReqPak: &USB_SETUP_REQ = EP0_Databuf.0.as_ptr().cast::<USB_SETUP_REQ>().as_ref().unwrap();
                SetupReqLen = pSetupReqPak.wLength;
                SetupReqCode = pSetupReqPak.bRequest;
                chtype = pSetupReqPak.bRequestType;

                len = 0;
                errflag = 0;

                if (pSetupReqPak.bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD {
                    /* Non-standard requests */
                    /* Other requests, such as class requests, vendor requests, etc. */
                    if (pSetupReqPak.bRequestType & 0x40) > 0 {
                        /* Manufacturer request */
                    } else if (pSetupReqPak.bRequestType & 0x20) > 0 {
                        match SetupReqCode {
                            DEF_USB_SET_IDLE => { /* 0x0A: SET_IDLE */
                                //The host wants to set the idle time interval for HID device-specific input reports
                                Idle_Value[pSetupReqPak.wIndex as usize] = (pSetupReqPak.wValue>>8) as u8;
                            }

                            DEF_USB_SET_REPORT => { /* 0x09: SET_REPORT */
                                //The host wants to set the report descriptor of the HID device
                            }

                            DEF_USB_SET_PROTOCOL => { /* 0x0B: SET_PROTOCOL */
                                //The host wants to set the protocol currently used by the HID device
                                Report_Value[pSetupReqPak.wIndex as usize] = pSetupReqPak.wValue as u8;
                            }

                            DEF_USB_GET_IDLE => { /* 0x02: GET_IDLE */
                                //The host wants to read the current idle ratio of the HID device specific input report.
                                EP0_Databuf.0[0] = Idle_Value[pSetupReqPak.wIndex as usize];
                                len = 1;
                            }

                            DEF_USB_GET_PROTOCOL => { /* 0x03: GET_PROTOCOL */
                                //The host wants to obtain the protocol currently used by the HID device
                                EP0_Databuf.0[0] = Report_Value[pSetupReqPak.wIndex as usize];
                                len = 1;
                            }

                            _ => {
                                errflag = 0xFF;
                            }
                        }
                    }
                } else /* standard request */ {
                    match SetupReqCode {
                        USB_GET_DESCRIPTOR => {
                            match (pSetupReqPak.wValue >> 8) as u8 {
                                USB_DESCR_TYP_DEVICE => {
                                    pDescr = &MyDevDescr;
                                    len = MyDevDescr[0];
                                }

                                USB_DESCR_TYP_CONFIG => {
                                    pDescr = &MyCfgDescr;
                                    len = MyCfgDescr[2];
                                }

                                USB_DESCR_TYP_HID => {
                                    match pSetupReqPak.wIndex & 0xff {
                                        /* select interface */
                                        0 => {
                                            pDescr = &MyCfgDescr[18..];
                                            len = 9;
                                        }

                                        1 => {
                                            pDescr = &MyCfgDescr[43..];
                                            len = 9;
                                        }

                                        _ => {
                                            /* unsupported string descriptor */
                                            errflag = 0xff;
                                        }
                                    }
                                }

                                USB_DESCR_TYP_REPORT => {
                                    if ((pSetupReqPak.wIndex) & 0xff) == 0 {
                                        //interface 0 report descriptor
                                        pDescr = &KeyRepDesc; //ready for upload
                                        len = KeyRepDesc.len() as u8;
                                    }
                                    else if ((pSetupReqPak.wIndex) & 0xff) == 1 {
                                        //interface 1 report descriptor
                                        pDescr = &MouseRepDesc; //ready for upload
                                        len = MouseRepDesc.len() as u8;
                                        Ready = 1; //last interface configured
                                    } else {
                                        len = 0xff; //only 2 interfaces; so, this is an error
                                    }
                                }

                                USB_DESCR_TYP_STRING => {
                                    match (pSetupReqPak.wValue) & 0xff {
                                        1 => {
                                            pDescr = &MyManuInfo;
                                            len = MyManuInfo[0];
                                        }
                                        2 => {
                                            pDescr = &MyProdInfo;
                                            len = MyProdInfo[0];
                                        }
                                        0 => {
                                            pDescr = &MyLangDescr;
                                            len = MyLangDescr[0];
                                        }
                                        _ => {
                                            errflag = 0xFF; // unsupported string descriptor
                                        }
                                    }
                                }

                                USB_DESCR_TYP_QUALIF => {
                                    pDescr = &My_QueDescr;
                                    len = My_QueDescr.len() as u8;
                                }

                                USB_DESCR_TYP_SPEED => {
                                    USB_FS_OSC_DESC[2..(MyCfgDescr.len())].copy_from_slice(&MyCfgDescr[2..]);
                                    pDescr = &USB_FS_OSC_DESC;
                                    len = USB_FS_OSC_DESC.len() as u8;
                                }

                                _ => {
                                    errflag = 0xff;
                                }
                            }

                            if SetupReqLen > len as u16 {
                                SetupReqLen = len as u16;
                            }
                            len = if SetupReqLen >= DevEP0SIZE as u16 { DevEP0SIZE } else { SetupReqLen as u8 };
                            EP0_Databuf.0[..len as usize].copy_from_slice(&pDescr[..len as usize]);
                            pDescr = &pDescr[len as usize..];
                        }

                        USB_SET_ADDRESS => {
                            SetupReqLen = pSetupReqPak.wValue & 0xff;
                        }

                        USB_GET_CONFIGURATION => {
                            EP0_Databuf.0[0] = DevConfig;
                            if SetupReqLen > 1 {
                                SetupReqLen = 1;
                            }
                        }

                        USB_SET_CONFIGURATION => {
                            DevConfig = pSetupReqPak.wValue as u8 & 0xff;
                        }

                        USB_CLEAR_FEATURE => {
                            if (pSetupReqPak.bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP {
                                // endpoints
                                match (pSetupReqPak.wIndex) & 0xff {
                                    0x83 => {
                                        usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK)
                                        });
                                    }
                                    0x03 => {
                                        usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK)
                                        });
                                    }
                                    0x82 => {
                                        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK)
                                        });
                                    }
                                    0x02 => {
                                        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK)
                                        });
                                    }
                                    0x81 => {
                                        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK)
                                        });
                                    }
                                    0x01 => {
                                        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK)
                                        });
                                    }
                                    _ => {
                                        errflag = 0xFF; // Unsupported endpoint
                                    }
                                }
                            } else if (pSetupReqPak.bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE {
                                if pSetupReqPak.wValue == 1 {
                                    USB_SleepStatus &= !0x01;
                                }
                            } else {
                                errflag = 0xFF;
                            }
                        }

                        USB_SET_FEATURE => {
                            if (pSetupReqPak.bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP {
                                /* endpoints */
                                match pSetupReqPak.wIndex {
                                    0x83 => {
                                        usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_STALL)
                                        });
                                    }
                                    0x03 => {
                                        usb.uep3_ctrl__r8_uh_tx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_STALL)
                                        });
                                    }
                                    0x82 => {
                                        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_STALL)
                                        });
                                    }
                                    0x02 => {
                                        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_STALL)
                                        });
                                    }
                                    0x81 => {
                                        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_STALL)
                                        });
                                    }
                                    0x01 => {
                                        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
                                            w.bits(r.bits() & !(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_STALL)
                                        });
                                    }
                                    _ => {
                                        errflag = 0xFF; // unsupported endpoint
                                    }
                                }
                            } else if (pSetupReqPak.bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE {
                                if pSetupReqPak.wValue == 1 {
                                    /* set sleep */
                                    USB_SleepStatus |= 0x01;
                                }
                            } else {
                                errflag = 0xFF;
                            }
                        }

                        USB_GET_INTERFACE => {
                            EP0_Databuf.0[0] = 0x00;
                            if SetupReqLen > 1 {
                                SetupReqLen = 1;
                            }
                        }

                        USB_SET_INTERFACE => {}

                        USB_GET_STATUS => {
                            if (pSetupReqPak.bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP {
                                /* endpoints */
                                EP0_Databuf.0[0] = 0x00;
                                match pSetupReqPak.wIndex {
                                    0x83 => {
                                        if (usb.uep3_ctrl__r8_uh_tx_ctrl().read().bits() & (RB_UEP_T_TOG | MASK_UEP_T_RES)) == UEP_T_RES_STALL {
                                            EP0_Databuf.0[0] = 0x01;
                                        }
                                    }

                                    0x03 => {
                                        if (usb.uep3_ctrl__r8_uh_tx_ctrl().read().bits() & (RB_UEP_R_TOG | MASK_UEP_R_RES)) == UEP_R_RES_STALL {
                                            EP0_Databuf.0[0] = 0x01;
                                        }
                                    }

                                    0x82 => {
                                        if (usb.uep2_ctrl_r8_uh_rx_ctrl().read().bits() & (RB_UEP_T_TOG | MASK_UEP_T_RES)) == UEP_T_RES_STALL {
                                            EP0_Databuf.0[0] = 0x01;
                                        }
                                    }

                                    0x02 => {
                                        if (usb.uep2_ctrl_r8_uh_rx_ctrl().read().bits() & (RB_UEP_R_TOG | MASK_UEP_R_RES)) == UEP_R_RES_STALL {
                                            EP0_Databuf.0[0] = 0x01;
                                        }
                                    }

                                    0x81 => {
                                        if (usb.uep1_ctrl__r8_uh_setup().read().bits() & (RB_UEP_T_TOG | MASK_UEP_T_RES)) == UEP_T_RES_STALL {
                                            EP0_Databuf.0[0] = 0x01;
                                        }
                                    }

                                    0x01 => {
                                        if (usb.uep1_ctrl__r8_uh_setup().read().bits() & (RB_UEP_R_TOG | MASK_UEP_R_RES)) == UEP_R_RES_STALL {
                                            EP0_Databuf.0[0] = 0x01;
                                        }
                                    }

                                    _ => {}
                                }
                            } else if (pSetupReqPak.bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE {
                                EP0_Databuf.0[0] = 0x00;
                                if USB_SleepStatus > 0 {
                                    EP0_Databuf.0[0] = 0x02;
                                } else {
                                    EP0_Databuf.0[0] = 0x00;
                                }
                            }

                            EP0_Databuf.0[1] = 0;

                            if SetupReqLen >= 2 {
                                SetupReqLen = 2;
                            }
                        }

                        _ => {
                            errflag = 0xff;
                        }
                    }
                }

                if errflag == 0xff { // error or unsupported
                    // STALL
                    usb.uep0_ctrl().write_with_zero(|w| {
                        w.bits(UEP_R_RES_STALL | UEP_T_RES_STALL);
                        w.uep_r_tog().set_bit();
                        w.uep_t_tog().set_bit()
                    });
                } else {
                    if (chtype & 0x80) > 0 { // upload
                        len = if (SetupReqLen as u8 > DevEP0SIZE) { DevEP0SIZE } else { SetupReqLen as u8 };
                        SetupReqLen -= len as u16;
                    } else {
                        len = 0; // download
                    }
                    usb.uep0_t_len().write(|w| w.bits(len));
                    usb.uep0_ctrl().write_with_zero(|w| {
                        w.bits(UEP_R_RES_ACK | UEP_T_RES_ACK);
                        w.uep_r_tog().set_bit();
                        w.uep_t_tog().set_bit()
                    });
                }

                usb.int_fg().write_with_zero(|w| w.uif_transfer().set_bit());
            }
        } else if int_fg.uif_bus_rst__rb_uif_detect().bit_is_set() {
            usb.dev_ad().write(|w| w.bits(0));
            usb.uep0_ctrl().write_with_zero(|w| {
               w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
            });
            usb.uep1_ctrl__r8_uh_setup().write_with_zero(|w| {
                w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
            });
            usb.uep2_ctrl_r8_uh_rx_ctrl().write_with_zero(|w| {
                w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
            });
            usb.uep3_ctrl__r8_uh_tx_ctrl().write_with_zero(|w| {
                w.bits(UEP_R_RES_ACK | UEP_T_RES_NAK)
            });
            usb.int_fg().write_with_zero(|w| w.uif_bus_rst__rb_uif_detect().set_bit());
        } else if int_fg.uif_suspend().bit_is_set() {
            if usb.mis_st().read().ums_suspend().bit_is_set() {
                // suspend
            } else {
                // wake
            }
            usb.int_fg().write_with_zero(|w| w.uif_suspend().set_bit());
        } else {
            usb.int_fg().write_with_zero(|w| w.bits(intflag) );
        }
    }
}

fn DevHIDMouseReport(mouse: u8) {
    unsafe {
        HIDMouse[0] = mouse;
        let in_databuf = &mut EP2_Databuf.0[64..];
        in_databuf[..HIDMouse.len()].copy_from_slice(&HIDMouse);
        DevEP2_IN_Deal(HIDMouse.len() as u8);
    }
}

fn DevHIDKeyReport(key: u8) {
    unsafe {
        HIDKey[2] = key;
        let in_databuf = &mut EP1_Databuf.0[64..];
        in_databuf[..HIDKey.len()].copy_from_slice(&HIDKey);
        DevEP1_IN_Deal(HIDKey.len() as u8);
    }
}

fn DevEP1_IN_Deal(l: u8) {
    unsafe {
        let usb = peripherals::USB::steal();
        usb.uep1_t_len().write(|w| w.bits(l));
        usb.uep1_ctrl__r8_uh_setup().modify(|r, w| {
            w.bits((r.bits() & !MASK_UEP_T_RES) | UEP_T_RES_ACK)
        });
    }
}

fn DevEP2_IN_Deal(l: u8) {
    unsafe {
        let usb = peripherals::USB::steal();
        usb.uep2_t_len_r8_uh_ep_pid().write(|w| w.bits(l));
        usb.uep2_ctrl_r8_uh_rx_ctrl().modify(|r, w| {
            w.bits((r.bits() & !MASK_UEP_T_RES) | UEP_T_RES_ACK)
        });
    }
}

fn DevEP1_OUT_Deal(l: u8)
{
    // uint8_t i;

    // for(i = 0; i < l; i++)
    // {
    //     pEP1_IN_DataBuf[i] = ~pEP1_OUT_DataBuf[i];
    // }
    // DevEP1_IN_Deal(l);
}




#[qingke_rt::entry]
#[highcode]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);
    let uart1 = p.UART1;
    let uart = Uart::new(uart1, p.PA9, p.PA8, Default::default()).unwrap();
    let (tx, _rx) = uart.split();

    critical_section::with(|cs| {
        G_SERIAL.replace(cs, Some(tx));
    });

    println!("\r\n\r\nUSB HID Composite. Start:\r");

    USB_DeviceInit();

    println!("DeviceInit\r");

    unsafe {
        // Enable USB interrupt
        qingke::pfic::enable_interrupt(hal::pac::Interrupt::USB as u8);
    }

    loop {
        hal::delay_ms(1000);
        // Mouse clicks left button
        DevHIDMouseReport(0x01);
        hal::delay_ms(100);
        DevHIDMouseReport(0x00);
        hal::delay_ms(200);

        // Keyboard types out "wch"
        DevHIDKeyReport(0x1A);
        hal::delay_ms(100);
        DevHIDKeyReport(0x00);
        hal::delay_ms(200);
        DevHIDKeyReport(0x06);
        hal::delay_ms(100);
        DevHIDKeyReport(0x00);
        hal::delay_ms(200);
        DevHIDKeyReport(0x0B);
        hal::delay_ms(100);
        DevHIDKeyReport(0x00);

        hal::delay_ms(1000);
    }
}

#[qingke_rt::interrupt]
#[highcode]
fn USB() {
    USB_DevTransProcess();
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
