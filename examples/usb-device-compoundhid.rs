#![no_std]
#![no_main]

use core::ptr::addr_of;
use core::cell::RefCell;
use ch58x_hal::usb::{HidRequest, SetupRecipient, UsbStandardRequest};
use critical_section::Mutex;

use ch58x_hal as hal;
use hal::peripherals;
use hal::uart::{UartTx, Uart};

// TODO: usb hal in progress
use hal::usb;
use usb::{
    UEP_R_RES_ACK,
    UEP_T_RES_ACK,
    UEP_T_RES_NAK,
    RB_UD_PD_DIS,
    MASK_UIS_TOKEN,
    MASK_UIS_ENDP,
    MASK_UEP_T_RES,
};
use usb::{
    UIS_TOKEN_IN,
    UIS_TOKEN_OUT,
};
use usb::{
    USB_REQ_TYP_MASK,
    USB_REQ_TYP_STANDARD,
};
use usb::{
    USB_REQ_RECIP_MASK,
    USB_REQ_RECIP_DEVICE,
    USB_REQ_RECIP_ENDP,
};
use usb::{
    USB_GET_STATUS,
    USB_CLEAR_FEATURE,
    USB_SET_FEATURE,
    USB_SET_ADDRESS,
    USB_GET_DESCRIPTOR,
    USB_SET_DESCRIPTOR,
    USB_GET_CONFIGURATION,
    USB_SET_CONFIGURATION,
    USB_GET_INTERFACE,
    USB_SET_INTERFACE,
};
use usb::{
    HID_SET_IDLE,
    HID_SET_REPORT,
    HID_SET_PROTOCOL,
    HID_GET_IDLE,
    HID_GET_PROTOCOL,
};
use usb::{
    RRes,
    TRes,
    UEPnCtrl,
    USB,
};
use usb::{
    SetupRequest,
    SetupRequestType,
    SetupTransferDirection,
    DescriptorType,
};
use usb::{
    USB_DESCR_TYP_DEVICE,
    USB_DESCR_TYP_CONFIG,
    USB_DESCR_TYP_STRING,
    USB_DESCR_TYP_QUALIF,
    USB_DESCR_TYP_SPEED,
    USB_DESCR_TYP_HID,
    USB_DESCR_TYP_REPORT,
};

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
static mut setup_request: SetupRequest = SetupRequest {
    bm_request_type: 0,
    bRequest: 0,
    wValue: 0,
    wIndex: 0,
    wLength: 0,
};
static mut SetupReqCode: u8 = 0;
static mut DevAddress: u8 = 0;
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

fn handle_class_setup_request() -> Result<usize, u8> {
    unsafe {
        let usb = peripherals::USB::steal();
        let usbh = USB::new(peripherals::USB::steal());
        println!(
            "SetupRequest: class request, req: {:?}\r",
            setup_request.hid_request(),
        );
        match setup_request.hid_request() {
            HidRequest::SetIdle => {
                //The host wants to set the idle time interval for HID device-specific input reports
                Idle_Value[setup_request.wIndex as usize] = (setup_request.wValue>>8) as u8;
                Ok(0)
            }
            HidRequest::SetReport => {
                //The host wants to set the report descriptor of the HID device
                Ok(0)
            }
            HidRequest::SetProtocol => {
                //The host wants to set the protocol currently used by the HID device
                Report_Value[setup_request.wIndex as usize] = setup_request.wValue as u8;
                Ok(0)
            }
            HidRequest::GetIdle => {
                //The host wants to read the current idle ratio of the HID device specific input report.
                EP0_Databuf.0[0] = Idle_Value[setup_request.wIndex as usize];
                Ok(1)
            }
            HidRequest::GetProtocol => {
                //The host wants to obtain the protocol currently used by the HID device
                EP0_Databuf.0[0] = Report_Value[setup_request.wIndex as usize];
                Ok(1)
            }
            _ => {
                Err(0xff)
            }
        }
    }
}

fn handle_standard_setup_request() -> Result<usize, u8> {
    unsafe {
        let usb = peripherals::USB::steal();
        let usbh = USB::new(peripherals::USB::steal());

        let mut errflag: u8 = 0;

        println!(
            "SetupRequest: standard request, req: {:?}\r",
            setup_request.usb_standard_request(),
        );
        match setup_request.usb_standard_request() {
            UsbStandardRequest::GetDescriptor => {
                println!(
                    "SetupRequest: descriptor type: {:?}\r",
                    setup_request.descriptor_type(),
                );
                match setup_request.descriptor_type() {
                    DescriptorType::Device => {
                        pDescr = &MyDevDescr;
                    }

                    DescriptorType::Configuration => {
                        pDescr = &MyCfgDescr;
                    }

                    DescriptorType::HID => {
                        match setup_request.wIndex & 0xff {
                            /* select interface */
                            0 => {
                                pDescr = &MyCfgDescr[18..27];
                            }

                            1 => {
                                pDescr = &MyCfgDescr[43..52];
                            }

                            _ => {
                                /* unsupported string descriptor */
                                errflag = 0xff;
                            }
                        }
                    }

                    DescriptorType::Report => {
                        if ((setup_request.wIndex) & 0xff) == 0 {
                            //interface 0 report descriptor
                            pDescr = &KeyRepDesc; //ready for upload
                        }
                        else if ((setup_request.wIndex) & 0xff) == 1 {
                            //interface 1 report descriptor
                            pDescr = &MouseRepDesc; //ready for upload
                            Ready = 1; //last interface configured
                        } else {
                            errflag = 0xff; //only 2 interfaces; so, this is an error
                        }
                    }

                    DescriptorType::String => {
                        match (setup_request.wValue) & 0xff {
                            1 => {
                                pDescr = &MyManuInfo;
                            }
                            2 => {
                                pDescr = &MyProdInfo;
                            }
                            0 => {
                                pDescr = &MyLangDescr;
                            }
                            _ => {
                                errflag = 0xFF; // unsupported string descriptor
                            }
                        }
                    }

                    DescriptorType::Qualifier => {
                        pDescr = &My_QueDescr;
                    }

                    DescriptorType::Speed => {
                        USB_FS_OSC_DESC[2..(MyCfgDescr.len())].copy_from_slice(&MyCfgDescr[2..]);
                        pDescr = &USB_FS_OSC_DESC;
                    }

                    _ => {
                        errflag = 0xff;
                    }
                }

                let offered_len = pDescr.len();
                let requested_len = setup_request.wLength as usize;
                let mut len = if requested_len < offered_len {
                    requested_len
                } else {
                    offered_len
                };
                len = if len >= DevEP0SIZE as usize { DevEP0SIZE as usize } else { len };
                EP0_Databuf.0[..len as usize].copy_from_slice(&pDescr[..len as usize]);
                pDescr = &pDescr[len as usize..];

                if errflag > 0 {
                    Err(errflag)
                } else {
                    Ok(len)
                }
            }
            UsbStandardRequest::SetAddress => {
                DevAddress = (setup_request.wValue & 0xff) as u8;
                Ok(0)
            }
            UsbStandardRequest::GetConfiguration => {
                EP0_Databuf.0[0] = DevConfig;
                Ok(1)
            }
            UsbStandardRequest::SetConfiguration => {
                DevConfig = setup_request.wValue as u8 & 0xff;
                Ok(0)
            }
            UsbStandardRequest::ClearFeature => {
                match setup_request.recipient() {
                    SetupRecipient::Endpoint => {
                        // endpoints
                        let direction = setup_request.wIndex & 0x80;
                        let endpoint_number = setup_request.wIndex as u8 & 0x0f;
                        match (direction, endpoint_number) {
                            (0x80, i) if i > 0 => {
                                usbh.UEPn(i).clear_t();
                                Ok(0)
                            }
                            (0x00, i) if i > 0 => {
                                usbh.UEPn(i).clear_r();
                                Ok(0)
                            }
                            _ => {
                                Err(0xff) // Unsupported endpoint
                            }
                        }
                    }
                    SetupRecipient::Device => {
                        if setup_request.wValue == 1 {
                            USB_SleepStatus &= !0x01;
                        }
                        Ok(0)
                    }
                    _ => {
                        Err(0xff)
                    }
                }
            }
            UsbStandardRequest::SetFeature => {
                match setup_request.recipient() {
                    SetupRecipient::Endpoint => {
                        /* endpoints */
                        let direction = setup_request.wIndex & 0x80;
                        let endpoint_number = setup_request.wIndex as u8 & 0x0f;
                        match (direction, endpoint_number) {
                            (0x80, i) if i > 0 => {
                                usbh.UEPn(i).set_t_res(TRes::Stall);
                                Ok(0)
                            }
                            (0x00, i) if i > 0 => {
                                usbh.UEPn(i).set_r_res(RRes::Stall);
                                Ok(0)
                            }
                            _ => {
                                Err(0xff) // unsupported endpoint
                            }
                        }
                    }
                    SetupRecipient::Device => {
                        if setup_request.wValue == 1 {
                            /* set sleep */
                            USB_SleepStatus |= 0x01;
                        }
                        Ok(0)
                    }
                    _ => {
                        Err(0xff)
                    }
                }
            }
            UsbStandardRequest::GetInterface => {
                EP0_Databuf.0[0] = 0x00;
                Ok(1)
            }
            UsbStandardRequest::SetInterface => {
                Ok(0)
            }
            UsbStandardRequest::GetStatus => {
                match setup_request.recipient() {
                    SetupRecipient::Endpoint => {
                        /* endpoints */
                        EP0_Databuf.0[0] = 0x00;
                        let direction = setup_request.wIndex & 0x80;
                        let endpoint_number = setup_request.wIndex as u8 & 0x0f;
                        match (direction, endpoint_number) {
                            (0x80, i) if i > 0 => {
                                if usbh.UEPn(i).is_t_stalled() {
                                    EP0_Databuf.0[0] = 0x01;
                                }
                            }
                            (0x00, i) if i > 0 => {
                                if usbh.UEPn(i).is_r_stalled() {
                                    EP0_Databuf.0[0] = 0x01;
                                }
                            }

                            _ => {}
                        }
                    }
                    SetupRecipient::Device => {
                        EP0_Databuf.0[0] = 0x00;
                        if USB_SleepStatus > 0 {
                            EP0_Databuf.0[0] = 0x02;
                        } else {
                            EP0_Databuf.0[0] = 0x00;
                        }
                    }
                    _ => {
                    }
                }

                EP0_Databuf.0[1] = 0;

                Ok(2)
            }
            _ => {
                Err(0xff)
            }
        }
    }
}

fn handle_setup_request() {
    unsafe {
        let usb = peripherals::USB::steal();
        let usbh = USB::new(peripherals::USB::steal());

        println!("\r");
        println!(
            "SetupRequest:(direction: {:?}, type: {:?}, recipient: {:?})\r",
            setup_request.direction(),
            setup_request.request_type(),
            setup_request.recipient(),
        );

        let setup_offer = match setup_request.request_type() {
            SetupRequestType::Reserved => { Ok(0) }
            SetupRequestType::Vendor => { Ok(0) }
            SetupRequestType::Class => {
                handle_class_setup_request()
            }
            SetupRequestType::Standard => {
                handle_standard_setup_request()
            }
        };

        match setup_offer {
            Err(_) => { // error or unsupported
                // STALL
                usbh.UEPn(0).set(true, true, RRes::Stall, TRes::Stall);
            }
            Ok(len) => {
                usb.uep0_t_len().write(|w| w.bits(len as u8));
                usbh.UEPn(0).set(true, true, RRes::Ack, TRes::Ack);
            }
        }
    }
}

fn USB_DevTransProcess() {
    unsafe {
        let usb = peripherals::USB::steal();
        let usbh = USB::new(peripherals::USB::steal());

        let int_fg = usb.int_fg().read();

        if int_fg.uif_transfer().bit_is_set() {
            let int_st = usb.int_st().read();

            if (int_st.bits() & int_st.mask_uis_token().bits()) != MASK_UIS_TOKEN { // Non-idle
                let token = int_st.bits() & MASK_UIS_TOKEN;
                let endp = int_st.bits() & MASK_UIS_ENDP;
                match (token, endp) {
                    (UIS_TOKEN_IN, 0) => {
                        match SetupReqCode as u8 {
                            USB_GET_DESCRIPTOR => {
                                let len: u8 = if pDescr.len() as u8 >= DevEP0SIZE { DevEP0SIZE } else { pDescr.len() as u8 };
                                EP0_Databuf.0[..len as usize].copy_from_slice(&pDescr[..len as usize]);
                                pDescr = &pDescr[len as usize..];
                                usb.uep0_t_len().write(|w| w.bits(len as u8));
                                usbh.UEPn(0).toggle_t();
                            }

                            USB_SET_ADDRESS => {
                                usbh.set_address(DevAddress);
                                usbh.UEPn(0).set(false, false, RRes::Ack, TRes::Nak)
                            }

                            USB_SET_FEATURE => {}

                            _ => {
                                // The status stage is completed and the
                                //    interruption is completed or the zero-length data
                                //    packet is forced to be uploaded to end the control
                                //    transmission.
                                usb.uep0_t_len().write(|w| w.bits(0));
                                usbh.UEPn(0).set(false, false, RRes::Ack, TRes::Nak)
                            }
                        }
                    }

                    (UIS_TOKEN_OUT, 0) => {
                        // len = usb.rx_len().read().bits();
                        if SetupReqCode == 0x09 {
                            println!("[{}] Num Lock\r", if EP0_Databuf.0[0] & (1 << 0) != 0 { "*" } else { " " });
                            println!("[{}] Caps Lock\r", if EP0_Databuf.0[0] & (1 << 1) != 0 { "*" } else { " " });
                            println!("[{}] Scroll Lock\r\n", if EP0_Databuf.0[0] & (1 << 2) != 0 { "*" } else { " " });
                        }
                    }

                    (UIS_TOKEN_OUT, i) => {
                        if int_st.uis_tog_ok().bit_is_set() {
                            // Discard out-of-sync packets
                            usbh.UEPn(i).toggle_r();
                        }
                    }

                    (UIS_TOKEN_IN, i) => {
                        usbh.UEPn(i).toggle_t();
                        usbh.UEPn(i).set_t_res(TRes::Nak);
                    }

                    _ => {}
                }

                usb.int_fg().write_with_zero(|w| w.uif_transfer().set_bit());
            }

            if int_st.uis_setup_act().bit_is_set() { // Setup processing
                usbh.UEPn(0).set(true, true, RRes::Ack, TRes::Nak);

                // read raw bytes from EP0 databuf into setup req struct

                let pSetupReqPak: &SetupRequest = EP0_Databuf.0.as_ptr().cast::<SetupRequest>().as_ref().unwrap();
                setup_request = pSetupReqPak.clone();
                SetupReqCode = setup_request.bRequest;

                handle_setup_request();

                usb.int_fg().write_with_zero(|w| w.uif_transfer().set_bit());
            }
        } else if int_fg.uif_bus_rst__rb_uif_detect().bit_is_set() {
            usb.dev_ad().write(|w| w.bits(0));
            usbh.UEPn(0).set(false, false, RRes::Ack, TRes::Nak);
            usbh.UEPn(1).set(false, false, RRes::Ack, TRes::Nak);
            usbh.UEPn(2).set(false, false, RRes::Ack, TRes::Nak);
            usbh.UEPn(3).set(false, false, RRes::Ack, TRes::Nak);
            usb.int_fg().write_with_zero(|w| w.uif_bus_rst__rb_uif_detect().set_bit());
        } else if int_fg.uif_suspend().bit_is_set() {
            if usb.mis_st().read().ums_suspend().bit_is_set() {
                // suspend
            } else {
                // wake
            }
            usb.int_fg().write_with_zero(|w| w.uif_suspend().set_bit());
        } else {
            usb.int_fg().modify(|r, w| w.bits(r.bits()) );
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
