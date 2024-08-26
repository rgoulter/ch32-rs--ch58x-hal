// Consts for USB registers
// TODO: HAL shouldn't need expose these.

pub const RB_UEP_AUTO_TOG: u8 = 0x10;      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle

pub const MASK_UEP_R_RES: u8 =     0x0C;     // bit mask of handshake response type for USB endpoint X receiving (OUT)

// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
pub const UEP_R_RES_ACK: u8 = 0x00;
pub const UEP_R_RES_TOUT: u8 = 0x04;
pub const UEP_R_RES_NAK: u8 = 0x08;
pub const UEP_R_RES_STALL: u8 = 0x0C;


pub const RB_UEP_R_TOG: u8 =       0x80;     // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
pub const RB_UEP_T_TOG: u8 =       0x40;     // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1

pub const MASK_UEP_T_RES: u8 =     0x03;     // bit mask of handshake response type for USB endpoint X transmittal (IN)

// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
pub const UEP_T_RES_ACK: u8 = 0x00;
pub const UEP_T_RES_TOUT: u8 = 0x01;
pub const UEP_T_RES_NAK: u8 = 0x02;
pub const UEP_T_RES_STALL: u8 = 0x03;

pub const RB_UD_PD_DIS: u8 = 0x80;

pub const RB_UIS_SETUP_ACT: u8 = 0x80;   // RO, indicate SETUP token & 8 bytes setup request received for USB device mode
pub const RB_UIS_TOG_OK: u8 = 0x40;      // RO, indicate current USB transfer toggle is OK
pub const RB_UIS_TOKEN1: u8 = 0x20;      // RO, current token PID code bit 1 received for USB device mode
pub const RB_UIS_TOKEN0: u8 = 0x10;      // RO, current token PID code bit 0 received for USB device mode
pub const MASK_UIS_TOKEN: u8 = 0x30;     // RO, bit mask of current token PID code received for USB device mode
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode, keep last status during SETUP token, clear RB_UIF_TRANSFER ( RB_UIF_TRANSFER from 1 to 0 ) to set free
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: free
pub const UIS_TOKEN_OUT: u8 = 0x00;
pub const UIS_TOKEN_IN: u8 = 0x20;
pub const UIS_TOKEN_SETUP: u8 = 0x30;

pub const MASK_UIS_ENDP: u8 = 0x0F;      // RO, bit mask of current transfer endpoint number for USB device mode






// USB standard device request code/
pub const USB_GET_STATUS: u8 =        0x00;
pub const USB_CLEAR_FEATURE: u8 =     0x01;
pub const USB_SET_FEATURE: u8 =       0x03;
pub const USB_SET_ADDRESS: u8 =       0x05;
pub const USB_GET_DESCRIPTOR: u8 =    0x06;
pub const USB_SET_DESCRIPTOR: u8 =    0x07;
pub const USB_GET_CONFIGURATION: u8 = 0x08;
pub const USB_SET_CONFIGURATION: u8 = 0x09;
pub const USB_GET_INTERFACE: u8 =     0x0A;
pub const USB_SET_INTERFACE: u8 =     0x0B;
pub const USB_SYNCH_FRAME: u8 =       0x0C;

/* Bit define for USB request type */
pub const USB_REQ_TYP_IN: u8        = 0x80;           /* control IN, device to host */
pub const USB_REQ_TYP_OUT: u8       = 0x00;           /* control OUT, host to device */
pub const USB_REQ_TYP_READ: u8      = 0x80;           /* control read, device to host */
pub const USB_REQ_TYP_WRITE: u8     = 0x00;           /* control write, host to device */
pub const USB_REQ_TYP_MASK: u8      = 0x60;           /* bit mask of request type */
pub const USB_REQ_TYP_STANDARD: u8  = 0x00;
pub const USB_REQ_TYP_CLASS: u8     = 0x20;
pub const USB_REQ_TYP_VENDOR: u8    = 0x40;
pub const USB_REQ_TYP_RESERVED: u8  = 0x60;
pub const USB_REQ_RECIP_MASK: u8    = 0x1F;           /* bit mask of request recipient */
pub const USB_REQ_RECIP_DEVICE: u8  = 0x00;
pub const USB_REQ_RECIP_INTERF: u8  = 0x01;
pub const USB_REQ_RECIP_ENDP: u8    = 0x02;
pub const USB_REQ_RECIP_OTHER: u8   = 0x03;

/* USB descriptor type */
pub const USB_DESCR_TYP_DEVICE: u8 =    0x01;
pub const USB_DESCR_TYP_CONFIG: u8 =    0x02;
pub const USB_DESCR_TYP_STRING: u8 =    0x03;
pub const USB_DESCR_TYP_INTERF: u8 =    0x04;
pub const USB_DESCR_TYP_ENDP: u8 =      0x05;
pub const USB_DESCR_TYP_QUALIF: u8 =    0x06;
pub const USB_DESCR_TYP_SPEED: u8 =     0x07;
pub const USB_DESCR_TYP_OTG: u8 =       0x09;
pub const USB_DESCR_TYP_HID: u8 =       0x21;
pub const USB_DESCR_TYP_REPORT: u8 =    0x22;
pub const USB_DESCR_TYP_PHYSIC: u8 =    0x23;
pub const USB_DESCR_TYP_CS_INTF: u8 =   0x24;
pub const USB_DESCR_TYP_CS_ENDP: u8 =   0x25;
pub const USB_DESCR_TYP_HUB: u8 =       0x29;

/* HID Class Request */
pub const HID_GET_REPORT: u8 =           0x01;
pub const HID_GET_IDLE: u8 =           0x02;
pub const HID_GET_PROTOCOL: u8 =       0x03;
pub const HID_SET_REPORT: u8 =         0x09;
pub const HID_SET_IDLE: u8 =           0x0A;
pub const HID_SET_PROTOCOL: u8 =       0x0B;

pub enum RRes {
    Ack = 0x00,
    Timeout = 0x04,
    Nak = 0x08,
    Stall = 0x0C,
}

pub enum TRes {
    Ack = 0x00,
    Timeout = 0x01,
    Nak = 0x02,
    Stall = 0x03,
}

#[derive(Clone, Copy)]
#[repr(packed, C)]
pub struct SetupRequest {
    pub bm_request_type: u8,
    pub bRequest: u8,
    pub wValue: u16,
    pub wIndex: u16,
    pub wLength: u16,
}

#[derive(Debug)]
pub enum SetupTransferDirection {
    HostToDevice,
    DeviceToHost,
}

#[derive(Debug)]
pub enum SetupRequestType {
    Standard,
    Class,
    Vendor,
    Reserved,
}

#[derive(Debug)]
pub enum SetupRecipient {
    Device,
    Interface,
    Endpoint,
    Other,
    Unknown(u8),
}

#[derive(Debug)]
pub enum SetupRequestRequest {
    GetStatus,
    ClearFeature,
    SetFeature,
    SetAddress,
    GetDescriptor,
    SetDescriptor,
    GetConfiguration,
    SetConfiguration,
    GetInterface,
    SetInterface,
    SynchFrame,
    Unknown(u8),
}

impl SetupRequest {
    pub fn request_type(&self) -> SetupRequestType {
        match self.bm_request_type & USB_REQ_TYP_MASK {
            USB_REQ_TYP_STANDARD => SetupRequestType::Standard,
            USB_REQ_TYP_CLASS => SetupRequestType::Class,
            USB_REQ_TYP_VENDOR => SetupRequestType::Vendor,
            USB_REQ_TYP_RESERVED => SetupRequestType::Reserved,
            _ => unreachable!(),
        }
    }

    pub fn recipient(&self) -> SetupRecipient {
        match self.bm_request_type & USB_REQ_RECIP_MASK {
            USB_REQ_RECIP_DEVICE => SetupRecipient::Device,
            USB_REQ_RECIP_INTERF => SetupRecipient::Interface,
            USB_REQ_RECIP_ENDP => SetupRecipient::Endpoint,
            USB_REQ_RECIP_OTHER => SetupRecipient::Other,
            x => SetupRecipient::Unknown(x),
        }
    }

    pub fn direction(&self) -> SetupTransferDirection {
        match self.bm_request_type & USB_REQ_TYP_IN {
            USB_REQ_TYP_IN => SetupTransferDirection::DeviceToHost,
            USB_REQ_TYP_OUT => SetupTransferDirection::HostToDevice,
            _ => unreachable!(),
        }
    }

    pub fn request(&self) -> SetupRequestRequest {
        match self.bRequest {
            USB_GET_STATUS => SetupRequestRequest::GetStatus,
            USB_CLEAR_FEATURE => SetupRequestRequest::ClearFeature,
            USB_SET_FEATURE => SetupRequestRequest::SetFeature,
            USB_SET_ADDRESS => SetupRequestRequest::SetAddress,
            USB_GET_DESCRIPTOR => SetupRequestRequest::GetDescriptor,
            USB_SET_DESCRIPTOR => SetupRequestRequest::SetDescriptor,
            USB_GET_CONFIGURATION => SetupRequestRequest::GetConfiguration,
            USB_SET_CONFIGURATION => SetupRequestRequest::SetConfiguration,
            USB_GET_INTERFACE => SetupRequestRequest::GetInterface,
            USB_SET_INTERFACE => SetupRequestRequest::SetInterface,
            USB_SYNCH_FRAME => SetupRequestRequest::SynchFrame,
            x => SetupRequestRequest::Unknown(x),
        }
    }
}

pub trait UEPnCtrl {
    fn toggle_r(&self);
    fn toggle_t(&self);
    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes);
}

pub struct USB {
    usb: crate::peripherals::USB,
}

impl USB {
    pub fn new(usb_: crate::peripherals::USB) -> Self {
        let usb = usb_;
        USB {
            usb,
        }
    }

    pub fn UEPn(&self, n: u8) -> UEPn {
        UEPn {
            n,
            uep0_ctrl: UEP0 { ctrl: &self.usb.uep0_ctrl() },
            uep1_ctrl: UEP1 { ctrl: &self.usb.uep1_ctrl__r8_uh_setup() },
            uep2_ctrl: UEP2 { ctrl: &self.usb.uep2_ctrl_r8_uh_rx_ctrl() },
            uep3_ctrl: UEP3 { ctrl: &self.usb.uep3_ctrl__r8_uh_tx_ctrl() },
            uep4_ctrl: UEP4 { ctrl: &self.usb.uep4_ctrl() },
        }
    }
}

pub struct UEP0<'a> {
    pub ctrl: &'a crate::pac::usb::UEP0_CTRL,
}

pub struct UEP1<'a> {
    pub ctrl: &'a crate::pac::usb::UEP1_CTRL__R8_UH_SETUP,
}

pub struct UEP2<'a> {
    pub ctrl: &'a crate::pac::usb::UEP2_CTRL_R8_UH_RX_CTRL,
}

pub struct UEP3<'a> {
    pub ctrl: &'a crate::pac::usb::UEP3_CTRL__R8_UH_TX_CTRL,
}

pub struct UEP4<'a> {
    pub ctrl: &'a crate::pac::usb::UEP4_CTRL,
}

impl<'a> UEPnCtrl for UEP0<'a> {
    fn toggle_r(&self) {
        self.ctrl.modify(|r, w| w.uep_r_tog().bit(r.uep_r_tog().bit() ^ true));
    }

    fn toggle_t(&self) {
        self.ctrl.modify(|r, w| w.uep_t_tog().bit(r.uep_t_tog().bit() ^ true));
    }

    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes) {
        self.ctrl.write(|w| {
            unsafe {
                w.bits(
                    (if r_tog { RB_UEP_R_TOG } else { 0x00 }) |
                    (if t_tog { RB_UEP_T_TOG } else { 0x00 }) |
                    (r_res as u8) |
                    (t_res as u8)
                )
            }
        });
    }
}

impl<'a> UEPnCtrl for UEP1<'a> {
    fn toggle_r(&self) {
        self.ctrl.modify(|r, w| w.uep_r_tog__rb_uh_pre_pid_en().bit(r.uep_r_tog__rb_uh_pre_pid_en().bit() ^ true));
    }

    fn toggle_t(&self) {
        self.ctrl.modify(|r, w| w.uep_t_tog__rb_uh_sof_en().bit(r.uep_t_tog__rb_uh_sof_en().bit() ^ true));
    }

    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes) {
        self.ctrl.write(|w| {
            unsafe {
                w.bits(
                    (if r_tog { RB_UEP_R_TOG } else { 0x00 }) |
                    (if t_tog { RB_UEP_T_TOG } else { 0x00 }) |
                    (r_res as u8) |
                    (t_res as u8)
                )
            }
        });
    }
}

impl<'a> UEPnCtrl for UEP2<'a> {
    fn toggle_r(&self) {
        self.ctrl.modify(|r, w| w.uep_r_tog__rb_uh_r_tog().bit(r.uep_r_tog__rb_uh_r_tog().bit() ^ true));
    }

    fn toggle_t(&self) {
        self.ctrl.modify(|r, w| w.uep_t_tog().bit(r.uep_t_tog().bit() ^ true));
    }

    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes) {
        self.ctrl.write(|w| {
            unsafe {
                w.bits(
                    (if r_tog { RB_UEP_R_TOG } else { 0x00 }) |
                    (if t_tog { RB_UEP_T_TOG } else { 0x00 }) |
                    (r_res as u8) |
                    (t_res as u8)
                )
            }
        });
    }
}

impl<'a> UEPnCtrl for UEP3<'a> {
    fn toggle_r(&self) {
        self.ctrl.modify(|r, w| w.uep_r_tog().bit(r.uep_r_tog().bit() ^ true));
    }

    fn toggle_t(&self) {
        self.ctrl.modify(|r, w| w.uep_t_tog_rb_uh_t_tog().bit(r.uep_t_tog_rb_uh_t_tog().bit() ^ true));
    }

    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes) {
        self.ctrl.write(|w| {
            unsafe {
                w.bits(
                    (if r_tog { RB_UEP_R_TOG } else { 0x00 }) |
                    (if t_tog { RB_UEP_T_TOG } else { 0x00 }) |
                    (r_res as u8) |
                    (t_res as u8)
                )
            }
        });
    }
}

impl<'a> UEPnCtrl for UEP4<'a> {
    fn toggle_r(&self) {
        self.ctrl.modify(|r, w| w.uep_r_tog().bit(r.uep_r_tog().bit() ^ true));
    }

    fn toggle_t(&self) {
        self.ctrl.modify(|r, w| w.uep_t_tog().bit(r.uep_t_tog().bit() ^ true));
    }

    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes) {
        self.ctrl.write(|w| {
            unsafe {
                w.bits(
                    (if r_tog { RB_UEP_R_TOG } else { 0x00 }) |
                    (if t_tog { RB_UEP_T_TOG } else { 0x00 }) |
                    (r_res as u8) |
                    (t_res as u8)
                )
            }
        });
    }
}

pub struct UEPn<'a> {
    n: u8,
    uep0_ctrl: UEP0<'a>,
    uep1_ctrl: UEP1<'a>,
    uep2_ctrl: UEP2<'a>,
    uep3_ctrl: UEP3<'a>,
    uep4_ctrl: UEP4<'a>,
}

impl<'a> UEPn<'a> {
    pub fn number(&self) -> u8 {
        self.n
    }
}

impl<'a> UEPnCtrl for UEPn<'a> {
    fn toggle_r(&self) {
        match self.n {
            0 => self.uep0_ctrl.toggle_r(),
            1 => self.uep1_ctrl.toggle_r(),
            2 => self.uep2_ctrl.toggle_r(),
            3 => self.uep3_ctrl.toggle_r(),
            4 => self.uep4_ctrl.toggle_r(),
            _ => unreachable!()
        }
    }

    fn toggle_t(&self) {
        match self.n {
            0 => self.uep0_ctrl.toggle_t(),
            1 => self.uep1_ctrl.toggle_t(),
            2 => self.uep2_ctrl.toggle_t(),
            3 => self.uep3_ctrl.toggle_t(),
            4 => self.uep4_ctrl.toggle_t(),
            _ => unreachable!()
        }
    }

    fn set(&self, r_tog: bool, t_tog: bool, r_res: RRes, t_res: TRes) {
        match self.n {
            0 => self.uep0_ctrl.set(r_tog, t_tog, r_res, t_res),
            1 => self.uep1_ctrl.set(r_tog, t_tog, r_res, t_res),
            2 => self.uep2_ctrl.set(r_tog, t_tog, r_res, t_res),
            3 => self.uep3_ctrl.set(r_tog, t_tog, r_res, t_res),
            4 => self.uep4_ctrl.set(r_tog, t_tog, r_res, t_res),
            _ => unreachable!()
        }
    }
}
