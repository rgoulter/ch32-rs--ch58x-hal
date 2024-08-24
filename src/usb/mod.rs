// Consts for USB registers
// TODO: HAL shouldn't need expose these.

pub const RB_UEP_AUTO_TOG: u8 = 0x10;      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle

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

pub const RB_UEP_T_RES1: u8 =      0x02;     // handshake response type high bit for USB endpoint X transmittal (IN)
pub const RB_UEP_T_RES0: u8 =      0x01;     // handshake response type low bit for USB endpoint X transmittal (IN)
pub const MASK_UEP_T_RES: u8 =     0x03;     // bit mask of handshake response type for USB endpoint X transmittal (IN)

pub const MASK_UEP_R_RES: u8 =     0x0C;     // bit mask of handshake response type for USB endpoint X receiving (OUT)

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


#[repr(packed, C)]
pub struct USB_SETUP_REQ {
    pub bRequestType: u8,
    pub bRequest: u8,
    pub wValue: u16,
    pub wIndex: u16,
    pub wLength: u16,
}
