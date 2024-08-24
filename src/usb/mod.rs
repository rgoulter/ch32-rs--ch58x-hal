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
