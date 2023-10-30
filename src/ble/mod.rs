use core::ffi::CStr;
use core::mem::MaybeUninit;
use core::num::NonZeroU8;

use crate::{pac, println};

pub mod ffi;

const HEAP_SIZE: usize = 1024 * 6;
// use u32 to align to 4

#[repr(C, align(4))]
struct BLEHeap([MaybeUninit<u8>; HEAP_SIZE]);

static mut BLE_HEAP: BLEHeap = BLEHeap([MaybeUninit::uninit(); HEAP_SIZE]);

/// "CH58x_BLE_LIB_V1.9"
pub fn lib_version() -> &'static str {
    unsafe {
        let version = CStr::from_ptr(ffi::VER_LIB.as_ptr());
        version.to_str().unwrap()
    }
}

/// Library format MAC Address, LSB first
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacAddress([u8; 6]);

impl MacAddress {
    #[inline(always)]
    pub fn from_raw(addr: [u8; 6]) -> Self {
        MacAddress(addr)
    }

    /// Convert from human readable format, MSB first
    pub fn from_msb(addr: [u8; 6]) -> Self {
        MacAddress([addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]])
    }
}

impl core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let MacAddress(addr) = self;
        write!(
            f,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
        )
    }
}

#[derive(Debug)]
pub struct Config {
    pub mac_addr: MacAddress,
}

impl Default for Config {
    fn default() -> Self {
        let mac_addr = MacAddress::from_raw(crate::isp::get_mac_address());
        Config { mac_addr }
    }
}

/// Wrapper of BLEInit and HAL_Init
pub fn init(config: Config) -> Result<(), NonZeroU8> {
    use ffi::{bleConfig_t, BLE_LibInit, BLE_RegInit, TMOS_TimerInit, LL_TX_POWEER_6_DBM};
    const BLE_TX_NUM_EVENT: u8 = 1;
    const BLE_BUFF_NUM: u8 = 5;
    const BLE_BUFF_MAX_LEN: u16 = 27;
    const BLE_TX_POWER: u8 = LL_TX_POWEER_6_DBM;
    const PERIPHERAL_MAX_CONNECTION: u8 = 1;
    const CENTRAL_MAX_CONNECTION: u8 = 3;

    let mut cfg: bleConfig_t = unsafe { core::mem::zeroed() };

    cfg.MEMAddr = unsafe { BLE_HEAP.0.as_ptr() as u32 };
    cfg.MEMLen = HEAP_SIZE as _;

    cfg.BufMaxLen = BLE_BUFF_MAX_LEN;
    cfg.BufNumber = BLE_BUFF_NUM;
    cfg.TxNumEvent = BLE_TX_NUM_EVENT;
    cfg.TxPower = BLE_TX_POWER;

    // No SNV (SNVAddr, SNVBlock, SNVNum, readFlashCB, writeFlashCB)

    cfg.SelRTCClock = 0; // use LSE: ( 0 外部(32768Hz)，默认:1：内部(32000Hz)，2：内部(32768Hz)

    cfg.ConnectNumber = (PERIPHERAL_MAX_CONNECTION & 3) | (CENTRAL_MAX_CONNECTION << 2);

    cfg.srandCB = Some(srand); // tmos_rand will call this

    cfg.rcCB = None; // use LSE, no calibrate
    cfg.tsCB = Some(get_raw_temperature);

    // No need to HAL_SLEEP
    // WakeUpTIme, sleepCB

    cfg.MacAddr = config.mac_addr.0;

    unsafe {
        BLE_LibInit(&cfg)?;
    }

    unsafe {
        TMOS_TimerInit(core::ptr::null_mut())?;

        BLE_RegInit();
    }

    Ok(())
}

pub unsafe extern "C" fn srand() -> u32 {
    let systick = unsafe { &*pac::SYSTICK::PTR };
    systick.cnt.read().bits() as u32
}

pub unsafe extern "C" fn get_raw_temperature() -> u16 {
    let mut regs: [u8; 4] = [0; 4];

    let rb = &*pac::ADC::PTR;
    let sys = &*pac::SYS::PTR; // TODO: refine rb
    regs[0] = sys.tkey_cfg.read().bits();
    regs[1] = rb.tem_sensor.read().bits();
    regs[2] = rb.channel.read().bits();
    regs[3] = rb.cfg.read().bits();

    let peri = crate::peripherals::ADC::steal();
    let mut adc = crate::adc::Adc::new(peri, crate::adc::Config::for_temperature());
    let mut temp_sensor = adc.enable_temperature();
    let data = adc.read(&mut temp_sensor);

    core::mem::forget(adc);

    // restore regs
    sys.tkey_cfg.write(|w| w.bits(regs[0]));
    rb.tem_sensor.write(|w| w.bits(regs[1]));
    rb.channel.write(|w| w.bits(regs[2]));
    rb.cfg.write(|w| w.bits(regs[3]));

    data
}
