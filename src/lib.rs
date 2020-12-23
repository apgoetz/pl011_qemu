//! Driver for the pl011 UARTs in the QEMU implementation
//!
//! This crate provides basic drivers for the UARTS exposed by
//! QEMU. You can see the implementation of these uarts
//! [here](https://github.com/qemu/qemu/blob/master/hw/arm/stellaris.c)
//!
//! The QEMU target actually exposes 4 different UARTS, that can each
//! be redirected to arbitary character devices or files. This crate
//! allows those UARTS to be accessed in order to support more
//! complicated use cases than can be provided by
//! [cortex_m_semihosting](https://crates.io/crates/cortex-m-semihosting).
#![no_std]
use core::fmt;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic;
use cortex_m::interrupt;
use embedded_hal::serial;
use nb;
use volatile_register::{RO, RW, WO};

/// Struct representing PL011 registers. Not intended to be directly
/// used
#[repr(C)]
pub struct PL011_Regs {
    /// Data Register
    pub uartdr: RW<u32>,
    /// receive status / error clear
    pub uartrsr: RW<u32>,
    reserved0: [u32; 4],
    /// flag register
    pub uartfr: RO<u32>,
    reserved1: u32,
    /// IrDA Low power counter register
    pub uartilpr: RW<u32>,
    /// integer baud rate
    pub uartibrd: RW<u32>,
    /// fractional baud rate
    pub uartfbrd: RW<u32>,
    /// line control
    pub uartlcr_h: RW<u32>,
    /// control
    pub uartcr: RW<u32>,
    /// interrupt fifo level select
    pub uartifls: RW<u32>,
    /// interrupt mask set/clear
    pub uartimsc: RW<u32>,
    /// raw interrupt status
    pub uartris: RO<u32>,
    /// masked interrupt status
    pub uartmis: RO<u32>,
    /// interrupt clear
    pub uarticr: WO<u32>,
    /// dma control
    pub uartdmacr: RW<u32>,
    reserved2: [u32; 997],
    /// UART Periph ID0
    pub uartperiphid0: RO<u32>,
    /// UART Periph ID1
    pub uartperiphid1: RO<u32>,
    /// UART Periph ID2
    pub uartperiphid2: RO<u32>,
    /// UART Periph ID3
    pub uartperiphid3: RO<u32>,
    /// UART PCell ID0
    pub uartpcellid0: RO<u32>,
    /// UART PCell ID1
    pub uartpcellid1: RO<u32>,
    /// UART PCell ID2
    pub uartpcellid2: RO<u32>,
    /// UART PCell ID3
    pub uartpcellid3: RO<u32>,
}

/// Error type necessary for embedded_hal usage. No errors supported
pub struct Error;

/// Struct representing the actual driver.
///
/// Notice that there are no silly ideas like setting the baud rate,
/// or assigning GPIO pins to the driver: the qemu implementation
/// doesnt need any of that, we can just write to the registers
/// directly.
///
/// Implements embedded_hal::serial as well as core::fmt::Write
///
/// # Examples
///
/// ```
/// use pl011_qemu;
/// // build a driver for UART1
/// let mut uart = pl011_qemu::PL011::new(pl011_qemu::UART1::take().unwrap());
/// ```
pub struct PL011<T> {
    /// owned copy of the underlying uart registers.
    ///
    /// Since they are
    /// moved into this struct, no one else can access them after the
    /// driver is initialized
    pub regs: T,
}

impl<T> PL011<T>
where
    T: Deref<Target = PL011_Regs>,
{
    /// Initialize a UART driver. Needs a UART struct to be passed in
    pub fn new(uart: T) -> Self {
        let pl011 = PL011 { regs: uart };
        pl011
    }
    /// writes a single byte out the uart
    ///
    /// spins until space is available in the fifo
    pub fn write_byte(&self, data: u8) {
        // while TXFF is set
        while self.regs.uartfr.read() & 0x20 > 0 {
            atomic::spin_loop_hint();
        }
        unsafe {
            self.regs.uartdr.write(data as u32);
        }
    }
    /// reads a single byte out the uart
    ///
    /// spins until a byte is available in the fifo
    pub fn read_byte(&self) -> u8 {
        // while RXFE is set
        while self.regs.uartfr.read() & 0x10 > 0 {
            atomic::spin_loop_hint();
        }
        (self.regs.uartdr.read() & 0xff) as u8
    }
}
impl<T> fmt::Write for PL011<T>
where
    T: Deref<Target = PL011_Regs>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.as_bytes().iter() {
            self.write_byte(*b);
        }
        Ok(())
    }
}



impl<T> serial::Read<u8> for PL011<T>
where
    T: Deref<Target = PL011_Regs>,
{
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        Ok(self.read_byte())
    }
}


impl<T> serial::Write<u8> for PL011<T>
where
    T: Deref<Target = PL011_Regs>,
{
    type Error = Error;
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word);
        Ok(())
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}

macro_rules! create_uart {
    (
        $(#[$attr:meta])* struct $uart:ident,
        $global:ident,
        $addr:literal
    ) => {
        $(#[$attr])*
            pub struct $uart {
                _marker: PhantomData<*const ()>,
            }
            unsafe impl Send for $uart {}
            impl Deref for $uart {
                type Target = PL011_Regs;
                #[inline(always)]
                fn deref(&self) -> &Self::Target {
                    unsafe {&*($addr as *const _)}
                }
            }
        static mut $global : bool = false;
        impl $uart {
            /// takes the UART HW
            ///
            /// Can only be taken once. If taken again returns None
            pub fn take() -> Option<Self> {
                    interrupt::free(|_| {
                        if unsafe {$global} {
                            None
                        } else {
                            Some(unsafe {$uart::steal()})
                        }
                    })
            }
            /// Unsafe function to steal hardware resources.
            ///
            /// Should not be needed if properly passing around the uart
            pub unsafe fn steal() -> Self {
                $global = true;
                $uart {_marker : PhantomData}
            }
        }
    }
}
create_uart!(
    /// Hardware Singleton for UART1 
    struct UART1,
    UART1_TAKEN, 0x4000_c000);
create_uart!(
    /// Hardware Singleton for UART2
    struct UART2,
    UART2_TAKEN, 0x4000_d000);
create_uart!(
    /// Hardware Singleton for UART3
    struct UART3,
    UART3_TAKEN, 0x4000_e000);
create_uart!(
    /// Hardware Singleton for UART4
    struct UART4,
    UART4_TAKEN, 0x4000_f000);
