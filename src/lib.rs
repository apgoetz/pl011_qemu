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

#![deny(missing_docs)]
#![no_std]
use core::fmt;
use core::marker::PhantomData;
use core::ops::Deref;
pub use cortex_m::interrupt;
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
pub struct PL011<UART> {
    /// owned copy of the underlying uart registers.
    ///
    ///Since they are moved into this struct, no one else can access them after the driver is initialized
    pub regs: UART,
    rx: Rx<UART>,
    tx: Tx<UART>,
}
/// Represents read half of UART.
pub struct Rx<UART> {
    _uart: PhantomData<UART>
}
/// Represents write half of UART
pub struct Tx<UART> {
    _uart: PhantomData<UART>
}



impl<T> Rx<T>
where
    T: detail::ConstRegBlockPtr<PL011_Regs>,
{
    /// reads a single byte out the uart
    ///
    /// spins until a byte is available in the fifo
    pub fn read_byte(&self) -> u8 {
        // loop while RXFE is set
        loop {
            // atomic read of register is side effect free
            let uartfr = unsafe { (*T::ptr()).uartfr.read() };
            if uartfr & 0x10 == 0 {
                break
            }
        }
        // read the data register. Atomic read is side effect free
        let data = unsafe {(*T::ptr()).uartdr.read() & 0xff};
        data as u8
    }    
}

impl<T> Tx<T>
where
    T: detail::ConstRegBlockPtr<PL011_Regs>,
{
    /// writes a single byte out the uart
    ///
    /// spins until space is available in the fifo
    pub fn write_byte(&self, data: u8) {
        loop {
            // atomic read of register is side effect free
            let uartfr = unsafe { (*T::ptr()).uartfr.read() };
            // if TXFF is not set
            if uartfr & 0x20 == 0 {
                break
            }
        }
        unsafe {(*T::ptr()).uartdr.write(data as u32)};
    }
}


impl<T> serial::Read<u8> for Rx<T>
    where
    T: detail::ConstRegBlockPtr<PL011_Regs>,
{
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        // atomic read of register is side effect free
        let uartfr = unsafe { (*T::ptr()).uartfr.read() };
        // if RXFE is set (rx fifo is empty)
        if uartfr & 0x10 > 0 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(unsafe {(*T::ptr()).uartdr.read() & 0xff} as u8)
        }
    }
}

impl<T> serial::Write<u8> for Tx<T>
    where
    T: detail::ConstRegBlockPtr<PL011_Regs>,
{
    type Error = Error;
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.flush()?;
        unsafe {(*T::ptr()).uartdr.write(word as u32)};
        Ok(())
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // atomic read of register is side effect free
        let uartfr = unsafe { (*T::ptr()).uartfr.read() };
        // if TXFF is set (transmit fifo full)
        if uartfr & 0x20 > 0 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }
}

impl<T> fmt::Write for Tx<T>
    where
    Tx<T>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        use embedded_hal::serial::Write;
        for b in s.as_bytes().iter() {
            if nb::block!(self.write(*b)).is_err() {
                return Err(fmt::Error);
            }
        }
        Ok(())
    }
}


impl<T> PL011<T>
    where
    T: detail::ConstRegBlockPtr<PL011_Regs>,
{
    /// Initialize a UART driver. Needs a UART struct to be passed in
    pub fn new(uart: T) -> Self {
        let pl011 = PL011 { regs: uart, tx: Tx{_uart:PhantomData}, rx: Rx{_uart:PhantomData} };
        pl011
    }
    /// splits a single PL011 uart object into separate Rx and Tx streams.
    ///
    /// Useful when you want to separate the two different halves of
    /// the UART and use them in different parts of the application
    ///
    /// Note that the Rx and Tx structs do not contain a reference to
    /// the underlying UART, this is because once you split the UART
    /// in half, it is no longer possible to interact with the uart
    /// hardware directly, since the other half (rx or tx) might be
    /// using it at the same time.
    pub fn split(self) -> (Tx<T>,Rx<T>) {
        (self.tx, self.rx)
    }
    /// writes a single byte out the uart
    ///
    /// spins until space is available in the fifo
    pub fn write_byte(&self, data: u8) {
        self.tx.write_byte(data);
    }
    /// reads a single byte out the uart
    ///
    /// spins until a byte is available in the fifo
    pub fn read_byte(&self) -> u8 {
        self.rx.read_byte()
    }
}

impl<T> fmt::Write for PL011<T>
    where
    Tx<T>: fmt::Write,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.tx.write_str(s)
    }
}

impl<T> serial::Read<u8> for PL011<T>
    where
    Rx<T>: serial::Read<u8, Error=Error>,
{
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}


impl<T> serial::Write<u8> for PL011<T>
    where
    Tx<T>: serial::Write<u8, Error=Error>,
{
    type Error = Error;
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

/// Create a new UART
#[macro_export]
macro_rules! create_uart {
    (
        $(#[$attr:meta])* struct $uart:ident,
        $global:ident,
        $addr:literal
    ) => {
        $(#[$attr])*
            pub struct $uart {
                _marker: core::marker::PhantomData<*const ()>,
            }
        impl $crate::detail::ConstRegBlockPtr<$crate::PL011_Regs> for $uart {
            /// returns a pointer to the register block
            fn ptr() -> *const $crate::PL011_Regs {
                $addr as *const _
            }
        }
        unsafe impl Send for $uart {}
        impl core::ops::Deref for $uart {
            type Target = $crate::PL011_Regs;
            #[inline(always)]
            fn deref(&self) -> &Self::Target {
                use $crate::detail::ConstRegBlockPtr;
                unsafe {&* $uart::ptr()}
            }
        }
        static mut $global : bool = false;
        impl $uart {
            /// takes the UART HW
            ///
            /// Can only be taken once. If taken again returns None
            pub fn take() -> Option<Self> {
                    $crate::interrupt::free(|_| {
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
                $uart {_marker: core::marker::PhantomData }
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

/// Implementation details for register block.
pub mod detail {
    /// Trait to indicate that the following type always points to a fixed hardware resource
    ///
    /// This trait is used since there is a uniqe type for each UART
    /// hardware component. If you have access to that type, you can
    /// access the registers of its underlying hardware through the
    /// type
    pub trait ConstRegBlockPtr<R> {
        /// returns a pointer to the fixed address of the underlying hardware resource  
        fn ptr() -> *const R where Self: Sized;
    }
}
