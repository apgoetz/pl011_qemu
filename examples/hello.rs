#![no_std]
#![no_main]

// pick a panicking behavior
extern crate pl011_qemu;
use panic_halt as _; 
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug};
use core::fmt::Write;
#[entry]
fn main() -> ! {
    let mut uart = pl011_qemu::PL011::new(pl011_qemu::UART1::take().unwrap());
    writeln!(uart, "Hello, World!").ok();        
    loop {
            debug::exit(debug::EXIT_SUCCESS);
    }    
}
