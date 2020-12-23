#![no_std]
#![no_main]

extern crate pl011_qemu;
use panic_halt as _; 
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug};
use core::fmt::Write;
#[entry]
fn main() -> ! {
    let mut uart1 = pl011_qemu::PL011::new(pl011_qemu::UART1::take().unwrap());
    let mut uart2 = pl011_qemu::PL011::new(pl011_qemu::UART2::take().unwrap());
    loop {
        let c = uart1.read_byte();
        if c == 4 {
            debug::exit(debug::EXIT_SUCCESS);
        }
        uart1.write_byte(c);
        uart2.write_byte(c);
    }    
}
