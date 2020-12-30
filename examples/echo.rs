#![no_std]
#![no_main]

extern crate pl011_qemu;
use panic_halt as _; 
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug};
use core::fmt::Write;
#[entry]
fn main() -> ! {
    let uart = pl011_qemu::PL011::new(pl011_qemu::UART1::take().unwrap());
    let (mut tx,rx) = uart.split();
    writeln!(tx, "Echo Test: Press ^D to exit").ok();        
    loop {
        let c = rx.read_byte();
        if c == 4 {
            debug::exit(debug::EXIT_SUCCESS);
        }
        tx.write_byte(c);
    }    

}
