# Embedded Hal driver for QEMU serial ports

This crate provides basic drivers for the UARTS exposed by QEMU. You
can see the implementation of these
uarts
[here](https://github.com/qemu/qemu/blob/master/hw/arm/stellaris.c)

The QEMU target actually exposes 4 different UARTS, that can each be
redirected to arbitary character devices or files. This crate allows
those UARTS to be accessed in order to support more complicated use
cases than can be provided
by
[cortex_m_semihosting](https://crates.io/crates/cortex-m-semihosting).

# EXAMPLES

To run the examples, you will need an arm QEMU. An example can be seen below:

_The classic example_

```
$ cargo run --example hello
Hello, World!
```

_Another example, this time tee-ing the input to a file:_
```
$ cargo run --example tee -- -serial file:test.txt
testing 123
^D
$ cat test.txt
testing 123
```
