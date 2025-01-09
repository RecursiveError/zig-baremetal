# Zig baremetal

this is an example of Zig-baremetal for STM32F103

[linkerscript source](https://github.com/haydenridd/stm32-baremetal-zig/tree/main)


bootloader: a simple bootloader that just configures the VTOR and jumps to the application.

app: a basic example of blinky using systick.