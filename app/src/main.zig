const std = @import("std");
const stm32f1xx = @import("stm32f1xx.zig");
const cortex_m3 = @import("cortexm3.zig");
const peripherals = stm32f1xx.devices.STM32F103.peripherals;
const CPU_CLOCK = 8_000_000;

const GPIOC = peripherals.GPIOC;
const RCC = peripherals.RCC;

//global time variable
var millis: u64 = 0;

fn delay(time: u32) void {
    const end = millis + time;
    //wait for millis
    while (millis < end) {
        std.mem.doNotOptimizeAway(end); //keep "end" in memory
    }
}

export fn main() noreturn {
    RCC.APB2ENR.IOPCEN = 1; //enable GPIOC clock
    GPIOC.CRH.MODE13 = 2; //output with max clock 2Mhz
    GPIOC.CRH.CNF13 = 0; //push pull

    //Enable System Tick
    cortex_m3.SCB.set_IRQPrio(15, 1);
    cortex_m3.SCB.enable_IRQ(15);
    init_SysTick(CPU_CLOCK / 1000); //1ms per tick
    cortex_m3.enableInterrupts();
    while (true) {
        GPIOC.ODR.ODR13 = 1;
        delay(1000);
        GPIOC.ODR.ODR13 = 0;
        delay(1000);
    }
}

fn init_SysTick(tick: u32) void {
    const sysTick = cortex_m3.SysTick;
    sysTick.CSR.* = 0; //disable SysTemTick
    sysTick.RVR.* = tick & 0x00FFFFFF; //set reload value
    sysTick.CVR.* = 0; //clear corrent value
    sysTick.CSR.* = 0b111; // enable Clock as the same as CPU(HSI) | enable IRQ on underflow | enable timer
}

//update millis every tick IRQ
export fn SystemTick_IRQ() callconv(.C) void {
    millis += 1;
}
