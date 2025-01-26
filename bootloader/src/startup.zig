const cortexm3 = @import("cortexm3.zig");
const disableInterrupts = cortexm3.disableInterrupts;
const enableInterrupts = cortexm3.enableInterrupts;

extern fn main() noreturn;

extern var _sbss: u8;
extern var _ebss: u8;
extern var _sdata: u8;
extern var _edata: u8;
extern var _sidata: u8;
extern var __stack: anyopaque;

fn default_handler() callconv(.Naked) void {}

pub fn resetHandler() callconv(.Naked) noreturn {

    // Disable interrupts during initialization, although they should start disabled anyways
    disableInterrupts();
    @setRuntimeSafety(false);
    {
        const bss_start: [*]u8 = @ptrCast(&_sbss);
        const bss_end: [*]u8 = @ptrCast(&_ebss);
        const bss_len = @intFromPtr(bss_end) - @intFromPtr(bss_start);

        @memset(bss_start[0..bss_len], 0);
    }

    // load .data from flash
    {
        const data_start: [*]u8 = @ptrCast(&_sdata);
        const data_end: [*]u8 = @ptrCast(&_edata);
        const data_len = @intFromPtr(data_end) - @intFromPtr(data_start);
        const data_src: [*]const u8 = @ptrCast(&_sidata);

        @memcpy(data_start[0..data_len], data_src[0..data_len]);
    }

    @setRuntimeSafety(true);
    enableInterrupts();

    asm volatile ("B main");

    while (true) {}
}

pub const VectorTable = extern struct {
    const Handler = *const fn () callconv(.Naked) void;
    const unhandled = default_handler;

    initial_stack_pointer: *anyopaque,
    Reset: Handler = resetHandler,
    NMI: Handler = unhandled,
    HardFault: Handler = unhandled,
    MemManageFault: Handler = unhandled,
    BusFault: Handler = unhandled,
    UsageFault: Handler = unhandled,
    reserved5: [4]u32 = undefined,
    SVCall: Handler = unhandled,
    reserved10: [2]u32 = undefined,
    PendSV: Handler = unhandled,
    SysTick: Handler = unhandled,

    //device IRQ
    WWDG: Handler = unhandled,
    PVD: Handler = unhandled,
    TAMPER: Handler = unhandled,
    RTC: Handler = unhandled,
    FLASH: Handler = unhandled,
    RCC: Handler = unhandled,
    EXTI0: Handler = unhandled,
    EXTI1: Handler = unhandled,
    EXTI2: Handler = unhandled,
    EXTI3: Handler = unhandled,
    EXTI4: Handler = unhandled,
    DMA1_Channel1: Handler = unhandled,
    DMA1_Channel2: Handler = unhandled,
    DMA1_Channel3: Handler = unhandled,
    DMA1_Channel4: Handler = unhandled,
    DMA1_Channel5: Handler = unhandled,
    DMA1_Channel6: Handler = unhandled,
    DMA1_Channel7: Handler = unhandled,
    ADC1_2: Handler = unhandled,
    USB_HP_CAN1_TX: Handler = unhandled,
    USB_LP_CAN1_RX0: Handler = unhandled,
    CAN1_RX1: Handler = unhandled,
    CAN1_SCE: Handler = unhandled,
    EXTI9_5: Handler = unhandled,
    TIM1_BRK: Handler = unhandled,
    TIM1_UP: Handler = unhandled,
    TIM1_TRG_COM: Handler = unhandled,
    TIM1_CC: Handler = unhandled,
    TIM2: Handler = unhandled,
    TIM3: Handler = unhandled,
    reserved44: [1]u32 = undefined,
    I2C1_EV: Handler = unhandled,
    I2C1_ER: Handler = unhandled,
    reserved47: [2]u32 = undefined,
    SPI1: Handler = unhandled,
    reserved50: [1]u32 = undefined,
    USART1: Handler = unhandled,
    USART2: Handler = unhandled,
    reserved53: [1]u32 = undefined,
    EXTI15_10: Handler = unhandled,
    RTC_Alarm: Handler = unhandled,
    USBWakeUp: Handler = unhandled,
};

const vector_table: VectorTable = .{
    .initial_stack_pointer = @extern(*usize, .{ .name = "__stack" }),
};

comptime {
    @export(vector_table, .{
        .name = "vector_table",
        .section = ".isr_vector",
        .linkage = .strong,
    });
}
