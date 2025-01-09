//some Cortex-M3 things

pub inline fn enableInterrupts() void {
    asm volatile ("cpsie i" ::: "memory");
}

pub inline fn disableInterrupts() void {
    asm volatile ("cpsid i" ::: "memory");
}

pub inline fn set_MSP(value: u32) void {
    asm volatile ("MSR MSP, %[stack];"
        :
        : [stack] "r" (value),
    );
}

pub const VTOR: *volatile u32 = @ptrFromInt(0xE000ED08);
