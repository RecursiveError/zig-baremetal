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

pub const SCB = struct {
    pub const VTOR: *volatile u32 = @ptrFromInt(0xE000ED08);
    pub const IPR: *volatile [59]u32 = @ptrFromInt(0xE000E400);
    pub const ISER: *volatile [7]u32 = @ptrFromInt(0xE000E100);
    pub const ICER: *volatile [7]u32 = @ptrFromInt(0xE000E180);
    pub const IABR: *volatile [7]u32 = @ptrFromInt(0xE000E300);

    pub fn set_IRQPrio(IRQn: u32, prio: u8) void {
        const IRQ_index = IRQn / 4;
        const IRQ_offset: u5 = @intCast((IRQn % 4) * 8);
        IPR[IRQ_index] &= ~(@as(u32, 0xFF) << IRQ_offset);
        IPR[IRQ_index] |= @as(u32, prio) << IRQ_offset;
    }

    pub fn enable_IRQ(IRQn: u32) void {
        const IRQ_index = IRQn / 32;
        const IRQ_offset: u5 = @intCast(IRQn % 32);

        ISER[IRQ_index] &= (@as(u32, 1) << IRQ_offset);
        ISER[IRQ_index] |= @as(u32, 1) << IRQ_offset;
    }

    pub fn disable_IRQ(IRQn: u32) void {
        const IRQ_index = IRQn / 32;
        const IRQ_offset: u5 = @intCast(IRQn % 32);

        ICER[IRQ_index] &= (@as(u32, 1) << IRQ_offset);
        ICER[IRQ_index] |= @as(u32, 1) << IRQ_offset;
    }

    pub fn read_IRQ_flag(IRQn: u32) bool {
        const IRQ_index = IRQn / 32;
        const IRQ_offset: u5 = @intCast(IRQn % 32);

        return IABR[IRQ_index] & (1 << IRQ_offset) != 0;
    }
};

pub const SysTick = struct {
    pub const CSR: *volatile u32 = @ptrFromInt(0xE000E010);
    pub const RVR: *volatile u32 = @ptrFromInt(0xE000E014);
    pub const CVR: *volatile u32 = @ptrFromInt(0xE000E018);
    pub const CALIB: *volatile u32 = @ptrFromInt(0xE000E01C);
};
