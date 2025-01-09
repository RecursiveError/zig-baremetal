const cortexm3 = @import("cortexm3.zig");

const app_addr: u32 = 0x08005000; //where the app start in memory
const app_handler = *const fn () callconv(.C) void; // app resetHandler signature

//more easy than calc the addrs
const app_struct = struct {
    stack_p: u32,
    reset: app_handler,
};

export fn main() noreturn {
    cortexm3.disableInterrupts();

    //bootloader stuff go here
    cortexm3.VTOR.* = app_addr;
    //load APP
    const app: *volatile app_struct = @ptrFromInt(app_addr);
    cortexm3.set_MSP(app.stack_p);
    app.reset();
    //should be unreachable

    while (true) {}
}
