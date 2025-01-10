const std = @import("std");
const stm32f1xx = @import("stm32f1xx.zig");
const cortex_m3 = @import("cortexm3.zig");
const Driver = @import("ESPAT");
const Client = Driver.Client;
const EspAT = Driver.EspAT;
const peripherals = stm32f1xx.devices.STM32F103.peripherals;
const CPU_CLOCK = 8_000_000;

const GPIOC = peripherals.GPIOC;
const GPIOA = peripherals.GPIOA;
const RCC = peripherals.RCC;
const UART = peripherals.USART1;

//global time variable
var millis: u64 = 0;

const wifi_ssid = "GUSTAVO";
const wifi_password = "anjos2018";
const server_port: u16 = 80;

var rv_internal_buf: [1024]u8 = undefined;
fn rx_callback(free_size: usize, user_data: ?*anyopaque) []u8 {
    _ = user_data;
    @memset(&rv_internal_buf, 0);
    const read = @min(rv_internal_buf.len, free_size);
    const bytes = uart_read_blocking(rv_internal_buf[0..read], 100);
    return rv_internal_buf[0..bytes];
}

fn TX_callback(data: []const u8, _: ?*anyopaque) void {
    uart_transmite_blocking(data);
}

fn result_callback(result: Driver.ReponseEvent, cmd: Driver.Commands, user_data: ?*anyopaque) void {
    _ = user_data;
    _ = result;
    _ = cmd;
}

fn WiFi_callback(event: Driver.WifiEvent, user_data: ?*anyopaque) void {
    _ = user_data;
    _ = event;
}

//tcp server example
var send_buffer: [2046]u8 = undefined;
fn echo_server(client: Client, user_data: ?*anyopaque) void {
    _ = user_data;
    switch (client.event) {
        .DataReport => |_| {
            client.accept() catch error_h();
        },
        .ReadData => |data| {
            std.mem.copyForwards(u8, &send_buffer, data);
            client.send(@constCast(send_buffer[0..data.len])) catch error_h();
            //client.close() catch error_h();
        },
        else => {},
    }
}

//UDP echo example
var req_buf_udp: [20]u8 = undefined;
var remote_host: [50]u8 = undefined;
fn udp_callback(client: Client, user_data: ?*anyopaque) void {
    _ = user_data;
    switch (client.event) {
        .ReadData => |data| {
            const host_len = client.remote_host.?.len;
            std.mem.copyForwards(u8, &remote_host, client.remote_host.?);
            const msg = std.fmt.bufPrint(&req_buf_udp, "{d}\r\n", .{data.len}) catch unreachable;
            client.send_to(msg, remote_host[0..host_len], client.remote_port.?) catch error_h();
        },
        else => {},
    }
}

const STA_config = Driver.WiFiSTAConfig{
    .ssid = wifi_ssid,
    .pwd = wifi_password,
    .wifi_protocol = .{
        .@"802.11b" = 1,
        .@"802.11g" = 1,
        .@"802.11n" = 1,
    },

    .wifi_ip = .{ .static = .{ .ip = "192.168.15.37" } },
};

const AP_config = Driver.WiFiAPConfig{
    .ssid = "banana",
    .channel = 5,
    .ecn = .OPEN,
    .wifi_protocol = .{
        .@"802.11b" = 1,
        .@"802.11g" = 1,
        .@"802.11n" = 1,
    },
    .mac = "00:C0:DA:F0:F0:00",
    .wifi_ip = .{ .DHCP = {} },
};

const config_udp = Driver.ConnectConfig{
    .recv_mode = .active,
    .remote_host = "0.0.0.0",
    .remote_port = 1234,
    .config = .{
        .udp = .{
            .local_port = 1234,
            .mode = .Change_all,
        },
    },
};

const server_config = Driver.ServerConfig{
    .recv_mode = .passive,
    .callback = echo_server,
    .user_data = null,
    .server_type = .TCP,
    .port = server_port,
    .timeout = 2600,
};

export fn main() noreturn {
    cortex_m3.disableInterrupts();
    //Enable System Tick
    cortex_m3.SCB.set_IRQPrio(15, 1);
    cortex_m3.SCB.enable_IRQ(15);
    init_SysTick(CPU_CLOCK / 1000); //1ms per tick
    init_UART();

    var my_drive = EspAT(.{ .TX_event_pool = 30 }).init(TX_callback, rx_callback, null);
    defer my_drive.deinit_driver();

    my_drive.set_WiFi_event_handler(WiFi_callback, null);
    my_drive.set_response_event_handler(result_callback, null);

    my_drive.init_driver() catch error_h();
    my_drive.set_WiFi_mode(Driver.WiFiDriverMode.AP_STA) catch error_h();
    my_drive.set_network_mode(Driver.NetworkDriveMode.SERVER_CLIENT) catch error_h();
    my_drive.WiFi_config_AP(AP_config) catch error_h();
    my_drive.WiFi_connect_AP(STA_config) catch error_h();

    const id_udp = my_drive.bind(udp_callback, null) catch error_h();
    my_drive.connect(id_udp, config_udp) catch error_h();
    my_drive.create_server(server_config) catch error_h();

    cortex_m3.enableInterrupts();
    while (true) {
        my_drive.process() catch error_h();
    }
}

fn init_SysTick(tick: u32) void {
    const sysTick = cortex_m3.SysTick;
    sysTick.CSR.* = 0; //disable SysTemTick
    sysTick.RVR.* = tick & 0x00FFFFFF; //set reload value
    sysTick.CVR.* = 0; //clear corrent value
    sysTick.CSR.* = 0b111; // enable Clock as the same as CPU(HSI) | enable IRQ on underflow | enable timer
}

fn delay(time: u32) void {
    const end = millis + time;
    //wait for millis
    while (millis < end) {
        std.mem.doNotOptimizeAway(end); //keep "end" in memory
    }
}

fn init_UART() void {
    RCC.APB2ENR.USART1EN = 1; //enable USART1 clock
    RCC.APB2ENR.IOPAEN = 1;
    GPIOA.CRH.MODE9 = 1;
    GPIOA.CRH.CNF9 = 0b10;

    //GPIOA.CRH.MODE10 = 1;
    //GPIOA.CRH.CNF10 = 0b10;

    GPIOA.CRH.MODE12 = 1;
    GPIOA.CRH.CNF12 = 0b10;

    //CLOCK = 8Mhz
    //BRR = 115200
    //8000000/(16*115200);
    //H_BRR = 4 = 0x004
    //L_BRR = 5 = 0x5

    UART.BRR.DIV_Fraction = 0x5;
    UART.BRR.DIV_Mantissa = 0x4;
    UART.CR1.TE = 1;
    UART.CR1.RE = 1;
    UART.CR3.RTSE = 1;
    UART.CR1.UE = 1;
}

fn uart_transmite_blocking(data: []const u8) void {
    for (data) |c| {
        while (UART.SR.TC != 1) {}
        UART.DR.DR = @intCast(c);
        while (UART.SR.TXE != 1) {}
    }
}

fn uart_read_blocking(data: []u8, timeout: u64) usize {
    var bytes: usize = 0;
    for (data) |*c| {
        const deadline = millis + timeout;
        while (UART.SR.RXNE != 1) {
            std.mem.doNotOptimizeAway(deadline);
            if (millis > deadline) return bytes;
        }
        c.* = @intCast(UART.DR.DR);
        bytes += 1;
    }
    return bytes;
}

//update millis every tick IRQ
export fn SystemTick_IRQ() callconv(.C) void {
    millis += 1;
}

fn error_h() noreturn {
    RCC.APB2ENR.IOPCEN = 1; //enable GPIOC clock
    GPIOC.CRH.MODE13 = 2; //output with max clock 2Mhz
    GPIOC.CRH.CNF13 = 0; //push pull

    while (true) {
        GPIOC.ODR.ODR13 = 1;
        delay(100);
        GPIOC.ODR.ODR13 = 0;
        delay(100);
    }
}
