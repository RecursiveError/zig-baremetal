const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .abi = .none,
        .cpu_model = std.zig.CrossTarget.CpuModel{ .explicit = &std.Target.arm.cpu.cortex_m3 },
        //.cpu_features_add = std.Target.arm.featureSet(&[_]std.Target.arm.Feature{std.Target.arm.Feature.v7m}),
    });
    const executable_name = "bootloader";

    const optimize = b.standardOptimizeOption(.{});
    const bootloader = b.addExecutable(.{
        .name = executable_name ++ ".elf",
        .target = target,
        .optimize = optimize,
        .link_libc = false,
        .linkage = .static,
        .single_threaded = true,
        .root_source_file = b.path("src/main.zig"),
    });
    bootloader.entry = .disabled;

    const startup = b.addObject(
        .{
            .name = "startup.o",
            .target = target,
            .optimize = optimize,
            .link_libc = false,
            .single_threaded = true,
            .root_source_file = b.path("src/startup.zig"),
        },
    );
    startup.entry = .disabled;

    bootloader.setLinkerScriptPath(b.path("stmf103.ld"));
    bootloader.link_gc_sections = true;
    bootloader.link_data_sections = true;
    bootloader.link_function_sections = true;

    bootloader.addObject(startup);

    // Produce .bin file from .elf
    const bin = b.addObjCopy(bootloader.getEmittedBin(), .{
        .format = .bin,
    });
    bin.step.dependOn(&bootloader.step);
    const copy_bin = b.addInstallBinFile(bin.getOutput(), executable_name ++ ".bin");
    b.default_step.dependOn(&copy_bin.step);

    // Produce .hex file from .elf
    const hex = b.addObjCopy(bootloader.getEmittedBin(), .{
        .format = .hex,
    });
    hex.step.dependOn(&bootloader.step);
    const copy_hex = b.addInstallBinFile(hex.getOutput(), executable_name ++ ".hex");
    b.default_step.dependOn(&copy_hex.step);
    b.installArtifact(bootloader);
}
