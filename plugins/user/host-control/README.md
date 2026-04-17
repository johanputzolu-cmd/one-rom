# Host Control

One ROM's Host Control plugin is a full implementation of the [ROM Bus Control Protocol (RBCP)](https://github.com/piersfinlayson/rom-bus-control-protocol), which enables bidirectional communication between a host computer and an RBCP-capable ROM emulator using only the ROM address and data buses — no additional hardware required.

This allows a host system to query and modify the state of the emulated ROM installed within it, allowing a wide range of applications, including:

- Remote debugging of code running on real retro systems
- ROM based bootloaders (think `grub` for the C64)
- Dynamic ROM patching for games, demos and other applications

## Building the Plugin

```bash
make
```

This creates `build/plugin_user.bin`, which can be loaded onto One ROM as a user plugin, enabling RBCP support

## Using the Plugin

The plugin is designed to be driven by the host system's CPU directly.  A [C64 kernal bootloader](https://github.com/piersfinlayson/rom-bus-control-protocol/tree/main/reference/host/6502/c64-boot) is available as part of the RBCP reference implementation.

To use, build the C64 kernal bootloader, and then install as the first non-plugin image on One ROM.  You will then need to follow it with one or more other C64 kernal images that you want to be able to switch between using the bootloader.
