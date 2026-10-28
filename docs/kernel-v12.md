# Kernel v12: 4 KB bulk reads

This branch preserves the v12 kernel experiment: 4096-byte bulk reads from the
Android Auto endpoint instead of the endpoint packet size (512 bytes), plus
read-path diagnostics.

## Changes

- `AA_BULK_READ_SLOT_SIZE` is used as the transfer length, so one completion
  fills one 4 KB ring slot. The 32-slot ring holds 128 KB instead of 16 KB.
- `vita_auto_usb.h` replaces the old stage-28 enum with
  `VITA_AUTO_USB_FAILURE_DETACH`, matching the detach-recovery session code.
- The diagnostics counters (`overflow`, `checksum`, `sequence`, ring depth)
  are populated from the read path.

## Result

Tested at 720p with VITA audio. v12 is the best kernel baseline so far, but it
does not fix the disconnect: 10 sessions, 7.94 MiB/session, median 28.2 s,
ending 6x `TlsReadFailed` and 4x `FrameParseFailed`. The controller still
aborts with `0x8024000A`; v12 makes that abort visible but cannot recover the
bytes it loses.

## Build

```
export VITASDK=/path/to/vitasdk
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

The kernel target produces `build/kernel/vita_auto_usb.skprx`, 14,844 bytes,
SHA-256 `F4AB7061E3CB69DD9AB5ACC6BACF19D2AB32BF02B886F49428A45A75B60B5F57`.

This is an experimental branch. `master` is unchanged.
