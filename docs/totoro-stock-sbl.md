# Stock SBL: verified kernel handoff (2026-11-04)

From the QEMU repository:

```sh
sh scripts/run-totoro-sbl.sh
# Optional graphical display:
sh scripts/run-totoro-sbl.sh -display gtk,gl=off
```

The launcher uses the original `../reference-firmware/sbl.bin`, direct entry
at 0x96800000, and the prepared `build/sbl-stock/main.img` + `oob.img`.
New checkouts use `local/flash/`; the legacy `build/sbl-stock/` pair is a fallback.
See [the main README](../README.md) for reproducible preparation and path overrides.
The generated flash is writable; original firmware files are not modified.
Ctrl-a x exits; Ctrl-a c opens the monitor. This is direct SBL entry, not mask
ROM emulation. No kernel is injected with -kernel or a second loader.

Both the original SBL and the S5360XXMK1 archive SBL were cold-started without
GDB or execution redirection. In each 10-second test the SBL read the boot
partition and execution reached Linux virtual PC 0xc01e47c0, LR 0xc0251c10,
with a kernel stack at 0xd2a2be00. UART reports AST_POWERON..
This establishes SBL-to-kernel handoff, not a complete Android userspace boot.

## QEMU changes and validation

`hw/arm/totoro-onenand-probe.h` now handles:

- Master/read/reset completion bits 15/7/4, and clearing interrupt status.
- Correct lock/unlock command codes and status.
- Separate OOB storage (`TOTORO_ONENAND_OOB`, 256 bytes per 4096-byte page).
  Reads/program/erase preserve the distinction between main data and OOB.
- OTP reads do not expose main-array contents.
- Synchronous cache-program command 0x7f.
- Program/erase completion bits even on read-only failure.

Built with `ninja -C build qemu-system-arm`. The MMIO regression passed:
`python3 scripts/test-totoro-onenand.py` (temporary images only, historical run).
The one-off test was subsequently archived locally under
`.local-artifacts/cleanup-2026-11-04/scripts/` and is not part of the published tree.
It checks reset, interrupt clear, lock status, cache program, main/OOB reads,
erase, and completion on read-only failures.

## Prepared flash provenance

Components were extracted from `../S5360XXMK1_DBTML1_SVC/` into
`build/sbl-stock/`, with sizes/hashes in `manifest.json`. PIT contains 2004
blocks of 256 KiB; 44 blocks remain for the reservoir. Component partitions
were placed sequentially, with PIT at block 17, boot at block 88, and backup
boot at block 108. This is a constructed emulator image, not a device dump.
STL filesystem/calibration completeness is not established.

A blank flash alone stops at SBL PC 0x9680d380 with BML error 0x90040000.
To provision metadata, the original SBL's FSR_BML_Format routine at
0x96845a28 was called through GDB on the scratch flash, with volume 0,
a FSRPART table derived from PIT in scratch RAM, and initial-format flag
0x10. It returned 0x10000000 and wrote reservoir metadata. This one-time
provisioning changes execution registers and scratch data, not SBL code;
it is separate from the verified cold boot. The diagnostic helper is
`build/sbl-stock/format-probe.py`; do not rerun it on a populated image.
The component payloads were then copied into the formatted main image.

Latest original-SBL cold-run evidence:
`build/sbl-stock/cold.qmp`, `cold.trace`, `cold.uart`.
Recheck with `python3 build/sbl-stock/cold-check.py ../reference-firmware/sbl.bin`.
These generated artifacts are under the ignored build directory: keep them
when cleaning builds, or the prepared flash will need to be regenerated.

Unmodified SBL SHA-256:

- Original: fdc8eecb28aa35d63c5286eeb645a3126ec2438d88e27fdd03b595fc8a6eb4d9
- S5360XXMK1: 530420d82e8e1321a86b5b2ef93013c3b8f3f6ecdc6f55fcb39d1f6824c184b4

Select the second with `TOTORO_SBL="$PWD/build/sbl-stock/Sbl.bin"`.
The model remains experimental: ECC, accurate NAND programming constraints,
and per-block lock enforcement are not fully modelled.

## Interactive SBL console

```sh
python3 scripts/run-totoro-sbl-console.py
```

Wait for `SBL>`, then use `help`, `printenv`, `showpart`, or `help <command>`.
Exit with Ctrl-a x. This launcher uses the original SBL only and validates its
SHA-256 before applying any address-specific debugger operations.

This is explicitly debugger-assisted entry, not a discovered stock key combo.
The launcher stops at the common initialized boot path (0x968116cc), enables
the runtime UART logging flag at 0x96a09a8c, sets r0 to a scratch input buffer,
and enters the stock command_loop at 0x96800494. It then detaches GDB.
The SBL file and its instructions are unchanged. Flash writes are disabled by
default; TOTORO_ONENAND_RW=1 enables writes to the supplied working image.

`python3 scripts/run-totoro-sbl-console.py --check` passed: the guest parsed
`help`, printed `Following commands are supported:`, listed its commands,
and returned to `SBL>`. The ordinary boot launcher remains separate.

## Guest-rendered param.lfs logo

The prepared working image now contains a real STL mapping for param_lfs
(partition 21), not a raw physical copy of its logical image. On 2026-11-04,
the stock FSR_STL_Format (0x9684bbb0), Open (0x9684c6cc), Write
(0x9684dccc), and close via j4fs_close (0x96820ce8) provisioned the original
765952-byte param.lfs. All returned zero. Preparation used the debugger only;
the subsequent cold boot did not change guest registers or memory.
Backups: build/sbl-stock/main.img.before-param and oob.img.before-param.
Provisioning diagnostic: build/sbl-stock/provision-param.py (do not rerun
on an image containing wanted param changes).

Cold-boot log now contains `loke_init: j4fs_open..success`. The SBL decoded
logo.jpg and sent its pixels to LCDC. QEMU now implements the DCS column/page
window commands (0x2a/0x2b), GRAM writes (0x2c), and PIO scanout. The existing
kernel framebuffer path remains selected when LCDC DMA is enabled.

Evidence: build/sbl-stock/sbl-logo.png (240x320 Samsung GALAXY Y Young
GT-S5360), sbl-logo.ppm and logo-sbl.log. The capture stopped at 0x96811774,
immediately after the ordinary logo drawing call returned. A breakpoint only
froze the screen for inspection; no host boot-param artwork or injected image
was used. The SBL code and original param.lfs were unchanged.

Run `sh scripts/run-totoro-sbl.sh -display gtk,gl=off` to see normal startup.
The image is experimental: persistent param.blk/switch_sel creation still
reports errors in the read-only diagnostic run; full writable J4FS behavior
and the other STL filesystem partitions are not validated.

## Reproducing this setup

Use `scripts/prepare-totoro-flash.py` and the detailed [README](../README.md).
The `build/sbl-stock/*.py` commands above record the original investigation;
they are local historical artifacts, not prerequisites for a fresh clone.
The maintained preparation helper creates a new directory and refuses to
overwrite an existing flash pair. Firmware and generated evidence are not
included in the repository.
