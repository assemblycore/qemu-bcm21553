#!/usr/bin/env python3
"""Build a new emulator flash from the S5360XXMK1 service package.

Calls the original SBL's BML/STL routines through QEMU's GDB stub.
Never overwrites an existing output directory or modifies input firmware.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]
SBL_SHA256 = "fdc8eecb28aa35d63c5286eeb645a3126ec2438d88e27fdd03b595fc8a6eb4d9"
BLOCK = 256 * 1024
EXPECTED_LAYOUT = [
    ("bcm_boot", 0, 1), ("loke", 1, 8), ("loke_bk", 2, 8),
    ("systemdata", 3, 1), ("modem", 4, 50), ("param_lfs", 21, 20),
    ("boot", 5, 20), ("boot_backup", 6, 20), ("system", 20, 920),
    ("cache", 22, 160), ("userdata", 23, 789), ("efs", 7, 1),
    ("sysparm_dep", 8, 1), ("umts_cal", 9, 1), ("cal", 10, 4),
]
COMPONENTS = {
    "bcmboot.img", "sbl.bin", "bcmcp.img", "param.lfs", "boot.img",
    "system.img", "csc.rfs", "userdata.img",
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def parse_pit(data):
    if len(data) < 28 or struct.unpack_from("<II", data) != (0x12349876, 15):
        raise ValueError("Expected the 15-entry totoro PIT")
    if len(data) != 28 + 132 * 15:
        raise ValueError("Unexpected PIT length")
    parts, start = [], 0
    for index in range(15):
        entry = data[28 + 132 * index:28 + 132 * (index + 1)]
        fields = struct.unpack_from("<9I", entry)
        name = entry[36:68].split(b"\0")[0].decode("ascii")
        filename = entry[68:100].split(b"\0")[0].decode("ascii")
        if (name, fields[2], fields[6]) != EXPECTED_LAYOUT[index] or fields[5] != 256:
            raise ValueError(f"Unsupported PIT entry: {name}")
        parts.append(dict(name=name, id=fields[2], attributes=fields[3],
                          start_block=start, blocks=fields[6], file=filename))
        start += fields[6]
    return parts


class Guest:
    def __init__(self, qemu, sbl, output, phase):
        self.sock, peer = socket.socketpair()
        self.sock.settimeout(60)
        self.log = (output / f"{phase}.log").open("wb")
        env = {**os.environ, "TOTORO_ONENAND_PROBE": "1",
               "TOTORO_ONENAND_IMAGE": str(output / "main.img"),
               "TOTORO_ONENAND_OOB": str(output / "oob.img"),
               "TOTORO_ONENAND_RW": "1"}
        try:
            self.proc = subprocess.Popen([
                str(qemu), "-M", "totoro", "-display", "none",
                "-serial", "stdio", "-monitor", "none", "-S",
                "-chardev", f"socket,id=dbg,fd={peer.fileno()}",
                "-gdb", "chardev:dbg", "-device",
                f"loader,file={sbl},addr=0x96800000,cpu-num=0,force-raw=on",
            ], env=env, pass_fds=[peer.fileno()], stdin=subprocess.DEVNULL,
                stdout=self.log, stderr=self.log)
        except BaseException:
            self.sock.close()
            self.log.close()
            raise
        finally:
            peer.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.sock.close()
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.log.close()

    def read(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise RuntimeError("QEMU GDB disconnected; inspect phase log")
            data.extend(chunk)
        return bytes(data)

    def send(self, command):
        payload = command.encode("ascii")
        self.sock.sendall(b"$" + payload + b"#" + f"{sum(payload) % 256:02x}".encode())
        while self.read(1) != b"$":
            pass
        reply = bytearray()
        while (byte := self.read(1)) != b"#":
            reply.extend(byte)
        if int(self.read(2), 16) != sum(reply) % 256:
            raise RuntimeError("GDB checksum mismatch")
        self.sock.sendall(b"+")
        return bytes(reply)

    def ok(self, command):
        reply = self.send(command)
        if reply != b"OK":
            raise RuntimeError(f"GDB {command[:48]}: {reply!r}")

    def regs(self):
        return bytearray.fromhex(self.send("g").decode())

    def at(self, address):
        self.send("?")
        self.ok(f"Z1,{address:x},4")
        self.send("c")
        regs = self.regs()
        pc = struct.unpack_from("<I", regs, 60)[0]
        if pc != address:
            raise RuntimeError(f"Unexpected SBL stop {pc:#x}, expected {address:#x}")
        return regs

    def write(self, address, data):
        for offset in range(0, len(data), 512):
            chunk = data[offset:offset + 512]
            self.ok(f"M{address + offset:x},{len(chunk):x}:" + chunk.hex())

    def call(self, saved, address, args, ret, expected):
        regs = saved[:]
        for index, value in enumerate(args[:4]):
            struct.pack_into("<I", regs, index * 4, value)
        for index, value in [(13, 0x95fff000), (14, ret), (15, address)]:
            struct.pack_into("<I", regs, index * 4, value)
        if len(args) > 4:
            self.write(0x95fff000, struct.pack("<" + "I" * len(args[4:]), *args[4:]))
        self.ok("G" + regs.hex())
        self.send("c")
        result = self.regs()
        r0, pc = struct.unpack_from("<I", result)[0], struct.unpack_from("<I", result, 60)[0]
        if pc != ret or r0 != expected:
            raise RuntimeError(f"SBL call {address:#x}: r0={r0:#x}, pc={pc:#x}")
        print(f"  SBL {address:#x} returned {r0:#x}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, required=True, help="service package directory")
    parser.add_argument("--pit", type=Path, help="default: FIRMWARE/totoro_0623.pit")
    parser.add_argument("--sbl", type=Path, default=Path(os.environ.get(
        "TOTORO_SBL", ROOT.parent / "reference-firmware/sbl.bin")))
    parser.add_argument("--qemu", type=Path, default=Path(os.environ.get(
        "QEMU_BINARY", ROOT / "build/qemu-system-arm")))
    parser.add_argument("--output", type=Path, default=ROOT / "local/flash",
                        help="must not already exist")
    args = parser.parse_args()
    sbl, qemu, output = args.sbl.resolve(), args.qemu.resolve(), args.output.resolve()
    if digest(sbl.read_bytes()) != SBL_SHA256:
        parser.error("Unsupported original SBL hash; formatter addresses are version-specific")
    if not qemu.is_file() or not os.access(qemu, os.X_OK):
        parser.error(f"Build QEMU first: {qemu}")
    if "," in str(sbl):
        parser.error("SBL path must not contain a comma (QEMU loader syntax)")
    if output.exists():
        parser.error(f"Output already exists; choose a NEW directory: {output}")
    pit_data = (args.pit or args.firmware / "totoro_0623.pit").read_bytes()
    parts = parse_pit(pit_data)
    # Read selected regular files only; no archive paths or links are extracted.
    payloads = {}
    for archive in sorted(args.firmware.glob("*.tar.md5")):
        with tarfile.open(archive, "r:") as tar:
            for member in tar:
                name = Path(member.name).name.lower()
                if name not in COMPONENTS or not member.isfile():
                    continue
                if name in payloads:
                    raise ValueError(f"Duplicate firmware component: {name}")
                if member.size > 256 * 1024 * 1024:
                    raise ValueError(f"Component too large: {name}")
                payloads[name] = tar.extractfile(member).read()
    if COMPONENTS - payloads.keys():
        parser.error(f"Missing components: {sorted(COMPONENTS - payloads.keys())}")
    payloads["totoro.pit"] = pit_data
    if len(payloads["param.lfs"]) % 512:
        parser.error("param.lfs must contain whole 512-byte sectors")
    for part in parts:
        filename = {"loke_bk": "sbl.bin", "boot_backup": "boot.img"}.get(
            part["name"], part["file"].lower())
        part["payload"] = filename if filename in payloads else None
        if filename in payloads and len(payloads[filename]) > part["blocks"] * BLOCK:
            parser.error(f"Component does not fit: {filename}")
    output.mkdir(parents=True)
    manifest = dict(status="incomplete", sbl_sha256=SBL_SHA256, block_bytes=BLOCK,
                    partitions=parts, files={name: dict(size=len(data), sha256=digest(data))
                                             for name, data in payloads.items()})
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print("Creating 512 MiB main + 32 MiB OOB, erased state 0xff", flush=True)
    for name, megabytes in [("main.img", 512), ("oob.img", 32)]:
        with (output / name).open("xb") as dest:
            for _ in range(megabytes):
                dest.write(b"\xff" * (1024 * 1024))
    print("Formatting BML with the original SBL", flush=True)
    with Guest(qemu, sbl, output, "format-bml") as guest:
        saved = guest.at(0x9680d380)
        table = b"FSRPART\0" + struct.pack("<II", 0x10000, len(parts))
        for part in parts:
            attr = 0x1101 if part["attributes"] == 2 else 0x1002
            table += struct.pack("<4H2I", part["id"], attr,
                                 part["start_block"], part["blocks"], 0, 0)
        guest.write(0x96000000, table)
        guest.call(saved, 0x96845a28, [0, 0x96000000, 0x10], 0x9680d380, 0x10000000)
    print("Placing PIT and firmware components", flush=True)
    with (output / "main.img").open("r+b") as dest:
        for part in parts:
            if part["payload"] is not None:
                dest.seek(part["start_block"] * BLOCK)
                dest.write(payloads[part["payload"]])
    print("Formatting STL and writing logical param.lfs", flush=True)
    with Guest(qemu, sbl, output, "format-param") as guest:
        saved = guest.at(0x968116cc)
        guest.write(0x96000000, struct.pack("<6I", 1, 0, 0, 0, 0, 0))
        guest.call(saved, 0x9684bbb0, [0, 21, 0x96000000], 0x968116cc, 0)
        guest.call(saved, 0x9684c6cc, [0, 21, 0x96000100, 0], 0x968116cc, 0)
        data = payloads["param.lfs"]
        guest.write(0x96100000, data)
        guest.call(saved, 0x9684dccc, [0, 21, 0, len(data) // 512, 0x96100000, 0],
                   0x968116cc, 0)
        guest.call(saved, 0x96820ce8, [], 0x968116cc, 0)
    manifest["status"] = "complete"
    manifest["note"] = "Constructed emulator flash; only param_lfs has provisioned STL."
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Ready: {output}\nLaunch with TOTORO_FLASH_DIR={output}", flush=True)


if __name__ == "__main__":
    main()
