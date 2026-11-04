#!/usr/bin/env python3
"""Enter the stock SBL command loop via QEMU GDB, without editing SBL code.

This is a debugger-assisted console, not the stock automatic boot path.
Changes only PC, r0 and the runtime UART logging flag after device init.
"""
import argparse
import hashlib
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = 'fdc8eecb28aa35d63c5286eeb645a3126ec2438d88e27fdd03b595fc8a6eb4d9'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--check', action='store_true', help='run help and exit')
args = parser.parse_args()
sbl = Path(os.environ.get('TOTORO_SBL', ROOT.parent / 'reference-firmware/sbl.bin')).resolve()
qemu = Path(os.environ.get('QEMU_BINARY', ROOT / 'build/qemu-system-arm')).resolve()
flash = Path(os.environ.get('TOTORO_FLASH_DIR', ROOT / 'local/flash'))
if 'TOTORO_FLASH_DIR' not in os.environ and not (flash / 'main.img').is_file():
    flash = ROOT / 'build/sbl-stock'
if not sbl.is_file() or not qemu.is_file():
    sys.exit('Missing QEMU or SBL: see README.md for setup.')
if ',' in str(sbl):
    sys.exit('SBL path must not contain a comma (QEMU loader syntax).')
if hashlib.sha256(sbl.read_bytes()).hexdigest() != EXPECTED:
    sys.exit('Unsupported SBL hash: debugger addresses are specific to the original image.')
env = {**os.environ, 'TOTORO_ONENAND_PROBE': '1'}
env.setdefault('TOTORO_ONENAND_IMAGE', str(flash / 'main.img'))
env.setdefault('TOTORO_ONENAND_OOB', str(flash / 'oob.img'))
env.setdefault('TOTORO_ONENAND_RW', '0')
for key in ('TOTORO_ONENAND_IMAGE', 'TOTORO_ONENAND_OOB'):
    if not Path(env[key]).is_file():
        sys.exit(f'Missing prepared flash: {env[key]}')
a, b = socket.socketpair()
a.settimeout(20)
command = [str(qemu), '-M', 'totoro',
    '-display', 'none', '-serial', 'stdio' if args.check else 'mon:stdio',
    '-monitor', 'none', '-S', '-chardev', f'socket,id=dbg,fd={b.fileno()}',
    '-gdb', 'chardev:dbg', '-device',
    f'loader,file={sbl},addr=0x96800000,cpu-num=0,force-raw=on']
print('SBL debug console: original code; runtime UART logging enabled. Ctrl-a x exits.', flush=True)
p = subprocess.Popen(command, env=env, pass_fds=[b.fileno()],
    stdin=subprocess.PIPE if args.check else None,
    stdout=subprocess.PIPE if args.check else None)
b.close()
def receive():
    while True:
        c = a.recv(1)
        if not c:
            raise RuntimeError('GDB disconnected')
        if c == b'$':
            break
    data = bytearray()
    while True:
        c = a.recv(1)
        if not c:
            raise RuntimeError('GDB disconnected')
        if c == b'#':
            break
        data += c
    checksum = b''
    while len(checksum) < 2:
        chunk = a.recv(2 - len(checksum))
        if not chunk:
            raise RuntimeError('GDB disconnected')
        checksum += chunk
    if int(checksum, 16) != sum(data) % 256:
        raise RuntimeError('GDB checksum mismatch')
    a.sendall(b'+')
    return bytes(data)
def send(command):
    payload = command.encode()
    a.sendall(b'$' + payload + b'#' + f'{sum(payload)%256:02x}'.encode())
    return receive()
def ok(command):
    result = send(command)
    if result != b'OK':
        raise RuntimeError(f'{command[:40]}: {result!r}')
try:
    send('?')
    # Common initialized boot path, before the boot-mode dispatch/kernel load.
    ok('Z1,968116cc,4')
    stop = send('c')
    registers = bytearray.fromhex(send('g').decode())
    pc = struct.unpack_from('<I', registers, 60)[0]
    if pc != 0x968116cc:
        raise RuntimeError(f'Unexpected stop PC {pc:#x}: {stop!r}')
    ok('z1,968116cc,4')
    # SBL's serial-output gate, normally loaded from the log_enable file.
    ok('M96a09a8c,1:01')
    struct.pack_into('<I', registers, 0, 0x96000000)  # command input buffer
    struct.pack_into('<I', registers, 60, 0x96800494)  # stock command_loop
    ok('G' + registers.hex())
    ok('D')  # detach resumes execution; no debugger needed during the console
    a.close()
    if args.check:
        try:
            output, _ = p.communicate(b'help\r', timeout=5)
        except subprocess.TimeoutExpired:
            p.terminate()
            output, _ = p.communicate(timeout=5)
        print(output.decode(errors='replace'))
        if b'Following commands are supported' not in output:
            raise RuntimeError('Stock help response was not observed')
        print('PASS: stock SBL parsed help and returned its command list')
    else:
        p.wait()
finally:
    a.close()
    if p.poll() is None:
        p.terminate()
        p.wait(timeout=5)
