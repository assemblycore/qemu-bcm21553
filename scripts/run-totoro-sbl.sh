#!/bin/sh
# Direct entry into unmodified stock SBL; boot.img is read by the guest from flash.
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
qemu=${QEMU_BINARY:-$repo/build/qemu-system-arm}
sbl=${TOTORO_SBL:-$repo/../reference-firmware/sbl.bin}
flash=${TOTORO_FLASH_DIR:-$repo/local/flash}
if [ -z "${TOTORO_FLASH_DIR:-}" ] && [ ! -f "$flash/main.img" ]; then
    flash=$repo/build/sbl-stock
fi
export TOTORO_ONENAND_PROBE=1
if [ -z "${TOTORO_ONENAND_IMAGE:-}" ]; then
    export TOTORO_ONENAND_IMAGE="$flash/main.img"
    export TOTORO_ONENAND_OOB="${TOTORO_ONENAND_OOB:-$flash/oob.img}"
    export TOTORO_ONENAND_RW="${TOTORO_ONENAND_RW:-1}"
else
    export TOTORO_ONENAND_RW="${TOTORO_ONENAND_RW:-0}"
fi
for file in "$qemu" "$sbl" "$TOTORO_ONENAND_IMAGE" "${TOTORO_ONENAND_OOB:-}"; do
    if [ ! -f "$file" ]; then
        echo "Missing file: $file (see README.md for build/flash preparation)" >&2
        exit 1
    fi
done
case "$sbl" in
    *,*) echo "SBL path must not contain a comma (QEMU loader syntax)" >&2; exit 1 ;;
esac
exec "$qemu" -M totoro -display none -serial mon:stdio \
    -device "loader,file=$sbl,addr=0x96800000,cpu-num=0,force-raw=on" "$@"
