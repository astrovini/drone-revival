#!/bin/sh
# build.sh — cross-compile a C source for the AR.Drone 2.0 (armv7 Cortex-A8, Linux 2.6.32).
#
# Usage:   ./build.sh [source.c]        (defaults to hello.c)
# Output:  a statically-linked armv7 Linux ELF next to the source, no extension.
#
# Toolchain: messense/macos-cross-toolchains  ->  armv7-unknown-linux-musleabihf
#   native macOS gcc that emits armv7-Linux ELF (a cross-compiler, not a VM).
# Static + musl = one self-contained file, no libc-version worries on the old rootfs.

set -eu

CC=armv7-unknown-linux-musleabihf-gcc
SRC="${1:-hello.c}"
OUT="$(basename "${SRC%.c}")"

if ! command -v "$CC" >/dev/null 2>&1; then
    echo "error: $CC not on PATH." >&2
    echo "install it with:  brew install armv7-unknown-linux-musleabihf" >&2
    exit 1
fi

# -static            : bundle libc, self-contained on the drone
# -march=armv7-a     : Cortex-A8 ISA
# -mfpu=neon         : the board has NEON/VFPv3 (musleabihf already implies hard-float)
# -O2                : reasonable optimization
set -x
"$CC" -static -march=armv7-a -mfpu=neon -O2 -o "$OUT" "$SRC"
set +x

echo "built: $OUT"
# Show what we actually produced (uses the toolchain's own readelf, runs on the Mac).
READELF=armv7-unknown-linux-musleabihf-readelf
if command -v "$READELF" >/dev/null 2>&1; then
    "$READELF" -h "$OUT" | grep -Ei 'class|machine|type'
fi
