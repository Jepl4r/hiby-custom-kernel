# Build environment for the HiBy X1600 kernel (4.4.94+), taken from
# MatthewBriggs/hiby-r1-linux-kernel-compiling (GPL-2.0).
#
# The SDK toolchain (mips-gcc720-glibc229) is made of x86-64 Linux binaries, so
# on Apple Silicon this runs as linux/amd64 under Rosetta. Debian bullseye pins
# a host toolchain old enough for a 4.4 kernel; snapshot.debian.org keeps the
# pin installable now that bullseye has left the live mirrors.
FROM --platform=linux/amd64 debian:bullseye

RUN printf '%s\n' \
        'deb http://snapshot.debian.org/archive/debian/20260815T000000Z bullseye main' \
        'deb http://snapshot.debian.org/archive/debian-security/20260815T000000Z bullseye-security main' \
        > /etc/apt/sources.list \
    && printf '%s\n' 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99no-check-valid-until \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential bc bison flex libssl-dev libncurses-dev u-boot-tools \
        device-tree-compiler python3 bzip2 lbzip2 rsync file ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /kit
