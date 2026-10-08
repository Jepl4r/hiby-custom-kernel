#!/usr/bin/env python3
"""Copies the HBC3000's FPGA configuration out of the stock sa_sound_hbc3000.ko.

    tools/hbc3000-firmware.py <stock sa_sound_hbc3000.ko> <out>

The open sa_sound_hbc3000.ko of boards/r3proii loads it with request_firmware()
from /lib/firmware/hbc3000.fw. It is HiBy's, so it is not in this repository:
it is the module's FS_data array, found through its symbol table.
"""
import struct, sys


def fs_data(elf):
	if elf[:6] != b"\x7fELF\x01\x01":
		sys.exit("not a 32-bit little-endian ELF")
	shoff, = struct.unpack_from("<I", elf, 0x20)
	shentsize, shnum = struct.unpack_from("<HH", elf, 0x2e)
	sh = [struct.unpack_from("<10I", elf, shoff + i * shentsize) for i in range(shnum)]
	for s in sh:
		if s[1] != 2:	# SHT_SYMTAB
			continue
		strtab = sh[s[6]][4]
		for o in range(0, s[5], 16):
			name, value, size, info, other, shndx = struct.unpack_from("<IIIBBH", elf, s[4] + o)
			end = elf.index(b"\0", strtab + name)
			if elf[strtab + name:end] == b"FS_data":
				start = sh[shndx][4] + value
				return elf[start:start + size]
	sys.exit("no FS_data: not the stock module?")


def main():
	if len(sys.argv) != 3:
		sys.exit(__doc__)
	data = fs_data(open(sys.argv[1], "rb").read())
	open(sys.argv[2], "wb").write(data)
	print("%s: %d bytes" % (sys.argv[2], len(data)))


main()
