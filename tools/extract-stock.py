#!/usr/bin/env python3
"""Takes a stock kernel apart for the build: its device tree and, when
vmlinux-to-elf is installed, its symbol table.

    tools/extract-stock.py MODEL <stock firmware .upt | stock xImage | mtd dump>

Writes, under boards/MODEL/ in this repo:
  stock.dtb        the board's device tree (the build needs it)
  vmlinux-stock    the decompressed kernel
  stock.kallsyms   its symbols, for tools/compare.py (optional)

A .upt is opened with 7z (p7zip). The kernel is found in it as the
xImage.NNNN.<md5> chunks of the OTA folder, joined in order. A kernel taken
straight from the device (dd if=/dev/mtd1) works as well.
"""
import glob, os, re, shutil, struct, subprocess, sys, tempfile, zlib

KIT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def ximage_from_upt(path, tmp):
	if not shutil.which("7z"):
		sys.exit("7z is needed to open a .upt (macOS: brew install p7zip, Debian: apt install p7zip-full)")
	subprocess.run(["7z", "x", "-y", "-o" + tmp, path], check=True, stdout=subprocess.DEVNULL)
	chunks = []
	for f in glob.glob(os.path.join(tmp, "**", "xImage.*"), recursive=True):
		m = re.match(r"xImage\.(\d+)\.", os.path.basename(f))
		if m:
			chunks.append((int(m.group(1)), f))
	if not chunks:
		sys.exit("no xImage.NNNN.* chunks in the .upt")
	return b"".join(open(f, "rb").read() for _, f in sorted(chunks))


def kernel_from_ximage(data):
	i = data.find(b"\x1f\x8b\x08")
	if i < 0:
		sys.exit("no gzip payload in the image: not a stock xImage?")
	return zlib.decompressobj(31).decompress(data[i:])


def dtb_from_kernel(raw):
	for m in re.finditer(b"\xd0\x0d\xfe\xed", raw):
		size = struct.unpack(">I", raw[m.start() + 4:m.start() + 8])[0]
		if 1000 < size < 200000:
			return raw[m.start():m.start() + size]
	sys.exit("no device tree in the kernel")


def main():
	if len(sys.argv) != 3:
		sys.exit(__doc__)
	model, src = sys.argv[1], sys.argv[2]
	rel = os.path.join("boards", model)
	OUT = os.path.join(KIT, rel)
	os.makedirs(OUT, exist_ok=True)
	with tempfile.TemporaryDirectory() as tmp:
		if src.lower().endswith(".upt"):
			image = ximage_from_upt(src, tmp)
		else:
			image = open(src, "rb").read()
	raw = kernel_from_ximage(image)

	vmlinux = os.path.join(OUT, "vmlinux-stock")
	open(vmlinux, "wb").write(raw)
	dtb = dtb_from_kernel(raw)
	open(os.path.join(OUT, "stock.dtb"), "wb").write(dtb)
	print(f"{rel}/stock.dtb: {len(dtb)} bytes")

	finder = shutil.which("kallsyms-finder")
	if not finder:
		print(f"{rel}/stock.kallsyms: skipped (pip install vmlinux-to-elf for kallsyms-finder)")
		return
	res = subprocess.run([finder, vmlinux], capture_output=True, text=True, errors="replace")
	lines = [l for l in res.stdout.splitlines() if re.match(r"^[0-9a-fA-F]{8,16} [A-Za-z] \S+$", l.strip())]
	open(os.path.join(OUT, "stock.kallsyms"), "w").write("\n".join(l.strip() for l in lines) + "\n")
	print(f"{rel}/stock.kallsyms: {len(lines)} symbols")


main()
