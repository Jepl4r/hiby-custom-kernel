#!/usr/bin/env python3
"""Compares a freshly built kernel with the stock one of its board.

Reads out/System.map-NAME and out/xImage-NAME and reports:
  * the version string the modules will be checked against;
  * every symbol the HiBy modules import that the new kernel does not export;
  * the symbols of the stock kernel missing from the new one, and the other way
    round, grouped by prefix -- the trail to follow when rebuilding the config;
  * whether the device tree inside the image is the stock one.
"""
import argparse, collections, gzip, os, re, struct, zlib


def load_map(path):
	syms = {}
	for line in open(path, errors="replace"):
		p = line.split()
		if len(p) >= 3:
			syms[p[2]] = (int(p[0], 16), p[1])
	return syms


def group(name):
	n = name.lstrip("_")
	n = re.sub(r"\.\d+$", "", n)
	head = re.split(r"[_.]", n, maxsplit=1)[0]
	return head or name


def interesting(name, kind):
	return kind.upper() in "TDBR" and "." not in name and not name.startswith(("__ksymtab", "__kstrtab", "__crc_"))


def text_kb(syms):
	a = syms.get("_text") or syms.get("_stext")
	b = syms.get("_etext")
	return (b[0] - a[0]) / 1024 if a and b else -1


def dtb_from_ximage(path):
	data = open(path, "rb").read()
	i = data.find(b"\x1f\x8b\x08")
	if i < 0:
		return None
	try:
		raw = zlib.decompressobj(31).decompress(data[i:])
	except zlib.error:
		return None
	for m in re.finditer(b"\xd0\x0d\xfe\xed", raw):
		size = struct.unpack(">I", raw[m.start() + 4:m.start() + 8])[0]
		if 1000 < size < 200000:
			return raw[m.start():m.start() + size]
	return None


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--kit", required=True)
	ap.add_argument("--tree", required=True)
	ap.add_argument("--name", required=True)
	ap.add_argument("--model", required=True)
	a = ap.parse_args()
	out = os.path.join(a.kit, "out")
	board = os.path.join(a.kit, "boards", a.model)

	rel = "?"
	try:
		rel = re.search(r'"(.*)"', open(os.path.join(a.tree, "include/generated/utsrelease.h")).read()).group(1)
	except Exception:
		pass
	print(f"version: {rel}   (the HiBy modules want 4.4.94+)")

	new = load_map(os.path.join(out, f"System.map-{a.name}"))
	stock_map = os.path.join(board, "stock.kallsyms")
	stock = load_map(stock_map) if os.path.exists(stock_map) else None
	if stock:
		print(f"text: new {text_kb(new):.0f} KB, stock {text_kb(stock):.0f} KB")
	else:
		print(f"text: new {text_kb(new):.0f} KB (no boards/{a.model}/stock.kallsyms to compare with)")

	need = [l.strip() for l in open(os.path.join(board, "modules-need.txt")) if l.strip()]
	missing = [s for s in need if "__ksymtab_" + s not in new]
	print(f"\nsymbols the HiBy modules import: {len(need)}, not exported by the new kernel: {len(missing)}")
	for s in missing:
		print(f"  MISSING {s}" + ("   (present but not exported)" if s in new else ""))

	if stock:
		sn = {n for n, (_, k) in stock.items() if interesting(n, k)}
		nn = {n for n, (_, k) in new.items() if interesting(n, k)}
		gone = collections.defaultdict(list)
		added = collections.defaultdict(list)
		for n in sn - nn:
			gone[group(n)].append(n)
		for n in nn - sn:
			added[group(n)].append(n)

		def show(title, groups):
			print(f"\n{title}: {sum(len(v) for v in groups.values())} symbols")
			for g, names in sorted(groups.items(), key=lambda kv: -len(kv[1]))[:45]:
				print(f"  {len(names):5d}  {g:24s} {' '.join(sorted(names)[:4])}")

		show("in the stock kernel, not in the new one", gone)
		show("in the new kernel, not in the stock one", added)

	dtb = dtb_from_ximage(os.path.join(out, f"xImage-{a.name}"))
	ref = open(os.path.join(board, "stock.dtb"), "rb").read()
	if dtb is None:
		print("\ndevice tree: not found inside the xImage (not appended, or another compression)")
	else:
		print(f"\ndevice tree: {len(dtb)} bytes, {'IDENTICAL to stock' if dtb == ref else 'DIFFERENT from stock'}")


main()
