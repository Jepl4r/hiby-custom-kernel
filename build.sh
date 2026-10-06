#!/bin/bash
# Builds the kernel (4.4.94+) of a HiBy X1600 player inside the Docker image in
# docker/.
#
#   cd <the folder with the SDK tarball and this repo>
#   docker run --platform linux/amd64 --rm -it \
#       -v "$PWD":/work -v hiby-custom-kbuild:/build \
#       hiby-custom-kernel bash /work/hiby-custom-kernel/build.sh --model MODEL [options]
#
# /work   that folder: the SDK tarball and this repo; results go to out/ in
#         the repo
# /build  a Docker volume: the extracted SDK and the kernel tree. Not the host
#         folder, because macOS (APFS) ignores case and the kernel has files
#         whose names differ only by case.
#
# Options:
#   --model MODEL      the board: a folder in boards/ (required)
#   --name NAME        tag of the build (default: the model)
#   --fragment FILE    one more config fragment, after configs/MODEL-required.config
#                      (repeatable; relative to configs/)
#   --patch FILE       one more patch, after the ones in patches/ (repeatable;
#                      relative to patches/)
#   --menuconfig       open menuconfig before building
#   --jobs N           parallel jobs (default: CPUs)
set -euo pipefail

KIT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORK=$(dirname "$KIT")
OUT=$KIT/out
MODEL=
NAME=
FRAGMENTS=()
PATCHES=()
MENUCONFIG=0
JOBS=$(nproc)

while [ $# -gt 0 ]; do
	case "$1" in
	--model) MODEL=$2; shift ;;
	--name) NAME=$2; shift ;;
	--fragment) FRAGMENTS+=("$2"); shift ;;
	--patch) PATCHES+=("$2"); shift ;;
	--menuconfig) MENUCONFIG=1 ;;
	--jobs) JOBS=$2; shift ;;
	*) echo "unknown option: $1" >&2; exit 2 ;;
	esac
	shift
done

if [ -z "$MODEL" ] || [ ! -d "$KIT/boards/$MODEL" ]; then
	echo "--model is one of: $(ls "$KIT/boards" | tr '\n' ' ')" >&2
	exit 2
fi
BOARD=$KIT/boards/$MODEL
NAME=${NAME:-$MODEL}

mkdir -p "$OUT"
LOG=$OUT/build-$NAME.log
exec > >(tee "$LOG") 2>&1
echo "=== build $NAME ($MODEL), $(date) ==="

# --- the SDK, extracted once into the volume --------------------------------
SDK=$(ls -d /build/sdk/*/ 2>/dev/null | head -1 || true)
if [ -z "$SDK" ] || [ ! -d "$SDK/kernel/kernel-4.4.94" ]; then
	TAR=$(ls "$WORK"/ingenic-linux-kernel4.4.94-x1600-*.tar.bz2 | head -1)
	echo "=== extracting $(basename "$TAR") (only the kernel and the toolchain, once) ==="
	rm -rf /build/sdk && mkdir -p /build/sdk
	tar -I lbzip2 -xf "$TAR" -C /build/sdk --wildcards \
		'*/kernel/kernel-4.4.94/*' '*/prebuilts/toolchains/mips-gcc720-glibc229/*'
	SDK=$(ls -d /build/sdk/*/ | head -1)
fi
SDK=${SDK%/}
TOOLCHAIN=$SDK/prebuilts/toolchains/mips-gcc720-glibc229
[ -x "$TOOLCHAIN/bin/mips-linux-gnu-gcc" ] || { echo "toolchain missing in $TOOLCHAIN" >&2; exit 1; }
export PATH=$TOOLCHAIN/bin:$PATH

# --- a clean tree for every build --------------------------------------------
SRC=/build/src
echo "=== fresh kernel tree ==="
rsync -a --delete "$SDK/kernel/kernel-4.4.94/" "$SRC/"
cd "$SRC"

MK=(make ARCH=mips CROSS_COMPILE=mips-linux-gnu- "HOSTCC=gcc -fcommon")
"${MK[@]}" mrproper >/dev/null
# vermagic: "4.4.94+" like the stock kernel, or none of the 32 modules loads
printf '+\n' > .scmversion

echo "=== patches ==="
# every patch in patches/, in order
for p in "$KIT"/patches/0*.patch; do
	echo "  $(basename "$p")"; patch -p1 -s < "$p"
done
for p in "${PATCHES[@]}"; do
	echo "  $p"; patch -p1 -s < "$KIT/patches/$p"
done

echo "=== defconfig + the $MODEL device tree ==="
"${MK[@]}" x1600_halley6_module_base_linux_sfc_nand_defconfig >/dev/null
DTS=arch/mips/boot/dts/ingenic/x1600_halley6_module_base.dts
[ -f "$DTS" ] || { echo "no $DTS in this tree:" >&2; ls arch/mips/boot/dts/ingenic >&2; exit 1; }
if [ ! -f "$BOARD/stock.dtb" ]; then
	echo "no boards/$MODEL/stock.dtb: extract it from the stock firmware with tools/extract-stock.py $MODEL" >&2
	exit 1
fi
dtc -q -I dtb -O dts -o "$DTS" "$BOARD/stock.dtb"
# dtc writes a string list as one string with \0 between the items, and
# "0\01\02" (the PWM dma-names) then reads back as octal escapes: split
# the lists so the device tree compiles back byte for byte
sed -i 's/\\0/", "/g' "$DTS"

echo "=== config fragments ==="
"${MK[@]}" olddefconfig >/dev/null
MERGE=("$KIT/configs/$MODEL-required.config")
for f in "${FRAGMENTS[@]}"; do MERGE+=("$KIT/configs/$f"); done
./scripts/kconfig/merge_config.sh -m -O . .config "${MERGE[@]}" | grep -E "^(Value|Previous|New|WARNING|warning)" || true
"${MK[@]}" olddefconfig >/dev/null

if [ "$MENUCONFIG" = 1 ]; then
	"${MK[@]}" menuconfig
	./scripts/kconfig/merge_config.sh -m -O . .config "${MERGE[@]}" >/dev/null
	"${MK[@]}" olddefconfig >/dev/null
fi

echo "=== requested options that did not stick ==="
LOST=0
for f in "${MERGE[@]}"; do
	while IFS= read -r line; do
		case "$line" in
		CONFIG_*=*) opt=${line%%=*}; want=$line ;;
		"# CONFIG_"*" is not set") opt=${line#\# }; opt=${opt%% *}; want="" ;;
		*) continue ;;
		esac
		have=$(grep -E "^$opt=" .config || true)
		if [ "$have" != "$want" ]; then
			echo "  $(basename "$f"): asked '${want:-$opt off}', got '${have:-off}'"
			LOST=1
		fi
	done < "$f"
done
[ "$LOST" = 0 ] && echo "  none"

echo "=== vermagic options ==="
for o in PREEMPT MODULE_UNLOAD CPU_MIPS32_R2 32BIT; do
	grep -q "^CONFIG_$o=y" .config || { echo "ABORT: CONFIG_$o must be y" >&2; exit 1; }
done
grep -q "^CONFIG_MODVERSIONS=y" .config && { echo "ABORT: MODVERSIONS would refuse the HiBy modules" >&2; exit 1; }
echo "  ok"

echo "=== building xImage with $JOBS jobs ==="
time "${MK[@]}" -j"$JOBS" xImage

cp arch/mips/boot/zcompressed/xImage "$OUT/xImage-$NAME"
cp System.map "$OUT/System.map-$NAME"
cp .config "$OUT/config-$NAME"

# Modules: the options a fragment sets to m, the modules in modules/ and in
# boards/MODEL/modules/, built out of tree from a copy so that the repo stays
# clean.
MODDIR=$OUT/modules-$NAME
rm -rf "$MODDIR" /build/ext && mkdir -p "$MODDIR" /build/ext
if grep -q '=m$' .config; then
	echo "=== building modules ==="
	"${MK[@]}" -j"$JOBS" modules
	find . -name '*.ko' -exec cp {} "$MODDIR/" \;
fi
for d in "$KIT"/modules/*/ "$BOARD"/modules/*/; do
	[ -f "$d/Makefile" ] || continue
	m=$(basename "$d")
	echo "=== building modules/$m ==="
	cp -r "$d" "/build/ext/$m"
	"${MK[@]}" M="/build/ext/$m" modules
	cp "/build/ext/$m"/*.ko "$MODDIR/"
done
if ls "$MODDIR"/*.ko >/dev/null 2>&1; then
	echo "=== module symbols not found in the kernel, the other modules or the HiBy modules ==="
	PROVIDED=$(mktemp)
	{
		awk '{print $3}' System.map
		for k in "$MODDIR"/*.ko; do mips-linux-gnu-nm "$k" | sed -n 's/.* __ksymtab_//p'; done
		grep -v '^#' "$BOARD/hiby-exports.txt"
	} | sort -u > "$PROVIDED"
	MISSING=0
	for k in "$MODDIR"/*.ko; do
		for s in $(mips-linux-gnu-nm -u "$k" | awk '{print $2}' | sort -u | comm -23 - "$PROVIDED"); do
			echo "  $(basename "$k"): $s"; MISSING=1
		done
	done
	[ "$MISSING" = 0 ] && echo "  none"
	rm -f "$PROVIDED"
fi
echo "=== checking against the stock kernel ==="
python3 "$KIT/tools/compare.py" --kit "$KIT" --tree "$SRC" --name "$NAME" --model "$MODEL" | tee "$OUT/report-$NAME.txt"
echo
echo "BUILD_OK -> out/xImage-$NAME, out/report-$NAME.txt, out/modules-$NAME"
