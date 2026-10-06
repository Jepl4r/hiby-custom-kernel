#!/bin/sh
#
# Compressed swap in RAM: /dev/zram0, LZ4, room for 24 MB of pages, which
# compress to a third or a half of that. Cold anonymous memory goes there
# instead of the page cache being thrown out. Sourced by rcS with "start"; a
# kernel without zram (the stock one) has no /sys/block/zram0 and nothing
# happens.

ZRAM=/sys/block/zram0
ZRAM_SIZE=24M

zram_start() {
	[ -d $ZRAM ] || return 0
	grep -q '^/dev/zram0 ' /proc/swaps && return 0

	# mdev may not have made the node yet
	if [ ! -b /dev/zram0 ]; then
		dev=$(cat $ZRAM/dev)
		mknod /dev/zram0 b ${dev%%:*} ${dev##*:}
	fi

	echo lz4 > $ZRAM/comp_algorithm 2>/dev/null
	echo 1 > $ZRAM/max_comp_streams 2>/dev/null
	echo $ZRAM_SIZE > $ZRAM/disksize || return 0
	mkswap /dev/zram0 > /dev/null && swapon /dev/zram0
	# anonymous pages to zram before the code and data the player reads back
	echo 100 > /proc/sys/vm/swappiness
}

case "$1" in
start) zram_start ;;
esac
