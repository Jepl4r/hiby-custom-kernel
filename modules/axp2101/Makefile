# Out-of-tree build against the configured kernel tree:
#   make -C <kernel tree> M=$PWD ARCH=mips CROSS_COMPILE=mips-linux-gnu- modules
obj-m := axp2101.o

# The symbols of the HiBy modules it calls, so that modpost records them in
# "depends" as the vendor module does.
KBUILD_EXTRA_SYMBOLS := $(KBUILD_EXTMOD)/hiby.symvers

# On the R1 the PMIC charges the battery, and VBUS wakes the system.
ifeq ($(HIBY_MODEL),r1)
ccflags-y += -DAXP2101_VBUS_WAKE
endif
