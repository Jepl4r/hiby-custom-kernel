# Out-of-tree build against the configured kernel tree, for one board:
#   make -C <kernel tree> M=$PWD ARCH=mips CROSS_COMPILE=mips-linux-gnu- \
#       HIBY_MODEL=r3proii modules
obj-m := sa_config_module.o

ifeq ($(HIBY_MODEL),)
$(error HIBY_MODEL is not set: the board, as named in boards/)
endif
ccflags-y := -DSA_CONFIG_MODEL='"$(HIBY_MODEL)"'
