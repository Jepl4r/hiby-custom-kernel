# Linux 4.4.94+ for HiBy X1600 players

A kit to rebuild the kernel of the HiBy players built on the **Ingenic X1600** from Ingenic's public SDK and put it into a firmware image, for anyone making custom firmware for them.

| board | `--model` | state |
|---|---|---|
| HiBy R3 Pro II | `r3proii` | working |
| HiBy R1 | `r1` | working |

On both players the resulting kernel:

- **loads HiBy's closed-source modules unchanged** (32 on the R3 Pro II, 28 on the R1): same vermagic `4.4.94+`, same exported symbols, same structures;
- **uses the board's original device tree**, byte for byte;
- **adds what the stock kernel lacks:** compressed swap in RAM (zram, LZ4), about 1.1 MB of RAM freed and the deadline I/O scheduler;
- **can replace the closed Wi-Fi/Bluetooth driver** `cywdhd.ko` with mainline `brcmfmac` and a small open board module, `bcm_wlbt_power` (see [Wi-Fi and Bluetooth](#wi-fi-and-bluetooth-without-cywdhd));
- **comes with open sources for eleven of HiBy's closed modules**, drop-in replacements with the same file names, parameters and behaviour: the two sound card drivers and nine small ones (see [Open HiBy modules](#open-hiby-modules)).

Working on both players: boot, display and touch, audio (3.5 mm, the R3 Pro II's balanced output, DSD), Wi-Fi, Bluetooth, microSD, USB (ADB, mass storage, USB DAC, OTG), charging and LEDs.

## Credits
The method, the Docker environment and three patches (0001, 0002, 0013) come from
[MatthewBriggs/hiby-r1-linux-kernel-compiling](https://github.com/MatthewBriggs/hiby-r1-linux-kernel-compiling), which did the same work for the HiBy R1.

## Not in this repository
- **Ingenic's SDK** (`ingenic-linux-kernel4.4.94-x1600-v6.0-20240606.tar.bz2`, kernel sources and toolchain). It is not redistributable and has to be obtained separately.
- **Anything from HiBy's firmware:** the stock device trees, their symbol tables, the closed modules, the firmware blobs and the stock scripts. `tools/extract-stock.py` takes what the build needs out of the official firmware.

## Requirements
- Docker. On Apple Silicon Macs: Docker Desktop with "Use Rosetta for x86_64/amd64 emulation" enabled.
- The SDK tarball.
- The stock firmware of the player (its `.upt`), or just its stock kernel image.
- Python 3 and `7z` (macOS: `brew install p7zip`). Optional: `pip install vmlinux-to-elf`, for the full symbol comparison.

## Layout
```
<work folder>/
  ingenic-linux-kernel4.4.94-x1600-v6.0-20240606.tar.bz2
  hiby-custom-kernel/           this repository
    build.sh                    the build
    docker/Dockerfile           the environment (Debian bullseye, amd64)
    patches/                    0001-0014, all applied, in order
    configs/                    configuration fragments, MODEL-*.config
    boards/MODEL/               one folder per player:
      modules-need.txt            the symbols the HiBy modules import
      hiby-exports.txt            the symbols the HiBy modules export to ours
      modules/                    that player's own modules, built out of tree
      rootfs/                     files for that player's root filesystem
      stock.dtb, stock.kallsyms   extracted from the stock firmware, not in git
    modules/                    modules for every board, built out of tree:
      bcm_wlbt_power/             Wi-Fi/Bluetooth power for brcmfmac
      soc_utils/, pwm_backlight/, sa_config_module/   open HiBy modules
    rootfs/                     files for the root filesystem of every player
    tools/extract-stock.py      device tree and symbols from the stock firmware
    tools/compare.py            comparison with the stock kernel
    out/                        xImage, modules, System.map, config, log and report
```

## One-time setup
```sh
cd <work folder>/hiby-custom-kernel
docker build --platform linux/amd64 -t hiby-custom-kernel docker/
docker volume create hiby-custom-kbuild
python3 tools/extract-stock.py r3proii /path/to/r3proii.upt
python3 tools/extract-stock.py r1 /path/to/the/r1/firmware.upt
```
`extract-stock.py MODEL` writes `boards/MODEL/stock.dtb` (required) and, if `vmlinux-to-elf` is installed, `boards/MODEL/stock.kallsyms`. It also accepts the stock kernel image alone, or a kernel read straight from the device (`dd if=/dev/mtd1`).

## Building
```sh
cd <work folder>
docker run --platform linux/amd64 --rm -it \
  -v "$PWD":/work -v hiby-custom-kbuild:/build \
  hiby-custom-kernel bash /work/hiby-custom-kernel/build.sh --model r3proii \
  --fragment r3proii-parity.config --fragment r3proii-ram.config \
  --fragment r3proii-brcmfmac.config
```
For the R1, `--model r1` and the `r1-*.config` fragments.
- **The first time** it extracts only the kernel and the toolchain (`mips-gcc720-glibc229`) from the tarball, into a Docker volume. The macOS disk is case-insensitive, and the kernel has files whose names differ only in case.
- **Steps:**
  1. all the patches;
  2. the defconfig `x1600_halley6_module_base_linux_sfc_nand_defconfig`;
  3. the board's stock device tree;
  4. `configs/MODEL-required.config` and the fragments, each option checked (an option that did not take effect is reported);
  5. a check of the options that decide the vermagic (`PREEMPT`, `MODULE_UNLOAD`, `CPU_MIPS32_R2`, `32BIT`, no `MODVERSIONS`);
  6. building `xImage`;
  7. building the modules a fragment sets to `m` and the ones in `modules/` and `boards/MODEL/modules/`, and checking that every symbol they import is in the kernel, in one of them or in `boards/MODEL/hiby-exports.txt`.
- **Output** in `out/`, with NAME the model unless `--name` says otherwise: `xImage-NAME`, `modules-NAME/`, `System.map-NAME`, `config-NAME`, `build-NAME.log` and `report-NAME.txt`.
- **The report** lists:
  - which of the symbols imported by the HiBy modules are missing (445 on the R3 Pro II, 431 on the R1; must be 0);
  - symbol differences against the stock kernel, if `stock.kallsyms` is present;
  - whether the device tree in the image is identical to the stock one.

`--menuconfig` opens menuconfig before building; the fragments are applied again afterwards.

## Into a firmware image
What the build makes, and where it goes in the stock firmware:

| from | to |
|---|---|
| `out/xImage-MODEL` | the kernel image of the `.upt`, in place of the stock one; not larger than it, which is all the room the kernel partition is known to have |
| `out/modules-MODEL/*.ko` | `/module_driver` |
| `rootfs/` and `boards/MODEL/rootfs/` | the root of the rootfs; scripts executable |

and, for `brcmfmac`, the four changes to the stock scripts in [Wi-Fi and Bluetooth](#wi-fi-and-bluetooth-without-cywdhd).

A `.upt` is an ISO 9660 image with `ota_v0/` inside. Each image (`xImage`, `rootfs.squashfs`) is split into 512 KiB chunks `NAME.NNNN.<md5>`:
- chunk `0000` is named after the md5 of the whole image, and every later chunk after the md5 of the one before;
- `ota_md5_NAME.<md5 of the image>` lists the md5 of every chunk;
- `ota_update.in` gives each image's size and md5.

The updater checks all of it, so a firmware packer has to rewrite the chunks, the list and `ota_update.in` for every image it changes. The rootfs is squashfs (lzo, all files owned by root).

To install, copy the `.upt` to the microSD under the stock file's name (`r3proii.upt` on the R3 Pro II) and start the recovery (Power + Volume up at power-on), which flashes it. The recovery lives in U-Boot, which no image touches, so a kernel that does not boot can always be recovered.

## The patches
All of them are applied for every board; on the R1, 0004 is not built (no `CONFIG_TYPEC`) and 0010 has no device to drive.

| | what it does |
|---|---|
| 0001 | `REGMAP_IRQ` selectable: needed by `axp2101.ko` (MatthewBriggs) |
| 0002 | `DMA_PRIVATE` on the Ingenic DMA, for `sa_hgl_dma.ko` (MatthewBriggs) |
| 0003 | **power_supply as on HiBy's kernel**: 201 properties in the same order, with their tables, for `mp2731`, `axp2101` and `cw2015` (the R1 modules use the same table) |
| 0004 | **Type-C** (R3 Pro II): typec class, TCPM and FUSB302 from Linux 4.16, with the board data from `fusb302b_add.ko`, the power role passed to the charger and the OTG ID GPIO |
| 0005 | USB gadget: `usb_gadget` class (`/sys/class/usb_gadget/android0`) and `soft_disconnect` |
| 0006 | mass storage: `inquiry_string` in configfs |
| 0007 | LEDs: `breathing` trigger for the charging LED |
| 0008 | **USB DAC**: the `uac_sa` gadget function and `/dev/uac_sa`, with descriptors, feedback table and ioctls matching the stock kernel |
| 0009 | Ingenic DMA: interleaved transfers, called by `sa_hgl_dma.ko` |
| 0010 | **AW95016 GPIO expander** (R3 Pro II, port F, GPIOs 160-175) in the pca953x driver: inverted direction bit and non-sleeping lines, as on the stock kernel. Wi-Fi, Bluetooth, the HBC3000, microSD power, LEDs and the charger sit behind it |
| 0011 | gpiolib: a GPIO used as an interrupt can still be driven, as on the stock kernel (GT9xx touch reset after suspend) |
| 0012 | **DMA to peripherals as on the stock kernel**: transfer size chosen by the controller and one full burst per request (RDIL). The SDK gives the AIC one word per request |
| 0013 | memory: no HighAtomic reserve when a pageblock is nearly the whole zone (with `FORCE_MAX_ZONEORDER=15` a pageblock is 64 MB, the zone 56) (MatthewBriggs) |
| 0014 | brcmfmac: the signal of the link in station mode, read with `GET_RSSI` when the firmware's `sta_info` has none, so `wpa_cli signal_poll` reports it |

## Configuration
The fragments of the two boards differ only where their stock kernels do: the R1's has no Type-C class and no memory compaction. The two stock kernels are otherwise the same build, function for function.

- `configs/r3proii-required.config`: what the HiBy modules and the system need (power_supply, regmap IRQ, LEDs, switch, input, squashfs/ubifs, FAT/exFAT/FUSE, USB host and gadget, cfg80211, Bluetooth over UART).
- `configs/r3proii-parity.config`: the choices that make the kernel match the stock one:
  - SLAB, as the HiBy modules expect;
  - `-Os`, compaction and LED triggers;
  - USB host for drives and DACs;
  - Type-C, `uac_sa` and the pca953x expander;
  - FAT long names and the NLS default in UTF-8;
  - NFS, ext4, CAN, Ethernet and the other defconfig parts the stock kernel lacks turned off.
- `configs/r3proii-ram.config`: memory and storage beyond parity.
  - **zram:** `SWAP`, `ZSMALLOC`, `ZRAM` with LZ4.
  - **Removed:**
    - mac80211 (`cywdhd` and `brcmfmac` use cfg80211 only);
    - the kernel's NTFS driver (ntfs-3g mounts NTFS);
    - data symbol names in kallsyms;
    - the crypto self-test vectors;
    - the GBK codepage (FAT mounts with codepage 437, exFAT with utf8);
    - the crypto algorithms only mac80211 used;
    - Bluetooth High Speed (AMP).
  - **Result:** the resident kernel shrinks by about 1.1 MB.
  - **I/O scheduler:** deadline instead of CFQ, for the microSD and the NAND.
  - **What stays, although nothing uses it:**
    - netfilter: `NETFILTER_INGRESS` adds a field to `struct net_device`, and `cywdhd` was built with that layout;
    - debugfs: `DEBUG_FS` adds fields to the ASoC codec, card and DAPM structures, which the codec module was built with.
- `configs/r3proii-brcmfmac.config`: `brcmfmac` and `brcmutil` as modules. The kernel image does not change.
- `configs/r1-*.config`: the same four for the R1, without Type-C and compaction in `r1-parity.config`.

## Wi-Fi and Bluetooth without cywdhd
Both players have a BCM43430 combo, either an AP6212A or an AzureWave AW-NB372SM module depending on the unit: Wi-Fi on SDIO, Bluetooth on `/dev/ttyS0`. HiBy drives both through the closed `cywdhd.ko`, about 860 KB of code loaded at every boot. In its place:

- **`brcmfmac.ko` + `brcmutil.ko`** (mainline, about 165 KB), loaded only while Wi-Fi is on.
- **`bcm_wlbt_power.ko`** (`modules/bcm_wlbt_power`), the board side of `cywdhd` and nothing else:
  - **Bluetooth:** the `bluetooth` rfkill, which is `rfkill0`, where the stock `bt_init` and most players look. Writing 1 to its state starts the 32 kHz LPO and raises `BT_REG_ON` for 300 ms, the same sequence as `cywdhd`; `BT_WAKE` is held high. The stock `brcm_patchram_plus` loads the Bluetooth patch as before.
  - **Wi-Fi:** `/sys/devices/platform/bcm_wlbt_power/wifi_power`. Writing 1 starts the LPO, pulses `WL_REG_ON` and powers the SDIO card up; 0 powers it down.
  - **`brcmfmac_sdio`** platform data: the out-of-band interrupt on `WL_HOST_WAKE`.
  - **`chipvendor`:** the attribute `cywdhd` puts on SDIO function 2 (`.../mmc0:0001:2/chipvendor`), read from the card's CIS. The stock `bt_init` reads it to choose the Bluetooth patch, and takes a different one when the file is missing, so it is published the same way.
- **`boards/MODEL/rootfs/module_driver/bcm_wlbt_power.sh`** loads the module with the board's pins, taken from its stock `cywdhd.sh`. Pins are given as the SoC names them, so another board only needs its own line:
  - R3 Pro II: `wl_reg_on=PF02 wl_host_wake=PC25 wl_mmc=0 bt_reg_on=PF03 host_wake_bt=PF05`;
  - R1: `wl_reg_on=PB03 wl_host_wake=PA08 wl_mmc=0 bt_reg_on=PB04 host_wake_bt=PB05`.
- **`rootfs/usr/bin/wlbt-wifi up|down`:**
  - **up:** powers the chip, links the stock blob for the module in `/lib/firmware/wifi_bcm/` (`cyw43438-7.46.58.35.bin` + `nvram_azw372.txt` when `chipvendor` is `0x81`, AzureWave; `fw_bcm43438a1.bin` + `nvram_ap6212a.txt` otherwise, AP6212A) under the name `brcmfmac` asks for, loads the driver, waits for `wlan0` and gives it the stock MAC address.
  - **down:** unloads the driver and powers the chip down.

Four changes to the stock scripts hand Wi-Fi over to it (the three scripts in `usr/bin` and `etc/init.d` are the same on both players):

| file | change |
|---|---|
| `module_driver/driver_default_init_script.sh` | `sh bcm_wlbt_power.sh` in place of `sh cywdhd.sh` |
| `usr/bin/wifi_on.sh` | `/usr/bin/wlbt-wifi up \|\| exit 1` after the `killall` lines |
| `usr/bin/wifi_off.sh` | `/usr/bin/wlbt-wifi down` before the final `exit 0` |
| `etc/init.d/S43wifi_bcm_init_config` | no waiting for `wlan0` at boot when `bcm_wlbt_power` is loaded |

A player that checks for `wlan0` to decide whether the device has Wi-Fi should also accept `/sys/devices/platform/bcm_wlbt_power/wifi_power`: `wlan0` now exists only while Wi-Fi is on.

To go back to `cywdhd`, leave the four changes and the three modules out.

## The sound card
`boards/MODEL/modules/x1600_hiby_MODEL_sound_card/` holds an open source for the board's ASoC machine driver, the module that ties the X1600 I2S controller (`soc_aic.ko`) to the DAC. Its `.ko` has the vendor module's file name and replaces it in `/module_driver`; the stock `x1600_hiby_MODEL_sound_card.sh` loads it unchanged.

It keeps everything the player sees: the platform device `hiby-hifi-board.0`, the card `hiby-sound-card`, the DAI links and the mixer controls, with the same names, ranges and behaviour. The two modules were checked against the vendor ones by running both, function by function, in an emulator with every external call recorded: every route, sample rate, DoP setting and error path gives the same calls with the same arguments, in the same order.

| board | DAC | links | controls |
|---|---|---|---|
| R1 | CS43131 (`codec_cs43131.ko`) | `x1600-i2s`, the X1600 is the I2S master | `Output Port Switch` 0-5: 2 and 3 power the DAC up |
| R3 Pro II | CS43198 (`codec_cs43198_dual.ko`) behind the HBC3000 (`sa_sound_hbc3000.ko`) | `x1600-i2s`, `x1600-spdif`, the HBC3000 is the I2S master | `Output Port Switch` 0-7, `Balance Lineout En`, `DOP_EN` |

`Output Port Switch` on the R3 Pro II: 1 and 2 the 3.5 mm jack (line out, headphones), 3 the 4.4 mm jack (headphones, or line out with `Balance Lineout En`), 4 S/PDIF, anything else off. Two vendor behaviours are kept on purpose, since players work around them: moving between 1 and 2 does nothing, and writing the current route does nothing except for 3, which applies `Balance Lineout En` again.

The only differences: on the R3 Pro II the workqueue is created before the GPIOs are requested, a failed allocation fails the probe, and removing the driver frees what the probe took (GPIOs, workqueue, the `soc_aic` callbacks), so the module can be unloaded and loaded again.

## Open HiBy modules
Open sources for closed modules of the stock firmware. Each `.ko` has the vendor module's file name and replaces it in `/module_driver`; the stock `.sh` that loads it stays as it is. They were checked like the sound card drivers: the vendor module and the rebuilt one run side by side in an emulator, every call into the kernel and the other HiBy modules recorded and compared, and their parameters (names, types, defaults, permissions) and tables (devices, drivers, file operations, sysfs attributes) compared too.

| module | where | boards | what it does |
|---|---|---|---|
| `soc_utils.ko` | `modules/soc_utils` | both | the 32 kHz clock out on PC26 for the Wi-Fi/Bluetooth chip: `ingenic_rtc32k_enable/disable`, counted |
| `pwm_backlight.ko` | `modules/pwm_backlight` | both | the display backlight, on a PWM channel of `soc_pwm.ko` |
| `sa_config_module.ko` | `modules/sa_config_module` | both | `/dev/sa-config`: two ioctls that return the model (`r1`, `r3proii`) and the firmware name (`r3proii.upt`) |
| `fusb302b_add.ko` | `boards/r3proii/modules` | R3 Pro II | registers the FUSB302B Type-C controller with its board data for the driver of patch 0004 |
| `gpio_aw95016_add.ko` | `boards/r3proii/modules` | R3 Pro II | registers the AW95016 GPIO expander for the driver of patch 0010 |
| `sau.ko` | `boards/r3proii/modules` | R3 Pro II | `/sys/devices/platform/sa_information/sa_verification`, read by HiBy's player and bluealsa |
| `keyboard_adc.ko` | `boards/r3proii/modules` | R3 Pro II | the keys on the ADC resistor ladder (`soc_adc.ko`), as the input device `jz adc keyboard` |
| `tcs1421_add.ko` | `boards/r1/modules` | R1 | the TCS1421 Type-C port controller, its role (`Sink`, `Source`, `StrongDRP`, `NormalDRP`) set on two pins through `/sys/devices/platform/tcs1421/tcs1421_cfg` |
| `leds_pwm_add.ko` | `boards/r1/modules` | R1 | the red and blue LEDs on PWM channels of `soc_pwm.ko`, as `/sys/class/leds/red` and `blue`; based on `drivers/leds/leds-pwm.c` |

`sa_config_module` is one source for every board: `build.sh` passes the board as `HIBY_MODEL`, which is also the model name the vendor module reports.

What differs from the vendor modules, on purpose:
- **`soc_utils`:** a disable with the count already at 0 is ignored; the vendor module takes the count below 0, and the next enable then leaves the clock off. It can also be unloaded.
- **`gpio_aw95016_add`, `sau`, `sa_config_module`:** unloading frees what loading took (the RSTN GPIO, the sysfs group and the device, an empty `release` for the platform devices). The vendor `sau.ko` does not unregister anything when unloaded.
- **`tcs1421_add`, `leds_pwm_add`, `keyboard_adc`:** a probe that fails, and unloading, give back what was taken (the CFG pins, the sysfs file, the PWM channels, the driver data), and the platform devices have an empty `release`.
- **`tcs1421_add`:** `tcs1421_cfg` reads at most 19 characters of the word and refuses a write with no word (`-EINVAL`); the vendor module copies the word onto its stack whatever its length, and with no word applies whatever the stack held.
- **`keyboard_adc`:** the key table ends after key 8 even when all eight codes are set, and a failed input allocation returns `-ENOMEM` (the vendor module returns 12).
- **`leds_pwm_add`:** without the device tree path of `leds-pwm.c`, which the module's own device, with its LEDs in parameters, never takes; a period of 0 gives 10 kHz without dividing by 0.

## zram on the device
`rootfs/etc/init.d/S12zram.sh`, which `rcS` runs at boot:
- creates a 24 MB `/dev/zram0` with LZ4;
- enables it as swap and sets `swappiness` to 100.

With the stock kernel, which has no zram, it does nothing. To check: `cat /proc/swaps` and `cat /sys/block/zram0/mm_stat`.

## Adding a board
1. `boards/MODEL/`: run `tools/extract-stock.py MODEL` on its stock firmware, and write `modules-need.txt`: the symbols its HiBy modules import (the `U` symbols of every `.ko` in `/module_driver`, minus those the modules export to each other).
2. `configs/MODEL-required.config`, and the other fragments the board wants.
3. Build with `--model MODEL`. The report says which of the needed symbols the kernel does not export yet; the patches here were written for the R3 Pro II and the R1 and may need company.
4. For `bcm_wlbt_power`, the pins from the board's stock `cywdhd.sh` in `boards/MODEL/rootfs/module_driver/bcm_wlbt_power.sh`.
5. Optionally, its own open modules in `boards/MODEL/modules/` (the sound card driver, say), with the symbols they take from HiBy modules in a `hiby.symvers` next to them and in `hiby-exports.txt`. The shared ones in `modules/` are built for it as well; `sa_config_module` reports the folder name as the model.

## License
GPL-2.0, like the Linux kernel the patches apply to (see `LICENSE`).
