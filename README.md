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
- **comes with open sources for thirty-seven of HiBy's closed modules**, drop-in replacements with the same file names, parameters and behaviour: the two sound card drivers, the I2S, PWM, I2C, LCD and MMC/SD controllers, the DACs of both boards, the R3 Pro II's HBC3000 FPGA, the panels and touch panels of both boards, the fuel gauge, the R3 Pro II's charger, the efuse, the ADC, the DAC's I2C bus, the GPIO keys, the reserved-memory manager, the helpers the other modules link against and fifteen smaller ones (see [Open HiBy modules](#open-hiby-modules)).

Working on both players: boot, display and touch, audio (3.5 mm, the R3 Pro II's balanced output, DSD), Wi-Fi, Bluetooth, microSD, USB (ADB, mass storage, USB DAC, OTG), charging and LEDs.

## Credits
The method, the Docker environment and three patches (0001, 0002, 0013) come from
[MatthewBriggs/hiby-r1-linux-kernel-compiling](https://github.com/MatthewBriggs/hiby-r1-linux-kernel-compiling), which did the same work for the HiBy R1.

## Not in this repository
- **Ingenic's SDK** (`ingenic-linux-kernel4.4.94-x1600-v6.0-20240606.tar.bz2`, kernel sources and toolchain). It is not redistributable and has to be obtained separately.
- **Anything from HiBy's firmware:** the stock device trees, their symbol tables, the closed modules, the firmware blobs (the HBC3000's FPGA configuration among them) and the stock scripts. `tools/extract-stock.py` takes what the build needs out of the official firmware, `tools/hbc3000-firmware.py` the FPGA configuration out of the stock `sa_sound_hbc3000.ko`.

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
    patches/                    0001-0015, all applied, in order
    configs/                    configuration fragments, MODEL-*.config
    boards/MODEL/               one folder per player:
      modules-need.txt            the symbols the HiBy modules import
      hiby-exports.txt            the symbols the HiBy modules export to ours
      modules/                    that player's own modules, built out of tree
      rootfs/                     files for that player's root filesystem
      stock.dtb, stock.kallsyms   extracted from the stock firmware, not in git
    modules/                    modules for every board, built out of tree:
      bcm_wlbt_power/             Wi-Fi/Bluetooth power for brcmfmac
      soc_utils/, pwm_backlight/, sa_config_module/,
      sa_sound_switch/, sa_earpods_adc/, soc_efuse/,
      cw2015/, soc_adc/, rmem_manager/,
      keyboard_gpio_add/, i2c_gpio_add/, utils/,
      soc_gpio/, sa_hgl_dma/, soc_aic/, soc_pwm/,
      soc_i2c/, soc_fb/,
      soc_msc/                  open HiBy modules
    rootfs/                     files for the root filesystem of every player
    tools/extract-stock.py      device tree and symbols from the stock firmware
    tools/compare.py            comparison with the stock kernel
    tools/hbc3000-firmware.py   the HBC3000's configuration from the stock module
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
| 0015 | ALSA: the sample rates numbered as on the stock kernel, with 705600 Hz (bit 15) before 768000 Hz (bit 16). The HiBy modules use that numbering; without it DSD256 over DoP is refused |

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

It keeps everything the player sees: the platform device `hiby-hifi-board.0`, the card `hiby-sound-card`, the DAI links and the mixer controls, with the same names, ranges and behaviour. The two modules were checked against the vendor ones by running both, function by function, in an emulator with every external call recorded: every sample rate, DoP setting and error path gives the same calls with the same arguments, in the same order, and so does every route on the R1.

| board | DAC | links | controls |
|---|---|---|---|
| R1 | CS43131 (`codec_cs43131.ko`) | `x1600-i2s`, the X1600 is the I2S master | `Output Port Switch` 0-5: 2 and 3 power the DAC up |
| R3 Pro II | CS43198 (`codec_cs43198_dual.ko`) behind the HBC3000 (`sa_sound_hbc3000.ko`) | `x1600-i2s`, `x1600-spdif`, the HBC3000 is the I2S master | `Output Port Switch` 0-7, `Balance Lineout En`, `DOP_EN` |

`Output Port Switch` on the R3 Pro II: 1 and 2 the 3.5 mm jack (line out, headphones), 3 the 4.4 mm jack (headphones, or line out with `Balance Lineout En`), 4 S/PDIF, anything else off. Two vendor behaviours are kept on purpose, since players work around them: moving between 1 and 2 does nothing, and writing the current route does nothing except for 3, which applies `Balance Lineout En` again.

The only differences: on the R3 Pro II the workqueue is created before the GPIOs are requested, a failed allocation fails the probe, and removing the driver frees what the probe took (GPIOs, workqueue, the `soc_aic` callbacks), so the module can be unloaded and loaded again.

On the R3 Pro II, besides, a change of route stays silent:

- The vendor driver, moving to 3, mutes, sets the 4.4 mm switches and unmutes, and only then powers and loads the HBC3000 and power-cycles the DAC, with the outputs open: a pop on the 4.4 mm jack at every change to it and after every wake-up. Here the outputs stay muted through the whole sequence (HBC3000, DAC, switches) for all of 1, 2 and 3, and 50 ms more while the DAC settles. A write of 3 over 3 is the vendor's.
- In a suspend the HBC3000 loses its power, and the output switches with it, after the card is suspended. The card's `suspend_pre` mutes the outputs and powers the DAC down first, and forgets the route (`Output Port Switch` reads 0), so the next write of a route powers everything back as above.

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
| `keyboard_adc_multifunc.ko` | `boards/r1/modules` | R1 | the keys on the ADC resistor ladder, with a second code per key on a double click (`key_config`) |
| `sa_sound_switch.ko` | `modules/sa_sound_switch` | both | what is plugged into the outputs, read from a pin or an ADC channel, as `/sys/class/switch/headset` and the others; `get_switch_status()` |
| `sa_earpods_adc.ko` | `modules/sa_earpods_adc` | both | the buttons of a wired headset remote on the ADC: clicks, long presses and volume, as the input device `earpods_adc` |
| `leds_sgm31324_add.ko` | `boards/r3proii/modules` | R3 Pro II | the SGM31324 RGB LED on I2C and the white LED pin, set up through the `sgm31324` parameter, patterns through `led_pattern` |
| `codec_cs43198_dual.ko` | `boards/r3proii/modules` | R3 Pro II | the two CS43198 DACs as the ASoC codec `cs43198-hifi`: power and reset per stream, the PCM and DSD register sets, volume, filter, NOS and DRE controls; `cs43198_set_dsd_en()` |
| `lcd_st7701_sbtc033001.ko` | `boards/r3proii/modules` | R3 Pro II | the ST7701S panel: its timings for `soc_fb.ko` and its set-up over a bit-banged 3-wire SPI at every screen-on |
| `soc_efuse.ko` | `modules/soc_efuse` | both | the X1600 efuse: `/proc/jz/efuse/efuse_chip_id` and `efuse_user_id`, and `/dev/efuse-string-version` with its ioctls to read and program the segments |
| `cw2015.ko` | `modules/cw2015` | both | the CW2015 fuel gauge as the power supply `battery`: loads the battery profile from `fuel_gauge`, then capacity, voltage, status and time to empty every second |
| `codec_cs43131.ko` | `boards/r1/modules` | R1 | the CS43131 DAC as the ASoC codec `cs43131-hifi`: power and reset per stream, the PCM and DSD register sets, volume, digital filter, `NOS_EN` and `DOP_EN`; `cs43131_set_power()` |
| `cst8xx_touch.ko` | `boards/r1/modules` | R1 | the Hynitron CST8xx touch panel as the input device `hyn_ts`, one or two contacts (`cst_max_touch_number`), asleep while the screen is off |
| `gt9xx_touch.ko` | `boards/r3proii/modules` | R3 Pro II | the Goodix GT967 touch panel as the input device `goodix-ts`, up to ten contacts (`gtp_max_touch_number`; the panel tracks five), its configuration sent at probe and at every screen-on; gesture wake through `gesture_sw` (a gesture or a double tap on the dozing panel is `KEY_POWER`), and `/proc/gt9xx_config` |
| `soc_adc.ko` | `modules/soc_adc` | both | the X1600 SAR ADC: `adc_enable()`, `adc_disable()`, `adc_read_channel_voltage()` for the key, jack and remote modules, and `/dev/jz_adc_aux_0..3` |
| `rmem_manager.ko` | `modules/rmem_manager` | both | `/dev/rmem_manager`: physically contiguous buffers for user space, allocated, freed, synced and mapped through ioctls and `mmap`, from the `rmem=` region or DMA memory |
| `keyboard_gpio_add.ko` | `modules/keyboard_gpio_add` | both | the keys on pins (power, next or previous track) as the input device `md-gpio-keys`, set up through the `keyboard` parameter; based on `drivers/input/keyboard/gpio_keys.c` |
| `i2c_gpio_add.ko` | `modules/i2c_gpio_add` | both | bit-banged I2C buses added through the `i2c_bus` parameter (bus 3, the DAC's); `i2c-gpio.c` and `i2c-algo-bit.c` in one module, the algorithm as `md_i2c_bit_add_numbered_bus()` |
| `lcd_lg35583.ko` | `boards/r1/modules` | R1 | the R1's 480x800 panel for `soc_fb.ko`: an LG35583 (16-bit SPI words) or an ST7701S (9-bit), told apart by ADC channel 3, set up at every screen-on; `/proc/lcd_reg` sets the LG35583's register C700 |
| `sa_sound_hbc3000.ko` | `boards/r3proii/modules` | R3 Pro II | the HBC3000, a Gowin GW1N FPGA between the I2S controller and the DACs: it is loaded over JTAG with HiBy's configuration (see below), then makes the audio clocks, is the I2S master, unpacks DoP and drives the output switches on its GPO pins, set through a 12-bit SPI; `hbc3000_enable()`, `hbc3000_set_samplerate()` and the rest for the sound card |
| `utils.ko` | `modules/utils` | both | the helpers the other HiBy modules link against: pin names (`str_to_gpio()`, `gpio_to_str()`, the `param_gpio_ops` parameter type), `gpio_set_func()`, word splitting (`str_to_words()`), `i2c_register_device()`, `spi_register_device()`, `local_clock_us/ms()`, the `param_pmu_ops` parameter type of the PMU drivers, and `/dev/log_manager`, a ring of timestamped records |
| `soc_gpio.ko` | `modules/soc_gpio` | both | `/dev/gpio`: the function, pull and level of a pin, by name, through ioctls |
| `sa_hgl_dma.ko` | `modules/sa_hgl_dma` | both | `/dev/sa_hgl_dma`: a block from `rmem_manager.ko` that a program maps, and 2D copies out of it by the DMA controller, one per write |
| `soc_aic.ko` | `modules/soc_aic` | both | the X1600 AIC (I2S controller) as the ASoC CPU DAI and platform `ingenic-aic`: its pins and clocks, the PCM over cyclic DMA (an hrtimer for playback, a copy thread for capture), the period of silence before the first playback, and `aic_slave_trigger()` for the sound cards |
| `soc_pwm.ko` | `modules/soc_pwm` | both | the eight channels of the X1600 PWM controller: `pwm2_request()`, `pwm2_config()`, `pwm2_set_level()` and `pwm2_release()` for the backlight and the R1's LEDs, waveforms played by DMA, and `/dev/jz_pwm` with its ioctls |
| `soc_i2c.ko` | `modules/soc_i2c` | both | the two X1600 I2C controllers as adapters 0 and 1, the buses of the on-board chips such as the touch panel: pins and speed from the `i2cN_*` parameters, transfers through the FIFOs from the interrupt handler, the controller clock on only during a transfer |
| `soc_fb.ko` | `modules/soc_fb` | both | the X1600 LCD controller as the framebuffer `/dev/fb0`, for the panel the board's panel module describes with `jzfb_register_lcd()`: RGB and smart (8080, 6800, SPI) panels, `frame_num` frames from `rmem_manager.ko` shown in turn by pans (`pan_display_sync` waits for the frame interrupt), mapped write-through; the controller and the panel are set up at every unblank and stopped at every blank; `slcd_read_data_only()` reads a smart panel back |
| `mp2731.ko` | `boards/r3proii/modules` | R3 Pro II | the MPS MP2731 charger as the power supply `mp2731-charger`: its currents and voltages from the parameters at probe, the chip state read at every interrupt, and a worker that sets the input and charge currents for the source and writes what user space sets (`step_charging_enabled` turns charging off with the input path left on, `charge_control_limit`, `input_current_limited`, `input_suspend`, `usb_otg`); the interrupt wakes the system from suspend; and a charge limit of its own, `charge_limit_percent` (50 to 100, read against the fuel gauge's level, restarting 3 % below, the system kept awake on the cable while it is set) with `charge_limit_held` beside it |
| `soc_msc.ko` | `modules/soc_msc` | both | the two X1600 MMC/SD controllers as MMC hosts, msc0 for the Wi-Fi chip's SDIO and msc1 for the microSD slot, set up from the `mscN_*` parameters: commands and their responses from the threaded interrupt, data by descriptor DMA straight from the scatterlist (a single segment below 64 bytes through the FIFO), a timer for requests that never end; the card-detect pin debounced by 200 ms, the Wi-Fi card inserted by hand through `jzmmc_manual_detect()`; `present` in sysfs |

`sa_config_module` is one source for every board: `build.sh` passes the board as `HIBY_MODEL`, which is also the model name the vendor module reports.

What differs from the vendor modules, on purpose:
- **`soc_utils`:** a disable with the count already at 0 is ignored; the vendor module takes the count below 0, and the next enable then leaves the clock off. It can also be unloaded.
- **`gpio_aw95016_add`, `sau`, `sa_config_module`:** unloading frees what loading took (the RSTN GPIO, the sysfs group and the device, an empty `release` for the platform devices). The vendor `sau.ko` does not unregister anything when unloaded.
- **`tcs1421_add`, `leds_pwm_add`, `keyboard_adc`:** a probe that fails, and unloading, give back what was taken (the CFG pins, the sysfs file, the PWM channels, the driver data), and the platform devices have an empty `release`.
- **`tcs1421_add`:** `tcs1421_cfg` reads at most 19 characters of the word and refuses a write with no word (`-EINVAL`); the vendor module copies the word onto its stack whatever its length, and with no word applies whatever the stack held.
- **`keyboard_adc`:** the key table ends after key 8 even when all eight codes are set, and a failed input allocation returns `-ENOMEM` (the vendor module returns 12).
- **`leds_pwm_add`:** without the device tree path of `leds-pwm.c`, which the module's own device, with its LEDs in parameters, never takes; a period of 0 gives 10 kHz without dividing by 0.
- **`keyboard_adc_multifunc`, `sa_sound_switch`, `sa_earpods_adc`, `leds_sgm31324_add`:** unloading and failed probes give back what was taken (pins, switch devices, sysfs groups, the regulator, the timer, the pattern memory), and the platform devices have an empty `release`.
- **`keyboard_adc_multifunc`:** a spinlock in place of the vendor mutex, which its timer takes in interrupt context; the key table ends after key 8.
- **`sa_sound_switch`:** writing "on" to `enable` while it is on leaves it on; the vendor module turns polling off.
- **`sa_earpods_adc`:** a missing regulator is left out (the vendor module enables the error pointer); remove does not free the input device twice.
- **`leds_sgm31324_add`:** `alloc=0` frees the parsed words too; a failed driver registration fails the load.
- **`codec_cs43198_dual`:** the `write_reg_val` and `reg_val` files refuse a line they cannot parse (`-EINVAL`), where the vendor module writes or reads with whatever its stack held, and `reg_val` reads at most 11 characters of its command; a failed probe or remove gives back the pins and the sysfs group; a failed driver registration fails the load; after remove `cs43198_set_dsd_en()` does nothing; without the vendor module's unexported no-op `codec_*` and `cs43198_platform_*` functions.
- **`lcd_st7701_sbtc033001`:** a missing vccio regulator also gives back the vcc one. The SPI bits are timed with busy waits of 1 us, where the vendor module sleeps for 1 us at every step and each sleep costs a reschedule: the panel set-up at every screen-on is sent in a few milliseconds, with the same pin sequence and the panel's own delays unchanged.
- **`soc_efuse`:** the ioctls copy from and to user space with `copy_from_user`/`copy_to_user` and check offset and length against the segment; the vendor module uses the user pointers directly. The misc device has an owner, so the module cannot go while it is open; a failed misc registration fails the load, and a failed load gives back the clocks and the pin.
- **`cw2015`:** a load without the gauge on the bus, or with a `fuel_gauge` of the wrong length, fails (the vendor module loads and does nothing); a failed power supply or workqueue fails the probe, and remove stops the worker and frees what the probe took.
- **`codec_cs43131`:** as for `codec_cs43198_dual`: `write_reg_val` and `reg_val` refuse a line they cannot parse, failed probes and remove give back the pins and the sysfs group, a failed driver registration fails the load, after remove `cs43131_set_power()` does nothing. `Digital Filter` sets the PCM filter (0..3, as on the CS43198), and `NOS_EN` turns on the CS43131's non-oversampling emulation; the vendor module's `Digital Filter` does nothing and it has no `NOS_EN`.
- **`cst8xx_touch`:** it never writes firmware to the panel: the vendor module carries a firmware image and flashes it when the panel's version is older or its chip ID cannot be read. Without the vendor module's hex dump of every frame to the kernel log. The number of contacts comes from `cst_max_touch_number`, 1 or 2 (the vendor module reports one; Sonix patched its binary for two). Failed probes and unloading put each regulator once and give back the pins, the interrupt and the workqueue.
- **`gt9xx_touch`:** every contact of a frame is reported: the vendor module reports a contact only when three jiffies have passed since the last one, so one contact a frame and never the very first, and drops a frame with more contacts than `gtp_max_touch_number` (here the first ones are reported). A frame that cannot be read turns the interrupt back on (the vendor module leaves it off, and the panel dead until the next screen-on); when the contacts after the first cannot be read, the first one alone is reported (the vendor module reports uninitialised memory). A blank before the panel came back from the last one, or an unblank with gesture wake switched off meanwhile, does not free or request the interrupt a second time. Probe fails cleanly when the interrupt or reset pin, the input device or its registration fails, and gives back only the pins it took (the vendor module carries on, or frees pins it does not own); remove waits for the two workers, and a failed driver or device registration at load gives back the workqueue. `/proc/gt9xx_config` prints the panel's bytes above 0x7F as bytes (the vendor module prints them sign-extended, as 0xFFFFFF80) and copies at most the size asked for to user space (the vendor module writes the whole text through the user pointer). The INT pin sync leaves out the vendor module's `gpio_set_func(pin, 20)`, a value that selects no function, and the load does not print a build time. Left out: the firmware update code (`gup_update_proc()` and its helpers, which nothing calls) and the configuration copy only it uses.
- **`soc_adc`:** a missing clock, interrupt or misc device fails the load and gives back what was taken (the vendor module hits `BUG()` or carries on); the vref ioctl reads its argument with `get_user`.
- **`rmem_manager`:** it no longer depends on `utils.ko`. User pointers go through `get_user`/`put_user`/`copy_from_user`; a failed allocation is not recorded (the vendor module records address 0 and frees it later), and a full block table fails the allocation. Failed loads unwind, a failed misc registration fails the probe, and remove frees the block table and empties the lists (the vendor module frees the first free block, which can be the list head).
- **`keyboard_gpio_add`:** reading `keyboard` before `register` lists the keys written so far (the vendor module reads them through a pointer that is still NULL); a line refused frees its `tag`, `alloc=0` frees the parsed words, a failed registration gives the keys back and lists none. Unloading takes away the device registered through the parameter and the keys not registered.
- **`i2c_gpio_add`:** a bus without `sda=` is refused (the vendor module registers it and the probe then fails); a failed registration also puts the device.
- **`lcd_lg35583`:** as for `lcd_st7701_sbtc033001`, a missing vccio regulator also gives back the vcc one, and the SPI steps are busy waits of 1 us. `/proc/lcd_reg` reads at most 127 characters, terminates them and ignores a line without two numbers (the vendor module copies the whole write into a 128-byte stack buffer and parses it unterminated). Unloading removes `/proc/lcd_reg`; the vendor module leaves it pointing at freed code.
- **`sa_sound_hbc3000`:** the FPGA configuration is not compiled in: it is loaded with `request_firmware()` from `/lib/firmware/hbc3000.fw`, and the user code that tells a loaded FPGA is read from it. Without the file the probe fails and the exported functions return `-ENODEV` (the vendor module's would dereference a NULL device). A pin that cannot be requested is skipped without freeing it (the vendor module frees it, which releases another driver's pin); remove powers the FPGA off and gives back its pins, and unloading unregisters the driver and the device (the vendor module's exit only prints). Left out, as unreachable: the vendor module's delayed reload after a resume (its switch is never set), the regulator supply (no parameter sets one), and its unexported loader and SPI helpers that nothing calls.
- **`utils`:** `/dev/log_manager` copies from and to user space with `copy_from_user`/`copy_to_user` (the vendor module uses the user pointers directly), reads at most 1024 bytes of a record and returns the bytes it delivered, at most the count asked for (the vendor module returns the record's length); a failed allocation at open returns `-ENOMEM` (the vendor module hits `BUG()` or dereferences NULL), the misc device has an owner, and unloading frees the ring. `gpio_port_set_func()` stops at pin 31: the vendor loop shifts the mask by the pin number modulo 32 and never ends when pin 31 is set. `spi_register_device()` drops the references it takes on the devices it finds and on the master. A failed misc or driver registration fails the load (the vendor module hits `BUG()` or carries on). Left out: two unexported string-to-number helpers nothing calls.
- **`soc_gpio`:** the ioctls copy their arguments and pin and function names from user space and their results to it (the vendor module uses the user pointers directly); names are read up to 31 characters. The misc device has an owner, and a failed registration fails the load (the vendor module panics).
- **`sa_hgl_dma`:** `mmap` checks offset and length against the block it allocated, without overflow; the vendor module checks them against the parameter, which can change after the open, and an offset near 4 GB wraps past its check and maps memory outside the block.
- **`soc_aic`:** remove also unprepares the clocks it disables (the vendor module only disables them); the DMA slave configuration starts zeroed (the vendor module leaves the FIFO address of the other direction and `device_fc` with whatever its stack held). Left out: the vendor module's unexported register helpers and DMA filter that nothing calls, and two clock pointers it never uses.
- **`soc_pwm`:** a channel whose pin cannot be requested is released properly (the vendor module unlocks the mutex of the entry before the table and leaves its own held); releasing a channel that loops a DMA waveform stops it under the lock already held (the vendor module takes the lock a second time and hangs); a failed DMA descriptor frees only the memory it allocated, not the caller's looped buffer; `/dev/jz_pwm` refuses a channel number above 7 for its two flag ioctls (the vendor module writes past the table), and frees its copy of a waveform when the user one cannot be read. The DMA slave configuration starts zeroed. A missing clock or a failed misc registration fails the load and gives back what was taken (the vendor module hits `BUG()`). Left out: four unexported helpers nothing calls.
- **`soc_i2c`:** the APB clock is checked and put back: without it the SDA set-up time is 0 (the vendor module asks for the rate of the error pointer and keeps the reference). Unloading does not disable the controller clock, which runs only during a transfer (the vendor module disables it once more than it enabled it).
- **`mp2731`:** the charger interrupt is a wake source: plugging or pulling the cable wakes the system from suspend (the vendor module leaves it asleep, and a cable plugged in during standby goes unnoticed until the next wake-up), and an interrupt during the way into suspend aborts it. A failed probe after the interrupt and the power supply are set up gives them back, with the notifiers and their work (the vendor module leaves them registered), and the interrupt work does not touch the power supply before it is registered. The USB notifier goes on one phy only (the vendor module puts the same notifier block on two chains when both phys exist). Remove also stops the interrupt and its work, the notifier's and the USB work, takes the USB notifier off before the power supply, and frees the pins; the reference taken on the followed power supply is put back. A load without the device on the bus fails. Without the vendor module's messages at every interrupt, at load and probe, and for every register it sets. Added: the charge limit (`charge_limit_percent` and `charge_limit_held` in the power supply's folder, and a `charge_limit_percent` parameter for the starting value, 100 for none): at or above the limit the worker clears CHG_CONFIG, as `step_charging_enabled` does, and sets it again 3 % below; the cable going out releases it, and so do shutdown and remove, which stop the worker first. While the cable is in and a limit is set, a wakeup source keeps the system out of suspend.
- **`soc_msc`:** removing a controller (unbind, or unloading) undoes the probe: the card-detect interrupt and its timer stop first, the core lets go of the card, and the controller interrupt, the request timer, the slot and bus pins, the descriptor page, the regulator and the clocks are given back before the host is freed; it also leaves the list `jzmmc_manual_detect()` looks in. The vendor module frees the host first and then keeps using it, frees its two device-managed interrupts by hand (devres frees them again: `Trying to free already-free IRQ`), passes a pointer into the freed host to `kfree`, never frees a pin and leaves its timer and list entry behind, so a second bind fails to get its pins and the kernel fails soon after. A probe that fails unwinds and returns the error (the vendor module returns 0 for a missing clock or a rate it cannot set, prints through a NULL host when the allocation fails, and ignores a failed `mmc_add_host`); the slot pins are requested with lasting names (the vendor module names them from its stack); the bus pin loop stops at the six pins of the table (`bus_width=8` reads past it in the vendor module); writing `present` reads the removal method from the host (the vendor module reads it through the device's `platform_data`, which is NULL: an oops). A request timing out after a failed transfer does not unmap through a NULL data pointer, and the descriptor dump is skipped without descriptors (`mscN_pio_mode`); both oops in the vendor module. The descriptor page is written back from the cache once zeroed, before it is used uncached. `jzmmc_of_parse_voltage()` refuses an index beyond the two controllers; a failed registration at load gives back what was registered and fails the load. The bad `bus-width` message prints the value (the vendor module prints 0).

### The HBC3000's configuration
The open `sa_sound_hbc3000.ko` needs the FPGA configuration HiBy compiled into the vendor module, its 34304-byte `FS_data` array. It is not in this repository; take it out of the stock module and put it in the root filesystem:

```
python3 tools/hbc3000-firmware.py <rootfs>/module_driver/sa_sound_hbc3000.ko hbc3000.fw
cp hbc3000.fw <rootfs>/lib/firmware/hbc3000.fw
```

Without it the module does not load and the R3 Pro II has no sound.

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
