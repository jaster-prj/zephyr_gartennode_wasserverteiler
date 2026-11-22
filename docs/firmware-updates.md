# Water distributor firmware updates

The update fixes are ported from the cistern module. Cistern hardware tests
do not establish that this module has passed a hardware update test.
Node ID remains **14** (`0x0e`). Valve, relay, Hall-sensor and touch behavior,
NMT startup policy and application watchdog configuration are unchanged.

## Build and release

Use the Gartennode west workspace and matching private signing key at
`bootloader/mcuboot/gartennode.pem`. Never commit or publish the key.
From the workspace root:

```sh
west build -b gartennode application -d build/wasserverteiler
python application/scripts/check_firmware.py \
  build/wasserverteiler/zephyr/zephyr.signed.bin \
  build/wasserverteiler/zephyr/.config
python bootloader/mcuboot/scripts/imgtool.py verify \
  --key bootloader/mcuboot/gartennode.pem \
  build/wasserverteiler/zephyr/zephyr.signed.bin
python application/scripts/prepare_eds.py \
  build/wasserverteiler/zephyr/.config application/objdict/objdict.eds objdict.eds
python -m unittest discover -s application/scripts -p 'test_*.py'
```

Publish Zephyr's `zephyr.signed.bin`, not an image re-signed with a hard-coded
header size. Zephyr 4.5 uses a `0x400` STM32 header. The default signed version
is `1.3.1`; override `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` for each new release.
The running `0x100A` string and generated release EDS use that same version.

Both application and supplied MCUboot configuration use **swap using scratch**:

| Region | Address/offset | Size |
| --- | --- | --- |
| MCUboot, internal | `0x08000000` | 64 KiB |
| Primary, internal | `0x08010000` | 192 KiB |
| Secondary, external | `0` | 192 KiB |
| Scratch, external | `0x30000` | 192 KiB |

Build matching MCUboot separately:

```sh
west build -b gartennode bootloader/mcuboot/boot/zephyr \
  -d build/wasserverteiler-boot \
  -- -DEXTRA_CONF_FILE="$PWD/application/bootloader.conf"
```

Verify the installed bootloader's layout and signing key before deployment.
The supplied bootloader has watchdog support disabled; the application retains
`CONFIG_WATCHDOG=y`, `CONFIG_TASK_WDT=y` and its existing disable-at-boot setting.
This port does not change watchdog initialization, channel registration or feeding.
Zephyr 4.5 warns that this STM32 configuration does not support
`CONFIG_WDT_DISABLE_AT_BOOT`; the existing assignment is retained but does not
become effective in the generated configuration. Do not rely on it to disable
an already-running hardware watchdog.

## CAN update

Use node `0xe` and the uploaded image's version:

```sh
curl --cacert CA.pem \
  -H 'Content-Type: application/octet-stream' \
  --data-binary @build/wasserverteiler/zephyr/zephyr.signed.bin \
  'https://YOUR_GATEWAY/canopenrest/api/v1/flash?node=0xe&version=1.3.1'
```

Poll `GET /canopenrest/api/v1/flash?id=<returned-id>` to completion.
Require reboot, matching `0x100A`, and explicit confirmation—not just successful
data reception.

STOP/CLEAR/START retain upstream PRE-OPERATIONAL restrictions. CLEAR erases the
secondary slot; reception-time stream-flash erasing is disabled to avoid
erasing it again during SDO handling.

The adapter permits explicit `0x1F51:1 = 0x80` confirmation while OPERATIONAL
as well as PRE-OPERATIONAL, without requiring another NMT transition after
gateway verification. It never automatically confirms an image; MCUboot
rollback remains available for unconfirmed test images.

The cistern's swap/startup took about 16.8 seconds, longer than the gateway's
fixed ten-second initial wait. Allow adequate reboot/verification time;
water distributor timing has not been measured.

## SWD recovery

Back up internal flash before recovery. Use a **full padded BIN**, not sparse
HEX: sparse programming can leave old swap-status bytes in a primary trailer
page and cause MCUboot to interpret an invalid interrupted swap.

Generate a confirmed baseline separately:

```sh
west build -b gartennode application -d build/wasserverteiler-recovery \
  -- -DCONFIG_MCUBOOT_GENERATE_CONFIRMED_IMAGE=y
```

Verify layout/signature and exact `0x30000`-byte length before programming
`zephyr.signed.confirmed.bin` at `0x08010000`. Resolve pending external swap
state first; do not resume a pending swap against an unrelated replacement
primary. Avoid mass erases of the bootloader or external settings.
Do not use cistern or pool firmware as a water distributor recovery image.

## Optional diagnostics

Add `-DEXTRA_CONF_FILE=debug/update.conf` when building the application.
It records reset causes, download byte counts and callback latency using
deferred RTT logs. Sensor/touch log noise is filtered without disabling those
functions; valve and relay logs remain available.
Read `wasserverteiler_update_diagnostics` and RTT only after a transfer;
live SWD reads or halts can disturb timing.

`debug/bootloader.conf` is an optional second MCUboot fragment for RTT output.
Native-sim continues to disable MCUboot and program download.

With a UDP-CAN module that does not advertise its devicetree root, supply it
explicitly when building native-sim:

```sh
west build -b native_sim application -d build/wasserverteiler-native -- \
  -DZEPHYR_EXTRA_MODULES=/workspace/zephyr-native-linux-udp-can \
  -DDTS_ROOT=/workspace/zephyr-native-linux-udp-can \
  -DEXTRA_CONF_FILE=boards/native_sim.conf
```

## Port validation

The normal application, diagnostic application, matching MCUboot and native-sim
builds passed with Zephyr 4.5.0. Both hardware images are 196608 bytes and pass
layout/vector-table and signature verification; generated EDS version is `1.3.1`.
All 11 host regression tests pass. No water distributor hardware was flashed
or tested as part of this port.
