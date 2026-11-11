# libzk4500 – ZKSoftware ZK4500 fingerprint reader on Linux

Linux support for the **ZKSoftware ZK4500** optical fingerprint reader
(USB `1b55:0840`), which the vendor states is not supported on Linux.

The repository contains:

- **libzk4500**: a small C library (libusb-1.0) for image capture, finger
  detection, LEDs and beeper
- **a libfprint driver** (`packaging/zk4500.c`) that makes the reader work with
  fprintd, `fprintd-enroll` and `pam_fprintd` (login, lock screen, sudo)
- an **Arch/Manjaro PKGBUILD** for libfprint with the driver included

The official ZKFinger Linux SDK (`libzkfp.so`) only accepts `1b55:0120`
(ZK9500/SLK20R). The ZK4500 USB protocol was reconstructed for interoperability
by static analysis of the vendor's Windows capture library (`libzklibcap.dll`,
ZKFinger SDK 5.3) using [REA](https://github.com/morluto/rea) and Ghidra.
No vendor code or binaries are included in this repository.

## Status

| Feature                                            | Status |
|----------------------------------------------------|--------|
| Device discovery, calibration read from EEPROM     | ✅ tested |
| Raw 640×480 frame                                  | ✅ tested |
| Geometry-corrected 284×368 image (same as vendor SDK) | ✅ tested |
| Finger detection, `wait_finger`                    | ✅ tested |
| Green/red LED, beeper                              | ✅ tested |
| libfprint driver: enroll/verify with fprintd       | ✅ tested |
| Vendor matching algorithm (`libzkfinger10`)        | ❌ see below |

## Build

Dependency: `libusb-1.0` (headers and pkg-config).

```bash
# allow non-root access to the reader (re-plug afterwards)
sudo cp udev/60-zk4500.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger -s usb

make                                  # build/libzk4500.so, build/libzk4500.a, tools
./build/zk4500_capture finger.bmp     # place finger -> beep -> 8-bit BMP (500 dpi)
sudo make install                     # optional, PREFIX=/usr/local
```

## Library API (excerpt, `include/zk4500.h`)

```c
zk4500_t *dev;
zk4500_open(&dev, 0);                       // init, EEPROM calibration, illumination on
int n = zk4500_image_size(dev);             // 284 * 368
uint8_t *img = malloc(n);
if (zk4500_wait_finger(dev, img, n, 10000) == 1) {
    zk4500_set_led(dev, 1, 0);              // green on
    zk4500_beep(dev, 80);
    zk4500_write_bmp("finger.bmp", img, 284, 368);
}
zk4500_close(dev);
```

Further functions: `zk4500_capture()` (single frame, returns 1 if a finger is
present), `zk4500_capture_raw()`, `zk4500_wait_release()`, `zk4500_set_gpio()`,
`zk4500_set_detect()` (detection thresholds), `zk4500_get_info()`.
Set `ZK4500_DEBUG=1` to print finger-detection values to stderr.

## Fingerprint login with fprintd

The libfprint driver uses the same protocol and image pipeline; matching is done
by libfprint's built-in NBIS matcher. Build and install the package, then enroll:

```bash
cd packaging
makepkg -si                 # replaces the distribution's libfprint package
sudo pacman -S fprintd
fprintd-enroll -f right-index-finger
fprintd-verify
```

PAM setup (LightDM, xfce4-screensaver, sudo with a hardware token and password
fallback over SSH) is described in [packaging/LOGIN.md](packaging/LOGIN.md).

## Protocol

| Action          | bmRequestType / bRequest | wValue | wIndex  | Data |
|-----------------|--------------------------|--------|---------|------|
| Init            | `0x40` / `0xE0`          | 0      | 0       | – |
| Read EEPROM     | `0xC0` / `0xE7`          | 0      | address | 1 byte |
| Set GPIO        | `0x40` / `0xE1`          | state  | GPIO    | – |
| Request image   | `0x40` / `0xE5`          | 0      | 0       | then bulk IN EP `0x82`, 307200 bytes |
| Close           | `0x40` / `0xE1`          | 0      | `0x17`  | – |

GPIOs (verified on hardware): 0 = green LED, 1 = red LED, 2 = beeper.
On open the vendor driver sends `E1 v=0 i=0x32`. The green prism illumination is
hard-wired and cannot be switched off.

EEPROM calibration block at `0x3B`, 15 × u16 little endian: crop (x, y, w, h),
trapezoid corners BL/BR/TL/TR, output width (flags in bits 12–15), output
height, checksum `(1 + Σ) & 0x7FFF`. `0x9A` = sensor type (`0x07D1`),
`0x63..0x8A` = serial number (GUID string).

Image pipeline as in the vendor's `sensorCapture`: 2×2 smoothing → crop →
perspective correction of the prism trapezoid to 284×368 (bilinear) →
horizontal mirror (sensor type `0x07D1`).

## Why not the vendor's matching algorithm?

ZKTeco's `libzkfinger10.so` (BIOKEY) checks a hardware licence in the sensor via
a callback during `BIOKEY_INIT` (vendor request `0xF3`, challenge `0x84`). The
ZK4500 does not implement this request, so initialisation fails ("Limit:0").
This project deliberately does not bypass that licence check. Use an open-source
matcher instead: libfprint (NBIS) via the included driver, or e.g. SourceAFIS
with the 500 dpi images from libzk4500.

## Tools

- `build/zkprobe`: prints device info and EEPROM and takes a test frame (read-only)
- `build/zkctl`: sends raw vendor requests, e.g. `zkctl out:0xE1:1:0 img:frame.pgm`

## Repository layout

```
include/, src/          libzk4500
examples/               zk4500_capture
tools/                  zkprobe, zkctl
udev/                   udev rule
packaging/              libfprint driver (zk4500.c), PKGBUILD, LOGIN.md
pam/                    pam-is-ssh helper for sudo over SSH
```

## Disclaimer

Not affiliated with or endorsed by ZKTeco / ZKSoftware. All trademarks belong to
their owners. Fingerprint authentication with an optical sensor is convenient but
weaker than a password or a hardware token: optical sensors can be fooled with
fake fingerprints.

## License

LGPL-2.1-or-later, see [LICENSE](LICENSE). The libfprint driver must be
LGPL-2.1-or-later to be combined with libfprint.
