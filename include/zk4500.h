/*
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (C) 2026 Michael Sauer
 *
 * libzk4500 - Linux userspace driver for the ZKSoftware ZK4500 fingerprint
 * reader (USB 1b55:0840), built on libusb-1.0.
 *
 * The USB protocol was recovered from ZKTeco's libzklibcap.dll / libsilkidcap.so
 * (ZKFinger SDK 5.3) by static analysis.
 */
#ifndef ZK4500_H
#define ZK4500_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZK4500_VID 0x1b55
#define ZK4500_PID 0x0840

/* error codes (negative); >= 0 means success */
#define ZK4500_OK            0
#define ZK4500_ERR_USB      -1   /* libusb error                      */
#define ZK4500_ERR_NODEV    -2   /* no ZK4500 found                   */
#define ZK4500_ERR_ACCESS   -3   /* permission denied (udev rule?)    */
#define ZK4500_ERR_PARAM    -4   /* invalid argument / buffer small   */
#define ZK4500_ERR_EEPROM   -5   /* calibration block invalid         */
#define ZK4500_ERR_TIMEOUT  -6   /* no finger within timeout          */
#define ZK4500_ERR_NOMEM    -7

/* GPIO outputs (vendor request 0xE1, wIndex = gpio, wValue = state).
 * Verified on hardware: 0 = green status LED, 1 = red status LED, 2 = beeper.
 * GPIO 3 ("white light" in the Windows SDK) has no visible effect on the ZK4500.
 * The green prism illumination is hard-wired and cannot be switched off. */
#define ZK4500_GPIO_GREEN   0
#define ZK4500_GPIO_RED     1
#define ZK4500_GPIO_BEEP    2
#define ZK4500_GPIO_WHITE   3

typedef struct zk4500 zk4500_t;

typedef struct {
    char     manufacturer[64];
    char     product[64];
    char     serial[64];       /* GUID stored in EEPROM 0x63..0x8a       */
    uint16_t vid, pid, bcd_device;
    int      raw_width;        /* sensor frame, 640                       */
    int      raw_height;       /* sensor frame, 480                       */
    int      width;            /* corrected image (from EEPROM), 284      */
    int      height;           /* corrected image (from EEPROM), 368      */
    int      sensor_type;      /* EEPROM 0x9a, 0x07d1 on ZK4500           */
    int      quad[8];          /* source trapezoid: BLx,BLy,BRx,BRy,TLx,TLy,TRx,TRy */
} zk4500_info_t;

/* number of connected ZK4500 devices (or negative error) */
int  zk4500_count(void);

/* open device #index (0-based). Performs init, reads calibration from EEPROM,
 * switches the sensor illumination on. */
int  zk4500_open(zk4500_t **dev, int index);
void zk4500_close(zk4500_t *dev);

int  zk4500_get_info(zk4500_t *dev, zk4500_info_t *info);

/* corrected image size in bytes (width*height, 8 bit gray) */
int  zk4500_image_size(zk4500_t *dev);

/* Read one unprocessed 640x480 sensor frame. Returns bytes read. */
int  zk4500_capture_raw(zk4500_t *dev, uint8_t *buf, size_t len);

/* Capture one frame and produce the geometry-corrected image (width x height,
 * 8-bit grayscale, ridges dark). Returns 1 if a finger is present, 0 if not,
 * negative on error. The image is written in both cases. */
int  zk4500_capture(zk4500_t *dev, uint8_t *img, size_t len);

/* Poll until a finger is present (or timeout_ms elapsed, <0 = forever).
 * Returns 1 with img filled, ZK4500_ERR_TIMEOUT or other negative error. */
int  zk4500_wait_finger(zk4500_t *dev, uint8_t *img, size_t len, int timeout_ms);

/* Poll until the sensor is empty again. */
int  zk4500_wait_release(zk4500_t *dev, int timeout_ms);

/* GPIO/LED/beeper control (state 0/1) */
int  zk4500_set_gpio(zk4500_t *dev, int gpio, int state);
/* status LEDs: green/red each 0 = off, 1 = on */
int  zk4500_set_led(zk4500_t *dev, int green, int red);
/* beep for ms milliseconds (blocking) */
int  zk4500_beep(zk4500_t *dev, int ms);

/* Finger detection threshold tuning: finger if >= min_dark_ratio of the
 * pixels are darker than the background by dark_delta. Defaults 0.15 / 40. */
void zk4500_set_detect(zk4500_t *dev, double min_dark_ratio, int dark_delta);

/* image helpers */
int  zk4500_write_pgm(const char *path, const uint8_t *img, int w, int h);
int  zk4500_write_bmp(const char *path, const uint8_t *img, int w, int h);

const char *zk4500_strerror(int err);

#ifdef __cplusplus
}
#endif
#endif
