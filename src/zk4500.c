/*
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (C) 2026 Michael Sauer
 *
 * libzk4500 - Linux userspace driver for the ZKSoftware ZK4500 (1b55:0840).
 *
 * Protocol (recovered from ZKFinger SDK 5.3 libzklibcap.dll / libsilkidcap.so):
 *   open      : SET_CONFIGURATION 1, claim interface 0
 *               vendor OUT 0xE0 (init)
 *   EEPROM    : vendor IN  0xE7, wIndex = address, 1 byte
 *   light on  : vendor OUT 0xE1, wValue = 0, wIndex = 0x32
 *   GPIO      : vendor OUT 0xE1, wValue = state, wIndex = gpio
 *   capture   : vendor OUT 0xE5, then bulk IN EP 0x82, 640*480 bytes
 *   close     : vendor OUT 0xE1, wValue = 0, wIndex = 0x17
 *
 * EEPROM calibration block at 0x3b (15 little-endian u16, checksum last):
 *   [0..3]  crop x, crop y, crop w, crop h          (0,0,640,480)
 *   [4..11] trapezoid BL(x,y) BR(x,y) TL(x,y) TR(x,y)
 *   [12]    output width  (low 12 bits) + flags (bits 12..15)
 *   [13]    output height (low 12 bits)
 *   [14]    checksum = (1 + sum[0..13]) & 0x7fff
 *   0x98 u16, 0x9a u16 sensor type, 0x63..0x8a serial (GUID string)
 *
 * Image pipeline (as in sensorCapture/FUN_180001e00):
 *   optional invert (flag bit 0) -> 2x2 box smoothing -> crop
 *   -> perspective correction of the trapezoid onto width x height
 *   -> sensor type 0x7d1: mirror rows horizontally, otherwise flip vertically
 */
#include "zk4500.h"

#include <libusb.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define RAW_W 640
#define RAW_H 480
#define EP_IN 0x82
#define CTRL_TIMEOUT 1000
#define BULK_TIMEOUT 2000

#define REQ_INIT    0xE0
#define REQ_GPIO    0xE1
#define REQ_CAPTURE 0xE5
#define REQ_EEPROM  0xE7

struct zk4500 {
    libusb_context *ctx;
    libusb_device_handle *h;
    zk4500_info_t info;
    int invert;
    int mirror_h;
    int crop_x, crop_y, crop_w, crop_h;
    uint8_t *raw;          /* RAW_W*RAW_H */
    int32_t *lut_idx;      /* per output pixel: source index (top-left), -1 = outside */
    uint8_t *lut_fx;       /* bilinear weights in 1/16 */
    uint8_t *lut_fy;
    double detect_ratio;
    int detect_delta;
};

/* ------------------------------------------------------------------ */

static int map_usb_err(int r)
{
    switch (r) {
    case LIBUSB_ERROR_ACCESS:    return ZK4500_ERR_ACCESS;
    case LIBUSB_ERROR_NO_DEVICE:
    case LIBUSB_ERROR_NOT_FOUND: return ZK4500_ERR_NODEV;
    case LIBUSB_ERROR_NO_MEM:    return ZK4500_ERR_NOMEM;
    case LIBUSB_ERROR_TIMEOUT:   return ZK4500_ERR_TIMEOUT;
    default:                     return ZK4500_ERR_USB;
    }
}

const char *zk4500_strerror(int err)
{
    switch (err) {
    case ZK4500_OK:          return "ok";
    case ZK4500_ERR_USB:     return "USB communication error";
    case ZK4500_ERR_NODEV:   return "no ZK4500 device found";
    case ZK4500_ERR_ACCESS:  return "permission denied (install the udev rule)";
    case ZK4500_ERR_PARAM:   return "invalid parameter or buffer too small";
    case ZK4500_ERR_EEPROM:  return "invalid calibration data in device EEPROM";
    case ZK4500_ERR_TIMEOUT: return "timeout";
    case ZK4500_ERR_NOMEM:   return "out of memory";
    default:                 return "unknown error";
    }
}

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static int vendor_out(zk4500_t *d, uint8_t req, uint16_t val, uint16_t idx)
{
    int r = libusb_control_transfer(d->h, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
                                    req, val, idx, NULL, 0, CTRL_TIMEOUT);
    return r < 0 ? map_usb_err(r) : 0;
}

static int eeprom_read(zk4500_t *d, int addr, uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++) {
        int r = libusb_control_transfer(d->h, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
                                        REQ_EEPROM, 0, (uint16_t)(addr + i), &buf[i], 1, CTRL_TIMEOUT);
        if (r != 1) return r < 0 ? map_usb_err(r) : ZK4500_ERR_USB;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* perspective mapping: unit square -> quad (Heckbert)                 */

typedef struct { double a, b, c, d, e, f, g, h; } persp_t;

static void square_to_quad(persp_t *m, const double x[4], const double y[4])
{
    /* corners in order (0,0) (1,0) (1,1) (0,1) */
    double sx = x[0] - x[1] + x[2] - x[3];
    double sy = y[0] - y[1] + y[2] - y[3];
    if (fabs(sx) < 1e-9 && fabs(sy) < 1e-9) {
        m->a = x[1] - x[0]; m->b = x[2] - x[1]; m->c = x[0];
        m->d = y[1] - y[0]; m->e = y[2] - y[1]; m->f = y[0];
        m->g = m->h = 0;
    } else {
        double dx1 = x[1] - x[2], dx2 = x[3] - x[2];
        double dy1 = y[1] - y[2], dy2 = y[3] - y[2];
        double den = dx1 * dy2 - dx2 * dy1;
        m->g = (sx * dy2 - dx2 * sy) / den;
        m->h = (dx1 * sy - sx * dy1) / den;
        m->a = x[1] - x[0] + m->g * x[1];
        m->b = x[3] - x[0] + m->h * x[3];
        m->c = x[0];
        m->d = y[1] - y[0] + m->g * y[1];
        m->e = y[3] - y[0] + m->h * y[3];
        m->f = y[0];
    }
}

static int build_lut(zk4500_t *d)
{
    int W = d->info.width, H = d->info.height, n = W * H;
    const int *q = d->info.quad;   /* BL, BR, TL, TR */
    double qx[4] = { q[4], q[6], q[2], q[0] };   /* TL TR BR BL */
    double qy[4] = { q[5], q[7], q[3], q[1] };
    persp_t m;
    square_to_quad(&m, qx, qy);

    d->lut_idx = malloc(sizeof(int32_t) * n);
    d->lut_fx = malloc(n);
    d->lut_fy = malloc(n);
    if (!d->lut_idx || !d->lut_fx || !d->lut_fy) return ZK4500_ERR_NOMEM;

    for (int j = 0; j < H; j++) {
        double v = (j + 0.5) / H;
        for (int i = 0; i < W; i++) {
            double u = (i + 0.5) / W;
            double w = m.g * u + m.h * v + 1.0;
            double sx = (m.a * u + m.b * v + m.c) / w;
            double sy = (m.d * u + m.e * v + m.f) / w;
            int k = j * W + i;
            if (sx < 0 || sy < 0 || sx > d->crop_w - 1 || sy > d->crop_h - 1) {
                d->lut_idx[k] = -1;
                continue;
            }
            int ix = (int)sx, iy = (int)sy;
            int fx = (int)((sx - ix) * 16 + 0.5), fy = (int)((sy - iy) * 16 + 0.5);
            if (fx == 16) { ix++; fx = 0; }
            if (fy == 16) { iy++; fy = 0; }
            if (ix >= d->crop_w - 1) { ix = d->crop_w - 1; fx = 0; }
            if (iy >= d->crop_h - 1) { iy = d->crop_h - 1; fy = 0; }
            d->lut_idx[k] = iy * d->crop_w + ix;
            d->lut_fx[k] = (uint8_t)fx;
            d->lut_fy[k] = (uint8_t)fy;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */

static int read_calibration(zk4500_t *d)
{
    uint8_t blk[30], b[4], serial[0x28 + 1];
    int r;

    if ((r = eeprom_read(d, 0x0a, b, 4))) return r;
    if (b[0] == 'Z' && b[1] == 'K' && b[2] == 'M' && b[3] == 0)
        return ZK4500_ERR_EEPROM;   /* different sensor family, not handled */

    if ((r = eeprom_read(d, 0x9a, b, 2))) return r;
    d->info.sensor_type = b[0] | b[1] << 8;

    memset(serial, 0, sizeof serial);
    if (eeprom_read(d, 0x63, serial, 0x28) == 0 && serial[0] != 0xff)
        snprintf(d->info.serial, sizeof d->info.serial, "%s", (char *)serial);

    if ((r = eeprom_read(d, 0x3b, blk, sizeof blk))) return r;
    uint16_t s[15];
    for (int i = 0; i < 15; i++) s[i] = blk[2 * i] | blk[2 * i + 1] << 8;
    uint16_t sum = 1;
    for (int i = 0; i < 14; i++) sum += s[i];
    if ((sum & 0x7fff) != s[14]) return ZK4500_ERR_EEPROM;

    d->crop_x = s[0]; d->crop_y = s[1]; d->crop_w = s[2]; d->crop_h = s[3];
    if (d->crop_w <= 0 || d->crop_w > RAW_W) d->crop_w = RAW_W;
    if (d->crop_h <= 0 || d->crop_h > RAW_H) d->crop_h = RAW_H;
    if (d->crop_x + d->crop_w > RAW_W) d->crop_x = RAW_W - d->crop_w;
    if (d->crop_y + d->crop_h > RAW_H) d->crop_y = RAW_H - d->crop_h;
    for (int i = 0; i < 8; i++) d->info.quad[i] = (int16_t)s[4 + i];
    d->info.width = s[12] & 0xfff;
    d->info.height = s[13] & 0xfff;
    d->invert = (s[12] >> 12) & 1;
    if (d->info.width <= 0 || d->info.height <= 0) return ZK4500_ERR_EEPROM;

    /* FUN_1800017f0: type 0x7d1 -> horizontal mirror, otherwise vertical flip */
    d->mirror_h = d->info.sensor_type == 0x7d1;
    return 0;
}

int zk4500_count(void)
{
    libusb_context *ctx;
    libusb_device **list;
    if (libusb_init(&ctx)) return ZK4500_ERR_USB;
    ssize_t n = libusb_get_device_list(ctx, &list);
    int cnt = 0;
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        if (!libusb_get_device_descriptor(list[i], &dd) &&
            dd.idVendor == ZK4500_VID && dd.idProduct == ZK4500_PID)
            cnt++;
    }
    if (n >= 0) libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return cnt;
}

int zk4500_open(zk4500_t **out, int index)
{
    int r, found = 0;
    libusb_device **list;
    if (!out || index < 0) return ZK4500_ERR_PARAM;
    *out = NULL;

    zk4500_t *d = calloc(1, sizeof *d);
    if (!d) return ZK4500_ERR_NOMEM;
    d->detect_ratio = 0.15;
    d->detect_delta = 40;
    if (libusb_init(&d->ctx)) { free(d); return ZK4500_ERR_USB; }

    ssize_t n = libusb_get_device_list(d->ctx, &list);
    r = ZK4500_ERR_NODEV;
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        if (libusb_get_device_descriptor(list[i], &dd) ||
            dd.idVendor != ZK4500_VID || dd.idProduct != ZK4500_PID)
            continue;
        if (found++ != index) continue;
        int e = libusb_open(list[i], &d->h);
        r = e ? map_usb_err(e) : 0;
        d->info.vid = dd.idVendor;
        d->info.pid = dd.idProduct;
        d->info.bcd_device = dd.bcdDevice;
        if (!e) {
            libusb_get_string_descriptor_ascii(d->h, dd.iManufacturer,
                (unsigned char *)d->info.manufacturer, sizeof d->info.manufacturer);
            libusb_get_string_descriptor_ascii(d->h, dd.iProduct,
                (unsigned char *)d->info.product, sizeof d->info.product);
        }
        break;
    }
    if (n >= 0) libusb_free_device_list(list, 1);
    if (r) goto fail;

    libusb_set_auto_detach_kernel_driver(d->h, 1);
    int cfg = 0;
    if (libusb_get_configuration(d->h, &cfg) || cfg != 1)
        libusb_set_configuration(d->h, 1);
    if ((r = libusb_claim_interface(d->h, 0))) { r = map_usb_err(r); goto fail; }

    /* drop stale frame data left by an aborted reader */
    for (int i = 0; i < 16; i++) {
        unsigned char junk[65536];
        int got = 0;
        if (libusb_bulk_transfer(d->h, EP_IN, junk, sizeof junk, &got, 100) || !got) break;
    }

    if ((r = vendor_out(d, REQ_INIT, 0, 0))) goto fail;
    if ((r = read_calibration(d))) goto fail;
    d->info.raw_width = RAW_W;
    d->info.raw_height = RAW_H;

    if ((r = vendor_out(d, REQ_GPIO, 0, 0x32))) goto fail;   /* illumination on */

    d->raw = malloc(RAW_W * RAW_H);
    if (!d->raw) { r = ZK4500_ERR_NOMEM; goto fail; }
    if ((r = build_lut(d))) goto fail;

    zk4500_capture_raw(d, d->raw, RAW_W * RAW_H);   /* discard first frame */
    *out = d;
    return 0;

fail:
    zk4500_close(d);
    return r;
}

void zk4500_close(zk4500_t *d)
{
    if (!d) return;
    if (d->h) {
        vendor_out(d, REQ_GPIO, 0, 0x17);
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
    }
    if (d->ctx) libusb_exit(d->ctx);
    free(d->raw);
    free(d->lut_idx);
    free(d->lut_fx);
    free(d->lut_fy);
    free(d);
}

int zk4500_get_info(zk4500_t *d, zk4500_info_t *info)
{
    if (!d || !info) return ZK4500_ERR_PARAM;
    *info = d->info;
    return 0;
}

int zk4500_image_size(zk4500_t *d)
{
    return d ? d->info.width * d->info.height : ZK4500_ERR_PARAM;
}

int zk4500_set_gpio(zk4500_t *d, int gpio, int state)
{
    if (!d || gpio < 0 || gpio > 0xff) return ZK4500_ERR_PARAM;
    return vendor_out(d, REQ_GPIO, state ? 1 : 0, (uint16_t)gpio);
}

int zk4500_set_led(zk4500_t *d, int green, int red)
{
    int r = zk4500_set_gpio(d, ZK4500_GPIO_GREEN, green);
    return r ? r : zk4500_set_gpio(d, ZK4500_GPIO_RED, red);
}

int zk4500_beep(zk4500_t *d, int ms)
{
    int r = zk4500_set_gpio(d, ZK4500_GPIO_BEEP, 1);
    if (r) return r;
    usleep(ms > 0 ? ms * 1000 : 0);
    return zk4500_set_gpio(d, ZK4500_GPIO_BEEP, 0);
}

void zk4500_set_detect(zk4500_t *d, double min_dark_ratio, int dark_delta)
{
    if (!d) return;
    d->detect_ratio = min_dark_ratio;
    d->detect_delta = dark_delta;
}

int zk4500_capture_raw(zk4500_t *d, uint8_t *buf, size_t len)
{
    const int n = RAW_W * RAW_H;
    if (!d || !buf || len < (size_t)n) return ZK4500_ERR_PARAM;
    int r = vendor_out(d, REQ_CAPTURE, 0, 0);
    if (r) return r;
    int tot = 0;
    while (tot < n) {
        int got = 0;
        r = libusb_bulk_transfer(d->h, EP_IN, buf + tot, n - tot, &got, BULK_TIMEOUT);
        tot += got;
        if (r || !got) break;
    }
    if (tot != n) {
        /* resync: flush whatever is left in the endpoint */
        unsigned char junk[65536];
        int got = 0;
        libusb_clear_halt(d->h, EP_IN);
        while (!libusb_bulk_transfer(d->h, EP_IN, junk, sizeof junk, &got, 100) && got) {}
        return r ? map_usb_err(r) : ZK4500_ERR_USB;
    }
    return tot;
}

static void process(zk4500_t *d, uint8_t *raw, uint8_t *img)
{
    const int n = RAW_W * RAW_H, W = d->info.width, H = d->info.height;

    if (d->invert)
        for (int i = 0; i < n; i++) raw[i] = 255 - raw[i];

    /* 2x2 box smoothing in place (FUN_1800079f0) */
    for (int y = 0; y < RAW_H - 1; y++) {
        uint8_t *p = raw + y * RAW_W, *q = p + RAW_W;
        for (int x = 0; x < RAW_W - 1; x++)
            p[x] = (uint8_t)((p[x] + p[x + 1] + q[x] + q[x + 1] + 2) >> 2);
    }

    /* crop is folded into the source pointer; LUT indices are crop-relative */
    const uint8_t *src = raw + d->crop_y * RAW_W + d->crop_x;
    const int cw = d->crop_w, ch = d->crop_h;
    for (int k = 0; k < W * H; k++) {
        int idx = d->lut_idx[k];
        if (idx < 0) { img[k] = 0xff; continue; }
        int iy = idx / cw, ix = idx % cw;
        const uint8_t *p = src + iy * RAW_W + ix;
        int fx = d->lut_fx[k], fy = d->lut_fy[k];
        int dx = ix + 1 < cw ? 1 : 0, dy = iy + 1 < ch ? RAW_W : 0;
        int top = p[0] * (16 - fx) + p[dx] * fx;
        int bot = p[dy] * (16 - fx) + p[dy + dx] * fx;
        img[k] = (uint8_t)((top * (16 - fy) + bot * fy + 128) >> 8);
    }

    if (d->mirror_h) {
        for (int y = 0; y < H; y++) {
            uint8_t *row = img + y * W;
            for (int a = 0, b = W - 1; a < b; a++, b--) {
                uint8_t t = row[a]; row[a] = row[b]; row[b] = t;
            }
        }
    } else {
        for (int a = 0, b = H - 1; a < b; a++, b--)
            for (int x = 0; x < W; x++) {
                uint8_t t = img[a * W + x]; img[a * W + x] = img[b * W + x]; img[b * W + x] = t;
            }
    }
}

static int finger_present(zk4500_t *d, const uint8_t *img)
{
    const int n = d->info.width * d->info.height;
    int hist[256] = {0};
    for (int i = 0; i < n; i++) hist[img[i]]++;
    /* background level = 90th percentile (sensor glass is bright) */
    int acc = 0, bg = 255;
    for (int v = 0; v < 256; v++) {
        acc += hist[v];
        if (acc >= n * 9 / 10) { bg = v; break; }
    }
    int thr = bg - d->detect_delta, dark = 0;
    for (int v = 0; v < thr && v < 256; v++) dark += hist[v];
    if (getenv("ZK4500_DEBUG"))
        fprintf(stderr, "zk4500: bg=%d dark_ratio=%.3f (need %.3f)\n", bg, (double)dark / n, d->detect_ratio);
    return dark >= d->detect_ratio * n;
}

int zk4500_capture(zk4500_t *d, uint8_t *img, size_t len)
{
    if (!d || !img || len < (size_t)(d->info.width * d->info.height)) return ZK4500_ERR_PARAM;
    int r = zk4500_capture_raw(d, d->raw, RAW_W * RAW_H);
    if (r < 0) return r;
    process(d, d->raw, img);
    return finger_present(d, img);
}

int zk4500_wait_finger(zk4500_t *d, uint8_t *img, size_t len, int timeout_ms)
{
    long t0 = now_ms();
    int hits = 0;
    for (;;) {
        int r = zk4500_capture(d, img, len);
        if (r < 0) return r;
        /* require two consecutive finger frames so the image is not taken mid-press */
        hits = r ? hits + 1 : 0;
        if (hits >= 2) return 1;
        if (timeout_ms >= 0 && now_ms() - t0 > timeout_ms) return ZK4500_ERR_TIMEOUT;
        usleep(50000);
    }
}

int zk4500_wait_release(zk4500_t *d, int timeout_ms)
{
    if (!d) return ZK4500_ERR_PARAM;
    uint8_t *img = malloc(d->info.width * d->info.height);
    if (!img) return ZK4500_ERR_NOMEM;
    long t0 = now_ms();
    int r;
    for (;;) {
        r = zk4500_capture(d, img, d->info.width * d->info.height);
        if (r <= 0) break;
        if (timeout_ms >= 0 && now_ms() - t0 > timeout_ms) { r = ZK4500_ERR_TIMEOUT; break; }
        usleep(50000);
    }
    free(img);
    return r < 0 ? r : 0;
}

/* ------------------------------------------------------------------ */

int zk4500_write_pgm(const char *path, const uint8_t *img, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f) return ZK4500_ERR_PARAM;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    size_t n = fwrite(img, 1, (size_t)w * h, f);
    fclose(f);
    return n == (size_t)w * h ? 0 : ZK4500_ERR_PARAM;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

int zk4500_write_bmp(const char *path, const uint8_t *img, int w, int h)
{
    int stride = (w + 3) & ~3;
    uint32_t off = 14 + 40 + 256 * 4, size = off + stride * h;
    uint8_t hdr[14 + 40] = {0}, pal[256 * 4], pad[3] = {0};
    hdr[0] = 'B'; hdr[1] = 'M';
    put32(hdr + 2, size); put32(hdr + 10, off);
    put32(hdr + 14, 40); put32(hdr + 18, w); put32(hdr + 22, h);
    put16(hdr + 26, 1); put16(hdr + 28, 8);
    put32(hdr + 34, stride * h);
    put32(hdr + 38, 19685); put32(hdr + 42, 19685);   /* 500 dpi */
    put32(hdr + 46, 256);
    for (int i = 0; i < 256; i++) { pal[4*i] = pal[4*i+1] = pal[4*i+2] = i; pal[4*i+3] = 0; }
    FILE *f = fopen(path, "wb");
    if (!f) return ZK4500_ERR_PARAM;
    fwrite(hdr, 1, sizeof hdr, f);
    fwrite(pal, 1, sizeof pal, f);
    for (int y = h - 1; y >= 0; y--) {
        fwrite(img + (size_t)y * w, 1, w, f);
        fwrite(pad, 1, stride - w, f);
    }
    fclose(f);
    return 0;
}
