/*
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (C) 2026 Michael Sauer
 *
 * zkprobe - read-only protocol probe for the ZK4500 (1b55:0840).
 *
 * Uses the ZKFPI vendor requests recovered from libsilkidcap.so:
 *   0x40/0xE0  init              0x40/0xE5  start image (bulk on EP 0x82)
 *   0xC0/0xE2  get GPIO          0xC0/0xE4  read camera register (2 bytes)
 *   0xC0/0xE7  read EEPROM byte  0xC0/0xF3  check sensor (4 bytes)
 * No write requests (EEPROM/camera/GPIO) are issued.
 *
 * build: cc -O2 -o zkprobe zkprobe.c $(pkg-config --cflags --libs libusb-1.0)
 */
#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VID 0x1b55
#define PID 0x0840
#define TO 2000

static libusb_device_handle *h;

static int vin(uint8_t req, uint16_t val, uint16_t idx, uint8_t *buf, uint16_t len)
{
    return libusb_control_transfer(h, 0xC0, req, val, idx, buf, len, TO);
}
static int vout(uint8_t req, uint16_t val, uint16_t idx)
{
    return libusb_control_transfer(h, 0x40, req, val, idx, NULL, 0, TO);
}

static void hexdump(const uint8_t *p, int n, int base)
{
    for (int i = 0; i < n; i += 16) {
        printf("  %04x:", base + i);
        for (int j = 0; j < 16 && i + j < n; j++) printf(" %02x", p[i + j]);
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    int r;
    const char *out = argc > 1 ? argv[1] : "zk4500_raw.bin";
    if ((r = libusb_init(NULL))) { fprintf(stderr, "libusb_init: %s\n", libusb_error_name(r)); return 1; }
    h = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!h) { fprintf(stderr, "cannot open %04x:%04x (not present or no permission)\n", VID, PID); return 1; }
    libusb_set_auto_detach_kernel_driver(h, 1);
    if ((r = libusb_claim_interface(h, 0))) { fprintf(stderr, "claim: %s\n", libusb_error_name(r)); return 1; }

    unsigned char s[128];
    struct libusb_device_descriptor dd;
    libusb_get_device_descriptor(libusb_get_device(h), &dd);
    libusb_get_string_descriptor_ascii(h, dd.iManufacturer, s, sizeof s); printf("manufacturer: %s\n", s);
    libusb_get_string_descriptor_ascii(h, dd.iProduct, s, sizeof s);      printf("product     : %s\n", s);
    printf("bcdDevice   : %x.%02x\n", dd.bcdDevice >> 8, dd.bcdDevice & 0xff);

    r = vout(0xE0, 0, 0);
    printf("E0 init      -> %d %s\n", r, r < 0 ? libusb_error_name(r) : "");

    uint8_t b[8] = {0};
    r = vin(0xE2, 0, 0x55, b, 2);
    printf("E2 gpio 0x55 -> %d  [%02x %02x]\n", r, b[0], b[1]);

    memset(b, 0, sizeof b);
    r = vin(0xF3, 0, 0, b, 4);
    printf("F3 checksens -> %d  [%02x %02x %02x %02x]\n", r, b[0], b[1], b[2], b[3]);

    printf("EEPROM (E7, 256 bytes):\n");
    uint8_t ee[256]; int eeok = 0;
    for (int a = 0; a < 256; a++) {
        ee[a] = 0;
        r = vin(0xE7, 0, a, &ee[a], 1);
        if (r != 1) { printf("  stop at 0x%02x: %d %s\n", a, r, libusb_error_name(r)); break; }
        eeok++;
    }
    hexdump(ee, eeok, 0);

    printf("camera registers (E4, 2 bytes each, 0x00-0xff):\n");
    for (int a = 0; a < 256; a++) {
        memset(b, 0, 2);
        r = vin(0xE4, 0, a, b, 2);
        if (r != 2) { printf("  stop at 0x%02x: %d %s\n", a, r, libusb_error_name(r)); break; }
        printf("%s%02x=%02x%02x", (a % 8) ? "  " : "\n  ", a, b[0], b[1]);
    }
    printf("\n");

    /* capture: E5 then read until short packet / timeout */
    const int max = 1024 * 1024;
    uint8_t *img = malloc(max);
    r = vout(0xE5, 0, 0);
    printf("E5 getimage  -> %d %s\n", r, r < 0 ? libusb_error_name(r) : "");
    int total = 0;
    for (;;) {
        int got = 0;
        r = libusb_bulk_transfer(h, 0x82, img + total, 64 * 1024, &got, TO);
        total += got;
        printf("  bulk chunk: r=%d got=%d total=%d\n", r, got, total);
        if (r || got < 64 * 1024 || total + 64 * 1024 > max) break;
    }
    if (total > 0) {
        FILE *f = fopen(out, "wb");
        fwrite(img, 1, total, f); fclose(f);
        printf("wrote %d bytes to %s\n", total, out);
        hexdump(img, total < 64 ? total : 64, 0);
    }
    free(img);
    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(NULL);
    return 0;
}
