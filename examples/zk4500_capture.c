/*
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (C) 2026 Michael Sauer
 *
 * zk4500_capture - capture a fingerprint from a ZK4500 and save it.
 *
 * usage: zk4500_capture [-t timeout_ms] [-r raw.pgm] OUTPUT.(bmp|pgm)
 */
#include "zk4500.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    int timeout = 15000, opt;
    const char *rawpath = NULL;
    while ((opt = getopt(argc, argv, "t:r:")) != -1) {
        if (opt == 't') timeout = atoi(optarg);
        else if (opt == 'r') rawpath = optarg;
        else { fprintf(stderr, "usage: %s [-t ms] [-r raw.pgm] out.bmp|out.pgm\n", argv[0]); return 2; }
    }
    if (optind >= argc) { fprintf(stderr, "usage: %s [-t ms] [-r raw.pgm] out.bmp|out.pgm\n", argv[0]); return 2; }
    const char *out = argv[optind];

    printf("devices: %d\n", zk4500_count());
    zk4500_t *dev;
    int r = zk4500_open(&dev, 0);
    if (r) { fprintf(stderr, "open: %s\n", zk4500_strerror(r)); return 1; }

    zk4500_info_t in;
    zk4500_get_info(dev, &in);
    printf("%s / %s, serial %s, sensor type 0x%04x\n", in.manufacturer, in.product, in.serial, in.sensor_type);
    printf("raw %dx%d -> image %dx%d\n", in.raw_width, in.raw_height, in.width, in.height);

    int n = zk4500_image_size(dev);
    uint8_t *img = malloc(n);
    zk4500_set_led(dev, 0, 0);
    printf("place finger on the sensor ...\n");
    fflush(stdout);
    r = zk4500_wait_finger(dev, img, n, timeout);
    if (r != 1) {
        fprintf(stderr, "capture: %s\n", zk4500_strerror(r));
        zk4500_set_led(dev, 0, 1);
        usleep(500000);
        zk4500_set_led(dev, 0, 0);
        zk4500_close(dev);
        return 1;
    }
    zk4500_set_led(dev, 1, 0);
    zk4500_beep(dev, 80);

    const char *ext = strrchr(out, '.');
    if (ext && !strcmp(ext, ".pgm")) r = zk4500_write_pgm(out, img, in.width, in.height);
    else r = zk4500_write_bmp(out, img, in.width, in.height);
    printf("saved %s (%s)\n", out, r ? "FAILED" : "ok");

    if (rawpath) {
        uint8_t *raw = malloc(in.raw_width * in.raw_height);
        if (zk4500_capture_raw(dev, raw, in.raw_width * in.raw_height) > 0)
            zk4500_write_pgm(rawpath, raw, in.raw_width, in.raw_height);
        free(raw);
    }
    usleep(300000);
    zk4500_set_led(dev, 0, 0);
    free(img);
    zk4500_close(dev);
    return 0;
}
