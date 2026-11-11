/*
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (C) 2026 Michael Sauer
 *
 * zkctl - send raw ZKFPI vendor requests to a ZK4500 (exploration tool).
 *
 * usage: zkctl CMD [CMD ...]
 *   in:REQ:VAL:IDX:LEN    vendor IN  (0xC0) request, prints returned bytes
 *   out:REQ:VAL:IDX       vendor OUT (0x40) request without data
 *   img:FILE              0xE5 + bulk read 640x480 into FILE, prints stats
 *   sleep:MS
 * numbers accept 0x prefix.
 */
#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    libusb_init(NULL);
    libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, 0x1b55, 0x0840);
    if (!h) { fprintf(stderr, "open failed\n"); return 1; }
    libusb_set_auto_detach_kernel_driver(h, 1);
    if (libusb_claim_interface(h, 0)) { fprintf(stderr, "claim failed\n"); return 1; }
    for (int i = 1; i < argc; i++) {
        char *a = strdup(argv[i]), *t = strtok(a, ":");
        if (!strcmp(t, "in")) {
            unsigned req = strtoul(strtok(NULL, ":"), 0, 0), val = strtoul(strtok(NULL, ":"), 0, 0);
            unsigned idx = strtoul(strtok(NULL, ":"), 0, 0), len = strtoul(strtok(NULL, ":"), 0, 0);
            unsigned char b[4096] = {0};
            int r = libusb_control_transfer(h, 0xC0, req, val, idx, b, len, 2000);
            printf("in  %02x v=%04x i=%04x -> %d:", req, val, idx, r);
            for (int k = 0; k < r; k++) printf(" %02x", b[k]);
            printf("\n");
        } else if (!strcmp(t, "out")) {
            unsigned req = strtoul(strtok(NULL, ":"), 0, 0), val = strtoul(strtok(NULL, ":"), 0, 0);
            unsigned idx = strtoul(strtok(NULL, ":"), 0, 0);
            int r = libusb_control_transfer(h, 0x40, req, val, idx, NULL, 0, 2000);
            printf("out %02x v=%04x i=%04x -> %d\n", req, val, idx, r);
        } else if (!strcmp(t, "img")) {
            const char *fn = strtok(NULL, ":");
            int n = 640 * 480, got = 0, tot = 0;
            unsigned char *p = malloc(n);
            int r = libusb_control_transfer(h, 0x40, 0xE5, 0, 0, NULL, 0, 2000);
            while (r >= 0 && tot < n) {
                r = libusb_bulk_transfer(h, 0x82, p + tot, n - tot, &got, 2000);
                tot += got;
                if (!got) break;
            }
            long sum = 0; int mn = 255, mx = 0;
            for (int k = 0; k < tot; k++) { sum += p[k]; if (p[k] < mn) mn = p[k]; if (p[k] > mx) mx = p[k]; }
            printf("img %d bytes r=%d min=%d max=%d mean=%.1f\n", tot, r, mn, mx, tot ? (double)sum / tot : 0);
            if (fn) { FILE *f = fopen(fn, "wb"); fprintf(f, "P5\n640 480\n255\n"); fwrite(p, 1, tot, f); fclose(f); }
            free(p);
        } else if (!strcmp(t, "wait")) {
            /* wait:PREFIX:SECONDS  capture until a frame darker than mean 230 appears, save 5 frames */
            const char *pre = strtok(NULL, ":");
            int secs = atoi(strtok(NULL, ":")), saved = 0, n = 640 * 480;
            unsigned char *p = malloc(n);
            for (int it = 0; it < secs * 5 && saved < 5; it++) {
                int got = 0, tot = 0;
                if (libusb_control_transfer(h, 0x40, 0xE5, 0, 0, NULL, 0, 2000) < 0) break;
                while (tot < n && !libusb_bulk_transfer(h, 0x82, p + tot, n - tot, &got, 2000) && got) tot += got;
                long sum = 0;
                for (int k = 0; k < tot; k++) sum += p[k];
                double mean = tot ? (double)sum / tot : 255;
                {
                    int dark = 0;
                    for (int k = 0; k < tot; k++) dark += p[k] < 200;
                    printf("t=%.1fs mean=%.1f dark=%d\n", it * 0.2, mean, dark);
                    fflush(stdout);
                }
                if (tot == n && mean < 230) {
                    char fn[512];
                    snprintf(fn, sizeof fn, "%s%d.pgm", pre, saved++);
                    FILE *f = fopen(fn, "wb"); fprintf(f, "P5\n640 480\n255\n"); fwrite(p, 1, n, f); fclose(f);
                    printf("saved %s mean=%.1f\n", fn, mean);
                    fflush(stdout);
                }
                usleep(200000);
            }
            free(p);
        } else if (!strcmp(t, "sleep")) {
            usleep(1000 * atoi(strtok(NULL, ":")));
        }
        free(a);
    }
    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(NULL);
    return 0;
}
