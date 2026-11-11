/*
 * ZKSoftware ZK4500 driver for libfprint
 * Copyright (C) 2026 Michael Sauer
 *
 * The protocol was recovered for interoperability by static analysis of
 * ZKTeco's libzklibcap.dll (ZKFinger SDK 5.3); see the libzk4500 README.
 * The capture loop structure follows the vcom5s driver (Daniel Drake).
 *
 *   init      : vendor OUT 0xE0
 *   EEPROM    : vendor IN  0xE7, wIndex = address, 1 byte
 *   light on  : vendor OUT 0xE1, wValue 0, wIndex 0x32
 *   GPIO      : vendor OUT 0xE1, wValue = state, wIndex = gpio
 *               (0 = green LED, 1 = red LED, 2 = beeper)
 *   capture   : vendor OUT 0xE5, then bulk IN 0x82, 640x480 8-bit gray
 *   close     : vendor OUT 0xE1, wValue 0, wIndex 0x17
 *
 * The raw frame shows the prism trapezoid; it is corrected with a
 * perspective mapping using calibration data from the device EEPROM.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "zk4500"

#include "drivers_api.h"
#include <math.h>

#define EP_IN            (2 | FPI_USB_ENDPOINT_IN)
#define CTRL_TIMEOUT     1000
#define BULK_TIMEOUT     2000
#define RAW_W            640
#define RAW_H            480
#define RAW_SIZE         (RAW_W * RAW_H)
#define DEFAULT_W        284
#define DEFAULT_H        368
#define FRAME_DELAY_MS   40

#define REQ_INIT         0xE0
#define REQ_GPIO         0xE1
#define REQ_CAPTURE      0xE5
#define REQ_EEPROM       0xE7

#define GPIO_GREEN       0
#define GPIO_RED         1
#define GPIO_LIGHT       0x32
#define GPIO_CLOSE       0x17

/* finger detection: share of pixels darker than (background - DELTA) */
#define DETECT_RATIO     0.15
#define DETECT_DELTA     40
#define DEBOUNCE_FRAMES  2

struct _FpDeviceZk4500
{
  FpImageDevice       parent;

  /* calibration */
  int                 width, height;
  int                 crop_x, crop_y, crop_w, crop_h;
  int                 quad[8];
  gboolean            invert;
  gboolean            mirror_h;
  gint32             *lut_idx;
  guint8             *lut_fx, *lut_fy;

  /* capture loop */
  guint8             *raw;
  FpiImageDeviceState state;
  gboolean            loop_running;
  gboolean            deactivating;
  int                 on_count, off_count;
  int                 reported;      /* -1 unknown, 0 off, 1 on */
  int                 read_errors;
  FpImage            *best_img;
  double              best_ratio, last_ratio;
};

G_DECLARE_FINAL_TYPE (FpDeviceZk4500, fpi_device_zk4500, FPI, DEVICE_ZK4500,
                      FpImageDevice);
G_DEFINE_TYPE (FpDeviceZk4500, fpi_device_zk4500, FP_TYPE_IMAGE_DEVICE);

/* ------------------------------------------------------------------ */
/* synchronous helpers (only used during open/close)                   */

static gboolean
ctrl_out_sync (FpDevice *dev, guint8 req, guint16 val, guint16 idx, GError **error)
{
  g_autoptr(FpiUsbTransfer) t = fpi_usb_transfer_new (dev);
  g_autoptr(GError) local_error = NULL;

  fpi_usb_transfer_fill_control (t, G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE,
                                 G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                 G_USB_DEVICE_RECIPIENT_DEVICE, req, val, idx, 0);
  /* fpi_usb_transfer_submit_sync() requires a non-NULL error pointer */
  if (!fpi_usb_transfer_submit_sync (t, CTRL_TIMEOUT, &local_error))
    {
      if (error)
        *error = g_steal_pointer (&local_error);
      return FALSE;
    }
  return TRUE;
}

/* discard stale image data (e.g. a frame left over by an aborted reader) */
static void
drain_sync (FpDevice *dev)
{
  for (int i = 0; i < 16; i++)
    {
      g_autoptr(FpiUsbTransfer) t = fpi_usb_transfer_new (dev);
      g_autoptr(GError) err = NULL;

      fpi_usb_transfer_fill_bulk (t, EP_IN, 65536);
      if (!fpi_usb_transfer_submit_sync (t, 100, &err))
        break;
      fp_dbg ("drained %zd stale bytes", t->actual_length);
    }
}

static gboolean
eeprom_read_sync (FpDevice *dev, int addr, guint8 *buf, int len, GError **error)
{
  for (int i = 0; i < len; i++)
    {
      g_autoptr(FpiUsbTransfer) t = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_control (t, G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST,
                                     G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                     G_USB_DEVICE_RECIPIENT_DEVICE,
                                     REQ_EEPROM, 0, addr + i, 1);
      t->short_is_error = TRUE;
      if (!fpi_usb_transfer_submit_sync (t, CTRL_TIMEOUT, error))
        return FALSE;
      buf[i] = t->buffer[0];
    }
  return TRUE;
}

/* ------------------------------------------------------------------ */
/* calibration + perspective lookup table                              */

static gboolean
read_calibration (FpDeviceZk4500 *self, GError **error)
{
  FpDevice *dev = FP_DEVICE (self);
  guint8 blk[30], b[4];
  guint16 s[15], sum = 1;
  int sensor_type;

  if (!eeprom_read_sync (dev, 0x0a, b, 4, error))
    return FALSE;
  if (b[0] == 'Z' && b[1] == 'K' && b[2] == 'M' && b[3] == 0)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED,
                   "Unsupported ZK sensor variant (ZKM)");
      return FALSE;
    }
  if (!eeprom_read_sync (dev, 0x9a, b, 2, error))
    return FALSE;
  sensor_type = b[0] | b[1] << 8;

  if (!eeprom_read_sync (dev, 0x3b, blk, sizeof (blk), error))
    return FALSE;
  for (int i = 0; i < 15; i++)
    s[i] = blk[2 * i] | blk[2 * i + 1] << 8;
  for (int i = 0; i < 14; i++)
    sum += s[i];
  if ((sum & 0x7fff) != s[14])
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL,
                   "Invalid calibration checksum in EEPROM");
      return FALSE;
    }

  self->crop_x = s[0];
  self->crop_y = s[1];
  self->crop_w = (s[2] > 0 && s[2] <= RAW_W) ? s[2] : RAW_W;
  self->crop_h = (s[3] > 0 && s[3] <= RAW_H) ? s[3] : RAW_H;
  if (self->crop_x + self->crop_w > RAW_W)
    self->crop_x = RAW_W - self->crop_w;
  if (self->crop_y + self->crop_h > RAW_H)
    self->crop_y = RAW_H - self->crop_h;
  for (int i = 0; i < 8; i++)
    self->quad[i] = (gint16) s[4 + i];
  self->width = s[12] & 0xfff;
  self->height = s[13] & 0xfff;
  self->invert = (s[12] >> 12) & 1;
  self->mirror_h = sensor_type == 0x7d1;
  if (self->width <= 0 || self->height <= 0)
    {
      self->width = DEFAULT_W;
      self->height = DEFAULT_H;
    }

  fp_dbg ("sensor type 0x%04x, crop %d,%d %dx%d, out %dx%d, quad %d,%d %d,%d %d,%d %d,%d",
          sensor_type, self->crop_x, self->crop_y, self->crop_w, self->crop_h,
          self->width, self->height, self->quad[0], self->quad[1], self->quad[2],
          self->quad[3], self->quad[4], self->quad[5], self->quad[6], self->quad[7]);
  return TRUE;
}

static void
build_lut (FpDeviceZk4500 *self)
{
  const int W = self->width, H = self->height;
  const int *q = self->quad;  /* BL, BR, TL, TR */
  /* unit square corners (0,0) (1,0) (1,1) (0,1) -> TL TR BR BL */
  double x[4] = { q[4], q[6], q[2], q[0] };
  double y[4] = { q[5], q[7], q[3], q[1] };
  double a, b, c, d, e, f, g, h;
  double sx = x[0] - x[1] + x[2] - x[3];
  double sy = y[0] - y[1] + y[2] - y[3];

  if (fabs (sx) < 1e-9 && fabs (sy) < 1e-9)
    {
      a = x[1] - x[0]; b = x[2] - x[1]; c = x[0];
      d = y[1] - y[0]; e = y[2] - y[1]; f = y[0];
      g = h = 0;
    }
  else
    {
      double dx1 = x[1] - x[2], dx2 = x[3] - x[2];
      double dy1 = y[1] - y[2], dy2 = y[3] - y[2];
      double den = dx1 * dy2 - dx2 * dy1;
      g = (sx * dy2 - dx2 * sy) / den;
      h = (dx1 * sy - sx * dy1) / den;
      a = x[1] - x[0] + g * x[1];
      b = x[3] - x[0] + h * x[3];
      c = x[0];
      d = y[1] - y[0] + g * y[1];
      e = y[3] - y[0] + h * y[3];
      f = y[0];
    }

  self->lut_idx = g_new (gint32, W * H);
  self->lut_fx = g_new (guint8, W * H);
  self->lut_fy = g_new (guint8, W * H);

  for (int j = 0; j < H; j++)
    {
      double v = (j + 0.5) / H;
      for (int i = 0; i < W; i++)
        {
          double u = (i + 0.5) / W;
          double w = g * u + h * v + 1.0;
          double px = (a * u + b * v + c) / w;
          double py = (d * u + e * v + f) / w;
          int k = j * W + i, ix, iy, fx, fy;

          if (px < 0 || py < 0 || px > self->crop_w - 1 || py > self->crop_h - 1)
            {
              self->lut_idx[k] = -1;
              continue;
            }
          ix = (int) px;
          iy = (int) py;
          fx = (int) ((px - ix) * 16 + 0.5);
          fy = (int) ((py - iy) * 16 + 0.5);
          if (fx == 16)
            {
              ix++;
              fx = 0;
            }
          if (fy == 16)
            {
              iy++;
              fy = 0;
            }
          if (ix >= self->crop_w - 1)
            {
              ix = self->crop_w - 1;
              fx = 0;
            }
          if (iy >= self->crop_h - 1)
            {
              iy = self->crop_h - 1;
              fy = 0;
            }
          self->lut_idx[k] = iy * self->crop_w + ix;
          self->lut_fx[k] = fx;
          self->lut_fy[k] = fy;
        }
    }
}

/* raw (modified in place) -> corrected image */
static void
process_frame (FpDeviceZk4500 *self, guint8 *raw, guint8 *img)
{
  const int W = self->width, H = self->height, cw = self->crop_w, ch = self->crop_h;
  const guint8 *src;

  if (self->invert)
    for (int i = 0; i < RAW_SIZE; i++)
      raw[i] = 255 - raw[i];

  /* 2x2 box smoothing, as done by the vendor driver */
  for (int yy = 0; yy < RAW_H - 1; yy++)
    {
      guint8 *p = raw + yy * RAW_W, *n = p + RAW_W;
      for (int xx = 0; xx < RAW_W - 1; xx++)
        p[xx] = (p[xx] + p[xx + 1] + n[xx] + n[xx + 1] + 2) >> 2;
    }

  src = raw + self->crop_y * RAW_W + self->crop_x;
  for (int k = 0; k < W * H; k++)
    {
      int idx = self->lut_idx[k], ix, iy, fx, fy, dx, dy, top, bot;
      const guint8 *p;

      if (idx < 0)
        {
          img[k] = 0xff;
          continue;
        }
      iy = idx / cw;
      ix = idx % cw;
      p = src + iy * RAW_W + ix;
      fx = self->lut_fx[k];
      fy = self->lut_fy[k];
      dx = ix + 1 < cw ? 1 : 0;
      dy = iy + 1 < ch ? RAW_W : 0;
      top = p[0] * (16 - fx) + p[dx] * fx;
      bot = p[dy] * (16 - fx) + p[dy + dx] * fx;
      img[k] = (top * (16 - fy) + bot * fy + 128) >> 8;
    }

  if (self->mirror_h)
    {
      for (int yy = 0; yy < H; yy++)
        {
          guint8 *row = img + yy * W;
          for (int l = 0, r = W - 1; l < r; l++, r--)
            {
              guint8 t = row[l];
              row[l] = row[r];
              row[r] = t;
            }
        }
    }
  else
    {
      for (int t = 0, bt = H - 1; t < bt; t++, bt--)
        for (int xx = 0; xx < W; xx++)
          {
            guint8 tmp = img[t * W + xx];
            img[t * W + xx] = img[bt * W + xx];
            img[bt * W + xx] = tmp;
          }
    }
}

static double
dark_ratio (FpDeviceZk4500 *self, const guint8 *img)
{
  const int n = self->width * self->height;
  int hist[256] = { 0 }, acc = 0, bg = 255, thr, dark = 0;

  for (int i = 0; i < n; i++)
    hist[img[i]]++;
  for (int v = 0; v < 256; v++)
    {
      acc += hist[v];
      if (acc >= n * 9 / 10)
        {
          bg = v;
          break;
        }
    }
  thr = bg - DETECT_DELTA;
  for (int v = 0; v < thr && v < 256; v++)
    dark += hist[v];
  fp_dbg ("bg %d dark ratio %.3f", bg, (double) dark / n);
  return (double) dark / n;
}

/* ------------------------------------------------------------------ */
/* capture loop                                                        */

enum loop_states {
  LOOP_REQUEST,
  LOOP_READ,
  LOOP_PROCESS,
  LOOP_DRAIN,
  LOOP_NUM_STATES,
};

#define MAX_READ_ERRORS 3

static void
read_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);

  if (error)
    {
      if (++self->read_errors > MAX_READ_ERRORS ||
          g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        {
          fpi_ssm_mark_failed (transfer->ssm, error);
          return;
        }
      /* stream out of sync: flush the endpoint and request a new frame */
      fp_dbg ("read error (%s), resyncing", error->message);
      g_error_free (error);
      fpi_ssm_jump_to_state (transfer->ssm, LOOP_DRAIN);
      return;
    }
  self->read_errors = 0;
  memcpy (self->raw, transfer->buffer, RAW_SIZE);
  fpi_ssm_next_state (transfer->ssm);
}

static void
drain_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  if (error)
    {
      /* timeout = endpoint empty */
      g_error_free (error);
      fpi_ssm_jump_to_state (transfer->ssm, LOOP_REQUEST);
      return;
    }
  fpi_ssm_jump_to_state (transfer->ssm, LOOP_DRAIN);
}

/* Capture policy: once a finger is seen, keep sampling until the contact area
 * stops growing (or MAX_SETTLE_FRAMES), then hand over the frame with the
 * largest contact area. This avoids images taken while the finger lands. */
#define MAX_SETTLE_FRAMES 6
#define SETTLE_EPSILON    0.01

static void
handle_frame (FpiSsm *ssm, FpDeviceZk4500 *self)
{
  FpImageDevice *imgdev = FP_IMAGE_DEVICE (self);
  FpImage *img = fp_image_new (self->width, self->height);
  double ratio;
  gboolean present;

  process_frame (self, self->raw, img->data);
  ratio = dark_ratio (self, img->data);
  present = ratio >= DETECT_RATIO;

  if (present)
    {
      self->on_count++;
      self->off_count = 0;
    }
  else
    {
      self->off_count++;
      self->on_count = 0;
      g_clear_object (&self->best_img);
      self->best_ratio = 0;
    }

  if (present && self->reported != 1)
    {
      gboolean settled = self->on_count >= DEBOUNCE_FRAMES &&
                         (ratio <= self->last_ratio + SETTLE_EPSILON ||
                          self->on_count >= MAX_SETTLE_FRAMES);

      if (ratio > self->best_ratio)
        {
          g_clear_object (&self->best_img);
          self->best_img = g_object_ref (img);
          self->best_ratio = ratio;
        }

      if (settled)
        {
          self->reported = 1;
          fpi_image_device_report_finger_status (imgdev, TRUE);
          if (self->state == FPI_IMAGE_DEVICE_STATE_CAPTURE)
            {
              FpImage *best = g_steal_pointer (&self->best_img);
              fp_dbg ("capturing frame with dark ratio %.3f", self->best_ratio);
              best->ppmm = 500.0 / 25.4;
              fpi_image_device_image_captured (imgdev, best);
            }
          g_clear_object (&self->best_img);
          self->best_ratio = 0;
        }
    }
  else if (self->off_count >= DEBOUNCE_FRAMES && self->reported != 0)
    {
      self->reported = 0;
      fpi_image_device_report_finger_status (imgdev, FALSE);
    }

  self->last_ratio = ratio;
  g_clear_object (&img);

  if (self->deactivating)
    fpi_ssm_mark_completed (ssm);
  else
    fpi_ssm_jump_to_state_delayed (ssm, LOOP_REQUEST, FRAME_DELAY_MS);
}

static void
loop_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);
  FpiUsbTransfer *transfer;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case LOOP_REQUEST:
      if (self->deactivating)
        {
          fpi_ssm_mark_completed (ssm);
          return;
        }
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_control (transfer, G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE,
                                     G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                     G_USB_DEVICE_RECIPIENT_DEVICE,
                                     REQ_CAPTURE, 0, 0, 0);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, CTRL_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case LOOP_READ:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, EP_IN, RAW_SIZE);
      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;
      fpi_usb_transfer_submit (transfer, BULK_TIMEOUT, NULL, read_cb, NULL);
      break;

    case LOOP_PROCESS:
      handle_frame (ssm, self);
      break;

    case LOOP_DRAIN:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, EP_IN, 65536);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, 100, NULL, drain_cb, NULL);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
loop_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpImageDevice *imgdev = FP_IMAGE_DEVICE (dev);
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);

  self->loop_running = FALSE;
  g_clear_object (&self->best_img);

  if (error && !self->deactivating)
    fpi_image_device_session_error (imgdev, error);
  else if (error)
    g_error_free (error);

  if (self->deactivating)
    {
      self->deactivating = FALSE;
      fpi_image_device_deactivate_complete (imgdev, NULL);
    }
}

/* ------------------------------------------------------------------ */
/* device vfuncs                                                       */

static void
dev_change_state (FpImageDevice *dev, FpiImageDeviceState state)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);

  self->state = state;
}

static void
dev_activate (FpImageDevice *dev)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);
  FpiSsm *ssm = fpi_ssm_new (FP_DEVICE (dev), loop_run_state, LOOP_NUM_STATES);

  self->deactivating = FALSE;
  self->on_count = self->off_count = 0;
  self->reported = -1;
  self->read_errors = 0;
  self->best_ratio = self->last_ratio = 0;
  g_clear_object (&self->best_img);
  self->loop_running = TRUE;
  fpi_ssm_start (ssm, loop_complete);
  fpi_image_device_activate_complete (dev, NULL);
}

static void
dev_deactivate (FpImageDevice *dev)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);

  if (self->loop_running)
    self->deactivating = TRUE;
  else
    fpi_image_device_deactivate_complete (dev, NULL);
}

static void
dev_open (FpImageDevice *dev)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);
  GUsbDevice *usb = fpi_device_get_usb_device (FP_DEVICE (dev));
  GError *error = NULL;

  if (!g_usb_device_claim_interface (usb, 0, 0, &error))
    goto out;
  drain_sync (FP_DEVICE (dev));
  if (!ctrl_out_sync (FP_DEVICE (dev), REQ_INIT, 0, 0, &error))
    goto out;
  if (!read_calibration (self, &error))
    goto out;
  if (!ctrl_out_sync (FP_DEVICE (dev), REQ_GPIO, 0, GPIO_LIGHT, &error))
    goto out;
  ctrl_out_sync (FP_DEVICE (dev), REQ_GPIO, 0, GPIO_GREEN, NULL);
  ctrl_out_sync (FP_DEVICE (dev), REQ_GPIO, 0, GPIO_RED, NULL);

  build_lut (self);
  self->raw = g_malloc (RAW_SIZE);

out:
  fpi_image_device_open_complete (dev, error);
}

static void
dev_close (FpImageDevice *dev)
{
  FpDeviceZk4500 *self = FPI_DEVICE_ZK4500 (dev);
  GError *error = NULL;

  ctrl_out_sync (FP_DEVICE (dev), REQ_GPIO, 0, GPIO_CLOSE, NULL);
  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                  0, 0, &error);
  g_clear_pointer (&self->lut_idx, g_free);
  g_clear_pointer (&self->lut_fx, g_free);
  g_clear_pointer (&self->lut_fy, g_free);
  g_clear_pointer (&self->raw, g_free);
  fpi_image_device_close_complete (dev, error);
}

static const FpIdEntry id_table[] = {
  { .vid = 0x1b55, .pid = 0x0840, },
  { .vid = 0, .pid = 0, .driver_data = 0 },
};

static void
fpi_device_zk4500_init (FpDeviceZk4500 *self)
{
  self->width = DEFAULT_W;
  self->height = DEFAULT_H;
}

static void
fpi_device_zk4500_class_init (FpDeviceZk4500Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = "zk4500";
  dev_class->full_name = "ZKSoftware ZK4500";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;

  img_class->img_open = dev_open;
  img_class->img_close = dev_close;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
  img_class->change_state = dev_change_state;

  img_class->img_width = DEFAULT_W;
  img_class->img_height = DEFAULT_H;
}
