#define _DEFAULT_SOURCE
#include "kinect_ir.h"

#include <math.h>
#include <string.h>

void kinect_ir_tune_default(kinect_ir_tune *t) {
  if (!t) return;
  t->boost = 2.2f;
  t->brightness = 8.f;
  t->contrast = 1.35f;
  t->gamma = 0.70f;
  t->black_pct = 4;
  t->white_pct = 96;
  t->invert = 0;
}

void kinect_ir_tune_see(kinect_ir_tune *t) {
  if (!t) return;
  t->boost = 3.4f;
  t->brightness = 16.f;
  t->contrast = 1.65f;
  t->gamma = 0.55f;
  t->black_pct = 6;
  t->white_pct = 88;
  t->invert = 0;
}

void kinect_unpack_y10b_u16(const uint8_t *buf, size_t blen, uint16_t *out, int npix) {
  size_t bit = 0, bits = blen * 8;
  int p;
  if (!buf || !out) return;
  for (p = 0; p < npix; p++) {
    int v = 0, k;
    if (bit + 10 > bits) {
      out[p] = 0;
      continue;
    }
    for (k = 0; k < 10; k++) {
      unsigned byte = buf[bit / 8];
      v = (v << 1) | ((byte >> (7 - (int)(bit % 8))) & 1);
      bit++;
    }
    out[p] = (uint16_t)v;
  }
}

void kinect_ir_grade(const uint16_t *in, uint8_t *out, int n, const kinect_ir_tune *t) {
  int hist[1024];
  int i, lo, hi, acc, want_lo, want_hi;
  float boost, bright, contrast, gamma, scale;
  kinect_ir_tune def;
  if (!in || !out || n <= 0) return;
  if (!t) {
    kinect_ir_tune_default(&def);
    t = &def;
  }
  memset(hist, 0, sizeof hist);
  for (i = 0; i < n; i++) {
    int v = in[i];
    if (v < 0) v = 0;
    if (v > 1023) v = 1023;
    hist[v]++;
  }
  if (t->black_pct < 0) want_lo = 0;
  else if (t->black_pct > 30) want_lo = (int)(n * 0.30);
  else want_lo = (int)(n * (t->black_pct / 100.f));
  if (t->white_pct < 60) want_hi = (int)(n * 0.60);
  else if (t->white_pct > 100) want_hi = n;
  else want_hi = (int)(n * (t->white_pct / 100.f));
  if (want_hi <= want_lo) want_hi = want_lo + 1;
  acc = 0;
  lo = 0;
  for (i = 0; i < 1024; i++) {
    acc += hist[i];
    if (acc >= want_lo) {
      lo = i;
      break;
    }
  }
  acc = 0;
  hi = 1023;
  for (i = 0; i < 1024; i++) {
    acc += hist[i];
    if (acc >= want_hi) {
      hi = i;
      break;
    }
  }
  if (hi <= lo) hi = lo + 1;
  boost = t->boost;
  if (boost < 0.2f) boost = 0.2f;
  if (boost > 10.f) boost = 10.f;
  bright = t->brightness;
  contrast = t->contrast;
  if (contrast < 0.2f) contrast = 0.2f;
  gamma = t->gamma;
  if (gamma < 0.2f) gamma = 0.2f;
  scale = 1.f / (float)(hi - lo);
  for (i = 0; i < n; i++) {
    float x = ((int)in[i] - lo) * scale;
    float y;
    int o;
    if (x < 0) x = 0;
    if (x > 1) x = 1;
    x *= boost;
    if (x > 1) x = 1;
    y = powf(x, gamma);
    y = (y - 0.5f) * contrast + 0.5f + bright / 255.f;
    if (y < 0) y = 0;
    if (y > 1) y = 1;
    if (t->invert) y = 1.f - y;
    o = (int)(y * 255.f + 0.5f);
    if (o < 0) o = 0;
    if (o > 255) o = 255;
    out[i] = (uint8_t)o;
  }
}

void kinect_ir_grey_to_rgb640(const uint8_t *grey, int w, int h, uint8_t *rgb) {
  int x, y;
  if (!grey || !rgb || w < 1 || h < 1) return;
  for (y = 0; y < 480; y++) {
    int sy = y * h / 480;
    for (x = 0; x < 640; x++) {
      int sx = x * w / 640;
      uint8_t g = grey[sy * w + sx];
      int o = (y * 640 + x) * 3;
      rgb[o] = rgb[o + 1] = rgb[o + 2] = g;
    }
  }
}
