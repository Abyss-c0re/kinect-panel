#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "kinect_bridge.h"
#include "kinect_ir.h"
#include "kinect_v4l.h"
#include "fg/fg.h"

#include <linux/videodev2.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef V4L2_PIX_FMT_Y10BPACK
#define V4L2_PIX_FMT_Y10BPACK v4l2_fourcc('Y', '1', '0', 'B')
#endif

static pthread_t th;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static int running;
static int stop_flag;
static int started;
static char mode[8] = "RGB";
static char source[64];
static char vcam[64];
static char err[160];
static uint8_t preview[KINECT_RGB_W * KINECT_RGB_H * 3];
static int have_preview;
static kinect_ir_tune ir_tune;
static fg_engine *fg;
static int fg_boost;

static void set_err(const char *s) {
  pthread_mutex_lock(&mu);
  snprintf(err, sizeof err, "%s", s);
  pthread_mutex_unlock(&mu);
}

static void *bridge_main(void *arg) {
  kinect_cap cap;
  int vfd = -1;
  uint8_t *raw = NULL, *yuyv = NULL, *grey = NULL;
  uint16_t *ir10 = NULL;
  int ir = (mode[0] == 'I' || mode[0] == 'i');
  size_t raw_len = ir ? ((size_t)KINECT_IR_W * KINECT_IR_H * 10 / 8)
                      : ((size_t)KINECT_RGB_W * KINECT_RGB_H * 2);
  (void)arg;

  uint8_t *fgbuf = NULL;
  raw = (uint8_t *)malloc(raw_len + 32);
  yuyv = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 2);
  fgbuf = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
  if (ir) {
    grey = (uint8_t *)malloc((size_t)KINECT_IR_W * KINECT_IR_H);
    ir10 = (uint16_t *)malloc((size_t)KINECT_IR_W * KINECT_IR_H * sizeof(uint16_t));
  }
  if (!raw || !yuyv || !fgbuf || (ir && (!grey || !ir10))) {
    set_err("oom");
    goto out;
  }
  if (!fg) fg = fg_create();

  if (ir) {
    if (kinect_cap_open(&cap, source, V4L2_PIX_FMT_Y10BPACK, KINECT_IR_W, KINECT_IR_H) < 0) {
      set_err("IR capture open failed");
      goto out;
    }
  } else {
    if (kinect_cap_open(&cap, source, V4L2_PIX_FMT_UYVY, KINECT_RGB_W, KINECT_RGB_H) < 0) {
      set_err("RGB capture open failed");
      goto out;
    }
  }
  vfd = kinect_vcam_open(vcam);
  if (vfd < 0) {
    set_err("virtual cam open failed");
    kinect_cap_close(&cap);
    goto out;
  }

  pthread_mutex_lock(&mu);
  running = 1;
  pthread_mutex_unlock(&mu);

  while (!stop_flag) {
    uint8_t rgb[KINECT_RGB_W * KINECT_RGB_H * 3];
    if (kinect_cap_frame(&cap, raw, raw_len) < 0) {
      usleep(20000);
      continue;
    }
    if (ir) {
      kinect_ir_tune t;
      pthread_mutex_lock(&mu);
      t = ir_tune;
      pthread_mutex_unlock(&mu);
      kinect_unpack_y10b_u16(raw, raw_len, ir10, KINECT_IR_W * KINECT_IR_H);
      kinect_ir_grade(ir10, grey, KINECT_IR_W * KINECT_IR_H, &t);
      kinect_grey_to_yuyv(grey, KINECT_IR_W, KINECT_IR_H, yuyv);
      kinect_ir_grey_to_rgb640(grey, KINECT_IR_W, KINECT_IR_H, rgb);
    } else {
      kinect_uyvy_to_yuyv(raw, yuyv, KINECT_RGB_W * KINECT_RGB_H);
      /* UYVY → RGB for preview */
      {
        int i, npix = KINECT_RGB_W * KINECT_RGB_H;
        for (i = 0; i + 1 < npix; i += 2) {
          int u = (int)raw[i * 2 + 0] - 128;
          int y0 = raw[i * 2 + 1];
          int v = (int)raw[i * 2 + 2] - 128;
          int y1 = raw[i * 2 + 3];
          int r, g, b, k;
          r = y0 + (int)(1.402f * v);
          g = y0 - (int)(0.344f * u + 0.714f * v);
          b = y0 + (int)(1.772f * u);
          if (r < 0) r = 0;
          if (r > 255) r = 255;
          if (g < 0) g = 0;
          if (g > 255) g = 255;
          if (b < 0) b = 0;
          if (b > 255) b = 255;
          k = i * 3;
          rgb[k] = (uint8_t)r;
          rgb[k + 1] = (uint8_t)g;
          rgb[k + 2] = (uint8_t)b;
          r = y1 + (int)(1.402f * v);
          g = y1 - (int)(0.344f * u + 0.714f * v);
          b = y1 + (int)(1.772f * u);
          if (r < 0) r = 0;
          if (r > 255) r = 255;
          if (g < 0) g = 0;
          if (g > 255) g = 255;
          if (b < 0) b = 0;
          if (b > 255) b = 255;
          rgb[k + 3] = (uint8_t)r;
          rgb[k + 4] = (uint8_t)g;
          rgb[k + 5] = (uint8_t)b;
        }
      }
    }
    {
      int nout, fi, mid;
      pthread_mutex_lock(&mu);
      mid = fg_boost;
      pthread_mutex_unlock(&mu);
      if (fg) {
        fg_set_boost(fg, mid);
        nout = fg_push_rgb(fg, rgb);
      } else {
        nout = 1;
      }
      for (fi = 0; fi < nout; fi++) {
        uint8_t *fr = rgb;
        if (fg && fg_copy(fg, fi, fgbuf) == 0) fr = fgbuf;
        kinect_rgb_to_yuyv(fr, yuyv);
        if (kinect_vcam_write_yuyv(vfd, yuyv) < 0) usleep(4000);
        pthread_mutex_lock(&mu);
        memcpy(preview, fr, sizeof preview);
        have_preview = 1;
        pthread_mutex_unlock(&mu);
        if (fi + 1 < nout) usleep(8000);
      }
    }
  }

  kinect_cap_close(&cap);
  if (vfd >= 0) close(vfd);
out:
  free(raw);
  free(yuyv);
  free(grey);
  free(ir10);
  free(fgbuf);
  pthread_mutex_lock(&mu);
  running = 0;
  pthread_mutex_unlock(&mu);
  return NULL;
}

int kinect_bridge_start(const char *want_mode) {
  kinect_bridge_stop();
  pthread_mutex_lock(&mu);
  err[0] = 0;
  have_preview = 0;
  if (ir_tune.boost < 0.1f) kinect_ir_tune_default(&ir_tune);
  snprintf(mode, sizeof mode, "%s", (want_mode && want_mode[0] == 'I') ? "IR" : "RGB");
  pthread_mutex_unlock(&mu);
  if (kinect_find_source(source, sizeof source) < 0) {
    set_err("No Kinect /dev/video* (gspca)");
    return -1;
  }
  if (kinect_find_vcam(vcam, sizeof vcam) < 0) {
    set_err("No virtual webcam. Run: setup_loopback.sh --load");
    return -1;
  }
  if (kinect_set_mode(source, mode) < 0) {
    set_err("v4l2 S_FMT failed");
    return -1;
  }
  stop_flag = 0;
  if (pthread_create(&th, NULL, bridge_main, NULL) != 0) {
    set_err("bridge thread");
    return -1;
  }
  started = 1;
  /* wait until running or error */
  {
    int i;
    for (i = 0; i < 40; i++) {
      int run;
      pthread_mutex_lock(&mu);
      run = running;
      pthread_mutex_unlock(&mu);
      if (run) return 0;
      if (err[0] && !run && i > 2) return -1;
      usleep(50000);
    }
  }
  if (!kinect_bridge_running()) {
    if (!err[0]) set_err("bridge did not start");
    return -1;
  }
  return 0;
}

void kinect_bridge_stop(void) {
  if (!started) return;
  stop_flag = 1;
  pthread_join(th, NULL);
  started = 0;
  running = 0;
  if (fg) {
    fg_destroy(fg);
    fg = NULL;
  }
}

int kinect_bridge_running(void) {
  int r;
  pthread_mutex_lock(&mu);
  r = running;
  pthread_mutex_unlock(&mu);
  return r;
}

const char *kinect_bridge_error(void) { return err; }
const char *kinect_bridge_vcam(void) { return vcam[0] ? vcam : NULL; }

int kinect_bridge_preview_rgb(uint8_t *rgb) {
  int ok;
  if (!rgb) return -1;
  pthread_mutex_lock(&mu);
  ok = have_preview;
  if (ok) memcpy(rgb, preview, sizeof preview);
  pthread_mutex_unlock(&mu);
  return ok ? 0 : -1;
}

void kinect_bridge_set_fg_boost(int mid_frames) {
  pthread_mutex_lock(&mu);
  fg_boost = mid_frames;
  pthread_mutex_unlock(&mu);
  if (fg) fg_set_boost(fg, mid_frames);
}

int kinect_bridge_get_fg_boost(void) {
  int b;
  pthread_mutex_lock(&mu);
  b = fg_boost;
  pthread_mutex_unlock(&mu);
  return b;
}

const char *kinect_bridge_fg_backend(void) {
  return fg ? fg_backend(fg) : "off";
}

void kinect_bridge_set_ir_tune(const kinect_ir_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  ir_tune = *t;
  pthread_mutex_unlock(&mu);
}

void kinect_bridge_get_ir_tune(kinect_ir_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  if (ir_tune.boost < 0.1f) kinect_ir_tune_default(&ir_tune);
  *t = ir_tune;
  pthread_mutex_unlock(&mu);
}
