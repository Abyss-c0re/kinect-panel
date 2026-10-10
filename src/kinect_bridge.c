#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "kinect_bridge.h"
#include "kinect_ir.h"
#include "kinect_v4l.h"
#include "kinect2_device.h"
#include "kinect_fuse.h"
#include "kinect_net.h"
#include "fg/fg.h"

#include <linux/videodev2.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
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
static int sensor; /* 1 = Kinect gspca, 2 = Kinect 2, while running */
static int from_net;
static int sensor_plan;
static uint8_t preview[KINECT_RGB_W * KINECT_RGB_H * 3];
static int have_preview;
static kinect_ir_tune ir_tune;
static fg_engine *fg;
static int fg_boost;

static uint64_t bridge_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void offer_local_image(const uint8_t *rgb, int cam, int ir) {
  if (!rgb) return;
  if (ir)
    kinect_net_offer_ir(rgb, cam, bridge_now());
  else
    kinect_net_offer_rgb(rgb, cam, bridge_now());
}

static int peer_has(int bit) {
  int i;
  if (kinect_net_peer_sensors() & bit) return 1;
  if (!kinect_net_active()) return 0;
  for (i = 0; i < 40 && !stop_flag; i++) {
    if (kinect_net_peer_sensors() & bit) return 1;
    if (!kinect_net_connected() && i >= 6) return 0;
    usleep(50000);
  }
  return (kinect_net_peer_sensors() & bit) != 0;
}

static void set_err(const char *s) {
  pthread_mutex_lock(&mu);
  snprintf(err, sizeof err, "%s", s);
  pthread_mutex_unlock(&mu);
}

/* Preview first, then optional frame-gen mids, then the loopback. */
static void publish_rgb(uint8_t *rgb, uint8_t *yuyv, int vfd) {
  int nout = 1, fi, mid;
  uint8_t *gen = NULL;
  pthread_mutex_lock(&mu);
  mid = fg_boost;
  pthread_mutex_unlock(&mu);
  if (mid > 0 && !fg) fg = fg_create();
  if (fg && mid > 0) {
    fg_set_boost(fg, mid);
    nout = fg_push_rgb(fg, rgb);
    if (nout < 1) nout = 1;
    gen = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
  }
  for (fi = 0; fi < nout; fi++) {
    uint8_t *fr = rgb;
    if (gen && fg_copy(fg, fi, gen) == 0) fr = gen;
    pthread_mutex_lock(&mu);
    memcpy(preview, fr, sizeof preview);
    have_preview = 1;
    pthread_mutex_unlock(&mu);
    if (vfd >= 0) {
      kinect_rgb_to_yuyv(fr, yuyv);
      (void)kinect_vcam_write_yuyv(vfd, yuyv);
    }
    if (fi + 1 < nout) usleep(6000);
  }
  free(gen);
}

/* Kinect camera if it has a gspca node. Kinect 2 or Both uses libfreenect2.
 * Auto uses gspca when that node exists, otherwise Kinect 2. */
static int image_from_v2(void) {
  kinect_src s = kinect_fuse_source();
  char node[64];
  if (s == KINECT_SRC_V2 || s == KINECT_SRC_BOTH) return 1;
  if (s == KINECT_SRC_V1) return 0;
  if (kinect_find_source(node, sizeof node) == 0) return 0;
  return 1;
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
  /* Preview must work even if the loopback write side is missing/blocked. */

  pthread_mutex_lock(&mu);
  sensor = 1;
  running = 1;
  pthread_mutex_unlock(&mu);

  while (!stop_flag) {
    uint8_t *rgb = fgbuf;
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
    offer_local_image(rgb, 1, ir);
    publish_rgb(rgb, yuyv, vfd);
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
  sensor = 0;
  pthread_mutex_unlock(&mu);
  return NULL;
}

static void *bridge_v2(void *arg) {
  int vfd = -1;
  int ir = (mode[0] == 'I' || mode[0] == 'i');
  int opened = 0;
  uint8_t *rgb = NULL, *yuyv = NULL, *grey = NULL;
  uint16_t *ir16 = NULL;
  const char *why;
  (void)arg;

  rgb = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
  yuyv = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 2);
  if (ir) {
    grey = (uint8_t *)malloc((size_t)KINECT2_N);
    ir16 = (uint16_t *)malloc((size_t)KINECT2_N * sizeof(uint16_t));
  }
  if (!rgb || !yuyv || (ir && (!grey || !ir16))) {
    set_err("oom");
    goto out;
  }
  if (ir) {
    if (kinect2_open_ir() != 0) {
      why = kinect2_error();
      set_err(why && why[0] ? why : "kinect 2 ir failed");
      goto out;
    }
  } else if (kinect2_open_color() != 0) {
    why = kinect2_error();
    set_err(why && why[0] ? why : "kinect 2 color failed");
    goto out;
  }
  opened = 1;
  vfd = kinect_vcam_open(vcam);

  pthread_mutex_lock(&mu);
  sensor = 2;
  running = 1;
  pthread_mutex_unlock(&mu);

  while (!stop_flag) {
    if (ir) {
      kinect_ir_tune t;
      int got = kinect2_take_ir(ir16, 200);
      if (got == 0) continue;
      if (got < 0) {
        set_err("kinect 2 ir read failed");
        break;
      }
      pthread_mutex_lock(&mu);
      t = ir_tune;
      pthread_mutex_unlock(&mu);
      /* Grade expects the Kinect 10-bit range. Kinect 2 IR is 16-bit. */
      {
        int i;
        for (i = 0; i < KINECT2_N; i++) ir16[i] = (uint16_t)(ir16[i] >> 6);
      }
      kinect_ir_grade(ir16, grey, KINECT2_N, &t);
      kinect_ir_grey_to_rgb640(grey, KINECT2_W, KINECT2_H, rgb);
    } else {
      int got = kinect2_take_rgb(rgb, 200);
      if (got == 0) continue;
      if (got < 0) {
        set_err("kinect 2 color read failed");
        break;
      }
    }
    offer_local_image(rgb, 2, ir);
    publish_rgb(rgb, yuyv, vfd);
  }

out:
  if (opened) kinect2_close();
  if (vfd >= 0) close(vfd);
  free(rgb);
  free(yuyv);
  free(grey);
  free(ir16);
  pthread_mutex_lock(&mu);
  running = 0;
  sensor = 0;
  pthread_mutex_unlock(&mu);
  return NULL;
}

static void *bridge_net(void *arg) {
  int ir = (mode[0] == 'I' || mode[0] == 'i');
  int vfd = -1;
  uint8_t *rgb = NULL, *yuyv = NULL;
  (void)arg;
  rgb = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
  yuyv = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 2);
  if (!rgb || !yuyv) {
    set_err("oom");
    goto out;
  }
  kinect_net_want(ir ? 3 : 2);
  vfd = vcam[0] ? kinect_vcam_open(vcam) : -1;
  pthread_mutex_lock(&mu);
  sensor = sensor_plan ? sensor_plan : 1;
  from_net = 1;
  running = 1;
  pthread_mutex_unlock(&mu);
  while (!stop_flag) {
    uint64_t ts = 0;
    if (kinect_net_take_rgb(rgb, &ts) != 1) {
      usleep(4000);
      continue;
    }
    publish_rgb(rgb, yuyv, vfd);
  }
out:
  if (vfd >= 0) close(vfd);
  free(rgb);
  free(yuyv);
  pthread_mutex_lock(&mu);
  running = 0;
  sensor = 0;
  from_net = 0;
  pthread_mutex_unlock(&mu);
  return NULL;
}

int kinect_bridge_start(const char *want_mode) {
  int use_v2, spins, i;
  void *(*fn)(void *);
  kinect_bridge_stop();
  stop_flag = 0;
  pthread_mutex_lock(&mu);
  err[0] = 0;
  have_preview = 0;
  from_net = 0;
  sensor_plan = 0;
  if (ir_tune.boost < 0.1f) kinect_ir_tune_default(&ir_tune);
  snprintf(mode, sizeof mode, "%s", (want_mode && want_mode[0] == 'I') ? "IR" : "RGB");
  pthread_mutex_unlock(&mu);
  if (kinect_find_vcam(vcam, sizeof vcam) < 0)
    vcam[0] = 0; /* preview / headless still run; loopback is OUT only */
  use_v2 = image_from_v2();
  if (use_v2) {
    if (kinect2_count() > 0) {
      snprintf(source, sizeof source, "kinect2");
      kinect_net_set_local(2);
    } else if (peer_has(2)) {
      from_net = 1;
      sensor_plan = 2;
      snprintf(source, sizeof source, "kinect2-net");
    } else {
      set_err("no kinect 2");
      return -1;
    }
  } else if (kinect_find_source(source, sizeof source) == 0) {
    if (kinect_set_mode(source, mode) < 0) {
      set_err("v4l2 S_FMT failed");
      return -1;
    }
    kinect_net_set_local(1);
  } else if (peer_has(1)) {
    from_net = 1;
    sensor_plan = 1;
    snprintf(source, sizeof source, "kinect-net");
  } else {
    set_err("No Kinect /dev/video* (gspca)");
    return -1;
  }
  fn = from_net ? bridge_net : (use_v2 ? bridge_v2 : bridge_main);
  if (pthread_create(&th, NULL, fn, NULL) != 0) {
    set_err("bridge thread");
    return -1;
  }
  started = 1;
  /* Kinect 2 color open can take a few seconds. Return as soon as it is up. */
  spins = use_v2 ? 140 : 40;
  for (i = 0; i < spins; i++) {
    int run, failed;
    pthread_mutex_lock(&mu);
    run = running;
    failed = err[0] && !running;
    pthread_mutex_unlock(&mu);
    if (run) return 0;
    if (failed && i > 2) {
      kinect_bridge_stop();
      return -1;
    }
    usleep(50000);
  }
  if (!kinect_bridge_running()) {
    if (!err[0]) set_err("bridge did not start");
    kinect_bridge_stop();
    return -1;
  }
  return 0;
}

void kinect_bridge_stop(void) {
  if (!started) return;
  stop_flag = 1;
  pthread_join(th, NULL);
  started = 0;
  pthread_mutex_lock(&mu);
  running = 0;
  sensor = 0;
  from_net = 0;
  pthread_mutex_unlock(&mu);
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

int kinect_bridge_sensor(void) {
  int s;
  pthread_mutex_lock(&mu);
  s = running ? sensor : 0;
  pthread_mutex_unlock(&mu);
  return s;
}

int kinect_bridge_net(void) {
  int n;
  pthread_mutex_lock(&mu);
  n = running && from_net;
  pthread_mutex_unlock(&mu);
  return n;
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
