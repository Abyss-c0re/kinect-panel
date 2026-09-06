#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* Live depth in-process. No fork, no freenect_sync (those crash GTK + hold USB). */
#include "kinect_cloud_view.h"
#include "cube_kinect_cloud.h"
#include "cube_paths.h"
#include "kinect_motor.h"
#include "kinect_v4l.h"

#include <libfreenect.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define DEP_W 640
#define DEP_H 480
#define DEP_N (DEP_W * DEP_H)

static pthread_t th;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static int started, running, stop_flag;
static char err[160];
static uint8_t *front, *back;
static int have_preview, npoints;
static float elev = 18.f, azim = -60.f, dist = 3.2f;
static double last_tilt, pending_tilt;
static int last_tilt_st, have_pending_tilt;

static uint16_t *depth_keep;
static int depth_new;
static pthread_mutex_t depth_mu = PTHREAD_MUTEX_INITIALIZER;
static freenect_device *live_dev;

static void set_err(const char *s) {
  pthread_mutex_lock(&mu);
  snprintf(err, sizeof err, "%s", s);
  pthread_mutex_unlock(&mu);
}

static void depth_cb(freenect_device *dev, void *depth, uint32_t ts) {
  (void)dev;
  (void)ts;
  if (!depth_keep || !depth) return;
  pthread_mutex_lock(&depth_mu);
  memcpy(depth_keep, depth, (size_t)DEP_N * sizeof(uint16_t));
  depth_new = 1;
  pthread_mutex_unlock(&depth_mu);
}

static void paint(cube_cloud *cloud, float e, float a, float d) {
  uint8_t *tmp;
  if (!back) return;
  if (cube_cloud_render_rgb(cloud, back, KINECT_RGB_W, KINECT_RGB_H, e, a, d) < 0)
    return;
  pthread_mutex_lock(&mu);
  tmp = front;
  front = back;
  back = tmp;
  have_preview = 1;
  npoints = cloud->n;
  if (cloud->n > 0) err[0] = 0;
  pthread_mutex_unlock(&mu);
}

static void *cloud_main(void *arg) {
  cube_cloud cloud;
  freenect_context *ctx = NULL;
  freenect_device *dev = NULL;
  freenect_frame_mode mode;
  int frames = 0;
  (void)arg;

  cube_cloud_init(&cloud);
  front = (uint8_t *)calloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3, 1);
  back = (uint8_t *)calloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3, 1);
  depth_keep = (uint16_t *)calloc(DEP_N, sizeof(uint16_t));
  if (!front || !back || !depth_keep) {
    set_err("oom");
    goto done;
  }

  /* Last file first so the preview is never blank. */
  if (cube_cloud_load_xyz(&cloud, CUBE_KINECT_XYZ, CUBE_CLOUD_MAX) == 0)
    paint(&cloud, elev, azim, dist);

  kinect_motor_close();
  cube_cloud_free_holders();
  usleep(250000);

  if (freenect_init(&ctx, NULL) < 0) {
    set_err("freenect_init failed");
    goto done;
  }
  freenect_set_log_level(ctx, FREENECT_LOG_WARNING);
  freenect_select_subdevices(ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
  if (freenect_num_devices(ctx) < 1 || freenect_open_device(ctx, &dev, 0) < 0) {
    set_err("open Kinect camera+motor failed");
    goto done;
  }
  mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT);
  if (!mode.is_valid || freenect_set_depth_mode(dev, mode) < 0) {
    set_err("depth mode 11-bit failed");
    goto done;
  }
  freenect_set_depth_callback(dev, depth_cb);
  if (freenect_start_depth(dev) < 0) {
    set_err("start_depth failed");
    goto done;
  }
  freenect_set_led(dev, LED_YELLOW);
  live_dev = dev;

  pthread_mutex_lock(&mu);
  running = 1;
  pthread_mutex_unlock(&mu);

  while (!stop_flag) {
    struct timeval tv = {0, 30000};
    float e, a, d;
    int fresh = 0;
    freenect_process_events_timeout(ctx, &tv);

    pthread_mutex_lock(&mu);
    e = elev;
    a = azim;
    d = dist;
    if (have_pending_tilt && live_dev) {
      freenect_set_tilt_degs(live_dev, pending_tilt);
      have_pending_tilt = 0;
    }
    pthread_mutex_unlock(&mu);

    if (live_dev) {
      freenect_raw_tilt_state *st;
      freenect_update_tilt_state(live_dev);
      st = freenect_get_tilt_state(live_dev);
      if (st) {
        pthread_mutex_lock(&mu);
        last_tilt = freenect_get_tilt_degs(st);
        last_tilt_st = (int)freenect_get_tilt_status(st);
        pthread_mutex_unlock(&mu);
      }
    }

    pthread_mutex_lock(&depth_mu);
    fresh = depth_new;
    depth_new = 0;
    pthread_mutex_unlock(&depth_mu);
    if (!fresh) continue;

    if (cube_cloud_from_depth(&cloud, depth_keep, 3) == 0 && cloud.n > 0) {
      paint(&cloud, e, a, d);
      if ((++frames % 8) == 0) cube_cloud_publish(&cloud);
    }
  }

  live_dev = NULL;
  if (dev) {
    freenect_stop_depth(dev);
    freenect_set_led(dev, LED_OFF);
    freenect_close_device(dev);
    dev = NULL;
  }
done:
  if (ctx) {
    freenect_shutdown(ctx);
    ctx = NULL;
  }
  cube_cloud_free(&cloud);
  pthread_mutex_lock(&mu);
  running = 0;
  pthread_mutex_unlock(&mu);
  /* Keep last preview buffers until stop() so GTK never sees a freed frame. */
  return NULL;
}

int kinect_cloud_start(void) {
  kinect_cloud_stop();
  pthread_mutex_lock(&mu);
  err[0] = 0;
  npoints = 0;
  stop_flag = 0;
  pthread_mutex_unlock(&mu);

  if (pthread_create(&th, NULL, cloud_main, NULL) != 0) {
    set_err("cloud thread");
    return -1;
  }
  started = 1;
  {
    int i;
    for (i = 0; i < 160; i++) {
      int run;
      char e0[160];
      pthread_mutex_lock(&mu);
      run = running;
      snprintf(e0, sizeof e0, "%s", err);
      pthread_mutex_unlock(&mu);
      if (run) return 0;
      if (e0[0] && i > 8) return -1;
      usleep(25000);
    }
  }
  if (kinect_cloud_running()) return 0;
  if (!err[0]) set_err("cloud did not start");
  return -1;
}

void kinect_cloud_stop(void) {
  if (!started) return;
  stop_flag = 1;
  pthread_join(th, NULL);
  started = 0;
  running = 0;
  live_dev = NULL;
  pthread_mutex_lock(&mu);
  free(front);
  free(back);
  front = back = NULL;
  have_preview = 0;
  pthread_mutex_unlock(&mu);
  free(depth_keep);
  depth_keep = NULL;
  kinect_motor_open();
}

int kinect_cloud_running(void) {
  int r;
  pthread_mutex_lock(&mu);
  r = running;
  pthread_mutex_unlock(&mu);
  return r;
}

const char *kinect_cloud_error(void) { return err; }

int kinect_cloud_preview_rgb(uint8_t *rgb) {
  int ok;
  if (!rgb) return -1;
  pthread_mutex_lock(&mu);
  ok = have_preview && front;
  if (ok) memcpy(rgb, front, (size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
  pthread_mutex_unlock(&mu);
  return ok ? 0 : -1;
}

int kinect_cloud_points(void) {
  int n;
  pthread_mutex_lock(&mu);
  n = npoints;
  pthread_mutex_unlock(&mu);
  return n;
}

void kinect_cloud_orbit(float e, float a, float d) {
  pthread_mutex_lock(&mu);
  elev = e;
  azim = a;
  dist = d < 0.5f ? 0.5f : d;
  pthread_mutex_unlock(&mu);
}

void kinect_cloud_get_orbit(float *e, float *a, float *d) {
  pthread_mutex_lock(&mu);
  if (e) *e = elev;
  if (a) *a = azim;
  if (d) *d = dist;
  pthread_mutex_unlock(&mu);
}

int kinect_cloud_set_tilt(double degrees) {
  if (degrees < -30) degrees = -30;
  if (degrees > 30) degrees = 30;
  pthread_mutex_lock(&mu);
  pending_tilt = degrees;
  have_pending_tilt = 1;
  pthread_mutex_unlock(&mu);
  return 0;
}

int kinect_cloud_get_tilt(double *degrees, int *status) {
  pthread_mutex_lock(&mu);
  if (degrees) *degrees = last_tilt;
  if (status) *status = last_tilt_st;
  pthread_mutex_unlock(&mu);
  return 0;
}
