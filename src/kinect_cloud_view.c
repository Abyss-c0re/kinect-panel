#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* One USB owner. Motor inhibited. Live frames only. Dump via posix_spawn if needed. */
#include "kinect_cloud_view.h"
#include "cube_kinect_cloud.h"
#include "cube_paths.h"
#include "kinect_motor.h"
#include "kinect_v4l.h"

#include <fcntl.h>
#include <libfreenect.h>
#include <pthread.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

#define DEP_W 640
#define DEP_H 480
#define DEP_N (DEP_W * DEP_H)

static pthread_t th;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static int started, running, stop_flag, live_frames;
static char err[160];
static uint8_t *front, *back;
static int have_preview, npoints;
static float elev = 18.f, azim = -60.f, dist = 3.2f;
static double last_tilt, pending_tilt;
static int last_tilt_st, have_pending_tilt;
static cube_cloud_tune tune;
static int tune_inited, tune_dirty;

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
  if (cube_cloud_render_rgb_ex(cloud, back, KINECT_RGB_W, KINECT_RGB_H, e, a, d, tune.dot) < 0)
    return;
  pthread_mutex_lock(&mu);
  tmp = front;
  front = back;
  back = tmp;
  have_preview = 1;
  npoints = cloud->n;
  pthread_mutex_unlock(&mu);
}

static int find_dump(char *out, size_t n) {
  char self[4096], dir[4096], *slash;
  ssize_t k;
  const char *cands[6];
  int i;
  k = readlink("/proc/self/exe", self, sizeof self - 1);
  dir[0] = 0;
  if (k > 0) {
    self[k] = 0;
    slash = strrchr(self, '/');
    if (slash) {
      *slash = 0;
      snprintf(dir, sizeof dir, "%s", self);
    }
  }
  cands[0] = getenv("KINECT_DEPTH_DUMP");
  cands[1] = dir[0] ? dir : NULL;
  cands[2] = "/home/voldemar/Dev/lab/kinect-panel/bin/kinect_depth_dump";
  cands[3] = "/home/voldemar/Dev/lab/prophecy_cube/os/trackers/kinect_depth_dump";
  cands[4] = NULL;
  if (dir[0]) {
    snprintf(out, n, "%s/kinect_depth_dump", dir);
    if (access(out, X_OK) == 0) return 0;
  }
  for (i = 0; cands[i]; i++) {
    if (!cands[i][0]) continue;
    if (i == 1) continue;
    if (access(cands[i], X_OK) == 0) {
      snprintf(out, n, "%s", cands[i]);
      return 0;
    }
  }
  return -1;
}

static int spawn_dump(const char *bin, const char *xyz, int stride) {
  pid_t pid = 0;
  int st = 0;
  char step[8];
  char *argv[5];
  posix_spawn_file_actions_t fa;
  if (stride < 1) stride = 4;
  snprintf(step, sizeof step, "%d", stride);
  argv[0] = (char *)bin;
  argv[1] = (char *)xyz;
  argv[2] = step;
  argv[3] = NULL;
  if (posix_spawn_file_actions_init(&fa) != 0) return -1;
  posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  if (posix_spawn(&pid, bin, &fa, NULL, argv, environ) != 0) {
    posix_spawn_file_actions_destroy(&fa);
    return -1;
  }
  posix_spawn_file_actions_destroy(&fa);
  if (waitpid(pid, &st, 0) < 0) return -1;
  return (WIFEXITED(st) && WEXITSTATUS(st) == 0) ? 0 : -1;
}

static int loop_dump(cube_cloud *cloud) {
  char bin[512];
  if (find_dump(bin, sizeof bin) < 0) {
    set_err("no kinect_depth_dump");
    return -1;
  }
  pthread_mutex_lock(&mu);
  running = 1;
  pthread_mutex_unlock(&mu);
  while (!stop_flag) {
    float e, a, d;
    pthread_mutex_lock(&mu);
    e = elev;
    a = azim;
    d = dist;
    pthread_mutex_unlock(&mu);
    cube_cloud_tune tcopy;
    int i;
    pthread_mutex_lock(&mu);
    if (!tune_inited) {
      cube_cloud_tune_default(&tune);
      tune_inited = 1;
    }
    tcopy = tune;
    pthread_mutex_unlock(&mu);
    if (spawn_dump(bin, CUBE_KINECT_XYZ, tcopy.stride) == 0 &&
        cube_cloud_load_xyz(cloud, CUBE_KINECT_XYZ, CUBE_CLOUD_MAX) == 0 &&
        cloud->n > 0) {
      for (i = 0; i < cloud->n; i++) cube_cloud_color_point(&cloud->pt[i], &tcopy);
      paint(cloud, e, a, d);
      pthread_mutex_lock(&mu);
      live_frames++;
      err[0] = 0;
      pthread_mutex_unlock(&mu);
    } else
      set_err("dump failed (Kinect busy?)");
    usleep(120000);
  }
  return 0;
}

static int loop_live(cube_cloud *cloud) {
  freenect_context *ctx = NULL;
  freenect_device *dev = NULL;
  freenect_frame_mode mode;
  int frames = 0, waited = 0;

  if (freenect_init(&ctx, NULL) < 0) return -1;
  freenect_set_log_level(ctx, FREENECT_LOG_FATAL);
  freenect_select_subdevices(ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
  if (freenect_num_devices(ctx) < 1 || freenect_open_device(ctx, &dev, 0) < 0) {
    freenect_shutdown(ctx);
    return -1;
  }
  mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT);
  if (!mode.is_valid || freenect_set_depth_mode(dev, mode) < 0 ||
      freenect_start_depth(dev) < 0) {
    freenect_close_device(dev);
    freenect_shutdown(ctx);
    return -1;
  }
  freenect_set_depth_callback(dev, depth_cb);
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
    {
      cube_cloud_tune tcopy;
      int dirty;
      pthread_mutex_lock(&mu);
      if (!tune_inited) {
        cube_cloud_tune_default(&tune);
        tune_inited = 1;
      }
      tcopy = tune;
      dirty = tune_dirty;
      tune_dirty = 0;
      pthread_mutex_unlock(&mu);
      if (!fresh && !dirty) {
        if (!live_frames && ++waited > 80) break; /* ~2.4s no frames → dump */
        continue;
      }
      waited = 0;
      if (cube_cloud_from_depth_ex(cloud, depth_keep, &tcopy) == 0 && cloud->n > 0) {
        paint(cloud, e, a, d);
        pthread_mutex_lock(&mu);
        live_frames++;
        err[0] = 0;
        pthread_mutex_unlock(&mu);
        if ((++frames % 8) == 0) cube_cloud_publish(cloud);
      }
    }
  }

  live_dev = NULL;
  freenect_stop_depth(dev);
  freenect_set_led(dev, LED_OFF);
  freenect_close_device(dev);
  freenect_shutdown(ctx);
  return live_frames > 0 ? 0 : -1;
}

static void *cloud_main(void *arg) {
  cube_cloud cloud;
  (void)arg;
  cube_cloud_init(&cloud);
  front = (uint8_t *)calloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3, 1);
  back = (uint8_t *)calloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3, 1);
  depth_keep = (uint16_t *)calloc(DEP_N, sizeof(uint16_t));
  if (!front || !back || !depth_keep) {
    set_err("oom");
    goto done;
  }

  /* Exclusive USB: inhibit motor reopen, then close the boot handle. */
  kinect_motor_inhibit(1);
  kinect_motor_close();
  cube_cloud_free_holders();
  usleep(300000);

  if (loop_live(&cloud) < 0 && !stop_flag) {
    /* Honest fallback: exclusive dump child, parent holds no libusb. */
    set_err("live open failed — dump");
    loop_dump(&cloud);
  }

done:
  cube_cloud_free(&cloud);
  pthread_mutex_lock(&mu);
  running = 0;
  pthread_mutex_unlock(&mu);
  return NULL;
}

int kinect_cloud_start(void) {
  kinect_cloud_stop();
  pthread_mutex_lock(&mu);
  err[0] = 0;
  npoints = 0;
  live_frames = 0;
  stop_flag = 0;
  pthread_mutex_unlock(&mu);

  kinect_motor_inhibit(1);
  kinect_motor_close();

  if (pthread_create(&th, NULL, cloud_main, NULL) != 0) {
    kinect_motor_inhibit(0);
    set_err("cloud thread");
    return -1;
  }
  started = 1;
  {
    int i;
    for (i = 0; i < 200; i++) {
      int run, lf;
      char e0[160];
      pthread_mutex_lock(&mu);
      run = running;
      lf = live_frames;
      snprintf(e0, sizeof e0, "%s", err);
      pthread_mutex_unlock(&mu);
      if (lf > 0) return 0;
      if (run && i > 12) return 0; /* thread owns USB; UI waits for live */
      if (e0[0] && !run && i > 16) return -1;
      usleep(25000);
    }
  }
  if (kinect_cloud_running() || live_frames > 0) return 0;
  if (!err[0]) set_err("no live depth");
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
  kinect_motor_inhibit(0);
  kinect_motor_open();
}

int kinect_cloud_running(void) {
  int r;
  pthread_mutex_lock(&mu);
  r = running;
  pthread_mutex_unlock(&mu);
  return r;
}

int kinect_cloud_live(void) {
  int n;
  pthread_mutex_lock(&mu);
  n = live_frames;
  pthread_mutex_unlock(&mu);
  return n;
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

void kinect_cloud_set_tune(const cube_cloud_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  tune = *t;
  if (tune.stride < 1) tune.stride = 1;
  if (tune.stride > 8) tune.stride = 8;
  if (tune.z_near < 0.3f) tune.z_near = 0.3f;
  if (tune.z_far < tune.z_near + 0.15f) tune.z_far = tune.z_near + 0.15f;
  if (tune.dot < 1) tune.dot = 1;
  if (tune.dot > 4) tune.dot = 4;
  if (tune.color < 0 || tune.color > 2) tune.color = 0;
  tune_inited = 1;
  tune_dirty = 1;
  pthread_mutex_unlock(&mu);
}

void kinect_cloud_get_tune(cube_cloud_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  if (!tune_inited) {
    cube_cloud_tune_default(&tune);
    tune_inited = 1;
  }
  *t = tune;
  pthread_mutex_unlock(&mu);
}

void kinect_cloud_reset_view(void) {
  pthread_mutex_lock(&mu);
  elev = 18.f;
  azim = -60.f;
  dist = 3.2f;
  tune_dirty = 1;
  pthread_mutex_unlock(&mu);
}
