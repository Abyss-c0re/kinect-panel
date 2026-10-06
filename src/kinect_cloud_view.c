#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* One USB owner. Motor inhibited. Live frames only. Dump via posix_spawn if needed. */
#include "kinect_cloud_view.h"
#include "cube_kinect_cloud.h"
#include "cube_kinect_mesh.h"
#include "cube_paths.h"
#include "kinect_fuse.h"
#include "kinect_motor.h"
#include "kinect_v4l.h"
#include "fg/fg.h"

#include <fcntl.h>
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
static cube_cloud_tune tune;
static int tune_inited, tune_dirty;
static fg_engine *fg;
static int fg_boost;

static uint16_t *depth_keep;
static pthread_mutex_t depth_mu = PTHREAD_MUTEX_INITIALIZER;
static cube_cloud snap_cloud;
static int have_snap;
static cube_mesh snap_mesh;
static int solid_3d = 1;

static void set_err(const char *s) {
  pthread_mutex_lock(&mu);
  snprintf(err, sizeof err, "%s", s);
  pthread_mutex_unlock(&mu);
}

static void paint(cube_cloud *cloud, float e, float a, float d) {
  uint8_t *tmp;
  int use_mesh;
  if (!back) return;
  pthread_mutex_lock(&mu);
  use_mesh = solid_3d && snap_mesh.ok;
  pthread_mutex_unlock(&mu);
  if (use_mesh) {
    if (cube_mesh_render_rgb(&snap_mesh, back, KINECT_RGB_W, KINECT_RGB_H, e, a, d) < 0)
      return;
  } else if (cube_cloud_render_rgb_ex(cloud, back, KINECT_RGB_W, KINECT_RGB_H, e, a, d,
                                      tune.dot) < 0)
    return;
  pthread_mutex_lock(&mu);
  tmp = front;
  front = back;
  back = tmp;
  have_preview = 1;
  npoints = cloud->n;
  if (cube_cloud_copy(&snap_cloud, cloud) == 0)
    have_snap = 1;
  pthread_mutex_unlock(&mu);
}

static void emit_fg(void) {
  int n, i, mid;
  uint8_t *tmp;
  if (!fg || !front) return;
  pthread_mutex_lock(&mu);
  mid = fg_boost;
  pthread_mutex_unlock(&mu);
  if (mid < 1) return;
  fg_set_boost(fg, mid);
  n = fg_push_rgb(fg, front);
  if (n < 2) return;
  tmp = (uint8_t *)malloc(FG_RGB);
  if (!tmp) return;
  for (i = 0; i < n; i++) {
    if (fg_copy(fg, i, tmp) != 0) continue;
    pthread_mutex_lock(&mu);
    if (front) memcpy(front, tmp, FG_RGB);
    pthread_mutex_unlock(&mu);
    if (i + 1 < n) usleep(8000);
  }
  free(tmp);
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
      emit_fg();
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

static void *cloud_main(void *arg) {
  cube_cloud cloud;
  uint32_t seen = 0;
  (void)arg;
  cube_cloud_init(&cloud);
  pthread_mutex_lock(&mu);
  if (!snap_cloud.pt) cube_cloud_init(&snap_cloud);
  cube_mesh_init(&snap_mesh);
  have_snap = 0;
  pthread_mutex_unlock(&mu);
  front = (uint8_t *)calloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3, 1);
  back = (uint8_t *)calloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3, 1);
  depth_keep = (uint16_t *)calloc(DEP_N, sizeof(uint16_t));
  if (!front || !back || !depth_keep) {
    set_err("oom");
    goto done;
  }
  if (!fg) fg = fg_create();

  /* Exclusive USB: inhibit motor reopen, then close the boot handle. */
  kinect_motor_inhibit(1);
  kinect_motor_close();
  cube_cloud_free_holders();
  usleep(300000);
  if (tune_inited) kinect_fuse_set_tune(&tune);

  if (kinect_fuse_start() != 0) {
    if (!stop_flag) {
      const char *fe = kinect_fuse_error();
      kinect_src src = kinect_fuse_source();
      set_err(fe && fe[0] ? fe : "live open failed");
      /* Dump child is the v1-only fallback. It cannot see Kinect 2. */
      if (src == KINECT_SRC_AUTO || src == KINECT_SRC_V1) loop_dump(&cloud);
    }
    goto done;
  }

  pthread_mutex_lock(&mu);
  running = 1;
  pthread_mutex_unlock(&mu);

  while (!stop_flag) {
    kinect_fuse_stats st;
    float e, a, d;
    int solid;
    cube_cloud_tune tcopy;
    kinect_fuse_get_stats(&st);
    pthread_mutex_lock(&mu);
    e = elev;
    a = azim;
    d = dist;
    solid = solid_3d;
    if (!tune_inited) {
      cube_cloud_tune_default(&tune);
      tune.color = 3;
      tune_inited = 1;
    }
    tcopy = tune;
    pthread_mutex_unlock(&mu);
    if (st.seq != seen && kinect_fuse_snapshot(&cloud) == 0 && cloud.n > 0) {
      seen = st.seq;
      if (solid && st.active == KINECT_SRC_V1 && depth_keep &&
          kinect_fuse_copy_v1_depth(depth_keep) == 0)
        cube_mesh_from_depth(&snap_mesh, depth_keep, &tcopy);
      else
        cube_mesh_clear(&snap_mesh);
      paint(&cloud, e, a, d);
      pthread_mutex_lock(&mu);
      live_frames++;
      err[0] = 0;
      pthread_mutex_unlock(&mu);
      emit_fg();
      cube_cloud_publish(&cloud);
    }
    usleep(12000);
  }
  kinect_fuse_stop();

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
    for (i = 0; i < 900; i++) {
      int run, lf;
      char e0[160];
      pthread_mutex_lock(&mu);
      run = running;
      lf = live_frames;
      snprintf(e0, sizeof e0, "%s", err);
      pthread_mutex_unlock(&mu);
      if (lf > 0 || run) return 0;
      if (e0[0] && !run && i > 15) return -1;
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
  kinect_fuse_request_stop();
  pthread_join(th, NULL);
  started = 0;
  running = 0;
  pthread_mutex_lock(&mu);
  free(front);
  free(back);
  front = back = NULL;
  have_preview = 0;
  have_snap = 0;
  cube_cloud_free(&snap_cloud);
  cube_mesh_free(&snap_mesh);
  pthread_mutex_unlock(&mu);
  free(depth_keep);
  depth_keep = NULL;
  if (fg) {
    fg_destroy(fg);
    fg = NULL;
  }
  kinect_motor_inhibit(0);
  /* Session owns power. Do not reopen motor — that left the LED/USB live. */
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

int kinect_cloud_snapshot(cube_cloud *out) {
  int rc;
  if (!out) return -1;
  pthread_mutex_lock(&mu);
  if (!have_snap) {
    pthread_mutex_unlock(&mu);
    return -1;
  }
  rc = cube_cloud_copy(out, &snap_cloud);
  pthread_mutex_unlock(&mu);
  return rc;
}

int kinect_cloud_copy_depth(uint16_t *depth) {
  if (!depth || !depth_keep) return -1;
  pthread_mutex_lock(&depth_mu);
  memcpy(depth, depth_keep, (size_t)DEP_N * sizeof(uint16_t));
  pthread_mutex_unlock(&depth_mu);
  return 0;
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

int kinect_cloud_set_tilt(double degrees) { return kinect_fuse_set_tilt(degrees); }

int kinect_cloud_get_tilt(double *degrees, int *status) {
  return kinect_fuse_get_tilt(degrees, status);
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
  if (tune.color < 0 || tune.color > 3) tune.color = 0;
  tune_inited = 1;
  tune_dirty = 1;
  pthread_mutex_unlock(&mu);
  kinect_fuse_set_tune(&tune);
}

void kinect_cloud_get_tune(cube_cloud_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  if (!tune_inited) {
    cube_cloud_tune_default(&tune);
    tune.color = 3;
    tune_inited = 1;
  }
  *t = tune;
  pthread_mutex_unlock(&mu);
}

void kinect_cloud_set_solid(int on) {
  pthread_mutex_lock(&mu);
  solid_3d = on ? 1 : 0;
  tune_dirty = 1;
  pthread_mutex_unlock(&mu);
}

int kinect_cloud_get_solid(void) {
  int v;
  pthread_mutex_lock(&mu);
  v = solid_3d;
  pthread_mutex_unlock(&mu);
  return v;
}

void kinect_cloud_set_fg_boost(int mid_frames) {
  pthread_mutex_lock(&mu);
  fg_boost = mid_frames;
  pthread_mutex_unlock(&mu);
  if (fg) fg_set_boost(fg, mid_frames);
}

int kinect_cloud_get_fg_boost(void) {
  int b;
  pthread_mutex_lock(&mu);
  b = fg_boost;
  pthread_mutex_unlock(&mu);
  return b;
}

const char *kinect_cloud_fg_backend(void) { return fg ? fg_backend(fg) : "off"; }

void kinect_cloud_reset_view(void) {
  pthread_mutex_lock(&mu);
  elev = 18.f;
  azim = -60.f;
  dist = 3.2f;
  tune_dirty = 1;
  pthread_mutex_unlock(&mu);
}
