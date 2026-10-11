#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* v1 (libfreenect) and v2 (kinect2_*) stay on their own USB devices.
 * Frames are stamped with CLOCK_MONOTONIC when they arrive and paired
 * inside KSYNC_PAIR_US. v2 points are moved by the saved rigid into the
 * v1 camera, which is the cloud the panel and OpenXR already use. */
#include "kinect_fuse.h"
#include "kinect2_device.h"
#include "kinect_net.h"

#include <libfreenect.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define RING 4
#define V1_W 640
#define V1_H 480
#define V1_N (V1_W * V1_H)

static pthread_t th;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static volatile int stop_flag;
static volatile int state; /* 0 idle, 1 opening, 2 up, 3 failed */
static int started;

static kinect_src want_src = KINECT_SRC_AUTO;
static int want_inited;
static kinect_src active = KINECT_SRC_AUTO;

static freenect_context *ctx;
static freenect_device *dev;
static uint16_t *cb_depth;
static uint64_t cb_ts;
static int cb_new;
static pthread_mutex_t cb_mu = PTHREAD_MUTEX_INITIALIZER;

static uint16_t *ring1[RING];
static float *ring2[RING];
static uint64_t stamp1[RING], stamp2[RING];
static int nring1, nring2, w1, w2;
static uint16_t *pub_depth;

static cube_cloud snap, sens1, sens2;
static int clouds_ready;
static cube_cloud_tune tune;
static int tune_ready, tune_dirty;
static ksync_rigid rigid;
static double pending_tilt, last_tilt;
static int have_pending, last_tilt_st;
static char err[160];
static kinect_fuse_stats stats;
static uint32_t seq;
static int remote1, remote2;
static int local_v1_usb;
static uint16_t *net_u16;
static float *wire_mm;
static float *remote_m;
static uint64_t last_offer2_us;

static uint64_t mono_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void set_err(const char *s) {
  pthread_mutex_lock(&mu);
  snprintf(err, sizeof err, "%s", s ? s : "");
  snprintf(stats.err, sizeof stats.err, "%s", err);
  pthread_mutex_unlock(&mu);
}

static void ensure_src(void) {
  if (want_inited) return;
  want_src = kinect_src_from_env();
  want_inited = 1;
}

const char *kinect_src_name(kinect_src s) {
  switch (s) {
    case KINECT_SRC_V1:
      return "v1";
    case KINECT_SRC_V2:
      return "v2";
    case KINECT_SRC_BOTH:
      return "both";
    case KINECT_SRC_AUTO:
    default:
      return "auto";
  }
}

kinect_src kinect_src_parse(const char *s) {
  kinect_src out = KINECT_SRC_AUTO;
  if (kinect_src_known(s, &out)) return out;
  return KINECT_SRC_AUTO;
}

int kinect_src_known(const char *s, kinect_src *out) {
  kinect_src v;
  if (!s || !s[0]) return 0;
  if (strcmp(s, "auto") == 0 || strcmp(s, "a") == 0)
    v = KINECT_SRC_AUTO;
  else if (strcmp(s, "v1") == 0 || strcmp(s, "1") == 0 || strcmp(s, "kinect") == 0 ||
           strcmp(s, "og") == 0)
    v = KINECT_SRC_V1;
  else if (strcmp(s, "v2") == 0 || strcmp(s, "2") == 0 || strcmp(s, "kinect2") == 0 ||
           strcmp(s, "k2") == 0 || strcmp(s, "cloud2") == 0)
    v = KINECT_SRC_V2;
  else if (strcmp(s, "both") == 0 || strcmp(s, "sync") == 0 || strcmp(s, "pair") == 0)
    v = KINECT_SRC_BOTH;
  else
    return 0;
  if (out) *out = v;
  return 1;
}

kinect_src kinect_src_from_env(void) {
  const char *e = getenv("KINECT_SRC");
  if (!e || !e[0]) return KINECT_SRC_AUTO;
  return kinect_src_parse(e);
}

void kinect_fuse_set_source(kinect_src s) {
  pthread_mutex_lock(&mu);
  want_src = s;
  want_inited = 1;
  stats.requested = s;
  pthread_mutex_unlock(&mu);
}

kinect_src kinect_fuse_source(void) {
  kinect_src s;
  ensure_src();
  pthread_mutex_lock(&mu);
  s = want_src;
  pthread_mutex_unlock(&mu);
  return s;
}

static int probe_v1(void) {
  freenect_context *c = NULL;
  int n;
  if (freenect_init(&c, NULL) < 0) return 0;
  freenect_set_log_level(c, FREENECT_LOG_FATAL);
  freenect_select_subdevices(c, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
  n = freenect_num_devices(c);
  freenect_shutdown(c);
  return n > 0 ? n : 0;
}

int kinect_fuse_probe(int *v1, int *v2) {
  int a, b, up;
  pthread_mutex_lock(&mu);
  up = state == 2;
  a = stats.v1_open;
  b = stats.v2_open;
  pthread_mutex_unlock(&mu);
  if (up) {
    if (v1) *v1 = a;
    if (v2) *v2 = b;
    return 0;
  }
  a = probe_v1();
  b = kinect2_count();
  if (v1) *v1 = a;
  if (v2) *v2 = b;
  return 0;
}

static void depth_cb(freenect_device *d, void *depth, uint32_t ts) {
  (void)d;
  (void)ts;
  if (!cb_depth || !depth) return;
  pthread_mutex_lock(&cb_mu);
  memcpy(cb_depth, depth, (size_t)V1_N * sizeof(uint16_t));
  cb_ts = mono_us();
  cb_new = 1;
  pthread_mutex_unlock(&cb_mu);
}

static void close_v1(void) {
  if (dev) {
    freenect_stop_depth(dev);
    freenect_set_led(dev, LED_OFF);
    freenect_close_device(dev);
    dev = NULL;
  }
  if (ctx) {
    freenect_shutdown(ctx);
    ctx = NULL;
  }
}

static int open_v1(void) {
  freenect_frame_mode mode;
  double tilt = 0;
  int do_tilt = 0;
  if (freenect_init(&ctx, NULL) < 0) {
    set_err("freenect_init");
    return -1;
  }
  freenect_set_log_level(ctx, FREENECT_LOG_FATAL);
  freenect_select_subdevices(ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
  if (freenect_num_devices(ctx) < 1 || freenect_open_device(ctx, &dev, 0) < 0) {
    freenect_shutdown(ctx);
    ctx = NULL;
    set_err("kinect busy or missing");
    return -1;
  }
  mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT);
  if (!mode.is_valid || freenect_set_depth_mode(dev, mode) < 0 || freenect_start_depth(dev) < 0) {
    close_v1();
    set_err("kinect depth start");
    return -1;
  }
  freenect_set_depth_callback(dev, depth_cb);
  freenect_set_led(dev, LED_YELLOW);
  pthread_mutex_lock(&mu);
  if (have_pending) {
    tilt = pending_tilt;
    have_pending = 0;
    do_tilt = 1;
  }
  pthread_mutex_unlock(&mu);
  if (do_tilt) freenect_set_tilt_degs(dev, tilt);
  return 0;
}

static int alloc_rings(void) {
  int i;
  cb_depth = (uint16_t *)calloc(V1_N, sizeof(uint16_t));
  pub_depth = (uint16_t *)calloc(V1_N, sizeof(uint16_t));
  net_u16 = (uint16_t *)calloc(V1_N, sizeof(uint16_t));
  wire_mm = (float *)calloc(KINECT2_N, sizeof(float));
  remote_m = (float *)calloc(KINECT2_N, sizeof(float));
  if (!cb_depth || !pub_depth || !net_u16 || !wire_mm || !remote_m) return -1;
  for (i = 0; i < RING; i++) {
    ring1[i] = (uint16_t *)calloc(V1_N, sizeof(uint16_t));
    ring2[i] = (float *)calloc(KINECT2_N, sizeof(float));
    if (!ring1[i] || !ring2[i]) return -1;
  }
  return 0;
}

static void free_rings(void) {
  int i;
  free(cb_depth);
  free(pub_depth);
  free(net_u16);
  free(wire_mm);
  free(remote_m);
  cb_depth = pub_depth = net_u16 = NULL;
  wire_mm = remote_m = NULL;
  for (i = 0; i < RING; i++) {
    free(ring1[i]);
    free(ring2[i]);
    ring1[i] = NULL;
    ring2[i] = NULL;
  }
  nring1 = nring2 = w1 = w2 = 0;
  cb_new = 0;
}

static int latest_i(int n, int w) {
  if (n < 1) return -1;
  return (w + RING - 1) % RING;
}

static void push1(void) {
  memcpy(ring1[w1], cb_depth, (size_t)V1_N * sizeof(uint16_t));
  stamp1[w1] = cb_ts;
  w1 = (w1 + 1) % RING;
  if (nring1 < RING) nring1++;
}

static void load_rigid(void) {
  char path[512];
  ksync_rigid_identity(&rigid);
  if (ksync_extrinsic_path(path, sizeof path) == 0)
    ksync_rigid_load(&rigid, path);
}

static int open_all(void) {
  kinect_src req, use;
  int have1 = 0, have2 = 0;
  ensure_src();
  pthread_mutex_lock(&mu);
  req = want_src;
  if (!tune_ready) {
    cube_cloud_tune_default(&tune);
    tune.color = 3;
    tune_ready = 1;
  }
  pthread_mutex_unlock(&mu);
  load_rigid();
  if (alloc_rings() < 0) {
    set_err("oom");
    return -1;
  }
  if (!clouds_ready) {
    cube_cloud_init(&snap);
    cube_cloud_init(&sens1);
    cube_cloud_init(&sens2);
    clouds_ready = 1;
  }
  {
    int local1 = 0, local2 = 0, peer = 0, spins = 0, max_spins = 0;
    if (req != KINECT_SRC_V2) local1 = probe_v1() > 0;
    if (stop_flag) return -1;
    if (req != KINECT_SRC_V1) local2 = kinect2_count() > 0;
    if (stop_flag) return -1;
    kinect_net_set_local((local1 ? 1 : 0) | (local2 ? 2 : 0));
    if (kinect_net_active()) {
      int need = 0;
      if (req == KINECT_SRC_BOTH && (!local1 || !local2)) {
        need = 1;
        /* Local depth can start at once. A missing sensor is filled when the
         * peer connects. With nothing local, wait longer for that peer. */
        max_spins = (local1 || local2) ? 10 : 160;
      } else if (req == KINECT_SRC_V1 && !local1) {
        need = 1;
        max_spins = 60;
      } else if (req == KINECT_SRC_V2 && !local2) {
        need = 1;
        max_spins = 60;
      } else if (req == KINECT_SRC_AUTO && !(local1 && local2)) {
        need = 1;
        max_spins = 20;
      }
      if (need) {
        while (spins < max_spins && !stop_flag) {
          int h1, h2;
          peer = kinect_net_peer_sensors();
          h1 = local1 || (peer & 1);
          h2 = local2 || (peer & 2);
          if (req == KINECT_SRC_BOTH && h1 && h2) break;
          if (req == KINECT_SRC_V1 && h1) break;
          if (req == KINECT_SRC_V2 && h2) break;
          if (req == KINECT_SRC_AUTO && h1 && h2) break;
          if (req == KINECT_SRC_AUTO && !kinect_net_connected() && spins >= 6 && (local1 || local2))
            break;
          usleep(50000);
          spins++;
        }
        peer = kinect_net_peer_sensors();
      }
    }
    if (req != KINECT_SRC_V2) have1 = local1 || (peer & 1);
    if (req != KINECT_SRC_V1) have2 = local2 || (peer & 2);
    use = req;
    if (req == KINECT_SRC_AUTO) {
      if (have1 && have2)
        use = KINECT_SRC_BOTH;
      else if (have2)
        use = KINECT_SRC_V2;
      else if (have1)
        use = KINECT_SRC_V1;
      else {
        set_err("no kinect");
        return -1;
      }
    } else if (req == KINECT_SRC_V1 && !have1) {
      set_err("no kinect");
      return -1;
    } else if (req == KINECT_SRC_V2 && !have2) {
      set_err(kinect2_error()[0] ? kinect2_error() : "no kinect 2");
      return -1;
    } else if (req == KINECT_SRC_BOTH && (!have1 || !have2)) {
      /* Share stays up on the local camera until the other Kinect appears. */
      if (!(kinect_net_active() && (local1 || local2 || have1 || have2))) {
        set_err("need kinect and kinect 2");
        return -1;
      }
    }
    remote1 = (use == KINECT_SRC_V1 || use == KINECT_SRC_BOTH) && !local1;
    remote2 = (use == KINECT_SRC_V2 || use == KINECT_SRC_BOTH) && !local2;
    if ((use == KINECT_SRC_V1 || use == KINECT_SRC_BOTH) && local1) {
      if (open_v1() != 0) {
        if ((peer & 1) && req != KINECT_SRC_V2) {
          remote1 = 1;
          set_err("");
        } else if (req == KINECT_SRC_AUTO && (local2 || (peer & 2))) {
          remote1 = 0;
          use = KINECT_SRC_V2;
          set_err("");
        } else
          return -1;
      }
    }
    if (stop_flag) return -1;
    if ((use == KINECT_SRC_V2 || use == KINECT_SRC_BOTH) && local2) {
      if (kinect2_open() != 0) {
        if ((peer & 2) && req != KINECT_SRC_V1) {
          remote2 = 1;
          set_err("");
        } else if (req == KINECT_SRC_AUTO && (dev || remote1)) {
          remote2 = 0;
          use = (dev || remote1) ? KINECT_SRC_V1 : use;
          set_err("kinect 2 failed — kinect only");
        } else {
          if (!err[0]) set_err(kinect2_error()[0] ? kinect2_error() : "kinect 2 open failed");
          return -1;
        }
      }
    }
  }
  {
    int up1 = (dev != NULL) || remote1;
    int up2 = kinect2_is_open() || remote2;
    if (!up1 && !up2) {
      if (!err[0]) set_err("no depth device");
      return -1;
    }
    if (req == KINECT_SRC_BOTH && (!up1 || !up2) && !kinect_net_active()) {
      set_err("need kinect and kinect 2");
      return -1;
    }
    if (up1 && up2)
      use = KINECT_SRC_BOTH;
    else if (up2)
      use = KINECT_SRC_V2;
    else
      use = KINECT_SRC_V1;
    pthread_mutex_lock(&mu);
    active = use;
    local_v1_usb = dev != NULL;
    stats.requested = req;
    stats.active = use;
    stats.v1_open = up1;
    stats.v2_open = up2;
    stats.running = 1;
    stats.skew_us = -1;
    stats.paired = 0;
    stats.identity_rigid = ksync_rigid_is_identity(&rigid);
    snprintf(stats.pipeline, sizeof stats.pipeline, "%s", up2 && !remote2 ? kinect2_pipeline() : "");
    pthread_mutex_unlock(&mu);
  }
  return 0;
}

static void close_all(void) {
  kinect2_close();
  close_v1();
  remote1 = remote2 = 0;
  free_rings();
  pthread_mutex_lock(&mu);
  local_v1_usb = 0;
  stats.running = 0;
  stats.v1_open = 0;
  stats.v2_open = 0;
  stats.paired = 0;
  pthread_mutex_unlock(&mu);
}

static void note_sources(void) {
  int up1 = (dev != NULL) || remote1;
  int up2 = kinect2_is_open() || remote2;
  kinect_src use;
  if (up1 && up2)
    use = KINECT_SRC_BOTH;
  else if (up2)
    use = KINECT_SRC_V2;
  else if (up1)
    use = KINECT_SRC_V1;
  else
    use = active;
  pthread_mutex_lock(&mu);
  active = use;
  local_v1_usb = dev != NULL;
  stats.active = use;
  stats.v1_open = up1;
  stats.v2_open = up2;
  pthread_mutex_unlock(&mu);
}

static void adopt_peer(void) {
  kinect_src req;
  int peer;
  if (!kinect_net_active()) return;
  peer = kinect_net_peer_sensors();
  if (!peer) return;
  pthread_mutex_lock(&mu);
  req = want_src;
  pthread_mutex_unlock(&mu);
  if (!remote1 && !dev && (peer & 1) && req != KINECT_SRC_V2) remote1 = 1;
  if (!remote2 && !kinect2_is_open() && (peer & 2) && req != KINECT_SRC_V1) remote2 = 1;
  note_sources();
}

/* One undistort feeds the local cloud and the wire. The net thread already
 * limits how often a new depth image is packetised. */
static int project_local_v2(cube_cloud *dst, const float *z_mm, uint64_t ts, const cube_cloud_tune *t,
                            int budget) {
  float fx, fy, cx, cy;
  int metric = 0;
  uint64_t now;
  if (!dst || !z_mm) return -1;
  if (wire_mm && kinect2_metric_depth(z_mm, wire_mm) == 0 &&
      kinect2_cloud_from_undistorted(t, dst, budget) == 0)
    metric = 1;
  else if (kinect2_cloud_from_depth(z_mm, t, dst, budget) != 0)
    return -1;
  ksync_tint_v2(dst);
  now = mono_us();
  if (now - last_offer2_us >= 45000ull) {
    last_offer2_us = now;
    if (kinect2_ir_intrinsics(&fx, &fy, &cx, &cy) == 0)
      kinect_net_offer_intrinsics(2, fx, fy, cx, cy);
    if (metric)
      kinect_net_offer_depth(2, wire_mm, KINECT2_W, KINECT2_H, 1, ts);
    else
      kinect_net_offer_depth(2, z_mm, KINECT2_W, KINECT2_H, 1, ts);
  }
  return 0;
}

static int build_remote_v2(cube_cloud *dst, const float *z_mm, const cube_cloud_tune *t, int budget) {
  float fx, fy, cx, cy;
  int i;
  if (!dst || !z_mm || !remote_m) return -1;
  if (kinect_net_take_intrinsics(2, &fx, &fy, &cx, &cy) != 1) return -1;
  for (i = 0; i < KINECT2_N; i++) remote_m[i] = z_mm[i] * 0.001f;
  if (ksync_project_depth(dst, remote_m, KINECT2_W, KINECT2_H, fx, fy, cx, cy, t, budget) != 0)
    return -1;
  ksync_tint_v2(dst);
  return 0;
}

static void remember(const cube_cloud *merged, const cube_cloud *a, const cube_cloud *b,
                     const uint16_t *depth, int skew, int paired) {
  pthread_mutex_lock(&mu);
  if (merged && merged->n > 0 && cube_cloud_copy(&snap, merged) == 0) {
    seq++;
    stats.seq = seq;
    stats.n = snap.n;
  }
  if (a && a->n > 0) cube_cloud_copy(&sens1, a);
  if (b && b->n > 0) cube_cloud_copy(&sens2, b);
  stats.n1 = a ? a->n : 0;
  stats.n2 = b ? b->n : 0;
  stats.n = snap.n;
  stats.skew_us = skew;
  stats.paired = paired;
  stats.active = active;
  stats.identity_rigid = ksync_rigid_is_identity(&rigid);
  if (depth && pub_depth) memcpy(pub_depth, depth, (size_t)V1_N * sizeof(uint16_t));
  pthread_mutex_unlock(&mu);
}

static void pump(void) {
  cube_cloud c1, c2, merged;
  cube_cloud_init(&c1);
  cube_cloud_init(&c2);
  cube_cloud_init(&merged);
  while (!stop_flag) {
    int fresh1 = 0, fresh2 = 0, dirty, i1, i2, paired = 0, skew = -1;
    int up1, up2, waited;
    cube_cloud_tune tcopy;
    ksync_rigid rig;
    int budget, guard;
    adopt_peer();
    if (dev) {
      struct timeval tv = {0, 15000};
      double tilt = 0;
      int do_tilt = 0;
      freenect_process_events_timeout(ctx, &tv);
      pthread_mutex_lock(&mu);
      if (have_pending) {
        tilt = pending_tilt;
        have_pending = 0;
        do_tilt = 1;
      }
      pthread_mutex_unlock(&mu);
      if (do_tilt) freenect_set_tilt_degs(dev, tilt);
      if (dev) {
        freenect_raw_tilt_state *st;
        freenect_update_tilt_state(dev);
        st = freenect_get_tilt_state(dev);
        if (st) {
          pthread_mutex_lock(&mu);
          last_tilt = freenect_get_tilt_degs(st);
          last_tilt_st = (int)freenect_get_tilt_status(st);
          pthread_mutex_unlock(&mu);
        }
      }
      pthread_mutex_lock(&cb_mu);
      if (cb_new) {
        push1();
        cb_new = 0;
        fresh1 = 1;
      }
      pthread_mutex_unlock(&cb_mu);
      if (fresh1) {
        int prev = (w1 + RING - 1) % RING;
        kinect_net_offer_depth(1, ring1[prev], V1_W, V1_H, 0, stamp1[prev]);
      }
    }
    if (remote1 && net_u16) {
      uint64_t ts = 0;
      uint32_t seqn = 0;
      int rw = 0, rh = 0;
      if (kinect_net_take_depth(1, ring1[w1], V1_N, &rw, &rh, &ts, &seqn) == 1 && rw == V1_W &&
          rh == V1_H) {
        stamp1[w1] = ts ? ts : mono_us();
        w1 = (w1 + 1) % RING;
        if (nring1 < RING) nring1++;
        fresh1 = 1;
      }
    }
    if (kinect2_is_open()) {
      guard = 0;
      while (guard < 3 && !stop_flag) {
        uint64_t ts = 0;
        int wait = (guard == 0 && !dev) ? 20 : 0;
        int rc = kinect2_take(ring2[w2], &ts, wait);
        if (rc != 1) break;
        stamp2[w2] = ts;
        w2 = (w2 + 1) % RING;
        if (nring2 < RING) nring2++;
        fresh2 = 1;
        guard++;
      }
    }
    if (remote2 && net_u16) {
      uint64_t ts = 0;
      uint32_t seqn = 0;
      int rw = 0, rh = 0;
      if (kinect_net_take_depth(2, net_u16, V1_N, &rw, &rh, &ts, &seqn) == 1 && rw == KINECT2_W &&
          rh == KINECT2_H) {
        int pi;
        float *dst = ring2[w2];
        for (pi = 0; pi < KINECT2_N; pi++) dst[pi] = net_u16[pi] ? (float)net_u16[pi] : 0.f;
        stamp2[w2] = ts ? ts : mono_us();
        w2 = (w2 + 1) % RING;
        if (nring2 < RING) nring2++;
        fresh2 = 1;
      }
    }
    pthread_mutex_lock(&mu);
    dirty = tune_dirty;
    tune_dirty = 0;
    tcopy = tune;
    rig = rigid;
    pthread_mutex_unlock(&mu);
    waited = (dev != NULL) || kinect2_is_open();
    if (!fresh1 && !fresh2 && !dirty) {
      if (!waited) usleep(4000);
      continue;
    }

    up1 = (dev != NULL) || remote1;
    up2 = kinect2_is_open() || remote2;
    i1 = latest_i(nring1, w1);
    i2 = latest_i(nring2, w2);
    if (up1 && up2 && nring1 > 0 && nring2 > 0) {
      int a = 0, b = 0;
      uint64_t sk = 0;
      if (ksync_best_pair(stamp1, nring1, stamp2, nring2, mono_us(), &a, &b, &sk) == 0) {
        i1 = a;
        i2 = b;
        skew = sk > 2000000000ull ? 2000000000 : (int)sk;
        paired = sk <= KSYNC_PAIR_US;
        if (sk > KSYNC_HOLD_US) {
          paired = 0;
          if (stamp1[i1] >= stamp2[i2])
            i2 = -1;
          else
            i1 = -1;
        }
      }
    }

    cube_cloud_clear(&c1);
    cube_cloud_clear(&c2);
    budget = (up1 && up2) ? CUBE_CLOUD_MAX / 2 : CUBE_CLOUD_MAX;
    if (up1 && i1 >= 0) {
      cube_cloud_tune tv = tcopy;
      tv.stride = ksync_cover_stride(V1_W, V1_H, tcopy.stride, budget);
      if (cube_cloud_from_depth_ex(&c1, ring1[i1], &tv) != 0) cube_cloud_clear(&c1);
    }
    if (up2 && i2 >= 0) {
      if (remote2) {
        if (build_remote_v2(&c2, ring2[i2], &tcopy, budget) != 0) cube_cloud_clear(&c2);
      } else if (project_local_v2(&c2, ring2[i2], stamp2[i2], &tcopy, budget) != 0)
        cube_cloud_clear(&c2);
    }
    cube_cloud_clear(&merged);
    if (c1.n > 0 && c2.n > 0)
      ksync_merge(&merged, &c1, &c2, &rig, CUBE_CLOUD_MAX);
    else if (c1.n > 0)
      cube_cloud_copy(&merged, &c1);
    else if (c2.n > 0)
      cube_cloud_copy(&merged, &c2);
    if (merged.n > 0)
      remember(&merged, &c1, &c2, (up1 && i1 >= 0) ? ring1[i1] : NULL, skew, paired && c1.n && c2.n);
  }
  cube_cloud_free(&c1);
  cube_cloud_free(&c2);
  cube_cloud_free(&merged);
}

static void *fuse_main(void *arg) {
  (void)arg;
  if (open_all() != 0) {
    close_all();
    pthread_mutex_lock(&mu);
    if (!stop_flag) state = 3;
    else state = 0;
    pthread_mutex_unlock(&mu);
    return NULL;
  }
  pthread_mutex_lock(&mu);
  state = 2;
  pthread_mutex_unlock(&mu);
  pump();
  close_all();
  pthread_mutex_lock(&mu);
  state = 0;
  pthread_mutex_unlock(&mu);
  return NULL;
}

int kinect_fuse_start(void) {
  int i;
  ensure_src();
  if (started) kinect_fuse_stop();
  stop_flag = 0;
  pthread_mutex_lock(&mu);
  state = 1;
  err[0] = 0;
  stats.err[0] = 0;
  stats.n = stats.n1 = stats.n2 = 0;
  stats.paired = 0;
  stats.skew_us = -1;
  pthread_mutex_unlock(&mu);
  if (pthread_create(&th, NULL, fuse_main, NULL) != 0) {
    pthread_mutex_lock(&mu);
    state = 0;
    snprintf(err, sizeof err, "fuse thread");
    pthread_mutex_unlock(&mu);
    return -1;
  }
  started = 1;
  for (i = 0; i < 800 && !stop_flag; i++) {
    int st;
    pthread_mutex_lock(&mu);
    st = state;
    pthread_mutex_unlock(&mu);
    if (st == 2) return 0;
    if (st == 3) {
      pthread_join(th, NULL);
      started = 0;
      pthread_mutex_lock(&mu);
      state = 0;
      pthread_mutex_unlock(&mu);
      return -1;
    }
    usleep(25000);
  }
  stop_flag = 1;
  pthread_join(th, NULL);
  started = 0;
  pthread_mutex_lock(&mu);
  state = 0;
  if (!err[0]) snprintf(err, sizeof err, "kinect open timeout");
  pthread_mutex_unlock(&mu);
  return -1;
}

void kinect_fuse_request_stop(void) { stop_flag = 1; }

void kinect_fuse_stop(void) {
  if (!started) return;
  stop_flag = 1;
  pthread_join(th, NULL);
  started = 0;
  stop_flag = 0;
  pthread_mutex_lock(&mu);
  state = 0;
  stats.running = 0;
  pthread_mutex_unlock(&mu);
}

int kinect_fuse_running(void) {
  int up;
  pthread_mutex_lock(&mu);
  up = state == 2;
  pthread_mutex_unlock(&mu);
  return up;
}

static void clamp_tune(cube_cloud_tune *t) {
  if (t->stride < 1) t->stride = 1;
  if (t->stride > 8) t->stride = 8;
  if (t->z_near < 0.3f) t->z_near = 0.3f;
  if (t->z_far < t->z_near + 0.15f) t->z_far = t->z_near + 0.15f;
  if (t->dot < 1) t->dot = 1;
  if (t->dot > 4) t->dot = 4;
  if (t->color < 0 || t->color > 3) t->color = 0;
}

void kinect_fuse_set_tune(const cube_cloud_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  tune = *t;
  clamp_tune(&tune);
  tune_ready = 1;
  tune_dirty = 1;
  pthread_mutex_unlock(&mu);
}

void kinect_fuse_get_tune(cube_cloud_tune *t) {
  if (!t) return;
  pthread_mutex_lock(&mu);
  if (!tune_ready) {
    cube_cloud_tune_default(&tune);
    tune.color = 3;
    tune_ready = 1;
  }
  *t = tune;
  pthread_mutex_unlock(&mu);
}

int kinect_fuse_set_tilt(double degrees) {
  if (degrees < -30) degrees = -30;
  if (degrees > 30) degrees = 30;
  pthread_mutex_lock(&mu);
  pending_tilt = degrees;
  have_pending = 1;
  pthread_mutex_unlock(&mu);
  return 0;
}

int kinect_fuse_get_tilt(double *degrees, int *status) {
  pthread_mutex_lock(&mu);
  if (degrees) *degrees = last_tilt;
  if (status) *status = last_tilt_st;
  pthread_mutex_unlock(&mu);
  return 0;
}

int kinect_fuse_snapshot(cube_cloud *out) {
  int rc;
  if (!out) return -1;
  pthread_mutex_lock(&mu);
  if (snap.n < 1) {
    pthread_mutex_unlock(&mu);
    return -1;
  }
  rc = cube_cloud_copy(out, &snap);
  pthread_mutex_unlock(&mu);
  return rc;
}

int kinect_fuse_copy_v1_depth(uint16_t *depth) {
  if (!depth || !pub_depth) return -1;
  pthread_mutex_lock(&mu);
  if (!stats.v1_open && active != KINECT_SRC_V1 && active != KINECT_SRC_BOTH) {
    pthread_mutex_unlock(&mu);
    return -1;
  }
  memcpy(depth, pub_depth, (size_t)V1_N * sizeof(uint16_t));
  pthread_mutex_unlock(&mu);
  return 0;
}

void kinect_fuse_get_stats(kinect_fuse_stats *st) {
  if (!st) return;
  pthread_mutex_lock(&mu);
  stats.requested = want_src;
  stats.running = state == 2;
  stats.identity_rigid = ksync_rigid_is_identity(&rigid);
  *st = stats;
  pthread_mutex_unlock(&mu);
}

const char *kinect_fuse_error(void) { return err; }

int kinect_fuse_align(ksync_align_result *res) {
  cube_cloud a, b;
  ksync_rigid rig;
  char path[512];
  int rc;
  if (res) memset(res, 0, sizeof *res);
  cube_cloud_init(&a);
  cube_cloud_init(&b);
  pthread_mutex_lock(&mu);
  if (sens1.n < 30 || sens2.n < 30) {
    pthread_mutex_unlock(&mu);
    set_err("need both clouds to align");
    cube_cloud_free(&a);
    cube_cloud_free(&b);
    return -1;
  }
  cube_cloud_copy(&a, &sens1);
  cube_cloud_copy(&b, &sens2);
  pthread_mutex_unlock(&mu);
  rc = ksync_align(&rig, res, &a, &b);
  cube_cloud_free(&a);
  cube_cloud_free(&b);
  if (rc != 0) {
    char why[160];
    if (res && (res->inliers > 0 || res->residual_m > 0.f))
      snprintf(why, sizeof why, "align found no overlap (inl %d res %.3f yaw %.0f)", res->inliers,
               res->residual_m, res->yaw_deg);
    else
      snprintf(why, sizeof why, "align found no overlap");
    set_err(why);
    return -1;
  }
  pthread_mutex_lock(&mu);
  rigid = rig;
  tune_dirty = 1;
  stats.identity_rigid = 0;
  pthread_mutex_unlock(&mu);
  if (ksync_extrinsic_path(path, sizeof path) == 0) ksync_rigid_save(&rigid, path);
  return 0;
}

void kinect_fuse_reset_rigid(void) {
  char path[512];
  pthread_mutex_lock(&mu);
  ksync_rigid_identity(&rigid);
  tune_dirty = 1;
  stats.identity_rigid = 1;
  pthread_mutex_unlock(&mu);
  if (ksync_extrinsic_path(path, sizeof path) == 0) ksync_rigid_save(&rigid, path);
}

void kinect_fuse_get_rigid(ksync_rigid *r) {
  if (!r) return;
  pthread_mutex_lock(&mu);
  *r = rigid;
  pthread_mutex_unlock(&mu);
}

int kinect_fuse_local_v1(void) {
  int v;
  pthread_mutex_lock(&mu);
  v = local_v1_usb;
  pthread_mutex_unlock(&mu);
  return v;
}

int kinect_fuse_snapshot_parts(cube_cloud *merged, cube_cloud *v1, cube_cloud *v2) {
  int rc = -1;
  pthread_mutex_lock(&mu);
  if (merged) {
    if (snap.n > 0)
      rc = cube_cloud_copy(merged, &snap);
    else
      rc = -1;
  } else {
    rc = (snap.n > 0 || sens1.n > 0 || sens2.n > 0) ? 0 : -1;
  }
  if (v1) {
    if (sens1.n > 0)
      cube_cloud_copy(v1, &sens1);
    else
      cube_cloud_clear(v1);
  }
  if (v2) {
    if (sens2.n > 0)
      cube_cloud_copy(v2, &sens2);
    else
      cube_cloud_clear(v2);
  }
  pthread_mutex_unlock(&mu);
  return rc;
}
