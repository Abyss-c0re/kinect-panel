#define _POSIX_C_SOURCE 200809L
#include "kinect_xr.h"
#include "cube_kinect_mesh.h"
#include "kinect_fuse.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEP_W 640
#define DEP_H 480
#define DEP_N (DEP_W * DEP_H)

struct kxr_session {
  pthread_mutex_t mu;
  cube_cloud kinect;
  cube_cloud_tune tune;
  kxr_rig rig;
  kxr_view views[KXR_MAX_VIEWS];
  int nviews;
  kxr_pose xr_head;
  int have_head;
  kxr_head blob;
  int have_blob;
  float body_h;
  float body_score;
  float floor_y;
  kxr_vec3 samp_k[KXR_MAX_SAMP];
  kxr_vec3 samp_x[KXR_MAX_SAMP];
  int nsamp;
  float hide_radius;
  float scale;
  int have_xr_floor;
  float xr_floor_y;
  char err[160];

  int live;
  int live_stop;
  int live_failed;
  pthread_t live_th;
  double tilt_deg;
  cube_mesh mesh;

  /* Sensor reopen and v2 align run off the app thread.
   * kxr_live_stop must not join them: they call back into live_start. */
  int reopen_busy;
  int reopen_alive;
  int reopen_stop;
  unsigned reopen_gen;
  pthread_t reopen_th;
  int align_busy;
  int align_alive;
  int align_stop;
  pthread_t align_th;
  char note[160];
  unsigned note_seq;
};

static void set_err(kxr_session *s, const char *m) {
  if (!s) return;
  snprintf(s->err, sizeof s->err, "%s", m ? m : "");
}

static void note_set(kxr_session *s, const char *m) {
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  snprintf(s->note, sizeof s->note, "%s", m ? m : "");
  if (s->note[0]) s->note_seq++;
  pthread_mutex_unlock(&s->mu);
}

static const char *sensor_label(kinect_src s) {
  switch (s) {
  case KINECT_SRC_V1:
    return "KINECT";
  case KINECT_SRC_V2:
    return "KINECT 2";
  case KINECT_SRC_BOTH:
    return "BOTH";
  default:
    return "AUTO";
  }
}

static void join_side_threads(kxr_session *s) {
  int reopen_alive, align_alive;
  pthread_mutex_lock(&s->mu);
  s->reopen_stop = 1;
  s->align_stop = 1;
  reopen_alive = s->reopen_alive;
  align_alive = s->align_alive;
  pthread_mutex_unlock(&s->mu);
  if (reopen_alive) {
    pthread_join(s->reopen_th, NULL);
    s->reopen_alive = 0;
    s->reopen_busy = 0;
  }
  if (align_alive) {
    pthread_join(s->align_th, NULL);
    s->align_alive = 0;
    s->align_busy = 0;
  }
}

static int track_head_unlocked(kxr_session *s, kxr_head *out);

static float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

void kxr_pose_identity(kxr_pose *p) {
  if (!p) return;
  p->p.x = p->p.y = p->p.z = 0;
  p->q.x = p->q.y = p->q.z = 0;
  p->q.w = 1;
}

void kxr_map_kinect(kxr_vec3 k, kxr_vec3 *x) {
  if (!x) return;
  x->x = k.x;
  x->y = k.y;
  x->z = -k.z;
}

void kxr_unmap_kinect(kxr_vec3 x, kxr_vec3 *k) {
  if (!k) return;
  k->x = x.x;
  k->y = x.y;
  k->z = -x.z;
}

static kxr_quat quat_norm(kxr_quat q) {
  float n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-8f) {
    q.x = q.y = q.z = 0;
    q.w = 1;
    return q;
  }
  q.x /= n;
  q.y /= n;
  q.z /= n;
  q.w /= n;
  return q;
}

static kxr_quat quat_mul(kxr_quat a, kxr_quat b) {
  kxr_quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return quat_norm(r);
}

static kxr_quat quat_axis_deg(float ax, float ay, float az, float deg) {
  float a = deg * (float)M_PI / 180.f;
  float s = sinf(a * 0.5f), c = cosf(a * 0.5f);
  float n = sqrtf(ax * ax + ay * ay + az * az);
  kxr_quat q;
  if (n < 1e-8f) {
    q.x = q.y = q.z = 0;
    q.w = 1;
    return q;
  }
  ax /= n;
  ay /= n;
  az /= n;
  q.x = ax * s;
  q.y = ay * s;
  q.z = az * s;
  q.w = c;
  return q;
}

static kxr_vec3 quat_rot(kxr_quat q, kxr_vec3 v) {
  kxr_vec3 t, u, o;
  q = quat_norm(q);
  t.x = 2.f * (q.y * v.z - q.z * v.y);
  t.y = 2.f * (q.z * v.x - q.x * v.z);
  t.z = 2.f * (q.x * v.y - q.y * v.x);
  u.x = q.y * t.z - q.z * t.y;
  u.y = q.z * t.x - q.x * t.z;
  u.z = q.x * t.y - q.y * t.x;
  o.x = v.x + q.w * t.x + u.x;
  o.y = v.y + q.w * t.y + u.y;
  o.z = v.z + q.w * t.z + u.z;
  return o;
}

static kxr_vec3 vadd(kxr_vec3 a, kxr_vec3 b) {
  kxr_vec3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
  return r;
}

static kxr_vec3 vsub(kxr_vec3 a, kxr_vec3 b) {
  kxr_vec3 r = {a.x - b.x, a.y - b.y, a.z - b.z};
  return r;
}

static float vdot(kxr_vec3 a, kxr_vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float vlen(kxr_vec3 a) { return sqrtf(vdot(a, a)); }

static kxr_quat quat_conj(kxr_quat q) {
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return quat_norm(q);
}

static kxr_vec3 xform_k_to_x(const kxr_rig *r, kxr_vec3 k, float scale) {
  kxr_vec3 m;
  kxr_map_kinect(k, &m);
  if (scale != 1.f) {
    m.x *= scale;
    m.y *= scale;
    m.z *= scale;
  }
  return vadd(quat_rot(r->kinect_in_xr.q, m), r->kinect_in_xr.p);
}

/* Inverse of xform_k_to_x — XR head back into Kinect camera metres. */
static kxr_vec3 xform_x_to_k(const kxr_rig *r, kxr_vec3 x, float scale) {
  kxr_vec3 m, k;
  m = quat_rot(quat_conj(r->kinect_in_xr.q), vsub(x, r->kinect_in_xr.p));
  if (scale > 1e-4f && scale != 1.f) {
    m.x /= scale;
    m.y /= scale;
    m.z /= scale;
  }
  kxr_unmap_kinect(m, &k);
  return k;
}

void kxr_rig_identity(kxr_rig *r) {
  if (!r) return;
  memset(r, 0, sizeof *r);
  kxr_pose_identity(&r->kinect_in_xr);
}

void kxr_rig_place(kxr_rig *r, kxr_vec3 pos, float yaw_deg, float pitch_deg) {
  kxr_quat qy, qx;
  if (!r) return;
  kxr_rig_identity(r);
  r->kinect_in_xr.p = pos;
  /* pitch first (optical tilt), then yaw in the room */
  qx = quat_axis_deg(1, 0, 0, pitch_deg);
  qy = quat_axis_deg(0, 1, 0, yaw_deg);
  r->kinect_in_xr.q = quat_mul(qy, qx);
  r->locked = 1;
}

kxr_session *kxr_create(void) {
  kxr_session *s = (kxr_session *)calloc(1, sizeof *s);
  if (!s) return NULL;
  pthread_mutex_init(&s->mu, NULL);
  cube_cloud_init(&s->kinect);
  cube_mesh_init(&s->mesh);
  cube_cloud_tune_default(&s->tune);
  kxr_rig_identity(&s->rig);
  kxr_pose_identity(&s->xr_head);
  s->hide_radius = 0.18f;
  s->scale = 1.f;
  return s;
}

void kxr_destroy(kxr_session *s) {
  if (!s) return;
  /* Join the reopen worker before live_stop. The worker calls live_start,
   * and live_start calls live_stop. Joining it from live_stop deadlocks. */
  join_side_threads(s);
  kxr_live_stop(s);
  cube_cloud_free(&s->kinect);
  cube_mesh_free(&s->mesh);
  pthread_mutex_destroy(&s->mu);
  free(s);
}

void kxr_set_tune(kxr_session *s, const cube_cloud_tune *t) {
  int i;
  cube_cloud_tune local;
  if (!s || !t) return;
  pthread_mutex_lock(&s->mu);
  s->tune = *t;
  if (s->tune.stride < 1) s->tune.stride = 1;
  if (s->tune.stride > 10) s->tune.stride = 10;
  for (i = 0; i < s->kinect.n; i++)
    cube_cloud_color_point(&s->kinect.pt[i], &s->tune);
  local = s->tune;
  pthread_mutex_unlock(&s->mu);
  kinect_fuse_set_tune(&local);
}

/* 1 when a worker owns the status line (switching or a restart error). */
static int schedule_reopen(kxr_session *s);

void kxr_set_source(kxr_session *s, const char *name) {
  kinect_src src = KINECT_SRC_AUTO;
  char msg[64];
  if (!s) return;
  if (name && name[0] && !kinect_src_known(name, &src)) {
    set_err(s, "unknown kinect source");
    note_set(s, "unknown kinect source");
    return;
  }
  kinect_fuse_set_source(src);
  if (schedule_reopen(s)) return;
  snprintf(msg, sizeof msg, "sensor %s", sensor_label(src));
  note_set(s, msg);
}

int kxr_source_switching(const kxr_session *s) {
  int b;
  if (!s) return 0;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  b = s->reopen_busy;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return b;
}

int kxr_poll_note(kxr_session *s, unsigned *seen, char *buf, size_t n) {
  unsigned seq;
  if (!s || !seen || !buf || n == 0) return 0;
  pthread_mutex_lock(&s->mu);
  seq = s->note_seq;
  if (seq == *seen || !s->note[0]) {
    pthread_mutex_unlock(&s->mu);
    return 0;
  }
  snprintf(buf, n, "%s", s->note);
  pthread_mutex_unlock(&s->mu);
  *seen = seq;
  return 1;
}

void kxr_get_tune(kxr_session *s, cube_cloud_tune *t) {
  if (!s || !t) return;
  pthread_mutex_lock(&s->mu);
  *t = s->tune;
  pthread_mutex_unlock(&s->mu);
}

void kxr_set_hide_radius(kxr_session *s, float metres) {
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  s->hide_radius = metres < 0.f ? 0.f : metres;
  pthread_mutex_unlock(&s->mu);
}

float kxr_get_hide_radius(const kxr_session *s) {
  float r;
  if (!s) return 0.f;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  r = s->hide_radius;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return r;
}

static void relock_t_unlocked(kxr_session *s) {
  kxr_vec3 mapped, pred0;
  if (!s->have_head || !s->blob.ok) return;
  kxr_map_kinect(s->blob.p_kinect, &mapped);
  mapped.x *= s->scale;
  mapped.y *= s->scale;
  mapped.z *= s->scale;
  pred0 = quat_rot(s->rig.kinect_in_xr.q, mapped);
  s->rig.kinect_in_xr.p = vsub(s->xr_head.p, pred0);
  s->rig.locked = 1;
}

void kxr_set_scale(kxr_session *s, float scale) {
  if (!s) return;
  if (scale < 0.25f) scale = 0.25f;
  if (scale > 4.f) scale = 4.f;
  pthread_mutex_lock(&s->mu);
  s->scale = scale;
  if (s->rig.locked) relock_t_unlocked(s);
  pthread_mutex_unlock(&s->mu);
}

void kxr_set_rig_scale(kxr_session *s, const kxr_rig *r, float scale) {
  if (!s || !r) return;
  if (scale < 0.25f) scale = 0.25f;
  if (scale > 4.f) scale = 4.f;
  pthread_mutex_lock(&s->mu);
  s->rig = *r;
  s->scale = scale;
  pthread_mutex_unlock(&s->mu);
}

void kxr_orbit_about(kxr_session *s, kxr_vec3 pivot, float yaw_deg, float pitch_deg) {
  kxr_quat qy, qx, dq;
  kxr_vec3 t, d;
  if (!s) return;
  if (fabsf(yaw_deg) < 1e-5f && fabsf(pitch_deg) < 1e-5f) return;
  pthread_mutex_lock(&s->mu);
  qx = quat_axis_deg(1, 0, 0, pitch_deg);
  qy = quat_axis_deg(0, 1, 0, yaw_deg);
  dq = quat_mul(qy, qx);
  t = s->rig.kinect_in_xr.p;
  d = vsub(t, pivot);
  s->rig.kinect_in_xr.p = vadd(quat_rot(dq, d), pivot);
  s->rig.kinect_in_xr.q = quat_mul(dq, s->rig.kinect_in_xr.q);
  pthread_mutex_unlock(&s->mu);
}

void kxr_scale_about(kxr_session *s, kxr_vec3 pivot, float factor) {
  kxr_vec3 t, d;
  float sc;
  if (!s || factor <= 0.f) return;
  pthread_mutex_lock(&s->mu);
  sc = s->scale * factor;
  if (sc < 0.25f) {
    factor = 0.25f / s->scale;
    sc = 0.25f;
  } else if (sc > 4.f) {
    factor = 4.f / s->scale;
    sc = 4.f;
  }
  t = s->rig.kinect_in_xr.p;
  d = vsub(t, pivot);
  s->rig.kinect_in_xr.p.x = pivot.x + factor * d.x;
  s->rig.kinect_in_xr.p.y = pivot.y + factor * d.y;
  s->rig.kinect_in_xr.p.z = pivot.z + factor * d.z;
  s->scale = sc;
  pthread_mutex_unlock(&s->mu);
}

void kxr_translate(kxr_session *s, kxr_vec3 delta) {
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  s->rig.kinect_in_xr.p = vadd(s->rig.kinect_in_xr.p, delta);
  pthread_mutex_unlock(&s->mu);
}

float kxr_get_scale(const kxr_session *s) {
  float sc;
  if (!s) return 1.f;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  sc = s->scale;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return sc;
}

int kxr_autoscale_height(kxr_session *s) {
  kxr_head h;
  int i, n = 0;
  float ylo = 1e9f, hk, hx, sc;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  if (!s->have_head) {
    set_err(s, "no OpenXR head");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  if (track_head_unlocked(s, &h) != 0) {
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  for (i = 0; i < s->kinect.n; i++) {
    float y = s->kinect.pt[i].y, z = s->kinect.pt[i].z;
    if (z < 0.8f || z > 3.5f) continue;
    if (y < ylo) ylo = y;
    n++;
  }
  hk = h.p_kinect.y - ylo;
  hx = s->have_xr_floor ? (s->xr_head.p.y - s->xr_floor_y) : s->xr_head.p.y;
  /* STAGE eye height is ~1.4–1.9 m. LOCAL y≈0 — skip unless floor was calibrated. */
  if (n < 20 || hk < 0.7f || hx < 0.8f) {
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  sc = hx / hk;
  if (sc < 0.5f) sc = 0.5f;
  if (sc > 2.0f) sc = 2.0f;
  s->scale = sc;
  relock_t_unlocked(s);
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return 0;
}

int kxr_calibrate_floor(kxr_session *s, float xr_floor_y) {
  kxr_head h;
  float hk, hx, sc, ylo;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  s->have_xr_floor = 1;
  s->xr_floor_y = xr_floor_y;
  if (!s->have_head) {
    set_err(s, "no OpenXR head");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  if (track_head_unlocked(s, &h) != 0) {
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  ylo = s->floor_y;
  hk = h.p_kinect.y - ylo;
  if (hk < 0.55f && s->body_h > hk) hk = s->body_h;
  hx = s->xr_head.p.y - xr_floor_y;
  if (hk < 0.55f || hx < 0.55f || hx > 2.45f) {
    set_err(s, "floor/head height implausible");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  sc = hx / hk;
  if (sc < 0.5f) sc = 0.5f;
  if (sc > 2.0f) sc = 2.0f;
  s->scale = sc;
  relock_t_unlocked(s);
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return 0;
}

void kxr_clear_xr_floor(kxr_session *s) {
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  s->have_xr_floor = 0;
  s->xr_floor_y = 0.f;
  pthread_mutex_unlock(&s->mu);
}

int kxr_have_xr_floor(const kxr_session *s) {
  int v;
  if (!s) return 0;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  v = s->have_xr_floor;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return v;
}

float kxr_get_xr_floor(const kxr_session *s) {
  float y;
  if (!s) return 0.f;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  y = s->xr_floor_y;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return y;
}

float kxr_get_floor_k(const kxr_session *s) {
  float y;
  if (!s) return 0.f;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  y = s->floor_y;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return y;
}

float kxr_get_body_h(const kxr_session *s) {
  float h;
  if (!s) return 0.f;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  h = s->body_h;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return h;
}

const char *kxr_error(const kxr_session *s) { return s ? s->err : "null"; }

int kxr_set_cloud(kxr_session *s, const cube_cloud *c) {
  int rc;
  if (!s || !c) return -1;
  pthread_mutex_lock(&s->mu);
  rc = cube_cloud_copy(&s->kinect, c);
  if (rc != 0) set_err(s, s->kinect.err[0] ? s->kinect.err : "copy");
  else s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return rc;
}

int kxr_push_depth(kxr_session *s, const uint16_t *depth, uint32_t ts) {
  cube_cloud_tune t;
  int rc;
  if (!s || !depth) return -1;
  pthread_mutex_lock(&s->mu);
  t = s->tune;
  rc = cube_cloud_from_depth_ex(&s->kinect, depth, &t);
  s->kinect.ts = ts;
  if (rc != 0) set_err(s, s->kinect.err[0] ? s->kinect.err : "depth");
  else s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return rc;
}

int kxr_load_xyz(kxr_session *s, const char *path) {
  int rc;
  if (!s || !path) return -1;
  pthread_mutex_lock(&s->mu);
  rc = cube_cloud_load_xyz(&s->kinect, path, CUBE_CLOUD_MAX);
  if (rc != 0) set_err(s, s->kinect.err[0] ? s->kinect.err : "xyz");
  else s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return rc;
}

int kxr_get_cloud(kxr_session *s, cube_cloud *out) {
  int rc;
  if (!s || !out) return -1;
  pthread_mutex_lock(&s->mu);
  rc = cube_cloud_copy(out, &s->kinect);
  pthread_mutex_unlock(&s->mu);
  return rc;
}

int kxr_points(const kxr_session *s) {
  int n;
  if (!s) return 0;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  n = s->kinect.n;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return n;
}

void kxr_set_rig(kxr_session *s, const kxr_rig *r) {
  if (!s || !r) return;
  pthread_mutex_lock(&s->mu);
  s->rig = *r;
  pthread_mutex_unlock(&s->mu);
}

void kxr_get_rig(const kxr_session *s, kxr_rig *r) {
  if (!s || !r) return;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  *r = s->rig;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
}

void kxr_set_head(kxr_session *s, const kxr_pose *xr_head) {
  if (!s || !xr_head) return;
  pthread_mutex_lock(&s->mu);
  s->xr_head = *xr_head;
  s->xr_head.q = quat_norm(s->xr_head.q);
  s->have_head = 1;
  pthread_mutex_unlock(&s->mu);
}

void kxr_get_head(const kxr_session *s, kxr_pose *xr_head) {
  if (!s || !xr_head) return;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  *xr_head = s->xr_head;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
}

void kxr_set_views(kxr_session *s, const kxr_view *views, int n) {
  int i;
  if (!s || !views) return;
  if (n < 0) n = 0;
  if (n > KXR_MAX_VIEWS) n = KXR_MAX_VIEWS;
  pthread_mutex_lock(&s->mu);
  s->nviews = n;
  for (i = 0; i < n; i++) {
    s->views[i] = views[i];
    s->views[i].pose.q = quat_norm(s->views[i].pose.q);
  }
  pthread_mutex_unlock(&s->mu);
}

int kxr_nviews(const kxr_session *s) {
  int n;
  if (!s) return 0;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  n = s->nviews;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return n;
}

#define KXR_GX 48
#define KXR_GZ 48
#define KXR_GMAX (KXR_GX * KXR_GZ)
#define KXR_CELL 0.16f
#define KXR_MAX_BODY 24
#define KXR_YBINS 80
#define KXR_Y0 (-2.0f)
#define KXR_YSTEP 0.05f

typedef struct {
  int n, nhead, nbody;
  float sx, sy, sz;
  float hx, hy, hz;
  float ymin, ymax;
  float xmin, xmax, zmin, zmax;
} kxr_clump;

static int gfind(int *p, int a) {
  while (p[a] != a) {
    p[a] = p[p[a]];
    a = p[a];
  }
  return a;
}

static float body_score(const kxr_clump *b, float floor_y, float xr_eye_y) {
  float h, w, d, aspect, s = 0.f;
  if (!b || b->n < 10) return -1.f;
  h = b->ymax - floor_y;
  w = b->xmax - b->xmin;
  d = b->zmax - b->zmin;
  if (d > w) w = d;
  if (w < 0.05f) w = 0.05f;
  aspect = h / w;

  if (h >= 1.15f && h <= 2.15f)
    s += 0.35f;
  else if (h >= 0.78f && h < 1.15f)
    s += 0.22f; /* seated / cropped legs */
  else if (h >= 0.55f && h < 0.78f)
    s += 0.08f;
  else
    s -= 0.25f;

  if (w >= 0.18f && w <= 0.90f)
    s += 0.20f;
  else if (w < 0.14f)
    s -= 0.15f;
  else if (w > 1.15f)
    s -= 0.30f; /* wall / sofa / curtain */

  if (aspect >= 1.25f && aspect <= 6.5f)
    s += 0.15f;

  if (b->nhead >= 8)
    s += 0.15f;
  else if (b->nhead < 5)
    s -= 0.20f;

  if (b->nbody >= 16)
    s += 0.15f;
  else
    s -= 0.30f; /* lamp / shelf: crown, no torso */

  {
    float hz = b->nhead > 0 ? b->hz / (float)b->nhead : 0.5f * (b->zmin + b->zmax);
    if (hz >= 0.85f && hz <= 3.30f) s += 0.10f;
  }

  if (xr_eye_y > 0.80f && h > 0.70f) {
    float ratio = h / xr_eye_y;
    if (ratio > 0.72f && ratio < 1.40f) s += 0.10f;
  }

  if (b->n > 1800 && w > 0.90f) s -= 0.20f;
  return s;
}

static int track_head_unlocked(kxr_session *s, kxr_head *out) {
  const cube_cloud *c = &s->kinect;
  int i, ncell = 0, nclu = 0, best = -1, fallback = -1;
  int yhist[KXR_YBINS];
  int occ_n[KXR_GMAX];
  int parent[KXR_GMAX];
  int label_of[KXR_GMAX];
  kxr_clump cl[KXR_MAX_BODY];
  float floor_y, xr_eye = 0.f;
  int have_prior = 0;
  kxr_vec3 prior_k = {0, 0, 0};
  float best_s = -1e9f, fall_s = -1e9f;

  memset(out, 0, sizeof *out);
  if (c->n < 20) {
    set_err(s, "too few points to track");
    return -1;
  }

  memset(yhist, 0, sizeof yhist);
  memset(occ_n, 0, sizeof occ_n);
  for (i = 0; i < KXR_GMAX; i++) {
    parent[i] = i;
    label_of[i] = -1;
  }

  for (i = 0; i < c->n; i++) {
    const cube_cloud_pt *p = &c->pt[i];
    int yb;
    if (p->z < 0.45f || p->z > 4.20f) continue;
    yb = (int)((p->y - KXR_Y0) / KXR_YSTEP);
    if (yb >= 0 && yb < KXR_YBINS) yhist[yb]++;
  }
  {
    int need = c->n / 10;
    int cum = 0;
    if (need < 8) need = 8;
    floor_y = -0.90f;
    for (i = 0; i < KXR_YBINS; i++) {
      cum += yhist[i];
      if (cum >= need) {
        floor_y = KXR_Y0 + ((float)i + 0.5f) * KXR_YSTEP;
        break;
      }
    }
  }

  /* Drop the floor plane so a person does not weld to furniture via the rug. */
  for (i = 0; i < c->n; i++) {
    const cube_cloud_pt *p = &c->pt[i];
    int ix, iz, id;
    if (p->z < 0.45f || p->z > 4.20f) continue;
    if (p->y < floor_y + 0.18f) continue;
    ix = (int)floorf(p->x / KXR_CELL) + KXR_GX / 2;
    iz = (int)floorf(p->z / KXR_CELL);
    if (ix < 0 || ix >= KXR_GX || iz < 0 || iz >= KXR_GZ) continue;
    id = iz * KXR_GX + ix;
    if (occ_n[id] == 0) ncell++;
    occ_n[id]++;
  }
  if (ncell < 1) {
    set_err(s, "too few body-range points");
    return -1;
  }

  /* 4-neighbour merge of occupied xz cells. */
  for (i = 0; i < KXR_GMAX; i++) {
    int iz, ix, r, u;
    if (occ_n[i] < 1) continue;
    iz = i / KXR_GX;
    ix = i % KXR_GX;
    if (ix + 1 < KXR_GX) {
      r = i + 1;
      if (occ_n[r] > 0) {
        int a = gfind(parent, i), b = gfind(parent, r);
        if (a != b) parent[b] = a;
      }
    }
    if (iz + 1 < KXR_GZ) {
      u = i + KXR_GX;
      if (occ_n[u] > 0) {
        int a = gfind(parent, i), b = gfind(parent, u);
        if (a != b) parent[b] = a;
      }
    }
  }

  memset(cl, 0, sizeof cl);
  for (i = 0; i < KXR_GMAX; i++) {
    int root, lab;
    if (occ_n[i] < 1) continue;
    root = gfind(parent, i);
    if (label_of[root] < 0) {
      if (nclu >= KXR_MAX_BODY) continue;
      label_of[root] = nclu;
      cl[nclu].ymin = 1e9f;
      cl[nclu].ymax = -1e9f;
      cl[nclu].xmin = 1e9f;
      cl[nclu].xmax = -1e9f;
      cl[nclu].zmin = 1e9f;
      cl[nclu].zmax = -1e9f;
      nclu++;
    }
    lab = label_of[root];
    label_of[i] = lab;
    (void)lab;
  }
  if (nclu < 1) {
    set_err(s, "no depth clusters");
    return -1;
  }

  for (i = 0; i < c->n; i++) {
    const cube_cloud_pt *p = &c->pt[i];
    int ix, iz, id, lab;
    kxr_clump *b;
    if (p->z < 0.45f || p->z > 4.20f) continue;
    if (p->y < floor_y + 0.18f) continue;
    ix = (int)floorf(p->x / KXR_CELL) + KXR_GX / 2;
    iz = (int)floorf(p->z / KXR_CELL);
    if (ix < 0 || ix >= KXR_GX || iz < 0 || iz >= KXR_GZ) continue;
    id = iz * KXR_GX + ix;
    lab = label_of[id];
    if (lab < 0 || lab >= nclu) continue;
    b = &cl[lab];
    b->n++;
    b->sx += p->x;
    b->sy += p->y;
    b->sz += p->z;
    if (p->y < b->ymin) b->ymin = p->y;
    if (p->y > b->ymax) b->ymax = p->y;
    if (p->x < b->xmin) b->xmin = p->x;
    if (p->x > b->xmax) b->xmax = p->x;
    if (p->z < b->zmin) b->zmin = p->z;
    if (p->z > b->zmax) b->zmax = p->z;
  }

  /* Head = top 28 cm of the column; torso = between knees and neck. */
  for (i = 0; i < c->n; i++) {
    const cube_cloud_pt *p = &c->pt[i];
    int ix, iz, id, lab;
    kxr_clump *b;
    float neck, hip;
    if (p->z < 0.45f || p->z > 4.20f) continue;
    ix = (int)floorf(p->x / KXR_CELL) + KXR_GX / 2;
    iz = (int)floorf(p->z / KXR_CELL);
    if (ix < 0 || ix >= KXR_GX || iz < 0 || iz >= KXR_GZ) continue;
    id = iz * KXR_GX + ix;
    lab = label_of[id];
    if (lab < 0 || lab >= nclu) continue;
    b = &cl[lab];
    neck = b->ymax - 0.28f;
    hip = floor_y + 0.35f;
    if (p->y >= neck) {
      b->nhead++;
      b->hx += p->x;
      b->hy += p->y;
      b->hz += p->z;
    } else if (p->y >= hip && p->y < b->ymax - 0.20f) {
      b->nbody++;
    }
  }

  if (s->have_head) {
    xr_eye = s->xr_head.p.y;
    if (s->have_xr_floor) xr_eye -= s->xr_floor_y;
  }
  if (s->rig.locked && s->have_head) {
    prior_k = xform_x_to_k(&s->rig, s->xr_head.p, s->scale);
    have_prior = 1;
  } else if (s->have_blob) {
    prior_k = s->blob.p_kinect;
    have_prior = 1;
  }

  for (i = 0; i < nclu; i++) {
    kxr_clump *b = &cl[i];
    float sc, hx, hy, hz, dx, dy, dz;
    if (b->n < 10 || b->nhead < 4) continue;
    sc = body_score(b, floor_y, xr_eye);
    hx = b->hx / (float)b->nhead;
    hy = b->hy / (float)b->nhead;
    hz = b->hz / (float)b->nhead;
    if (have_prior) {
      dx = hx - prior_k.x;
      dy = hy - prior_k.y;
      dz = hz - prior_k.z;
      sc += 0.28f * expf(-(dx * dx + dy * dy + dz * dz) / 0.22f);
    }
    if (sc > fall_s) {
      fall_s = sc;
      fallback = i;
    }
    /* After a lock, ignore furniture that does not look like a body
     * unless it is the only cluster near the HMD prior. */
    if (sc > best_s) {
      best_s = sc;
      best = i;
    }
  }
  if (best < 0) best = fallback;
  if (best < 0) {
    set_err(s, "no human-shaped cluster");
    return -1;
  }

  {
    kxr_clump *b = &cl[best];
    kxr_vec3 raw;
    float alpha;
    raw.x = b->hx / (float)b->nhead;
    raw.y = b->hy / (float)b->nhead;
    raw.z = b->hz / (float)b->nhead;
    /* After lock, stick to the same person (no swim). Raw heads
     * are required for walk samples / Umeyama. */
    if (s->have_blob && s->rig.locked) {
      kxr_vec3 d = vsub(raw, s->blob.p_kinect);
      if (vdot(d, d) < 0.45f * 0.45f) {
        alpha = 0.38f;
        raw.x = alpha * raw.x + (1.f - alpha) * s->blob.p_kinect.x;
        raw.y = alpha * raw.y + (1.f - alpha) * s->blob.p_kinect.y;
        raw.z = alpha * raw.z + (1.f - alpha) * s->blob.p_kinect.z;
      }
    }
    out->ok = 1;
    out->n = b->nhead;
    out->p_kinect = raw;
    out->conf = clampf(best_s, 0.f, 1.f);
    s->blob = *out;
    s->have_blob = 1;
    s->body_h = b->ymax - floor_y;
    s->body_score = best_s;
    s->floor_y = floor_y;
    s->err[0] = 0;
  }
  return 0;
}

int kxr_track_head(kxr_session *s, kxr_head *out) {
  kxr_head tmp;
  int rc;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  rc = track_head_unlocked(s, &tmp);
  pthread_mutex_unlock(&s->mu);
  if (out) *out = tmp;
  return rc;
}

int kxr_lock_head(kxr_session *s) {
  kxr_head h;
  kxr_vec3 mapped, pred0;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  if (!s->have_head) {
    set_err(s, "no OpenXR head pose");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  if (track_head_unlocked(s, &h) != 0) {
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  kxr_map_kinect(h.p_kinect, &mapped);
  mapped.x *= s->scale;
  mapped.y *= s->scale;
  mapped.z *= s->scale;
  pred0 = quat_rot(s->rig.kinect_in_xr.q, mapped);
  s->rig.kinect_in_xr.p = vsub(s->xr_head.p, pred0);
  s->rig.locked = 1;
  s->rig.residual_m = vlen(vsub(xform_k_to_x(&s->rig, h.p_kinect, s->scale), s->xr_head.p));
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return 0;
}

int kxr_lock_sample(kxr_session *s) {
  kxr_head h;
  kxr_vec3 mapped;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  if (!s->have_head) {
    set_err(s, "no OpenXR head pose");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  if (track_head_unlocked(s, &h) != 0) {
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  if (h.conf < 0.25f) {
    set_err(s, "head conf too low");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  kxr_map_kinect(h.p_kinect, &mapped);
  if (s->nsamp > 0) {
    kxr_vec3 dk = vsub(mapped, s->samp_k[s->nsamp - 1]);
    kxr_vec3 dx = vsub(s->xr_head.p, s->samp_x[s->nsamp - 1]);
    /* ignore jitter: need a few centimetres of motion */
    if (vlen(dk) < 0.06f && vlen(dx) < 0.06f) {
      pthread_mutex_unlock(&s->mu);
      return 0;
    }
  }
  if (s->nsamp >= KXR_MAX_SAMP) {
    memmove(s->samp_k, s->samp_k + 1, (KXR_MAX_SAMP - 1) * sizeof(kxr_vec3));
    memmove(s->samp_x, s->samp_x + 1, (KXR_MAX_SAMP - 1) * sizeof(kxr_vec3));
    s->nsamp = KXR_MAX_SAMP - 1;
  }
  s->samp_k[s->nsamp] = mapped;
  s->samp_x[s->nsamp] = s->xr_head.p;
  s->nsamp++;
  s->rig.nsamples = s->nsamp;
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return 0;
}

/* Symmetric 4×4 Jacobi. V columns are eigenvectors. */
static void jacobi4(const float a_in[16], float ev[4], float v[16]) {
  float a[16];
  int it, i, p, q, r, c;
  memcpy(a, a_in, sizeof a);
  memset(v, 0, 16 * sizeof(float));
  v[0] = v[5] = v[10] = v[15] = 1.f;
  for (it = 0; it < 48; it++) {
    float maxv = 0;
    p = 0;
    q = 1;
    for (r = 0; r < 4; r++) {
      for (c = r + 1; c < 4; c++) {
        float av = fabsf(a[r * 4 + c]);
        if (av > maxv) {
          maxv = av;
          p = r;
          q = c;
        }
      }
    }
    if (maxv < 1e-10f) break;
    {
      float app = a[p * 4 + p], aqq = a[q * 4 + q], apq = a[p * 4 + q];
      float tau = (aqq - app) / (2.f * apq);
      float t = (tau >= 0 ? 1.f : -1.f) / (fabsf(tau) + sqrtf(1.f + tau * tau));
      float cv = 1.f / sqrtf(1.f + t * t);
      float sv = t * cv;
      float rp[4], rq[4];
      for (i = 0; i < 4; i++) {
        rp[i] = cv * a[p * 4 + i] - sv * a[q * 4 + i];
        rq[i] = sv * a[p * 4 + i] + cv * a[q * 4 + i];
      }
      for (i = 0; i < 4; i++) {
        a[p * 4 + i] = a[i * 4 + p] = rp[i];
        a[q * 4 + i] = a[i * 4 + q] = rq[i];
      }
      a[p * 4 + p] = cv * cv * app - 2.f * sv * cv * apq + sv * sv * aqq;
      a[q * 4 + q] = sv * sv * app + 2.f * sv * cv * apq + cv * cv * aqq;
      a[p * 4 + q] = a[q * 4 + p] = 0;
      for (i = 0; i < 4; i++) {
        float vip = v[i * 4 + p], viq = v[i * 4 + q];
        v[i * 4 + p] = cv * vip - sv * viq;
        v[i * 4 + q] = sv * vip + cv * viq;
      }
    }
  }
  ev[0] = a[0];
  ev[1] = a[5];
  ev[2] = a[10];
  ev[3] = a[15];
}

/* Horn 1987: R,t minimizing ||R k + t - x||. k,x already axis-mapped. */
static int umeyama(const kxr_vec3 *k, const kxr_vec3 *x, int n, kxr_pose *out) {
  kxr_vec3 ck = {0, 0, 0}, cx = {0, 0, 0};
  float Sxx = 0, Sxy = 0, Sxz = 0, Syx = 0, Syy = 0, Syz = 0, Szx = 0, Szy = 0, Szz = 0;
  float N[16], ev[4], V[16];
  int i, best = 0;
  if (n < 3 || !k || !x || !out) return -1;
  for (i = 0; i < n; i++) {
    ck = vadd(ck, k[i]);
    cx = vadd(cx, x[i]);
  }
  ck.x /= (float)n;
  ck.y /= (float)n;
  ck.z /= (float)n;
  cx.x /= (float)n;
  cx.y /= (float)n;
  cx.z /= (float)n;
  for (i = 0; i < n; i++) {
    kxr_vec3 pk = vsub(k[i], ck), px = vsub(x[i], cx);
    Sxx += pk.x * px.x;
    Sxy += pk.x * px.y;
    Sxz += pk.x * px.z;
    Syx += pk.y * px.x;
    Syy += pk.y * px.y;
    Syz += pk.y * px.z;
    Szx += pk.z * px.x;
    Szy += pk.z * px.y;
    Szz += pk.z * px.z;
  }
  /* N is 4×4, row-major, quaternion (w,x,y,z). */
  N[0] = Sxx + Syy + Szz;
  N[1] = Syz - Szy;
  N[2] = Szx - Sxz;
  N[3] = Sxy - Syx;
  N[4] = Syz - Szy;
  N[5] = Sxx - Syy - Szz;
  N[6] = Sxy + Syx;
  N[7] = Szx + Sxz;
  N[8] = Szx - Sxz;
  N[9] = Sxy + Syx;
  N[10] = -Sxx + Syy - Szz;
  N[11] = Syz + Szy;
  N[12] = Sxy - Syx;
  N[13] = Szx + Sxz;
  N[14] = Syz + Szy;
  N[15] = -Sxx - Syy + Szz;
  jacobi4(N, ev, V);
  for (i = 1; i < 4; i++)
    if (ev[i] > ev[best]) best = i;
  out->q.w = V[0 * 4 + best];
  out->q.x = V[1 * 4 + best];
  out->q.y = V[2 * 4 + best];
  out->q.z = V[3 * 4 + best];
  out->q = quat_norm(out->q);
  {
    kxr_vec3 rk = quat_rot(out->q, ck);
    out->p = vsub(cx, rk);
  }
  return 0;
}

int kxr_lock_solve(kxr_session *s) {
  kxr_pose extra;
  int i;
  float resid = 0;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  if (s->nsamp < 3) {
    set_err(s, "need ≥3 lock samples");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  if (umeyama(s->samp_k, s->samp_x, s->nsamp, &extra) != 0) {
    set_err(s, "umeyama failed");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  s->rig.kinect_in_xr = extra;
  s->rig.locked = 1;
  s->rig.nsamples = s->nsamp;
  for (i = 0; i < s->nsamp; i++) {
    kxr_vec3 p = vadd(quat_rot(extra.q, s->samp_k[i]), extra.p);
    resid += vlen(vsub(p, s->samp_x[i]));
  }
  s->rig.residual_m = resid / (float)s->nsamp;
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return 0;
}

int kxr_lock_yaw_walk(kxr_session *s) {
  kxr_vec3 dk, dx;
  float yk, yx, dyaw;
  kxr_quat qy;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  if (s->nsamp < 2) {
    set_err(s, "need 2 walk samples");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  dk = vsub(s->samp_k[s->nsamp - 1], s->samp_k[0]);
  dx = vsub(s->samp_x[s->nsamp - 1], s->samp_x[0]);
  dk.y = 0;
  dx.y = 0;
  if (vlen(dk) < 0.15f || vlen(dx) < 0.15f) {
    set_err(s, "walk too short");
    pthread_mutex_unlock(&s->mu);
    return -1;
  }
  /* mapped Kinect looks along -Z after axis map, same as OpenXR */
  yk = atan2f(dk.x, -dk.z);
  yx = atan2f(dx.x, -dx.z);
  dyaw = (yx - yk) * 180.f / (float)M_PI;
  qy = quat_axis_deg(0, 1, 0, dyaw);
  s->rig.kinect_in_xr.q = quat_mul(qy, s->rig.kinect_in_xr.q);
  if (s->have_head && s->blob.ok) {
    kxr_vec3 mapped, pred0;
    kxr_map_kinect(s->blob.p_kinect, &mapped);
    pred0 = quat_rot(s->rig.kinect_in_xr.q, mapped);
    s->rig.kinect_in_xr.p = vsub(s->xr_head.p, pred0);
    s->rig.residual_m = vlen(vsub(xform_k_to_x(&s->rig, s->blob.p_kinect, s->scale), s->xr_head.p));
  }
  s->rig.locked = 1;
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);
  return 0;
}

void kxr_lock_reset(kxr_session *s) {
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  kxr_rig_identity(&s->rig);
  s->nsamp = 0;
  pthread_mutex_unlock(&s->mu);
}

int kxr_predict_head(const kxr_session *s, kxr_vec3 *out) {
  kxr_vec3 p;
  if (!s || !out) return -1;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  if (!s->blob.ok) {
    pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
    return -1;
  }
  p = xform_k_to_x(&s->rig, s->blob.p_kinect, s->scale);
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  *out = p;
  return 0;
}

static int world_cloud_unlocked(const kxr_session *s, cube_cloud *out) {
  int i, n = 0;
  float hide = s->hide_radius;
  kxr_vec3 head = s->xr_head.p;
  int clip = s->have_head && hide > 0.f;
  cube_cloud_clear(out);
  if (s->kinect.n <= 0) return -1;
  if (cube_cloud_copy(out, &s->kinect) != 0) return -1;
  for (i = 0; i < s->kinect.n; i++) {
    kxr_vec3 k, x;
    k.x = s->kinect.pt[i].x;
    k.y = s->kinect.pt[i].y;
    k.z = s->kinect.pt[i].z;
    x = xform_k_to_x(&s->rig, k, s->scale);
    if (clip) {
      kxr_vec3 d = vsub(x, head);
      if (vdot(d, d) < hide * hide) continue;
    }
    out->pt[n] = s->kinect.pt[i];
    out->pt[n].x = x.x;
    out->pt[n].y = x.y;
    out->pt[n].z = x.z;
    n++;
  }
  out->n = n;
  out->ok = n > 0;
  return out->ok ? 0 : -1;
}

int kxr_world_cloud(const kxr_session *s, cube_cloud *out) {
  int rc;
  if (!s || !out) return -1;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  rc = world_cloud_unlocked(s, out);
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return rc;
}

int kxr_fill_vertices(const kxr_session *s, float *xyzrgb, int max_pts, int *n) {
  cube_cloud w;
  int i, m;
  if (!s || !xyzrgb || max_pts < 1) return -1;
  cube_cloud_init(&w);
  if (kxr_world_cloud(s, &w) != 0) {
    cube_cloud_free(&w);
    if (n) *n = 0;
    return -1;
  }
  m = w.n < max_pts ? w.n : max_pts;
  for (i = 0; i < m; i++) {
    float *d = xyzrgb + i * 6;
    d[0] = w.pt[i].x;
    d[1] = w.pt[i].y;
    d[2] = w.pt[i].z;
    d[3] = w.pt[i].r / 255.f;
    d[4] = w.pt[i].g / 255.f;
    d[5] = w.pt[i].b / 255.f;
  }
  if (n) *n = m;
  cube_cloud_free(&w);
  return 0;
}

int kxr_fill_mesh(const kxr_session *s, float *xyznrgb, int max_v, int *nv,
                  unsigned *idx, int max_i, int *ni) {
  int i, nout = 0, iout = 0;
  int *map;
  float sc, hide2;
  kxr_rig rig;
  kxr_vec3 head;
  if (!s || !xyznrgb || !idx || max_v < 3 || max_i < 3) return -1;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  if (!s->mesh.ok || s->mesh.nv < 3) {
    pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
    if (nv) *nv = 0;
    if (ni) *ni = 0;
    return -1;
  }
  map = (int *)malloc((size_t)s->mesh.nv * sizeof(int));
  if (!map) {
    pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
    return -1;
  }
  sc = s->scale;
  rig = s->rig;
  hide2 = s->hide_radius * s->hide_radius;
  head = s->xr_head.p;
  for (i = 0; i < s->mesh.nv && nout < max_v; i++) {
    kxr_vec3 k, x, n0, n1;
    const cube_mesh_v *v = &s->mesh.v[i];
    float dx, dy, dz;
    k.x = v->x;
    k.y = v->y;
    k.z = v->z;
    x = xform_k_to_x(&rig, k, sc);
    dx = x.x - head.x;
    dy = x.y - head.y;
    dz = x.z - head.z;
    if (s->have_head && hide2 > 0.f && dx * dx + dy * dy + dz * dz < hide2) {
      map[i] = -1;
      continue;
    }
    n0.x = v->nx;
    n0.y = v->ny;
    n0.z = v->nz;
    n1 = xform_k_to_x(&rig, n0, 1.f);
    n1.x -= rig.kinect_in_xr.p.x;
    n1.y -= rig.kinect_in_xr.p.y;
    n1.z -= rig.kinect_in_xr.p.z;
    {
      float ln = sqrtf(n1.x * n1.x + n1.y * n1.y + n1.z * n1.z);
      float *d = xyznrgb + nout * 9;
      if (ln > 1e-6f) {
        n1.x /= ln;
        n1.y /= ln;
        n1.z /= ln;
      }
      d[0] = x.x;
      d[1] = x.y;
      d[2] = x.z;
      d[3] = n1.x;
      d[4] = n1.y;
      d[5] = n1.z;
      d[6] = v->r / 255.f;
      d[7] = v->g / 255.f;
      d[8] = v->b / 255.f;
    }
    map[i] = nout++;
  }
  for (i = 0; i + 2 < s->mesh.ni && iout + 3 <= max_i; i += 3) {
    int a = map[s->mesh.idx[i]], b = map[s->mesh.idx[i + 1]], c = map[s->mesh.idx[i + 2]];
    if (a < 0 || b < 0 || c < 0) continue;
    idx[iout++] = (unsigned)a;
    idx[iout++] = (unsigned)b;
    idx[iout++] = (unsigned)c;
  }
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  free(map);
  if (nv) *nv = nout;
  if (ni) *ni = iout;
  return iout > 0 ? 0 : -1;
}

int kxr_mesh_objects(const kxr_session *s) {
  int n;
  if (!s) return 0;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  n = s->mesh.nobj;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return n;
}

void kxr_view_matrix(const kxr_pose *eye, float out16[16]) {
  kxr_quat q;
  kxr_vec3 t, x, y, z, xa = {1, 0, 0}, ya = {0, 1, 0}, za = {0, 0, 1};
  if (!eye || !out16) return;
  q = quat_norm(eye->q);
  /* camera axes in world = rotated basis; view = R^T, t' = -R^T t */
  x = quat_rot(q, xa);
  y = quat_rot(q, ya);
  z = quat_rot(q, za);
  t = eye->p;
  memset(out16, 0, 16 * sizeof(float));
  out16[0] = x.x;
  out16[4] = x.y;
  out16[8] = x.z;
  out16[1] = y.x;
  out16[5] = y.y;
  out16[9] = y.z;
  out16[2] = z.x;
  out16[6] = z.y;
  out16[10] = z.z;
  out16[12] = -vdot(x, t);
  out16[13] = -vdot(y, t);
  out16[14] = -vdot(z, t);
  out16[15] = 1.f;
}

void kxr_proj_matrix(const kxr_fov *fov, float znear, float zfar, float out16[16]) {
  float l, r, u, d, w, h;
  if (!fov || !out16) return;
  if (znear < 0.01f) znear = 0.01f;
  if (zfar <= znear + 0.1f) zfar = znear + 0.1f;
  l = tanf(fov->left);
  r = tanf(fov->right);
  u = tanf(fov->up);
  d = tanf(fov->down);
  w = r - l;
  h = u - d;
  if (fabsf(w) < 1e-6f) w = 1e-6f;
  if (fabsf(h) < 1e-6f) h = 1e-6f;
  memset(out16, 0, 16 * sizeof(float));
  out16[0] = 2.f / w;
  out16[5] = 2.f / h;
  out16[8] = (r + l) / w;
  out16[9] = (u + d) / h;
  out16[10] = -(zfar + znear) / (zfar - znear);
  out16[11] = -1.f;
  out16[14] = -(2.f * zfar * znear) / (zfar - znear);
}

static void mat4_mul(const float a[16], const float b[16], float o[16]) {
  int c, r, k;
  float t[16];
  for (c = 0; c < 4; c++) {
    for (r = 0; r < 4; r++) {
      float s = 0;
      for (k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      t[c * 4 + r] = s;
    }
  }
  memcpy(o, t, sizeof t);
}

void kxr_mvp(const kxr_session *s, int eye, float znear, float zfar, float out16[16]) {
  float V[16], P[16];
  kxr_view v;
  if (!s || !out16) return;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  if (s->nviews < 1) {
    pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
    memset(out16, 0, 16 * sizeof(float));
    out16[0] = out16[5] = out16[10] = out16[15] = 1.f;
    return;
  }
  if (eye < 0) eye = 0;
  if (eye >= s->nviews) eye = s->nviews - 1;
  v = s->views[eye];
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  kxr_view_matrix(&v.pose, V);
  kxr_proj_matrix(&v.fov, znear, zfar, P);
  mat4_mul(P, V, out16);
}

int kxr_render_eye(const kxr_session *s, int eye, uint8_t *rgb, int w, int h,
                   float znear, float zfar) {
  cube_cloud world;
  float mvp[16];
  float *zbuf;
  int i, npix;
  if (!s || !rgb || w < 8 || h < 8) return -1;
  cube_cloud_init(&world);
  if (kxr_world_cloud(s, &world) != 0) {
    cube_cloud_free(&world);
    return -1;
  }
  kxr_mvp(s, eye, znear, zfar, mvp);
  npix = w * h;
  memset(rgb, 8, (size_t)npix * 3);
  for (i = 0; i < npix; i++) {
    rgb[i * 3 + 0] = 8;
    rgb[i * 3 + 1] = 2;
    rgb[i * 3 + 2] = 4;
  }
  zbuf = (float *)malloc((size_t)npix * sizeof(float));
  if (!zbuf) {
    cube_cloud_free(&world);
    return -1;
  }
  for (i = 0; i < npix; i++) zbuf[i] = 1e9f;
  for (i = 0; i < world.n; i++) {
    float px = world.pt[i].x, py = world.pt[i].y, pz = world.pt[i].z;
    float cx = mvp[0] * px + mvp[4] * py + mvp[8] * pz + mvp[12];
    float cy = mvp[1] * px + mvp[5] * py + mvp[9] * pz + mvp[13];
    float cz = mvp[2] * px + mvp[6] * py + mvp[10] * pz + mvp[14];
    float cw = mvp[3] * px + mvp[7] * py + mvp[11] * pz + mvp[15];
    float inv, ndc_x, ndc_y;
    int ix, iy, k, dx, dy;
    if (cw <= 1e-5f) continue;
    inv = 1.f / cw;
    ndc_x = cx * inv;
    ndc_y = cy * inv;
    if (ndc_x < -1.2f || ndc_x > 1.2f || ndc_y < -1.2f || ndc_y > 1.2f) continue;
    ix = (int)((ndc_x * 0.5f + 0.5f) * (float)w);
    iy = (int)((0.5f - ndc_y * 0.5f) * (float)h);
    for (dy = -1; dy <= 1; dy++) {
      for (dx = -1; dx <= 1; dx++) {
        int xx = ix + dx, yy = iy + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        k = yy * w + xx;
        if (cz >= zbuf[k]) continue;
        zbuf[k] = cz;
        rgb[k * 3 + 0] = world.pt[i].r;
        rgb[k * 3 + 1] = world.pt[i].g;
        rgb[k * 3 + 2] = world.pt[i].b;
      }
    }
  }
  free(zbuf);
  cube_cloud_free(&world);
  return 0;
}

void kxr_status_json(const kxr_session *s, char *buf, size_t n) {
  kxr_rig r;
  kxr_head h;
  int pts, live, locked, nv;
  float sc, bh, bs, fy, xfy;
  int have_xf;
  if (!buf || n < 8) return;
  if (!s) {
    snprintf(buf, n, "{\"ok\":false}\n");
    return;
  }
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  r = s->rig;
  h = s->blob;
  pts = s->kinect.n;
  live = s->live;
  locked = r.locked;
  nv = s->nviews;
  sc = s->scale;
  bh = s->body_h;
  bs = s->body_score;
  fy = s->floor_y;
  xfy = s->xr_floor_y;
  have_xf = s->have_xr_floor;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  snprintf(buf, n,
           "{\"ok\":true,\"lib\":\"kinect_xr\",\"points\":%d,\"live\":%s,"
           "\"locked\":%s,\"scale\":%.3f,\"residual_m\":%.4f,\"samples\":%d,\"views\":%d,"
           "\"head_ok\":%s,\"head_k\":[%.4f,%.4f,%.4f],\"conf\":%.3f,"
           "\"body_h\":%.3f,\"body_score\":%.3f,\"floor_y\":%.3f,"
           "\"xr_floor\":%.3f,\"have_xr_floor\":%s,"
           "\"kinect_in_xr\":{\"p\":[%.4f,%.4f,%.4f],\"q\":[%.4f,%.4f,%.4f,%.4f]},"
           "\"collect\":false}\n",
           pts, live ? "true" : "false", locked ? "true" : "false", sc, r.residual_m,
           r.nsamples, nv, h.ok ? "true" : "false", h.p_kinect.x, h.p_kinect.y,
           h.p_kinect.z, h.conf, bh, bs, fy, xfy, have_xf ? "true" : "false",
           r.kinect_in_xr.p.x, r.kinect_in_xr.p.y, r.kinect_in_xr.p.z,
           r.kinect_in_xr.q.x, r.kinect_in_xr.q.y, r.kinect_in_xr.q.z,
           r.kinect_in_xr.q.w);
}

/* ---- live depth (v1, v2, or both). No GTK. ---- */

/* Drop the panel's USB hold so OpenXR can stream without the dashboard. */
static void ask_panel_off(void) {
  const char *rd = getenv("XDG_RUNTIME_DIR");
  char path[320];
  int fd;
  struct sockaddr_un addr;
  if (rd && rd[0])
    snprintf(path, sizeof path, "%s/kinect-panel.sock", rd);
  else
    snprintf(path, sizeof path, "/tmp/kinect-panel.sock");
  fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return;
  memset(&addr, 0, sizeof addr);
  addr.sun_family = AF_UNIX;
  snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path);
  if (connect(fd, (struct sockaddr *)&addr, sizeof addr) == 0) {
    ssize_t nw = write(fd, "OFF\n", 4);
    (void)nw;
    usleep(280000);
  }
  close(fd);
}

static void *live_main(void *arg) {
  kxr_session *s = (kxr_session *)arg;
  cube_cloud_tune t;
  uint16_t *depth;
  uint32_t seen = 0;

  pthread_mutex_lock(&s->mu);
  t = s->tune;
  pthread_mutex_unlock(&s->mu);
  kinect_fuse_set_tune(&t);
  depth = (uint16_t *)calloc(DEP_N, sizeof(uint16_t));
  if (kinect_fuse_start() != 0) {
    const char *e = kinect_fuse_error();
    set_err(s, e && e[0] ? e : "live open failed");
    free(depth);
    pthread_mutex_lock(&s->mu);
    s->live = 0;
    s->live_failed = 1;
    pthread_mutex_unlock(&s->mu);
    return NULL;
  }
  pthread_mutex_lock(&s->mu);
  s->live = 1;
  s->err[0] = 0;
  pthread_mutex_unlock(&s->mu);

  while (!s->live_stop) {
    kinect_fuse_stats st;
    cube_cloud snap;
    kinect_fuse_get_stats(&st);
    if (st.seq != seen) {
      cube_cloud_init(&snap);
      if (kinect_fuse_snapshot(&snap) == 0 && snap.n > 0) {
        seen = st.seq;
        pthread_mutex_lock(&s->mu);
        cube_cloud_copy(&s->kinect, &snap);
        t = s->tune;
        /* Solid mesh is the 640x480 Kinect. v2 and a fused cloud stay points. */
        if (st.active == KINECT_SRC_V1 && depth && kinect_fuse_copy_v1_depth(depth) == 0)
          cube_mesh_from_depth(&s->mesh, depth, &t);
        else
          cube_mesh_clear(&s->mesh);
        s->err[0] = 0;
        pthread_mutex_unlock(&s->mu);
      }
      cube_cloud_free(&snap);
    }
    usleep(12000);
  }
  free(depth);
  kinect_fuse_stop();
  pthread_mutex_lock(&s->mu);
  s->live = 0;
  pthread_mutex_unlock(&s->mu);
  return NULL;
}

int kxr_live_start(kxr_session *s) {
  int i, aborted = 0;
  if (!s) return -1;
  kxr_live_stop(s);
  pthread_mutex_lock(&s->mu);
  s->live_stop = 0;
  s->live_failed = 0;
  s->live = 0;
  s->err[0] = 0;
  /* Old points must not count as the new sensor already streaming. */
  s->kinect.n = 0;
  pthread_mutex_unlock(&s->mu);
  ask_panel_off();
  cube_cloud_free_holders();
  usleep(200000);
  pthread_mutex_lock(&s->mu);
  aborted = s->reopen_stop;
  pthread_mutex_unlock(&s->mu);
  if (aborted) return -1;
  if (pthread_create(&s->live_th, NULL, live_main, s) != 0) {
    set_err(s, "live thread");
    return -1;
  }
  for (i = 0; i < 900; i++) {
    int live, n, failed, stop;
    usleep(25000);
    pthread_mutex_lock(&s->mu);
    live = s->live;
    n = s->kinect.n;
    failed = s->live_failed;
    stop = s->reopen_stop;
    pthread_mutex_unlock(&s->mu);
    if (stop) {
      aborted = 1;
      break;
    }
    if (n > 0 || live) return 0;
    if (failed) break;
  }
  kxr_live_stop(s);
  if (aborted) return -1;
  if (!s->err[0]) set_err(s, "no live depth");
  return -1;
}

void kxr_live_stop(kxr_session *s) {
  if (!s || (!s->live && !s->live_th)) return;
  /* Do not join reopen_th / align_th here. The reopen worker is inside
   * kxr_live_start, which is this function's caller when a sensor changes. */
  s->live_stop = 1;
  kinect_fuse_request_stop();
  if (s->live_th) {
    pthread_join(s->live_th, NULL);
    s->live_th = 0;
  }
  pthread_mutex_lock(&s->mu);
  s->live = 0;
  s->live_failed = 0;
  pthread_mutex_unlock(&s->mu);
}

static int opened_matches(kinect_src want, kinect_src active) {
  if (want == KINECT_SRC_AUTO)
    return active == KINECT_SRC_V1 || active == KINECT_SRC_V2 || active == KINECT_SRC_BOTH;
  return active == want;
}

/* Restarts depth until the opened cloud matches kinect_fuse_source().
 * A newer set_source only bumps reopen_gen; this loop picks it up. */
static void *reopen_main(void *arg) {
  kxr_session *s = (kxr_session *)arg;
  int spins = 0;
  char msg[160];

  snprintf(msg, sizeof msg, "sensor restart failed");
  while (spins < 6) {
    unsigned gen;
    int stop, rc;
    kinect_src want;
    kinect_fuse_stats st;

    pthread_mutex_lock(&s->mu);
    stop = s->reopen_stop;
    gen = s->reopen_gen;
    pthread_mutex_unlock(&s->mu);
    if (stop) {
      msg[0] = 0;
      break;
    }
    want = kinect_fuse_source();
    rc = kxr_live_start(s);
    pthread_mutex_lock(&s->mu);
    stop = s->reopen_stop;
    if (stop) {
      pthread_mutex_unlock(&s->mu);
      msg[0] = 0;
      break;
    }
    if (s->reopen_gen != gen) {
      pthread_mutex_unlock(&s->mu);
      spins++;
      continue;
    }
    pthread_mutex_unlock(&s->mu);
    memset(&st, 0, sizeof st);
    kinect_fuse_get_stats(&st);
    if (rc == 0 && opened_matches(want, st.active)) {
      snprintf(msg, sizeof msg, "sensor %s", sensor_label(want));
      break;
    }
    if (rc != 0) {
      pthread_mutex_lock(&s->mu);
      snprintf(msg, sizeof msg, "%s", s->err[0] ? s->err : "no live depth");
      pthread_mutex_unlock(&s->mu);
      break;
    }
    spins++;
  }
  pthread_mutex_lock(&s->mu);
  s->reopen_busy = 0;
  if (!s->reopen_stop && msg[0]) {
    snprintf(s->note, sizeof s->note, "%s", msg);
    s->note_seq++;
  }
  pthread_mutex_unlock(&s->mu);
  return NULL;
}

static int schedule_reopen(kxr_session *s) {
  int live, busy, alive;
  char msg[64];
  pthread_mutex_lock(&s->mu);
  busy = s->reopen_busy;
  alive = s->reopen_alive;
  live = s->live;
  if (busy) s->reopen_gen++;
  pthread_mutex_unlock(&s->mu);
  snprintf(msg, sizeof msg, "switching to %s", sensor_label(kinect_fuse_source()));
  if (busy) {
    note_set(s, msg);
    return 1;
  }
  if (alive) {
    pthread_join(s->reopen_th, NULL);
    pthread_mutex_lock(&s->mu);
    s->reopen_alive = 0;
    live = s->live;
    pthread_mutex_unlock(&s->mu);
  }
  if (!live) return 0;
  pthread_mutex_lock(&s->mu);
  s->reopen_stop = 0;
  s->reopen_busy = 1;
  s->reopen_gen++;
  pthread_mutex_unlock(&s->mu);
  if (pthread_create(&s->reopen_th, NULL, reopen_main, s) != 0) {
    pthread_mutex_lock(&s->mu);
    s->reopen_busy = 0;
    pthread_mutex_unlock(&s->mu);
    set_err(s, "sensor restart failed");
    note_set(s, "sensor restart failed");
    return 1;
  }
  pthread_mutex_lock(&s->mu);
  s->reopen_alive = 1;
  pthread_mutex_unlock(&s->mu);
  note_set(s, msg);
  return 1;
}

static void *align_main(void *arg) {
  kxr_session *s = (kxr_session *)arg;
  ksync_align_result res;
  char msg[160];
  if (kinect_fuse_align(&res) != 0) {
    const char *e = kinect_fuse_error();
    snprintf(msg, sizeof msg, "%s", e && e[0] ? e : "align failed");
  } else {
    snprintf(msg, sizeof msg, "aligned yaw %.0f pitch %.0f", res.yaw_deg, res.pitch_deg);
  }
  pthread_mutex_lock(&s->mu);
  s->align_busy = 0;
  if (!s->align_stop) {
    snprintf(s->note, sizeof s->note, "%s", msg);
    s->note_seq++;
  }
  pthread_mutex_unlock(&s->mu);
  return NULL;
}

void kxr_align_v2(kxr_session *s) {
  int busy, alive;
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  busy = s->align_busy;
  alive = s->align_alive;
  pthread_mutex_unlock(&s->mu);
  if (busy) {
    note_set(s, "aligning kinect 2");
    return;
  }
  if (alive) {
    pthread_join(s->align_th, NULL);
    pthread_mutex_lock(&s->mu);
    s->align_alive = 0;
    pthread_mutex_unlock(&s->mu);
  }
  pthread_mutex_lock(&s->mu);
  s->align_stop = 0;
  s->align_busy = 1;
  pthread_mutex_unlock(&s->mu);
  if (pthread_create(&s->align_th, NULL, align_main, s) != 0) {
    pthread_mutex_lock(&s->mu);
    s->align_busy = 0;
    pthread_mutex_unlock(&s->mu);
    set_err(s, "align thread failed");
    note_set(s, "align thread failed");
    return;
  }
  pthread_mutex_lock(&s->mu);
  s->align_alive = 1;
  pthread_mutex_unlock(&s->mu);
  note_set(s, "aligning kinect 2");
}

void kxr_align_reset(kxr_session *s) {
  int busy;
  if (!s) return;
  pthread_mutex_lock(&s->mu);
  busy = s->align_busy;
  pthread_mutex_unlock(&s->mu);
  if (busy) {
    note_set(s, "align busy");
    return;
  }
  kinect_fuse_reset_rigid();
  note_set(s, "align reset");
}

int kxr_live_running(const kxr_session *s) {
  int r;
  if (!s) return 0;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  r = s->live;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return r;
}

int kxr_live_poll(kxr_session *s) {
  int n;
  if (!s) return -1;
  pthread_mutex_lock(&s->mu);
  n = s->kinect.n;
  pthread_mutex_unlock(&s->mu);
  return n > 0 ? 0 : -1;
}

int kxr_set_tilt(kxr_session *s, double degrees) {
  if (!s) return -1;
  if (degrees < -30) degrees = -30;
  if (degrees > 30) degrees = 30;
  pthread_mutex_lock(&s->mu);
  s->tilt_deg = degrees;
  pthread_mutex_unlock(&s->mu);
  return kinect_fuse_set_tilt(degrees);
}

float kxr_get_tilt(const kxr_session *s) {
  float d;
  if (!s) return 0.f;
  pthread_mutex_lock((pthread_mutex_t *)&s->mu);
  d = (float)s->tilt_deg;
  pthread_mutex_unlock((pthread_mutex_t *)&s->mu);
  return d;
}
