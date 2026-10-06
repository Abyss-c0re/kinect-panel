#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "kinect_sync.h"
#include "cube_paths.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct ksync_v3 {
  float x, y, z;
} ksync_v3;

void ksync_rigid_identity(ksync_rigid *r) {
  int i;
  if (!r) return;
  for (i = 0; i < 9; i++) r->R[i] = 0.f;
  r->R[0] = r->R[4] = r->R[8] = 1.f;
  r->t[0] = r->t[1] = r->t[2] = 0.f;
}

int ksync_rigid_is_identity(const ksync_rigid *r) {
  int i;
  if (!r) return 1;
  for (i = 0; i < 9; i++) {
    float e = (i == 0 || i == 4 || i == 8) ? 1.f : 0.f;
    if (fabsf(r->R[i] - e) > 1e-4f) return 0;
  }
  for (i = 0; i < 3; i++)
    if (fabsf(r->t[i]) > 1e-4f) return 0;
  return 1;
}

void ksync_rigid_apply(const ksync_rigid *r, float *x, float *y, float *z) {
  float X, Y, Z;
  if (!r || !x || !y || !z) return;
  X = *x;
  Y = *y;
  Z = *z;
  *x = r->R[0] * X + r->R[1] * Y + r->R[2] * Z + r->t[0];
  *y = r->R[3] * X + r->R[4] * Y + r->R[5] * Z + r->t[1];
  *z = r->R[6] * X + r->R[7] * Y + r->R[8] * Z + r->t[2];
}

void ksync_rigid_ypr(const ksync_rigid *r, float *yaw_deg, float *pitch_deg) {
  float yaw, pitch;
  if (!r) return;
  /* Ry then Rx. Yaw is about +Y, matching ksync_R_ypr(yaw, 0, 0). */
  yaw = atan2f(r->R[2], r->R[0]) * (180.f / (float)M_PI);
  pitch = atan2f(-r->R[5], r->R[4]) * (180.f / (float)M_PI);
  if (yaw_deg) *yaw_deg = yaw;
  if (pitch_deg) *pitch_deg = pitch;
}

static int mkdir_p(const char *path) {
  char buf[512];
  size_t n, i;
  if (!path || !path[0]) return -1;
  n = strlen(path);
  if (n >= sizeof buf) return -1;
  memcpy(buf, path, n + 1);
  for (i = 1; i < n; i++) {
    if (buf[i] != '/') continue;
    buf[i] = 0;
    if (mkdir(buf, 0755) < 0 && errno != EEXIST) return -1;
    buf[i] = '/';
  }
  if (mkdir(buf, 0755) < 0 && errno != EEXIST) return -1;
  return 0;
}

static int parent_mkdir(const char *file) {
  char buf[512];
  const char *slash;
  size_t n;
  slash = strrchr(file, '/');
  if (!slash) return 0;
  n = (size_t)(slash - file);
  if (n >= sizeof buf) return -1;
  memcpy(buf, file, n);
  buf[n] = 0;
  return mkdir_p(buf);
}

int ksync_extrinsic_path(char *out, size_t n) {
  const char *env;
  if (!out || !n) return -1;
  env = getenv("KINECT_EXTRINSIC");
  if (env && env[0]) {
    snprintf(out, n, "%s", env);
    return 0;
  }
  snprintf(out, n, "%s/kinect2_extrinsic.txt", CUBE_TRACK_DIR);
  return 0;
}

int ksync_rigid_save(const ksync_rigid *r, const char *path) {
  FILE *f;
  char tmp[512];
  if (!r || !path) return -1;
  if (parent_mkdir(path) < 0) return -1;
  snprintf(tmp, sizeof tmp, "%s.tmp", path);
  f = fopen(tmp, "w");
  if (!f) return -1;
  fprintf(f, "# p_v1 = R * p_v2 + t, row-major\n");
  fprintf(f, "%.8f %.8f %.8f %.8f\n", r->R[0], r->R[1], r->R[2], r->t[0]);
  fprintf(f, "%.8f %.8f %.8f %.8f\n", r->R[3], r->R[4], r->R[5], r->t[1]);
  fprintf(f, "%.8f %.8f %.8f %.8f\n", r->R[6], r->R[7], r->R[8], r->t[2]);
  if (fclose(f) != 0) return -1;
  if (rename(tmp, path) != 0) return -1;
  return 0;
}

int ksync_rigid_load(ksync_rigid *r, const char *path) {
  FILE *f;
  char line[256];
  int row = 0;
  float a, b, c, d;
  if (!r || !path) return -1;
  f = fopen(path, "r");
  if (!f) return -1;
  ksync_rigid_identity(r);
  while (fgets(line, sizeof line, f) && row < 3) {
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
    if (sscanf(line, "%f %f %f %f", &a, &b, &c, &d) != 4) {
      fclose(f);
      return -1;
    }
    r->R[row * 3 + 0] = a;
    r->R[row * 3 + 1] = b;
    r->R[row * 3 + 2] = c;
    r->t[row] = d;
    row++;
  }
  fclose(f);
  return row == 3 ? 0 : -1;
}

int ksync_cover_stride(int w, int h, int stride, int max_pts) {
  if (stride < 1) stride = 1;
  if (stride > 32) stride = 32;
  if (w < 1 || h < 1 || max_pts < 1) return stride;
  while (stride < 32) {
    int nx = (w + stride - 1) / stride;
    int ny = (h + stride - 1) / stride;
    if (nx * ny <= max_pts) break;
    stride++;
  }
  return stride;
}

static uint64_t abs_dt(uint64_t a, uint64_t b) { return a > b ? a - b : b - a; }

int ksync_best_pair(const uint64_t *t1, int n1, const uint64_t *t2, int n2,
                    uint64_t now_us, int *i1, int *i2, uint64_t *skew_us) {
  int i, j, bi = -1, bj = -1;
  uint64_t best = UINT64_MAX;
  if (i1) *i1 = -1;
  if (i2) *i2 = -1;
  if (skew_us) *skew_us = 0;
  if (!t1 || !t2 || n1 < 1 || n2 < 1) return -1;
  for (i = 0; i < n1; i++) {
    if (now_us > t1[i] && now_us - t1[i] > KSYNC_STALE_US) continue;
    for (j = 0; j < n2; j++) {
      uint64_t dt;
      if (now_us > t2[j] && now_us - t2[j] > KSYNC_STALE_US) continue;
      dt = abs_dt(t1[i], t2[j]);
      if (dt < best) {
        best = dt;
        bi = i;
        bj = j;
      }
    }
  }
  if (bi < 0) return -1;
  if (i1) *i1 = bi;
  if (i2) *i2 = bj;
  if (skew_us) *skew_us = best;
  return 0;
}

void ksync_tint_v2(cube_cloud *c) {
  int i;
  if (!c || !c->pt) return;
  for (i = 0; i < c->n; i++) {
    cube_cloud_pt *p = &c->pt[i];
    unsigned b = (unsigned)p->b / 4u + 180u;
    unsigned g = (unsigned)p->g / 2u + 30u;
    unsigned r = (unsigned)p->r / 4u;
    if (b > 255u) b = 255u;
    if (g > 255u) g = 255u;
    p->r = (uint8_t)r;
    p->g = (uint8_t)g;
    p->b = (uint8_t)b;
  }
}

int ksync_merge(cube_cloud *dst, const cube_cloud *v1, const cube_cloud *v2,
                const ksync_rigid *v2_to_v1, int max_pts) {
  ksync_rigid id;
  const ksync_rigid *rig = v2_to_v1;
  int na, nb, sa, sb, i, n = 0, take_a, take_b;
  if (!dst) return -1;
  if (!rig) {
    ksync_rigid_identity(&id);
    rig = &id;
  }
  if (max_pts < 1 || max_pts > CUBE_CLOUD_MAX) max_pts = CUBE_CLOUD_MAX;
  na = (v1 && v1->pt) ? v1->n : 0;
  nb = (v2 && v2->pt) ? v2->n : 0;
  if (na < 0) na = 0;
  if (nb < 0) nb = 0;
  sa = 1;
  sb = 1;
  take_a = na;
  take_b = nb;
  while (take_a + take_b > max_pts && (sa < na || sb < nb)) {
    if (take_a >= take_b && sa < na)
      sa++;
    else if (sb < nb)
      sb++;
    else
      sa++;
    take_a = sa > 0 ? (na + sa - 1) / sa : 0;
    take_b = sb > 0 ? (nb + sb - 1) / sb : 0;
    if (sa > 64 && sb > 64) break;
  }
  cube_cloud_clear(dst);
  if (cube_cloud_reserve(dst, take_a + take_b > 0 ? take_a + take_b : 1) < 0) {
    snprintf(dst->err, sizeof dst->err, "oom");
    return -1;
  }
  for (i = 0; i < na && n < dst->cap && n < max_pts; i += sa)
    dst->pt[n++] = v1->pt[i];
  for (i = 0; i < nb && n < dst->cap && n < max_pts; i += sb) {
    cube_cloud_pt p = v2->pt[i];
    ksync_rigid_apply(rig, &p.x, &p.y, &p.z);
    dst->pt[n++] = p;
  }
  dst->n = n;
  dst->ok = n > 0;
  if (!dst->ok) snprintf(dst->err, sizeof dst->err, "empty cloud");
  return dst->ok ? 0 : -1;
}

int ksync_project_depth(cube_cloud *out, const float *z_m, int w, int h,
                        float fx, float fy, float cx, float cy,
                        const cube_cloud_tune *t, int max_pts) {
  cube_cloud_tune def;
  int stride, v, u, n = 0;
  float zn, zf;
  if (!out || !z_m || w < 1 || h < 1 || fx < 1.f || fy < 1.f) return -1;
  if (!t) {
    cube_cloud_tune_default(&def);
    t = &def;
  }
  if (max_pts < 1 || max_pts > CUBE_CLOUD_MAX) max_pts = CUBE_CLOUD_MAX;
  stride = ksync_cover_stride(w, h, t->stride, max_pts);
  zn = t->z_near;
  zf = t->z_far;
  if (zn < 0.2f) zn = 0.2f;
  if (zf < zn + 0.1f) zf = zn + 0.1f;
  cube_cloud_clear(out);
  if (cube_cloud_reserve(out, max_pts) < 0) {
    snprintf(out->err, sizeof out->err, "oom");
    return -1;
  }
  for (v = 0; v < h && n < max_pts; v += stride) {
    for (u = 0; u < w && n < max_pts; u += stride) {
      float z = z_m[v * w + u];
      cube_cloud_pt *p;
      if (!(z > zn && z < zf)) continue;
      p = &out->pt[n++];
      p->x = (u - cx) * z / fx;
      p->y = -((v - cy) * z / fy);
      p->z = z;
      cube_cloud_color_point(p, t);
    }
  }
  out->n = n;
  out->ok = n > 0;
  if (!out->ok) snprintf(out->err, sizeof out->err, "empty cloud");
  return out->ok ? 0 : -1;
}

static void mul3(const float A[9], const float B[9], float C[9]) {
  int r, c, k;
  for (r = 0; r < 3; r++) {
    for (c = 0; c < 3; c++) {
      float s = 0.f;
      for (k = 0; k < 3; k++) s += A[r * 3 + k] * B[k * 3 + c];
      C[r * 3 + c] = s;
    }
  }
}

static void ksync_R_ypr(float yaw_deg, float pitch_deg, float roll_deg, float R[9]) {
  float y = yaw_deg * (float)M_PI / 180.f;
  float p = pitch_deg * (float)M_PI / 180.f;
  float r = roll_deg * (float)M_PI / 180.f;
  float cy = cosf(y), sy = sinf(y), cp = cosf(p), sp = sinf(p), cr = cosf(r), sr = sinf(r);
  float Ry[9] = {cy, 0, sy, 0, 1, 0, -sy, 0, cy};
  float Rx[9] = {1, 0, 0, 0, cp, -sp, 0, sp, cp};
  float Rz[9] = {cr, -sr, 0, sr, cr, 0, 0, 0, 1};
  float T[9];
  mul3(Rx, Rz, T);
  mul3(Ry, T, R);
}

static void apply_rt(const float R[9], const float t[3], ksync_v3 *p) {
  float x = p->x, y = p->y, z = p->z;
  p->x = R[0] * x + R[1] * y + R[2] * z + t[0];
  p->y = R[3] * x + R[4] * y + R[5] * z + t[1];
  p->z = R[6] * x + R[7] * y + R[8] * z + t[2];
}

static int sample_cloud(const cube_cloud *c, ksync_v3 *out, int maxn) {
  int step, i, n = 0;
  if (!c || !c->pt || c->n < 1 || maxn < 1) return 0;
  step = c->n / maxn;
  if (step < 1) step = 1;
  for (i = 0; i < c->n && n < maxn; i += step) {
    out[n].x = c->pt[i].x;
    out[n].y = c->pt[i].y;
    out[n].z = c->pt[i].z;
    n++;
  }
  return n;
}

static void centroid(const ksync_v3 *p, int n, ksync_v3 *c) {
  int i;
  double x = 0, y = 0, z = 0;
  if (n < 1) {
    c->x = c->y = c->z = 0;
    return;
  }
  for (i = 0; i < n; i++) {
    x += p[i].x;
    y += p[i].y;
    z += p[i].z;
  }
  c->x = (float)(x / n);
  c->y = (float)(y / n);
  c->z = (float)(z / n);
}

static float nearest2(const ksync_v3 *a, int na, ksync_v3 p) {
  int i;
  float best = 1e30f;
  for (i = 0; i < na; i++) {
    float dx = a[i].x - p.x, dy = a[i].y - p.y, dz = a[i].z - p.z;
    float d = dx * dx + dy * dy + dz * dz;
    if (d < best) best = d;
  }
  return best;
}

/* Lower is better. Thin overlap scores as a miss. */
static float score_rt(const ksync_v3 *a, int na, const ksync_v3 *b, int nb,
                      const float R[9], const float t[3], float thresh, int *inliers) {
  int i, inl = 0;
  double acc = 0;
  float th2 = thresh * thresh;
  for (i = 0; i < nb; i++) {
    ksync_v3 p = b[i];
    float d2;
    apply_rt(R, t, &p);
    d2 = nearest2(a, na, p);
    if (d2 <= th2) {
      acc += sqrt((double)d2);
      inl++;
    }
  }
  if (inliers) *inliers = inl;
  if (inl < 20) return 1000.f + (float)(nb - inl);
  return (float)(acc / inl);
}

static void t_from_centroids(const ksync_v3 *ca, const ksync_v3 *cb, const float R[9],
                             float t[3]) {
  float x = cb->x, y = cb->y, z = cb->z;
  t[0] = ca->x - (R[0] * x + R[1] * y + R[2] * z);
  t[1] = ca->y - (R[3] * x + R[4] * y + R[5] * z);
  t[2] = ca->z - (R[6] * x + R[7] * y + R[8] * z);
}

static void rigid_from_ypr(float yaw, float pitch, float roll, const ksync_v3 *ca,
                           const ksync_v3 *cb, ksync_rigid *r) {
  ksync_R_ypr(yaw, pitch, roll, r->R);
  t_from_centroids(ca, cb, r->R, r->t);
}

int ksync_align(ksync_rigid *out, ksync_align_result *res, const cube_cloud *v1,
                const cube_cloud *v2) {
  ksync_v3 sa[160], sb[160], ca, cb;
  ksync_rigid best, trial;
  float best_score = 1e9f, yaw, pitch, roll;
  int na, nb, best_inl = 0, yaw_i, pitch_i, roll_i, iter, ax;
  float thresh = 0.14f;
  if (res) memset(res, 0, sizeof *res);
  if (!out || !v1 || !v2 || v1->n < 30 || v2->n < 30) return -1;
  na = sample_cloud(v1, sa, 120);
  nb = sample_cloud(v2, sb, 120);
  if (na < 30 || nb < 30) return -1;
  centroid(sa, na, &ca);
  centroid(sb, nb, &cb);
  ksync_rigid_identity(&best);

  for (yaw_i = -15; yaw_i <= 15; yaw_i++) {
    yaw = (float)yaw_i * 4.f;
    for (pitch_i = -6; pitch_i <= 6; pitch_i++) {
      pitch = (float)pitch_i * 4.f;
      for (roll_i = -3; roll_i <= 3; roll_i++) {
        float sc;
        int inl = 0;
        roll = (float)roll_i * 4.f;
        rigid_from_ypr(yaw, pitch, roll, &ca, &cb, &trial);
        sc = score_rt(sa, na, sb, nb, trial.R, trial.t, thresh, &inl);
        if (sc < best_score) {
          best_score = sc;
          best = trial;
          best_inl = inl;
        }
      }
    }
  }

  /* Walk yaw, pitch, roll, then translation. Full overlap locks the rotation;
   * the steps pull a partial overlap off the centroid guess. */
  for (iter = 0; iter < 18; iter++) {
    float steps[6] = {2.f, 2.f, 2.f, 0.02f, 0.02f, 0.02f};
    int improved = 0;
    ksync_rigid_ypr(&best, &yaw, &pitch);
    roll = atan2f(best.R[3], best.R[4]) * (180.f / (float)M_PI);
    for (ax = 0; ax < 6; ax++) {
      int dir;
      for (dir = -1; dir <= 1; dir += 2) {
        float sc, y2 = yaw, p2 = pitch, r2 = roll;
        float t2[3] = {best.t[0], best.t[1], best.t[2]};
        int inl = 0;
        if (ax == 0) y2 += dir * steps[0];
        else if (ax == 1) p2 += dir * steps[1];
        else if (ax == 2) r2 += dir * steps[2];
        else
          t2[ax - 3] += dir * steps[ax];
        ksync_R_ypr(y2, p2, r2, trial.R);
        if (ax < 3)
          t_from_centroids(&ca, &cb, trial.R, trial.t);
        else {
          trial.t[0] = t2[0];
          trial.t[1] = t2[1];
          trial.t[2] = t2[2];
        }
        sc = score_rt(sa, na, sb, nb, trial.R, trial.t, thresh, &inl);
        if (sc + 1e-5f < best_score) {
          best_score = sc;
          best = trial;
          best_inl = inl;
          improved = 1;
        }
      }
    }
    if (!improved) thresh = thresh > 0.06f ? thresh * 0.85f : thresh;
  }

  if (best_inl < 25 || best_score > 0.08f) return -1;
  *out = best;
  if (res) {
    res->ok = 1;
    res->residual_m = best_score;
    res->inliers = best_inl;
    ksync_rigid_ypr(&best, &res->yaw_deg, &res->pitch_deg);
  }
  return 0;
}
