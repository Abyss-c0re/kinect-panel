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

/* Match distance. A stride of 120 points from a full cloud is coarser than
 * this, so those samples cannot see a real wall. Keep the surface spacing. */
#define KSYNC_MATCH_M 0.12f
#define KSYNC_HASH 8192

static unsigned khash_cell(int x, int y, int z) {
  unsigned h = (unsigned)x * 73856093u ^ (unsigned)y * 19349663u ^ (unsigned)z * 83492791u;
  return h & (KSYNC_HASH - 1u);
}

typedef struct ksync_hash {
  const ksync_v3 *p;
  int *next;
  int head[KSYNC_HASH];
  float cell;
} ksync_hash;

static int hash_build(ksync_hash *h, const ksync_v3 *p, int n, float cell) {
  int i;
  h->p = p;
  h->cell = cell;
  h->next = (int *)malloc(sizeof(int) * (n > 0 ? (size_t)n : 1));
  if (!h->next) return -1;
  for (i = 0; i < KSYNC_HASH; i++) h->head[i] = -1;
  for (i = 0; i < n; i++) {
    int ix = (int)floorf(p[i].x / cell);
    int iy = (int)floorf(p[i].y / cell);
    int iz = (int)floorf(p[i].z / cell);
    unsigned b = khash_cell(ix, iy, iz);
    h->next[i] = h->head[b];
    h->head[b] = i;
  }
  return 0;
}

/* Inliers of b under R,t against the hashed cloud. *mean is the inlier average. */
static void hash_count(const ksync_hash *h, const ksync_v3 *b, int nb, const float R[9],
                       const float t[3], float thresh, int *inliers, float *mean) {
  int i, inl = 0;
  double acc = 0;
  float th2 = thresh * thresh;
  float cell = h->cell;
  for (i = 0; i < nb; i++) {
    ksync_v3 p = b[i];
    float best = 1e30f;
    int ix, iy, iz, dx, dy, dz;
    apply_rt(R, t, &p);
    ix = (int)floorf(p.x / cell);
    iy = (int)floorf(p.y / cell);
    iz = (int)floorf(p.z / cell);
    for (dz = -1; dz <= 1; dz++) {
      for (dy = -1; dy <= 1; dy++) {
        for (dx = -1; dx <= 1; dx++) {
          int j = h->head[khash_cell(ix + dx, iy + dy, iz + dz)];
          while (j >= 0) {
            float ddx = h->p[j].x - p.x;
            float ddy = h->p[j].y - p.y;
            float ddz = h->p[j].z - p.z;
            float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
            if (d2 < best) best = d2;
            j = h->next[j];
          }
        }
      }
    }
    if (best <= th2) {
      acc += sqrt((double)best);
      inl++;
    }
  }
  if (inliers) *inliers = inl;
  if (mean) *mean = inl ? (float)(acc / (double)inl) : 1.f;
}

/* One point per spatial cell so a scanline stride cannot skip a wall. */
static int voxel_keep(const ksync_v3 *p, int n, ksync_v3 *out, int maxn, float cell) {
  enum { H = 4096 };
  int cx[H], cy[H], cz[H], used[H];
  int i, kept = 0;
  memset(used, 0, sizeof used);
  for (i = 0; i < n && kept < maxn; i++) {
    int x = (int)floorf(p[i].x / cell);
    int y = (int)floorf(p[i].y / cell);
    int z = (int)floorf(p[i].z / cell);
    unsigned h = khash_cell(x, y, z) & (unsigned)(H - 1);
    unsigned step;
    for (step = 0; step < H; step++) {
      unsigned s = (h + step) & (unsigned)(H - 1);
      if (!used[s]) {
        used[s] = 1;
        cx[s] = x;
        cy[s] = y;
        cz[s] = z;
        out[kept++] = p[i];
        break;
      }
      if (cx[s] == x && cy[s] == y && cz[s] == z) break;
    }
  }
  return kept;
}

static int copy_pts(const cube_cloud *c, ksync_v3 **out) {
  ksync_v3 *p;
  int i;
  if (!c || c->n < 1 || !c->pt) return 0;
  p = (ksync_v3 *)malloc(sizeof(*p) * (size_t)c->n);
  if (!p) return -1;
  for (i = 0; i < c->n; i++) {
    p[i].x = c->pt[i].x;
    p[i].y = c->pt[i].y;
    p[i].z = c->pt[i].z;
  }
  *out = p;
  return c->n;
}

static int pose_better(int inl, float mean, int best_inl, float best_mean) {
  if (inl > best_inl) return 1;
  if (inl == best_inl && mean < best_mean - 1e-4f) return 1;
  return 0;
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

/* Returns 1 when this pose replaces *best. Yaw, pitch, and roll stay the
 * angles that built R. Decomposing R later drops a coupled roll. */
static int consider_pose(const ksync_hash *h, const ksync_v3 *b, int nb, float yaw, float pitch,
                         float roll, const float t[3], ksync_rigid *best, int *best_inl,
                         float *best_mean) {
  ksync_rigid trial;
  float mean = 1.f;
  int inl = 0;
  ksync_R_ypr(yaw, pitch, roll, trial.R);
  trial.t[0] = t[0];
  trial.t[1] = t[1];
  trial.t[2] = t[2];
  hash_count(h, b, nb, trial.R, trial.t, KSYNC_MATCH_M, &inl, &mean);
  if (!pose_better(inl, mean, *best_inl, *best_mean)) return 0;
  *best = trial;
  *best_inl = inl;
  *best_mean = mean;
  return 1;
}

/* Cameras that face each other do not share a centroid. Walk the baseline
 * from the seed without snapping translation back onto the centroids. */
static void descend_facing(const ksync_hash *h, const ksync_v3 *b, int nb, float *yaw,
                           float *pitch, float *roll, ksync_rigid *best, int *best_inl,
                           float *best_mean) {
  static const float rounds[3][4] = {
      {4.f, 4.f, 4.f, 0.12f},
      {2.f, 2.f, 2.f, 0.06f},
      {1.f, 1.f, 1.f, 0.03f},
  };
  int round, iter;
  for (round = 0; round < 3; round++) {
    for (iter = 0; iter < 8; iter++) {
      int ax, improved = 0;
      for (ax = 0; ax < 6; ax++) {
        int dir;
        float step = ax < 3 ? rounds[round][ax] : rounds[round][3];
        for (dir = -1; dir <= 1; dir += 2) {
          float y2 = *yaw, p2 = *pitch, r2 = *roll;
          float t2[3] = {best->t[0], best->t[1], best->t[2]};
          if (ax == 0) y2 += (float)dir * step;
          else if (ax == 1) p2 += (float)dir * step;
          else if (ax == 2) r2 += (float)dir * step;
          else
            t2[ax - 3] += (float)dir * step;
          if (fabsf(fabsf(y2) - 180.f) > 40.f) continue;
          if (consider_pose(h, b, nb, y2, p2, r2, t2, best, best_inl, best_mean)) {
            *yaw = y2;
            *pitch = p2;
            *roll = r2;
            improved = 1;
          }
        }
      }
      if (!improved) break;
    }
  }
}

int ksync_align(ksync_rigid *out, ksync_align_result *res, const cube_cloud *v1,
                const cube_cloud *v2) {
  ksync_v3 *pa = NULL, *pb = NULL, *qb = NULL;
  ksync_hash hash;
  ksync_v3 ca, cb;
  ksync_rigid cent, face, chosen;
  float cent_mean = 1.f, face_mean = 1.f, full_mean = 1.f, yaw;
  float face_yaw = 180.f, face_pitch = 0.f, face_roll = 0.f;
  int na, nb, nq, cent_inl = 0, face_inl = 0, full_inl = 0;
  int base, ys, ps, rs, accept_cent, accept_face, use_face;
  hash.next = NULL;
  if (res) memset(res, 0, sizeof *res);
  if (!out || !v1 || !v2 || v1->n < 30 || v2->n < 30) return -1;
  na = copy_pts(v1, &pa);
  nb = copy_pts(v2, &pb);
  if (na < 30 || nb < 30) goto fail;
  if (hash_build(&hash, pa, na, KSYNC_MATCH_M) != 0) goto fail;
  centroid(pa, na, &ca);
  centroid(pb, nb, &cb);
  if (nb > 2500) {
    qb = (ksync_v3 *)malloc(sizeof(*qb) * 2400);
    if (!qb) goto fail;
    nq = voxel_keep(pb, nb, qb, 2400, 0.10f);
    if (nq < 30) goto fail;
  } else {
    qb = pb;
    nq = nb;
  }

  /* Same-scene fit. Translation stays on the centroids so a symmetric
   * corridor cannot slide one view onto the other and call it aligned. */
  ksync_rigid_identity(&cent);
  ksync_rigid_identity(&face);
  for (base = 0; base <= 180; base += 180) {
    for (ys = -5; ys <= 5; ys++) {
      float yaw_i = (float)base + (float)ys * 4.f;
      for (ps = -2; ps <= 2; ps++) {
        float pitch_i = (float)ps * 8.f;
        for (rs = -1; rs <= 1; rs++) {
          ksync_rigid trial;
          float mean = 1.f;
          int inl = 0;
          rigid_from_ypr(yaw_i, pitch_i, (float)rs * 8.f, &ca, &cb, &trial);
          hash_count(&hash, qb, nq, trial.R, trial.t, KSYNC_MATCH_M, &inl, &mean);
          if (pose_better(inl, mean, cent_inl, cent_mean)) {
            cent = trial;
            cent_inl = inl;
            cent_mean = mean;
          }
        }
      }
    }
  }

  /* Facing pair: the other camera sits metres along +Z, turned around.
   * Left of one view is right of the other, so centroids do not correspond. */
  {
    static const float tz_s[] = {3.2f, 4.2f, 5.2f, 6.0f, 6.8f};
    static const float ty_s[] = {-0.3f, 0.15f, 0.55f};
    static const float tx_s[] = {-0.45f, -0.1f, 0.25f};
    static const float yaw_s[] = {172.f, 180.f, 188.f};
    static const float pitch_s[] = {0.f, 8.f, 16.f};
    static const float roll_s[] = {-8.f, 0.f, 8.f};
    int iz, iy, ix, iw, ip, ir;
    for (iz = 0; iz < 5; iz++) {
      for (iy = 0; iy < 3; iy++) {
        for (ix = 0; ix < 3; ix++) {
          for (iw = 0; iw < 3; iw++) {
            for (ip = 0; ip < 3; ip++) {
              for (ir = 0; ir < 3; ir++) {
                float t[3] = {tx_s[ix], ty_s[iy], tz_s[iz]};
                if (consider_pose(&hash, qb, nq, yaw_s[iw], pitch_s[ip], roll_s[ir], t, &face,
                                  &face_inl, &face_mean)) {
                  face_yaw = yaw_s[iw];
                  face_pitch = pitch_s[ip];
                  face_roll = roll_s[ir];
                }
              }
            }
          }
        }
      }
    }
  }
  if (face_inl > 0)
    descend_facing(&hash, qb, nq, &face_yaw, &face_pitch, &face_roll, &face, &face_inl,
                   &face_mean);

  hash_count(&hash, pb, nb, cent.R, cent.t, KSYNC_MATCH_M, &cent_inl, &cent_mean);
  hash_count(&hash, pb, nb, face.R, face.t, KSYNC_MATCH_M, &face_inl, &face_mean);
  ksync_rigid_ypr(&face, &yaw, NULL);
  /* Majority overlap is one scene seen twice (the synthetic pair, or two
   * cameras on the same wall). Anything thinner has to be the facing room:
   * yaw near 180 and the cameras metres apart. A close stack is not a map. */
  accept_cent = cent_inl * 2 >= nb && cent_mean <= 0.08f;
  accept_face = face_inl * 6 >= nb && face_mean <= KSYNC_MATCH_M &&
                sqrtf(face.t[0] * face.t[0] + face.t[1] * face.t[1] + face.t[2] * face.t[2]) > 1.5f &&
                fabsf(fabsf(yaw) - 180.f) < 25.f;
  use_face = accept_face && (!accept_cent || face_inl > cent_inl);
  if (use_face) {
    chosen = face;
    full_inl = face_inl;
    full_mean = face_mean;
  } else if (accept_cent) {
    chosen = cent;
    full_inl = cent_inl;
    full_mean = cent_mean;
  } else {
    if (res) {
      int show_face = face_inl >= cent_inl;
      res->ok = 0;
      res->inliers = show_face ? face_inl : cent_inl;
      res->residual_m = show_face ? face_mean : cent_mean;
      ksync_rigid_ypr(show_face ? &face : &cent, &res->yaw_deg, &res->pitch_deg);
    }
    goto fail;
  }
  *out = chosen;
  if (res) {
    res->ok = 1;
    res->residual_m = full_mean;
    res->inliers = full_inl;
    ksync_rigid_ypr(&chosen, &res->yaw_deg, &res->pitch_deg);
  }
  free(hash.next);
  free(pa);
  free(pb);
  if (qb != pb) free(qb);
  return 0;

fail:
  free(hash.next);
  free(pa);
  free(pb);
  if (qb && qb != pb) free(qb);
  return -1;
}
