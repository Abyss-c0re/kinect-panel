#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "cube_kinect_cloud.h"
#include "cube_paths.h"
#include <stdint.h>

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float raw_to_m(uint16_t d) {
  if (d < 1 || d > 2046) return 0.f;
  return 0.1236f * tanf(d / 2842.5f + 1.1863f);
}

static int mkdir_p(const char *path) {
  char buf[512];
  size_t n = strlen(path);
  if (n >= sizeof buf) return -1;
  memcpy(buf, path, n + 1);
  for (char *p = buf + 1; *p; p++) {
    if (*p == '/') {
      *p = 0;
      if (mkdir(buf, 0755) < 0 && errno != EEXIST) return -1;
      *p = '/';
    }
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

void cube_cloud_init(cube_cloud *c) {
  if (!c) return;
  memset(c, 0, sizeof *c);
}

void cube_cloud_clear(cube_cloud *c) {
  if (!c) return;
  c->n = 0;
  c->ok = 0;
  c->ts = 0;
  c->err[0] = 0;
}

void cube_cloud_free(cube_cloud *c) {
  if (!c) return;
  free(c->pt);
  c->pt = NULL;
  c->cap = 0;
  cube_cloud_clear(c);
}

static int cloud_reserve(cube_cloud *c, int need) {
  cube_cloud_pt *p;
  int cap = c->cap;
  if (need <= cap) return 0;
  if (cap < 1024) cap = 1024;
  while (cap < need) cap *= 2;
  if (cap > CUBE_CLOUD_MAX) cap = CUBE_CLOUD_MAX;
  if (need > cap) need = cap;
  p = (cube_cloud_pt *)realloc(c->pt, (size_t)cap * sizeof *p);
  if (!p) return -1;
  c->pt = p;
  c->cap = cap;
  return 0;
}

void cube_cloud_free_holders(void) {
  static const char *names[] = {
      "cube_daemon", "kinect_ir_view", "freenect-glview",
      "kinect_lizard_track", NULL};
  DIR *d = opendir("/proc");
  struct dirent *e;
  if (!d) return;
  while ((e = readdir(d))) {
    char path[64], comm[64];
    FILE *f;
    pid_t pid;
    int i, hit = 0;
    char *end = NULL;
    if (e->d_name[0] < '1' || e->d_name[0] > '9') continue;
    pid = (pid_t)strtol(e->d_name, &end, 10);
    if (!end || *end || pid <= 1 || pid == getpid()) continue;
    snprintf(path, sizeof path, "/proc/%d/comm", (int)pid);
    f = fopen(path, "r");
    if (!f) continue;
    if (!fgets(comm, sizeof comm, f)) {
      fclose(f);
      continue;
    }
    fclose(f);
    comm[strcspn(comm, "\n")] = 0;
    for (i = 0; names[i]; i++) {
      if (strcmp(comm, names[i]) == 0) {
        hit = 1;
        break;
      }
    }
    if (hit) kill(pid, SIGTERM);
  }
  closedir(d);
  usleep(300000);
}

void cube_cloud_tune_default(cube_cloud_tune *t) {
  if (!t) return;
  t->stride = 3;
  t->z_near = 0.50f;
  t->z_far = 3.50f;
  t->color = 0;
  t->dot = 2;
}

void cube_cloud_color_point(cube_cloud_pt *p, const cube_cloud_tune *t) {
  float tn, tf, u;
  if (!p || !t) return;
  tn = t->z_near;
  tf = t->z_far;
  if (tf <= tn + 0.05f) tf = tn + 0.05f;
  if (t->color == 1) {
    /* height: y up, seated/standing band */
    u = (p->y + 0.4f) / 1.4f;
  } else {
    u = (p->z - tn) / (tf - tn);
  }
  if (u < 0) u = 0;
  if (u > 1) u = 1;
  if (t->color == 2) {
    uint8_t g = (uint8_t)(40 + u * 200);
    p->r = p->g = p->b = g;
  } else if (t->color == 1) {
    p->r = (uint8_t)(40 + u * 80);
    p->g = (uint8_t)(180 - u * 80);
    p->b = (uint8_t)(90 + (1.f - u) * 80);
  } else {
    p->r = 242;
    p->g = (uint8_t)(38 + u * 90);
    p->b = (uint8_t)(71 + (1.f - u) * 50);
  }
}

int cube_cloud_from_depth_ex(cube_cloud *c, const uint16_t *depth, const cube_cloud_tune *t) {
  const int W = 640, H = 480;
  const float fx = 594.21f, fy = 591.04f, cx = 339.5f, cy = 242.7f;
  cube_cloud_tune def;
  int stride, v, u, n = 0;
  float zn, zf;
  if (!c || !depth) return -1;
  if (!t) {
    cube_cloud_tune_default(&def);
    t = &def;
  }
  cube_cloud_clear(c);
  stride = t->stride;
  if (stride < 1) stride = 1;
  if (stride > 10) stride = 10;
  zn = t->z_near;
  zf = t->z_far;
  if (zn < 0.3f) zn = 0.3f;
  if (zf < zn + 0.1f) zf = zn + 0.1f;
  if (cloud_reserve(c, (W / stride) * (H / stride) + 8) < 0) {
    snprintf(c->err, sizeof c->err, "oom");
    return -1;
  }
  for (v = 0; v < H && n < c->cap; v += stride) {
    for (u = 0; u < W && n < c->cap; u += stride) {
      float z = raw_to_m(depth[v * W + u]);
      cube_cloud_pt *p;
      if (z < zn || z > zf) continue;
      p = &c->pt[n++];
      p->x = (u - cx) * z / fx;
      p->y = -((v - cy) * z / fy);
      p->z = z;
      cube_cloud_color_point(p, t);
    }
  }
  c->n = n;
  c->ok = n > 0;
  if (!c->ok) snprintf(c->err, sizeof c->err, "empty cloud");
  return c->ok ? 0 : -1;
}

int cube_cloud_from_depth(cube_cloud *c, const uint16_t *depth, int stride) {
  cube_cloud_tune t;
  cube_cloud_tune_default(&t);
  t.stride = stride;
  return cube_cloud_from_depth_ex(c, depth, &t);
}

int cube_cloud_capture(cube_cloud *c, int stride) {
  (void)stride;
  if (!c) return -1;
  snprintf(c->err, sizeof c->err, "sync capture unused");
  return -1;
}

void cube_cloud_release(void) {}

int cube_cloud_load_xyz(cube_cloud *c, const char *path, int max_pts) {
  FILE *f;
  char line[256];
  int n = 0;
  if (!c || !path) return -1;
  cube_cloud_clear(c);
  if (max_pts <= 0 || max_pts > CUBE_CLOUD_MAX) max_pts = CUBE_CLOUD_MAX;
  f = fopen(path, "r");
  if (!f) {
    snprintf(c->err, sizeof c->err, "cannot open %s", path);
    return -1;
  }
  if (cloud_reserve(c, max_pts) < 0) {
    fclose(f);
    snprintf(c->err, sizeof c->err, "oom");
    return -1;
  }
  while (fgets(line, sizeof line, f) && n < c->cap && n < max_pts) {
    float x, y, z;
    int r = 242, g = 38, b = 71;
    int got = sscanf(line, "%f %f %f %d %d %d", &x, &y, &z, &r, &g, &b);
    if (got < 3) continue;
    c->pt[n].x = x;
    c->pt[n].y = y;
    c->pt[n].z = z;
    c->pt[n].r = (uint8_t)(r < 0 ? 0 : r > 255 ? 255 : r);
    c->pt[n].g = (uint8_t)(g < 0 ? 0 : g > 255 ? 255 : g);
    c->pt[n].b = (uint8_t)(b < 0 ? 0 : b > 255 ? 255 : b);
    n++;
  }
  fclose(f);
  c->n = n;
  c->ok = n > 0;
  snprintf(c->path, sizeof c->path, "%s", path);
  if (!c->ok) snprintf(c->err, sizeof c->err, "no points in %s", path);
  return c->ok ? 0 : -1;
}

int cube_cloud_write_xyz(const cube_cloud *c, const char *path) {
  FILE *f;
  int i;
  if (!c || !path) return -1;
  if (parent_mkdir(path) < 0) return -1;
  f = fopen(path, "w");
  if (!f) return -1;
  for (i = 0; i < c->n; i++) {
    const cube_cloud_pt *p = &c->pt[i];
    fprintf(f, "%.5f %.5f %.5f %d %d %d\n", p->x, p->y, p->z, p->r, p->g, p->b);
  }
  fclose(f);
  return 0;
}

int cube_cloud_publish(const cube_cloud *c) {
  char a[400], b[400];
  if (!c) return -1;
  if (cube_cloud_write_xyz(c, CUBE_KINECT_XYZ) < 0) return -1;
  snprintf(a, sizeof a, "%s/clouds/kinect_latest.xyz", CUBE_STATE_DIR);
  snprintf(b, sizeof b, "%s/clouds/cloud_latest.xyz", CUBE_STATE_DIR);
  cube_cloud_write_xyz(c, a);
  cube_cloud_write_xyz(c, b);
  return 0;
}

int cube_cloud_render_rgb(const cube_cloud *c, uint8_t *rgb, int w, int h,
                          float elev_deg, float azim_deg, float dist) {
  return cube_cloud_render_rgb_ex(c, rgb, w, h, elev_deg, azim_deg, dist, 2);
}

int cube_cloud_render_rgb_ex(const cube_cloud *c, uint8_t *rgb, int w, int h,
                             float elev_deg, float azim_deg, float dist, int dot) {
  float *zbuf;
  int i, x, y, npix, rad;
  float elev, azim, se, ce, sa, ca;
  float cx = 0, cy = 0, cz = 0;
  if (!c || !rgb || w < 8 || h < 8) return -1;
  rad = dot < 1 ? 1 : (dot > 4 ? 4 : dot);
  npix = w * h;
  memset(rgb, 0, (size_t)npix * 3);
  zbuf = (float *)malloc((size_t)npix * sizeof(float));
  if (!zbuf) return -1;
  for (i = 0; i < npix; i++) zbuf[i] = 1e9f;
  if (c->n > 0) {
    for (i = 0; i < c->n; i++) {
      cx += c->pt[i].x;
      cy += c->pt[i].y;
      cz += c->pt[i].z;
    }
    cx /= (float)c->n;
    cy /= (float)c->n;
    cz /= (float)c->n;
  }
  elev = elev_deg * (float)M_PI / 180.f;
  azim = azim_deg * (float)M_PI / 180.f;
  se = sinf(elev);
  ce = cosf(elev);
  sa = sinf(azim);
  ca = cosf(azim);
  if (dist < 0.4f) dist = 0.4f;
  for (i = 0; i < c->n; i++) {
    float px = c->pt[i].x - cx;
    float py = c->pt[i].y - cy;
    float pz = c->pt[i].z - cz;
    float vx = px * ca + pz * sa;
    float vz = -px * sa * ce + py * se + pz * ca * ce;
    float vy = px * sa * se + py * ce - pz * ca * se;
    float zc = dist - vz;
    float inv, sx, sy;
    int ix, iy, k, dx, dy;
    if (zc < 0.15f) continue;
    inv = 1.8f / zc;
    sx = vx * inv;
    sy = vy * inv;
    ix = (int)((sx * 0.5f + 0.5f) * (float)w);
    iy = (int)((0.5f - sy * 0.5f) * (float)h);
    for (dy = -rad; dy <= rad; dy++) {
      for (dx = -rad; dx <= rad; dx++) {
        int xx = ix + dx, yy = iy + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        k = yy * w + xx;
        if (zc >= zbuf[k]) continue;
        zbuf[k] = zc;
        rgb[k * 3 + 0] = c->pt[i].r;
        rgb[k * 3 + 1] = c->pt[i].g;
        rgb[k * 3 + 2] = c->pt[i].b;
      }
    }
  }
  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      int k = (y * w + x) * 3;
      if (rgb[k] == 0 && rgb[k + 1] == 0 && rgb[k + 2] == 0) {
        rgb[k] = 8;
        rgb[k + 1] = 2;
        rgb[k + 2] = 4;
      }
    }
  }
  free(zbuf);
  return 0;
}

int cube_cloud_write_ppm(const cube_cloud *c, const char *path, int w, int h,
                         float elev_deg, float azim_deg, float dist) {
  uint8_t *pix;
  FILE *f;
  if (!c || !path || w < 8 || h < 8) return -1;
  pix = (uint8_t *)malloc((size_t)w * (size_t)h * 3);
  if (!pix) return -1;
  if (cube_cloud_render_rgb(c, pix, w, h, elev_deg, azim_deg, dist) < 0) {
    free(pix);
    return -1;
  }
  if (parent_mkdir(path) < 0) {
    free(pix);
    return -1;
  }
  f = fopen(path, "wb");
  if (!f) {
    free(pix);
    return -1;
  }
  fprintf(f, "P6\n%d %d\n255\n", w, h);
  fwrite(pix, 3, (size_t)w * (size_t)h, f);
  fclose(f);
  free(pix);
  return 0;
}
