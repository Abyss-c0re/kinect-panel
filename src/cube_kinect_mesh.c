#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "cube_kinect_mesh.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEP_W 640
#define DEP_H 480
#define GW 160
#define GH 120
#define GSTR 4

static float raw_to_m(uint16_t d) {
  if (d < 1 || d > 2046) return 0.f;
  return 0.1236f * tanf(d / 2842.5f + 1.1863f);
}

static const uint8_t PAL[][3] = {
    {96, 74, 52},    /* floor */
    {72, 118, 158},  /* wall */
    {228, 48, 78},   /* person */
    {186, 142, 58},  /* furniture */
    {68, 148, 96},   {148, 88, 168}, {58, 138, 148}, {200, 110, 70},
    {110, 110, 170}, {160, 70, 90},  {90, 160, 70},  {70, 90, 160},
};

void cube_mesh_init(cube_mesh *m) {
  if (!m) return;
  memset(m, 0, sizeof *m);
}

void cube_mesh_clear(cube_mesh *m) {
  if (!m) return;
  m->nv = m->ni = m->nobj = 0;
  m->ok = 0;
  m->err[0] = 0;
}

void cube_mesh_free(cube_mesh *m) {
  if (!m) return;
  free(m->v);
  free(m->idx);
  memset(m, 0, sizeof *m);
}

static int gfind(int *p, int a) {
  while (p[a] != a) {
    p[a] = p[p[a]];
    a = p[a];
  }
  return a;
}

static void obj_color(int kind, int id, uint8_t *r, uint8_t *g, uint8_t *b) {
  int k = kind;
  if (k < 0 || k > 2) k = 3 + (id % 9);
  if (k >= (int)(sizeof PAL / sizeof PAL[0])) k = 3 + (id % 9);
  *r = PAL[k][0];
  *g = PAL[k][1];
  *b = PAL[k][2];
}

int cube_mesh_from_depth(cube_mesh *m, const uint16_t *depth, const cube_cloud_tune *t) {
  const float fx = 594.21f, fy = 591.04f, cx = 339.5f, cy = 242.7f;
  cube_cloud_tune def;
  float zn, zf, zthr = 0.09f;
  int gx[GH][GW];
  float gz[GH][GW];
  int parent[GW * GH];
  int oc[CUBE_MESH_MAX_OBJ];
  float oymin[CUBE_MESH_MAX_OBJ], oymax[CUBE_MESH_MAX_OBJ];
  float oxmin[CUBE_MESH_MAX_OBJ], oxmax[CUBE_MESH_MAX_OBJ];
  float ozmin[CUBE_MESH_MAX_OBJ], ozmax[CUBE_MESH_MAX_OBJ];
  int kind[CUBE_MESH_MAX_OBJ];
  int i, j, nv, nobj, labn;
  int label_of[GW * GH];

  if (!m || !depth) return -1;
  if (!t) {
    cube_cloud_tune_default(&def);
    t = &def;
  }
  zn = t->z_near < 0.3f ? 0.3f : t->z_near;
  zf = t->z_far < zn + 0.1f ? zn + 0.1f : t->z_far;

  if (!m->v) {
    m->v = (cube_mesh_v *)malloc((size_t)CUBE_MESH_MAX_V * sizeof *m->v);
    m->idx = (unsigned *)malloc((size_t)CUBE_MESH_MAX_I * sizeof *m->idx);
    if (!m->v || !m->idx) {
      snprintf(m->err, sizeof m->err, "oom");
      return -1;
    }
  }
  cube_mesh_clear(m);

  nv = 0;
  for (j = 0; j < GH; j++) {
    for (i = 0; i < GW; i++) {
      int u = i * GSTR, v = j * GSTR;
      float z = raw_to_m(depth[v * DEP_W + u]);
      gx[j][i] = -1;
      gz[j][i] = 0;
      if (z < zn || z > zf) continue;
      if (nv >= CUBE_MESH_MAX_V) break;
      m->v[nv].x = (u - cx) * z / fx;
      m->v[nv].y = -((v - cy) * z / fy);
      m->v[nv].z = z;
      m->v[nv].nx = 0;
      m->v[nv].ny = 1;
      m->v[nv].nz = 0;
      m->v[nv].obj = 0;
      gx[j][i] = nv;
      gz[j][i] = z;
      nv++;
    }
  }
  m->nv = nv;
  if (nv < 8) {
    snprintf(m->err, sizeof m->err, "too few mesh verts");
    return -1;
  }

  for (i = 0; i < nv; i++) parent[i] = i;

  for (j = 0; j < GH; j++) {
    for (i = 0; i < GW; i++) {
      int a = gx[j][i];
      if (a < 0) continue;
      if (i + 1 < GW && gx[j][i + 1] >= 0 && fabsf(gz[j][i] - gz[j][i + 1]) < zthr) {
        int pa = gfind(parent, a), pb = gfind(parent, gx[j][i + 1]);
        if (pa != pb) parent[pb] = pa;
      }
      if (j + 1 < GH && gx[j + 1][i] >= 0 && fabsf(gz[j][i] - gz[j + 1][i]) < zthr) {
        int pa = gfind(parent, a), pb = gfind(parent, gx[j + 1][i]);
        if (pa != pb) parent[pb] = pa;
      }
    }
  }

  for (i = 0; i < nv; i++) label_of[i] = -1;
  nobj = 0;
  memset(oc, 0, sizeof oc);
  for (i = 0; i < CUBE_MESH_MAX_OBJ; i++) {
    oymin[i] = 1e9f;
    oymax[i] = -1e9f;
    oxmin[i] = 1e9f;
    oxmax[i] = -1e9f;
    ozmin[i] = 1e9f;
    ozmax[i] = -1e9f;
  }
  for (i = 0; i < nv; i++) {
    int root = gfind(parent, i), lab;
    if (label_of[root] < 0) {
      if (nobj >= CUBE_MESH_MAX_OBJ) continue;
      label_of[root] = nobj++;
    }
    lab = label_of[root];
    label_of[i] = lab;
    m->v[i].obj = lab;
    oc[lab]++;
    if (m->v[i].y < oymin[lab]) oymin[lab] = m->v[i].y;
    if (m->v[i].y > oymax[lab]) oymax[lab] = m->v[i].y;
    if (m->v[i].x < oxmin[lab]) oxmin[lab] = m->v[i].x;
    if (m->v[i].x > oxmax[lab]) oxmax[lab] = m->v[i].x;
    if (m->v[i].z < ozmin[lab]) ozmin[lab] = m->v[i].z;
    if (m->v[i].z > ozmax[lab]) ozmax[lab] = m->v[i].z;
  }
  m->nobj = nobj;

  {
    float floor_y = 1e9f;
    for (i = 0; i < nobj; i++)
      if (oc[i] > 40 && oymin[i] < floor_y) floor_y = oymin[i];
    if (floor_y > 1e8f) floor_y = -0.9f;
    for (i = 0; i < nobj; i++) {
      float h = oymax[i] - oymin[i];
      float w = oxmax[i] - oxmin[i];
      float d = ozmax[i] - ozmin[i];
      if (d > w) w = d;
      kind[i] = 3;
      if (oc[i] > 80 && h < 0.28f && (oymax[i] - floor_y) < 0.35f)
        kind[i] = 0; /* floor */
      else if (oc[i] > 80 && h > 1.1f && w > 1.1f)
        kind[i] = 1; /* wall */
      else if (h >= 0.85f && h <= 2.15f && w < 0.95f && oc[i] >= 20)
        kind[i] = 2; /* person */
    }
  }

  for (i = 0; i < nv; i++) {
    int lab = m->v[i].obj;
    if (lab < 0 || lab >= nobj) lab = 0;
    obj_color(kind[lab], lab, &m->v[i].r, &m->v[i].g, &m->v[i].b);
    if (t->color == 1) {
      float u = (m->v[i].y + 0.4f) / 1.4f;
      if (u < 0) u = 0;
      if (u > 1) u = 1;
      m->v[i].r = (uint8_t)(40 + u * 80);
      m->v[i].g = (uint8_t)(180 - u * 80);
      m->v[i].b = (uint8_t)(90 + (1.f - u) * 80);
    } else if (t->color == 2) {
      uint8_t g = (uint8_t)(50 + m->v[i].z / zf * 180);
      m->v[i].r = m->v[i].g = m->v[i].b = g;
    } else if (t->color == 0) {
      float u = (m->v[i].z - zn) / (zf - zn);
      if (u < 0) u = 0;
      if (u > 1) u = 1;
      m->v[i].r = 242;
      m->v[i].g = (uint8_t)(38 + u * 90);
      m->v[i].b = (uint8_t)(71 + (1.f - u) * 50);
    }
  }

  labn = 0;
  for (j = 0; j < GH - 1; j++) {
    for (i = 0; i < GW - 1; i++) {
      int a = gx[j][i], b = gx[j][i + 1], c = gx[j + 1][i], d = gx[j + 1][i + 1];
      int nval = (a >= 0) + (b >= 0) + (c >= 0) + (d >= 0);
      if (nval < 3 || labn + 6 > CUBE_MESH_MAX_I) continue;
      if (a >= 0 && b >= 0 && c >= 0 && fabsf(gz[j][i] - gz[j][i + 1]) < zthr &&
          fabsf(gz[j][i] - gz[j + 1][i]) < zthr) {
        m->idx[labn++] = (unsigned)a;
        m->idx[labn++] = (unsigned)c;
        m->idx[labn++] = (unsigned)b;
      }
      if (b >= 0 && c >= 0 && d >= 0 && fabsf(gz[j][i + 1] - gz[j + 1][i + 1]) < zthr &&
          fabsf(gz[j + 1][i] - gz[j + 1][i + 1]) < zthr) {
        m->idx[labn++] = (unsigned)b;
        m->idx[labn++] = (unsigned)c;
        m->idx[labn++] = (unsigned)d;
      }
    }
  }
  m->ni = labn;

  for (i = 0; i < labn; i += 3) {
    cube_mesh_v *a = &m->v[m->idx[i]], *b = &m->v[m->idx[i + 1]], *c = &m->v[m->idx[i + 2]];
    float ex = b->x - a->x, ey = b->y - a->y, ez = b->z - a->z;
    float fx = c->x - a->x, fy = c->y - a->y, fz = c->z - a->z;
    float nx = ey * fz - ez * fy, ny = ez * fx - ex * fz, nz = ex * fy - ey * fx;
    a->nx += nx;
    a->ny += ny;
    a->nz += nz;
    b->nx += nx;
    b->ny += ny;
    b->nz += nz;
    c->nx += nx;
    c->ny += ny;
    c->nz += nz;
  }
  for (i = 0; i < nv; i++) {
    float n = sqrtf(m->v[i].nx * m->v[i].nx + m->v[i].ny * m->v[i].ny +
                    m->v[i].nz * m->v[i].nz);
    if (n < 1e-6f) {
      m->v[i].nx = 0;
      m->v[i].ny = 1;
      m->v[i].nz = 0;
    } else {
      m->v[i].nx /= n;
      m->v[i].ny /= n;
      m->v[i].nz /= n;
    }
  }
  m->ok = m->ni > 0;
  if (!m->ok) snprintf(m->err, sizeof m->err, "no triangles");
  return m->ok ? 0 : -1;
}

static void putpx(uint8_t *rgb, float *zbuf, int w, int h, int x, int y, float z,
                  uint8_t r, uint8_t g, uint8_t b) {
  int k;
  if (x < 0 || y < 0 || x >= w || y >= h) return;
  k = y * w + x;
  if (z >= zbuf[k]) return;
  zbuf[k] = z;
  rgb[k * 3 + 0] = r;
  rgb[k * 3 + 1] = g;
  rgb[k * 3 + 2] = b;
}

int cube_mesh_render_rgb(const cube_mesh *m, uint8_t *rgb, int w, int h, float elev_deg,
                         float azim_deg, float dist) {
  float *zbuf;
  int i, t, npix;
  float elev, azim, se, ce, sa, ca, cx = 0, cy = 0, cz = 0;
  if (!m || !rgb || w < 8 || h < 8 || m->nv < 3) return -1;
  npix = w * h;
  memset(rgb, 8, (size_t)npix * 3);
  zbuf = (float *)malloc((size_t)npix * sizeof(float));
  if (!zbuf) return -1;
  for (i = 0; i < npix; i++) zbuf[i] = 1e9f;
  for (i = 0; i < m->nv; i++) {
    cx += m->v[i].x;
    cy += m->v[i].y;
    cz += m->v[i].z;
  }
  cx /= (float)m->nv;
  cy /= (float)m->nv;
  cz /= (float)m->nv;
  elev = elev_deg * (float)M_PI / 180.f;
  azim = azim_deg * (float)M_PI / 180.f;
  se = sinf(elev);
  ce = cosf(elev);
  sa = sinf(azim);
  ca = cosf(azim);
  if (dist < 0.4f) dist = 0.4f;

  for (t = 0; t < m->ni; t += 3) {
    float sx[3], sy[3], sz[3], lum;
    int ix[3], iy[3], minx, maxx, miny, maxy, x, y;
    const cube_mesh_v *tv[3];
    float nx, ny, nz, nlen;
    for (i = 0; i < 3; i++) {
      const cube_mesh_v *p = &m->v[m->idx[t + i]];
      float px = p->x - cx, py = p->y - cy, pz = p->z - cz;
      float vx = px * ca + pz * sa;
      float vz = -px * sa * ce + py * se + pz * ca * ce;
      float vy = px * sa * se + py * ce - pz * ca * se;
      float zc = dist - vz;
      float inv;
      tv[i] = p;
      if (zc < 0.15f) goto next_tri;
      inv = 1.f / zc;
      sx[i] = (float)w * 0.5f + vx * (float)h * 0.55f * inv;
      sy[i] = (float)h * 0.5f - vy * (float)h * 0.55f * inv;
      sz[i] = zc;
    }
    nx = tv[0]->nx + tv[1]->nx + tv[2]->nx;
    ny = tv[0]->ny + tv[1]->ny + tv[2]->ny;
    nz = tv[0]->nz + tv[1]->nz + tv[2]->nz;
    nlen = sqrtf(nx * nx + ny * ny + nz * nz);
    if (nlen > 1e-6f) {
      nx /= nlen;
      ny /= nlen;
      nz /= nlen;
    }
    lum = 0.32f + 0.68f * fmaxf(0.f, nx * 0.35f + ny * 0.82f + nz * 0.45f);
    if (lum < 0.15f) lum = 0.15f;
    if (lum > 1.f) lum = 1.f;
    minx = (int)fminf(sx[0], fminf(sx[1], sx[2]));
    maxx = (int)fmaxf(sx[0], fmaxf(sx[1], sx[2]));
    miny = (int)fminf(sy[0], fminf(sy[1], sy[2]));
    maxy = (int)fmaxf(sy[0], fmaxf(sy[1], sy[2]));
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx >= w) maxx = w - 1;
    if (maxy >= h) maxy = h - 1;
    for (i = 0; i < 3; i++) {
      ix[i] = (int)sx[i];
      iy[i] = (int)sy[i];
    }
    {
      float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
      if (fabsf(area) < 0.5f) goto next_tri;
      for (y = miny; y <= maxy; y++) {
        for (x = minx; x <= maxx; x++) {
          float px = (float)x + 0.5f, py = (float)y + 0.5f;
          float w0 = (sx[1] - px) * (sy[2] - py) - (sx[2] - px) * (sy[1] - py);
          float w1 = (sx[2] - px) * (sy[0] - py) - (sx[0] - px) * (sy[2] - py);
          float w2 = (sx[0] - px) * (sy[1] - py) - (sx[1] - px) * (sy[0] - py);
          if (area < 0) {
            w0 = -w0;
            w1 = -w1;
            w2 = -w2;
            area = -area;
          }
          if (w0 < 0 || w1 < 0 || w2 < 0) continue;
          {
            float inva = 1.f / area;
            float z = (w0 * sz[0] + w1 * sz[1] + w2 * sz[2]) * inva;
            uint8_t r = (uint8_t)(tv[0]->r * lum);
            uint8_t g = (uint8_t)(tv[0]->g * lum);
            uint8_t b = (uint8_t)(tv[0]->b * lum);
            putpx(rgb, zbuf, w, h, x, y, z, r, g, b);
          }
        }
      }
    }
  next_tri:;
  }
  free(zbuf);
  return 0;
}
