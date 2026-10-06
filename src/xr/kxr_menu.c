#include "kxr_menu.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

_Static_assert(KXR_MENU_Y0 + KXR_MENU_ROWS * KXR_MENU_PITCH <= KXR_MENU_HGT - 36,
               "menu rows overlap the footer");

/* 5×7 column-major, bit0 = top. Space..Z plus a few marks; lowercase → upper. */
static const uint8_t F5[][5] = {
    {0, 0, 0, 0, 0},                   /* space */
    {0, 0, 0x5f, 0, 0},                /* ! */
    {0, 0x07, 0, 0x07, 0},             /* " */
    {0x14, 0x7f, 0x14, 0x7f, 0x14},    /* # */
    {0x24, 0x2a, 0x7f, 0x2a, 0x12},    /* $ */
    {0x23, 0x13, 0x08, 0x64, 0x62},    /* % */
    {0x36, 0x49, 0x55, 0x22, 0x50},    /* & */
    {0, 0x05, 0x03, 0, 0},             /* ' */
    {0, 0x1c, 0x22, 0x41, 0},          /* ( */
    {0, 0x41, 0x22, 0x1c, 0},          /* ) */
    {0x14, 0x08, 0x3e, 0x08, 0x14},    /* * */
    {0x08, 0x08, 0x3e, 0x08, 0x08},    /* + */
    {0, 0x50, 0x30, 0, 0},             /* , */
    {0x08, 0x08, 0x08, 0x08, 0x08},    /* - */
    {0, 0x60, 0x60, 0, 0},             /* . */
    {0x20, 0x10, 0x08, 0x04, 0x02},    /* / */
    {0x3e, 0x51, 0x49, 0x45, 0x3e},    /* 0 */
    {0, 0x42, 0x7f, 0x40, 0},          /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46},    /* 2 */
    {0x21, 0x41, 0x45, 0x4b, 0x31},    /* 3 */
    {0x18, 0x14, 0x12, 0x7f, 0x10},    /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39},    /* 5 */
    {0x3c, 0x4a, 0x49, 0x49, 0x30},    /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03},    /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36},    /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1e},    /* 9 */
    {0, 0x36, 0x36, 0, 0},             /* : */
    {0, 0x56, 0x36, 0, 0},             /* ; */
    {0x08, 0x14, 0x22, 0x41, 0},       /* < */
    {0x14, 0x14, 0x14, 0x14, 0x14},    /* = */
    {0, 0x41, 0x22, 0x14, 0x08},       /* > */
    {0x02, 0x01, 0x51, 0x09, 0x06},    /* ? */
    {0x32, 0x49, 0x79, 0x41, 0x3e},    /* @ */
    {0x7e, 0x11, 0x11, 0x11, 0x7e},    /* A */
    {0x7f, 0x49, 0x49, 0x49, 0x36},    /* B */
    {0x3e, 0x41, 0x41, 0x41, 0x22},    /* C */
    {0x7f, 0x41, 0x41, 0x22, 0x1c},    /* D */
    {0x7f, 0x49, 0x49, 0x49, 0x41},    /* E */
    {0x7f, 0x09, 0x09, 0x09, 0x01},    /* F */
    {0x3e, 0x41, 0x49, 0x49, 0x7a},    /* G */
    {0x7f, 0x08, 0x08, 0x08, 0x7f},    /* H */
    {0, 0x41, 0x7f, 0x41, 0},          /* I */
    {0x20, 0x40, 0x41, 0x3f, 0x01},    /* J */
    {0x7f, 0x08, 0x14, 0x22, 0x41},    /* K */
    {0x7f, 0x40, 0x40, 0x40, 0x40},    /* L */
    {0x7f, 0x02, 0x0c, 0x02, 0x7f},    /* M */
    {0x7f, 0x04, 0x08, 0x10, 0x7f},    /* N */
    {0x3e, 0x41, 0x41, 0x41, 0x3e},    /* O */
    {0x7f, 0x09, 0x09, 0x09, 0x06},    /* P */
    {0x3e, 0x41, 0x51, 0x21, 0x5e},    /* Q */
    {0x7f, 0x09, 0x19, 0x29, 0x46},    /* R */
    {0x46, 0x49, 0x49, 0x49, 0x31},    /* S */
    {0x01, 0x01, 0x7f, 0x01, 0x01},    /* T */
    {0x3f, 0x40, 0x40, 0x40, 0x3f},    /* U */
    {0x1f, 0x20, 0x40, 0x20, 0x1f},    /* V */
    {0x3f, 0x40, 0x38, 0x40, 0x3f},    /* W */
    {0x63, 0x14, 0x08, 0x14, 0x63},    /* X */
    {0x07, 0x08, 0x70, 0x08, 0x07},    /* Y */
    {0x61, 0x51, 0x49, 0x45, 0x43},    /* Z */
};

static kxr_quat qn(kxr_quat q) {
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

static kxr_vec3 qrot(kxr_quat q, kxr_vec3 v) {
  kxr_vec3 t, u, o;
  q = qn(q);
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

static kxr_quat qyaw_from_fwd(kxr_vec3 f) {
  float yaw, h;
  kxr_quat q;
  h = sqrtf(f.x * f.x + f.z * f.z);
  if (h < 1e-5f) {
    q.x = q.y = q.z = 0;
    q.w = 1;
    return q;
  }
  f.x /= h;
  f.z /= h;
  yaw = atan2f(f.x, -f.z);
  q.x = 0;
  q.y = sinf(yaw * 0.5f);
  q.z = 0;
  q.w = cosf(yaw * 0.5f);
  return q;
}

void kxr_menu_init(kxr_menu *m) {
  if (!m) return;
  memset(m, 0, sizeof *m);
  m->width_m = 0.84f;
  m->height_m = 0.84f * ((float)KXR_MENU_HGT / (float)KXR_MENU_W);
  m->rgba = (uint8_t *)calloc((size_t)KXR_MENU_W * KXR_MENU_HGT * 4, 1);
  m->dirty = 1;
  snprintf(m->status, sizeof m->status, "A/+  B/-  stick X tilt both ways · menu close");
}

void kxr_menu_free(kxr_menu *m) {
  if (!m) return;
  free(m->rgba);
  memset(m, 0, sizeof *m);
}

void kxr_menu_recenter(kxr_menu *m, const kxr_pose *head) {
  kxr_vec3 fwd = {0, 0, -1}, f;
  if (!m || !head) return;
  f = qrot(head->q, fwd);
  m->pose.p.x = head->p.x + f.x * 1.05f;
  m->pose.p.y = head->p.y + f.y * 1.05f;
  m->pose.p.z = head->p.z + f.z * 1.05f;
  /* Upright, facing the wearer — follows look xz so it stays in front. */
  m->pose.q = qyaw_from_fwd(f);
  m->have_pose = 1;
}

void kxr_menu_open(kxr_menu *m, const kxr_pose *head) {
  if (!m) return;
  m->open = 1;
  m->dirty = 1;
  if (head) kxr_menu_recenter(m, head);
}

void kxr_menu_close(kxr_menu *m) {
  if (!m) return;
  m->open = 0;
}

void kxr_menu_toggle(kxr_menu *m, const kxr_pose *head) {
  if (!m) return;
  if (m->open)
    kxr_menu_close(m);
  else
    kxr_menu_open(m, head);
}

void kxr_menu_set_center(kxr_menu *m, kxr_vec3 c) {
  if (!m) return;
  m->pose.p = c;
}

static void px(uint8_t *rgba, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t *p;
  if (x < 0 || y < 0 || x >= KXR_MENU_W || y >= KXR_MENU_HGT) return;
  p = rgba + ((size_t)y * KXR_MENU_W + (size_t)x) * 4;
  p[0] = r;
  p[1] = g;
  p[2] = b;
  p[3] = 255;
}

static void fill(uint8_t *rgba, int x, int y, int w, int h, uint8_t r, uint8_t g,
                 uint8_t b) {
  int i, j;
  for (j = 0; j < h; j++)
    for (i = 0; i < w; i++) px(rgba, x + i, y + j, r, g, b);
}

static int glyph(int c) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c < 32 || c > 'Z') return 0;
  return c - 32;
}

static void glyph_at(uint8_t *rgba, int x, int y, int c, int s, uint8_t r, uint8_t g,
                     uint8_t b) {
  const uint8_t *col;
  int i, j;
  col = F5[glyph(c)];
  for (i = 0; i < 5; i++) {
    uint8_t bits = col[i];
    for (j = 0; j < 7; j++) {
      if (bits & (1u << j)) {
        int a, bb;
        for (bb = 0; bb < s; bb++)
          for (a = 0; a < s; a++) px(rgba, x + i * s + a, y + j * s + bb, r, g, b);
      }
    }
  }
}

static void text(uint8_t *rgba, int x, int y, const char *s, int sc, uint8_t r, uint8_t g,
                 uint8_t b) {
  int cx = x;
  if (!s) return;
  for (; *s; s++) {
    if (*s == '\n') {
      y += 8 * sc;
      cx = x;
      continue;
    }
    glyph_at(rgba, cx, y, (unsigned char)*s, sc, r, g, b);
    cx += 6 * sc;
  }
}

static const char *row_label(int row) {
  static const char *L[] = {"Near m",        "Far m",         "Density",
                            "Dot",           "Color",         "Tilt deg",
                            "Scale",         "Hide head m",   "Autolock",
                            "Lock head",     "Sample walk",   "Solve rigid",
                            "Yaw from walk", "Reset lock",    "Solid 3D",
                            "Map fly",       "Sensor",        "Align v2",
                            "Reset align",   "Close"};
  if (row < 0 || row >= KXR_MENU_ROWS) return "";
  if ((int)(sizeof L / sizeof L[0]) != KXR_MENU_ROWS) return "";
  return L[row];
}

static const char *sensor_name(int src) {
  switch (src) {
  case 1:
    return "KINECT";
  case 2:
    return "KINECT 2";
  case 3:
    return "BOTH";
  default:
    return "AUTO";
  }
}

static void row_value(const kxr_menu_snap *s, int row, char *out, size_t n) {
  static const char *col[] = {"Depth", "Height", "Grey", "Objects"};
  int ci;
  out[0] = 0;
  if (!s) return;
  switch (row) {
  case 0:
    snprintf(out, n, "%.2f", s->z_near);
    break;
  case 1:
    snprintf(out, n, "%.2f", s->z_far);
    break;
  case 2:
    snprintf(out, n, "%d", s->stride);
    break;
  case 3:
    snprintf(out, n, "%d", s->dot);
    break;
  case 4:
    ci = s->color;
    if (ci < 0) ci = 0;
    if (ci > 3) ci = 3;
    snprintf(out, n, "%s", col[ci]);
    break;
  case 5:
    snprintf(out, n, "< %.0f >", s->tilt_deg);
    break;
  case 6:
    snprintf(out, n, "%.2f", s->scale);
    break;
  case 7:
    snprintf(out, n, "%.2f", s->hide);
    break;
  case 8:
    snprintf(out, n, "%s", s->autolock ? "on" : "off");
    break;
  case 14:
    snprintf(out, n, "%s  %d obj", s->solid ? "on" : "off", s->nobj);
    break;
  case 15:
    snprintf(out, n, "%s", s->map ? "on" : "off");
    break;
  case 16:
    snprintf(out, n, "%s", sensor_name(s->src));
    break;
  case 17:
    if (s->identity_rigid)
      snprintf(out, n, "IDENTITY");
    else
      snprintf(out, n, "Y %.0f P %.0f", s->align_yaw, s->align_pitch);
    break;
  default:
    break;
  }
}

void kxr_menu_paint(kxr_menu *m, const kxr_menu_snap *snap) {
  uint8_t *rgba;
  int i, y0;
  char val[48], head[96];
  if (!m || !m->rgba) return;
  rgba = m->rgba;
  fill(rgba, 0, 0, KXR_MENU_W, KXR_MENU_HGT, 10, 2, 8);
  fill(rgba, 0, 0, KXR_MENU_W, 48, 140, 22, 42);
  text(rgba, 16, 12, "KINECT XR", 3, 250, 230, 235);
  snprintf(head, sizeof head, "n=%d %s %s", snap ? snap->points : 0,
           snap && snap->live ? "LIVE" : "xyz", snap && snap->locked ? "LOCK" : "");
  text(rgba, 430, 16, head, 2, 230, 180, 190);
  y0 = KXR_MENU_Y0;
  for (i = 0; i < KXR_MENU_ROWS; i++) {
    int y = y0 + i * KXR_MENU_PITCH;
    int sel = (i == m->row) || (m->hover_ok && i == m->hover_row);
    if (i == 0)
      text(rgba, 14, y - 2, "CLOUD", 1, 180, 70, 90);
    if (i == 6)
      text(rgba, 14, y - 2, "LOCK", 1, 180, 70, 90);
    if (i == 16)
      text(rgba, 14, y - 2, "SENSOR", 1, 180, 70, 90);
    if (sel)
      fill(rgba, 8, y - 2, KXR_MENU_W - 16, 20, 70, 14, 28);
    text(rgba, 24, y + 2, row_label(i), 2, 240, 220, 225);
    row_value(snap, i, val, sizeof val);
    if (val[0])
      text(rgba, 430, y + 2, val, 2, 242, 80, 100);
  }
  fill(rgba, 0, KXR_MENU_HGT - 36, KXR_MENU_W, 36, 20, 4, 12);
  text(rgba, 16, KXR_MENU_HGT - 26, m->status, 2, 220, 160, 170);
  m->dirty = 0;
}

static kxr_menu_act act_for(int row, int dir) {
  if (dir == 0) dir = 1;
  switch (row) {
  case 0:
    return dir < 0 ? KXR_ACT_NEAR_N : KXR_ACT_NEAR_P;
  case 1:
    return dir < 0 ? KXR_ACT_FAR_N : KXR_ACT_FAR_P;
  case 2:
    return dir < 0 ? KXR_ACT_STRIDE_N : KXR_ACT_STRIDE_P;
  case 3:
    return dir < 0 ? KXR_ACT_DOT_N : KXR_ACT_DOT_P;
  case 4:
    return KXR_ACT_COLOR;
  case 5:
    return dir < 0 ? KXR_ACT_TILT_N : KXR_ACT_TILT_P;
  case 6:
    return dir < 0 ? KXR_ACT_SCALE_N : KXR_ACT_SCALE_P;
  case 7:
    return dir < 0 ? KXR_ACT_HIDE_N : KXR_ACT_HIDE_P;
  case 8:
    return KXR_ACT_AUTOLOCK;
  case 9:
    return KXR_ACT_LOCK;
  case 10:
    return KXR_ACT_SAMPLE;
  case 11:
    return KXR_ACT_SOLVE;
  case 12:
    return KXR_ACT_YAW;
  case 13:
    return KXR_ACT_RESET;
  case 14:
    return KXR_ACT_SOLID;
  case 15:
    return KXR_ACT_MAP;
  case 16:
    return dir < 0 ? KXR_ACT_SRC_N : KXR_ACT_SRC_P;
  case 17:
    return KXR_ACT_ALIGN_V2;
  case 18:
    return KXR_ACT_ALIGN_RESET;
  case 19:
    return KXR_ACT_CLOSE;
  default:
    return KXR_ACT_NONE;
  }
}

kxr_menu_act kxr_menu_nav(kxr_menu *m, int dx, int dy, int activate) {
  if (!m || !m->open) return KXR_ACT_NONE;
  if (dy) {
    m->row += dy > 0 ? 1 : -1;
    if (m->row < 0) m->row = KXR_MENU_ROWS - 1;
    if (m->row >= KXR_MENU_ROWS) m->row = 0;
    m->dirty = 1;
    return KXR_ACT_NONE;
  }
  if (dx) return act_for(m->row, dx);
  if (activate) return act_for(m->row, 1);
  return KXR_ACT_NONE;
}

kxr_menu_act kxr_menu_click_row(kxr_menu *m, int row, int dir) {
  if (!m || row < 0 || row >= KXR_MENU_ROWS) return KXR_ACT_NONE;
  m->row = row;
  m->dirty = 1;
  return act_for(row, dir);
}

int kxr_menu_ray(const kxr_menu *m, kxr_vec3 origin, kxr_vec3 dir, int *row, float *hitx,
                 float *hity, float *hitz, float *hitu) {
  kxr_vec3 n, rgt, up, d, hit;
  float dn, t, u, v;
  int ir;
  if (!m || !m->open || !m->have_pose) return 0;
  rgt.x = 1;
  rgt.y = 0;
  rgt.z = 0;
  up.x = 0;
  up.y = 1;
  up.z = 0;
  n.x = 0;
  n.y = 0;
  n.z = 1;
  rgt = qrot(m->pose.q, rgt);
  up = qrot(m->pose.q, up);
  n = qrot(m->pose.q, n);
  dn = dir.x * n.x + dir.y * n.y + dir.z * n.z;
  if (fabsf(dn) < 1e-5f) return 0;
  d.x = m->pose.p.x - origin.x;
  d.y = m->pose.p.y - origin.y;
  d.z = m->pose.p.z - origin.z;
  t = (d.x * n.x + d.y * n.y + d.z * n.z) / dn;
  if (t < 0.05f || t > 3.5f) return 0;
  hit.x = origin.x + dir.x * t;
  hit.y = origin.y + dir.y * t;
  hit.z = origin.z + dir.z * t;
  u = ((hit.x - m->pose.p.x) * rgt.x + (hit.y - m->pose.p.y) * rgt.y +
       (hit.z - m->pose.p.z) * rgt.z) /
          m->width_m +
      0.5f;
  v = 0.5f - ((hit.x - m->pose.p.x) * up.x + (hit.y - m->pose.p.y) * up.y +
              (hit.z - m->pose.p.z) * up.z) /
                 m->height_m;
  if (u < 0.f || u > 1.f || v < 0.f || v > 1.f) return 0;
  if (hitx) *hitx = hit.x;
  if (hity) *hity = hit.y;
  if (hitz) *hitz = hit.z;
  if (hitu) *hitu = u;
  ir = (int)((v * (float)KXR_MENU_HGT - (float)KXR_MENU_Y0) / (float)KXR_MENU_PITCH);
  if (ir < 0 || ir >= KXR_MENU_ROWS) ir = -1;
  if (row) *row = ir;
  return 1;
}
