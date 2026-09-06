#define _DEFAULT_SOURCE
#include "fg.h"
#include "fg_hip.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct fg_engine {
  int mid;
  int have_prev;
  int nout;
  int hip;
  char backend[32];
  uint8_t *prev;
  uint8_t *curr;
  uint8_t *out[FG_MAX_MID + 1];
};

static void cpu_lerp(const uint8_t *a, const uint8_t *b, float t, uint8_t *o) {
  int i, n = FG_RGB;
  int ia = (int)((1.f - t) * 256.f);
  int ib = 256 - ia;
  for (i = 0; i < n; i++)
    o[i] = (uint8_t)((a[i] * ia + b[i] * ib) >> 8);
}

fg_engine *fg_create(void) {
  fg_engine *e = (fg_engine *)calloc(1, sizeof *e);
  int i;
  if (!e) return NULL;
  e->prev = (uint8_t *)malloc(FG_RGB);
  e->curr = (uint8_t *)malloc(FG_RGB);
  if (!e->prev || !e->curr) {
    fg_destroy(e);
    return NULL;
  }
  for (i = 0; i <= FG_MAX_MID; i++) {
    e->out[i] = (uint8_t *)malloc(FG_RGB);
    if (!e->out[i]) {
      fg_destroy(e);
      return NULL;
    }
  }
  e->hip = 0;
  snprintf(e->backend, sizeof e->backend, "cpu");
#ifdef FG_HAVE_HIP
  if (fg_hip_available() && fg_hip_init(FG_W, FG_H) == 0) {
    e->hip = 1;
    snprintf(e->backend, sizeof e->backend, "hip");
  }
#endif
  return e;
}

void fg_destroy(fg_engine *e) {
  int i;
  if (!e) return;
#ifdef FG_HAVE_HIP
  if (e->hip) fg_hip_shutdown();
#endif
  free(e->prev);
  free(e->curr);
  for (i = 0; i <= FG_MAX_MID; i++) free(e->out[i]);
  memset(e, 0, sizeof *e);
  free(e);
}

const char *fg_backend(const fg_engine *e) { return e && e->backend[0] ? e->backend : "none"; }
int fg_ok(const fg_engine *e) { return e != NULL; }

void fg_set_boost(fg_engine *e, int mid_frames) {
  if (!e) return;
  if (mid_frames < 0) mid_frames = 0;
  if (mid_frames > FG_MAX_MID) mid_frames = FG_MAX_MID;
  e->mid = mid_frames;
}

int fg_get_boost(const fg_engine *e) { return e ? e->mid : 0; }

int fg_push_rgb(fg_engine *e, const uint8_t *rgb) {
  uint8_t *tmp;
  int k, n;
  if (!e || !rgb) return 0;
  if (!e->have_prev) {
    memcpy(e->curr, rgb, FG_RGB);
    memcpy(e->out[0], rgb, FG_RGB);
    e->have_prev = 1;
    e->nout = 1;
    return 1;
  }
  tmp = e->prev;
  e->prev = e->curr;
  e->curr = tmp;
  memcpy(e->curr, rgb, FG_RGB);
  n = e->mid;
  for (k = 0; k < n; k++) {
    float t = (float)(k + 1) / (float)(n + 1);
    int ok = -1;
#ifdef FG_HAVE_HIP
    if (e->hip) ok = fg_hip_interp(e->prev, e->curr, t, e->out[k], FG_W, FG_H);
#endif
    if (ok != 0) cpu_lerp(e->prev, e->curr, t, e->out[k]);
  }
  memcpy(e->out[n], e->curr, FG_RGB);
  e->nout = n + 1;
  return e->nout;
}

int fg_count(const fg_engine *e) { return e ? e->nout : 0; }

int fg_copy(const fg_engine *e, int i, uint8_t *rgb) {
  if (!e || !rgb || i < 0 || i >= e->nout) return -1;
  memcpy(rgb, e->out[i], FG_RGB);
  return 0;
}
