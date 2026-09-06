/* HIP motion-compensated interpolator. Host never writes frames to disk. */
#include <hip/hip_runtime.h>
#include <stdint.h>
#include <string.h>

#define BS 8
#define SR 8

static int g_w, g_h, g_ok;
extern "C" void fg_hip_shutdown(void);
static uint8_t *d_a, *d_b, *d_o;
static short *d_mvx, *d_mvy;

static int blocks_x(void) { return (g_w + BS - 1) / BS; }
static int blocks_y(void) { return (g_h + BS - 1) / BS; }

__device__ static int luma(const uint8_t *p) {
  return (p[0] + 2 * p[1] + p[2]) >> 2;
}

__global__ void k_match(const uint8_t *a, const uint8_t *b, int w, int h,
                        short *mvx, short *mvy, int nbx) {
  int bx = (int)blockIdx.x;
  int by = (int)blockIdx.y;
  int x0 = bx * BS;
  int y0 = by * BS;
  if (x0 >= w || y0 >= h) return;
  int best = 0x7fffffff;
  int bdx = 0, bdy = 0;
  int dy, dx, j, i;
  for (dy = -SR; dy <= SR; dy++) {
    for (dx = -SR; dx <= SR; dx++) {
      int sad = 0;
      for (j = 0; j < BS; j++) {
        int y = y0 + j;
        if (y >= h) continue;
        for (i = 0; i < BS; i++) {
          int x = x0 + i;
          int x2, y2;
          if (x >= w) continue;
          x2 = x + dx;
          y2 = y + dy;
          if ((unsigned)x2 >= (unsigned)w || (unsigned)y2 >= (unsigned)h)
            sad += 48;
          else {
            int la = luma(a + (y * w + x) * 3);
            int lb = luma(b + (y2 * w + x2) * 3);
            sad += la > lb ? la - lb : lb - la;
          }
        }
      }
      if (sad < best) {
        best = sad;
        bdx = dx;
        bdy = dy;
      }
    }
  }
  mvx[by * nbx + bx] = (short)bdx;
  mvy[by * nbx + bx] = (short)bdy;
}

__device__ static void sample(const uint8_t *im, int w, int h, float fx, float fy,
                              int *r, int *g, int *b) {
  int x0, y0, x1, y1;
  float tx, ty;
  if (fx < 0) fx = 0;
  if (fy < 0) fy = 0;
  if (fx > (float)(w - 1)) fx = (float)(w - 1);
  if (fy > (float)(h - 1)) fy = (float)(h - 1);
  x0 = (int)fx;
  y0 = (int)fy;
  x1 = x0 + 1 < w ? x0 + 1 : x0;
  y1 = y0 + 1 < h ? y0 + 1 : y0;
  tx = fx - (float)x0;
  ty = fy - (float)y0;
  {
    const uint8_t *p00 = im + (y0 * w + x0) * 3;
    const uint8_t *p10 = im + (y0 * w + x1) * 3;
    const uint8_t *p01 = im + (y1 * w + x0) * 3;
    const uint8_t *p11 = im + (y1 * w + x1) * 3;
    float ix0, ix1;
    ix0 = p00[0] * (1.f - tx) + p10[0] * tx;
    ix1 = p01[0] * (1.f - tx) + p11[0] * tx;
    *r = (int)(ix0 * (1.f - ty) + ix1 * ty + 0.5f);
    ix0 = p00[1] * (1.f - tx) + p10[1] * tx;
    ix1 = p01[1] * (1.f - tx) + p11[1] * tx;
    *g = (int)(ix0 * (1.f - ty) + ix1 * ty + 0.5f);
    ix0 = p00[2] * (1.f - tx) + p10[2] * tx;
    ix1 = p01[2] * (1.f - tx) + p11[2] * tx;
    *b = (int)(ix0 * (1.f - ty) + ix1 * ty + 0.5f);
  }
}

__global__ void k_warp(const uint8_t *a, const uint8_t *b, const short *mvx,
                       const short *mvy, int w, int h, float t, uint8_t *o, int nbx) {
  int x = (int)(blockIdx.x * blockDim.x + threadIdx.x);
  int y = (int)(blockIdx.y * blockDim.y + threadIdx.y);
  int bx, by, ra, ga, ba, rb, gb, bb, i;
  float xa, ya, xb, yb;
  if (x >= w || y >= h) return;
  bx = x / BS;
  by = y / BS;
  {
    short mx = mvx[by * nbx + bx];
    short my = mvy[by * nbx + bx];
    xa = (float)x - t * (float)mx;
    ya = (float)y - t * (float)my;
    xb = (float)x + (1.f - t) * (float)mx;
    yb = (float)y + (1.f - t) * (float)my;
  }
  sample(a, w, h, xa, ya, &ra, &ga, &ba);
  sample(b, w, h, xb, yb, &rb, &gb, &bb);
  i = (y * w + x) * 3;
  o[i + 0] = (uint8_t)((1.f - t) * ra + t * rb);
  o[i + 1] = (uint8_t)((1.f - t) * ga + t * gb);
  o[i + 2] = (uint8_t)((1.f - t) * ba + t * bb);
}

extern "C" int fg_hip_available(void) {
  int n = 0;
  if (hipGetDeviceCount(&n) != hipSuccess) return 0;
  return n > 0;
}

extern "C" int fg_hip_init(int w, int h) {
  size_t pix = (size_t)w * (size_t)h * 3;
  size_t nblk;
  if (g_ok) fg_hip_shutdown();
  g_w = w;
  g_h = h;
  nblk = (size_t)blocks_x() * (size_t)blocks_y();
  if (hipMalloc(&d_a, pix) != hipSuccess) return -1;
  if (hipMalloc(&d_b, pix) != hipSuccess) return -1;
  if (hipMalloc(&d_o, pix) != hipSuccess) return -1;
  if (hipMalloc(&d_mvx, nblk * sizeof(short)) != hipSuccess) return -1;
  if (hipMalloc(&d_mvy, nblk * sizeof(short)) != hipSuccess) return -1;
  g_ok = 1;
  return 0;
}

extern "C" void fg_hip_shutdown(void) {
  if (d_a) hipFree(d_a);
  if (d_b) hipFree(d_b);
  if (d_o) hipFree(d_o);
  if (d_mvx) hipFree(d_mvx);
  if (d_mvy) hipFree(d_mvy);
  d_a = d_b = d_o = 0;
  d_mvx = d_mvy = 0;
  g_ok = 0;
}

extern "C" int fg_hip_interp(const uint8_t *a, const uint8_t *b, float t, uint8_t *out,
                             int w, int h) {
  dim3 gb, tb, gw, tw;
  size_t pix;
  int nbx;
  if (!g_ok || w != g_w || h != g_h) return -1;
  pix = (size_t)w * (size_t)h * 3;
  nbx = blocks_x();
  if (hipMemcpy(d_a, a, pix, hipMemcpyHostToDevice) != hipSuccess) return -1;
  if (hipMemcpy(d_b, b, pix, hipMemcpyHostToDevice) != hipSuccess) return -1;
  gb = dim3((unsigned)nbx, (unsigned)blocks_y(), 1);
  tb = dim3(1, 1, 1);
  hipLaunchKernelGGL(k_match, gb, tb, 0, 0, d_a, d_b, w, h, d_mvx, d_mvy, nbx);
  tw = dim3(16, 16, 1);
  gw = dim3((unsigned)((w + 15) / 16), (unsigned)((h + 15) / 16), 1);
  hipLaunchKernelGGL(k_warp, gw, tw, 0, 0, d_a, d_b, d_mvx, d_mvy, w, h, t, d_o, nbx);
  if (hipDeviceSynchronize() != hipSuccess) return -1;
  if (hipMemcpy(out, d_o, pix, hipMemcpyDeviceToHost) != hipSuccess) return -1;
  return 0;
}
