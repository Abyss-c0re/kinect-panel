/* Kinect depth → XYZ crimson cloud. FACE_BLUR_LAW: no RGB faces.
 * Exclusive process — parent must hold no libusb. Usage: kinect_depth_dump [out.xyz] [stride]
 */
#include <libfreenect_sync.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static float raw_to_m(uint16_t d) {
  if (d < 1 || d > 2046) return 0.f;
  return 0.1236f * tanf(d / 2842.5f + 1.1863f);
}

int main(int argc, char **argv) {
  const char *out = argc > 1 ? argv[1] : "/tmp/kinect_cloud.xyz";
  int stride = argc > 2 ? atoi(argv[2]) : 4;
  void *depth = NULL;
  uint32_t ts = 0;
  uint16_t *d;
  const int W = 640, H = 480;
  const float fx = 594.21f, fy = 591.04f, cx = 339.5f, cy = 242.7f;
  FILE *f;
  int n = 0, v, u;
  if (stride < 1) stride = 4;
  if (freenect_sync_get_depth(&depth, &ts, 0, FREENECT_DEPTH_11BIT) != 0) {
    fprintf(stderr, "kinect_depth_dump: no depth (is Kinect free?)\n");
    freenect_sync_stop();
    return 1;
  }
  d = (uint16_t *)depth;
  f = fopen(out, "w");
  if (!f) {
    freenect_sync_stop();
    return 2;
  }
  for (v = 0; v < H; v += stride) {
    for (u = 0; u < W; u += stride) {
      float z = raw_to_m(d[v * W + u]);
      float x, y, t;
      int r, g, b;
      if (z < 0.45f || z > 3.8f) continue;
      x = (u - cx) * z / fx;
      y = -((v - cy) * z / fy);
      t = (z - 0.45f) / 3.35f;
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      r = 242;
      g = (int)(38 + t * 90);
      b = (int)(71 + (1.f - t) * 50);
      fprintf(f, "%.5f %.5f %.5f %d %d %d\n", x, y, z, r, g, b);
      n++;
    }
  }
  fclose(f);
  freenect_sync_stop();
  printf("{\"ok\":true,\"points\":%d,\"out\":\"%s\",\"stride\":%d,\"schema\":\"cube.kinect_cloud.v1\"}\n",
         n, out, stride);
  return 0;
}
