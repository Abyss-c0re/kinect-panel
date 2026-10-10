#include "kinect2_device.h"

#include <stdio.h>

static char err[80] = "kinect 2 support not built";

int kinect2_count(void) { return 0; }
const char *kinect2_error(void) { return err; }
const char *kinect2_pipeline(void) { return ""; }
int kinect2_open(void) { return -1; }
int kinect2_open_color(void) { return -1; }
int kinect2_open_ir(void) { return -1; }
void kinect2_close(void) {}
int kinect2_is_open(void) { return 0; }
int kinect2_stream_kind(void) { return 0; }
int kinect2_take(float *z_mm, uint64_t *host_us, int wait_ms) {
  (void)z_mm;
  (void)host_us;
  (void)wait_ms;
  return -1;
}
int kinect2_take_rgb(uint8_t *rgb640, int wait_ms) {
  (void)rgb640;
  (void)wait_ms;
  return -1;
}
int kinect2_take_ir(uint16_t *ir, int wait_ms) {
  (void)ir;
  (void)wait_ms;
  return -1;
}
int kinect2_cloud_from_depth(const float *z_mm, const cube_cloud_tune *t, cube_cloud *out,
                             int max_pts) {
  (void)z_mm;
  (void)t;
  (void)max_pts;
  if (out) snprintf(out->err, sizeof out->err, "%s", err);
  return -1;
}
int kinect2_metric_depth(const float *z_mm, float *out_mm) {
  (void)z_mm;
  (void)out_mm;
  return -1;
}
int kinect2_ir_intrinsics(float *fx, float *fy, float *cx, float *cy) {
  (void)fx;
  (void)fy;
  (void)cx;
  (void)cy;
  return -1;
}
