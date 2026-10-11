/* Kinect v2 (Xbox One) depth, color, and IR. C API over libfreenect2. */
#ifndef KINECT2_DEVICE_H
#define KINECT2_DEVICE_H

#include "cube_kinect_cloud.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KINECT2_W 512
#define KINECT2_H 424
#define KINECT2_N (KINECT2_W * KINECT2_H)

int kinect2_count(void);
const char *kinect2_error(void);
const char *kinect2_pipeline(void);
/* Depth stream (512×424 mm). Closes a color or IR session first. */
int kinect2_open(void);
/* Color camera only. IR projector stays off. One device, same as depth. */
int kinect2_open_color(void);
/* IR camera (depth pipeline, float 512×424). Projector on. */
int kinect2_open_ir(void);
void kinect2_close(void);
int kinect2_is_open(void);
/* 0 closed, 1 depth, 2 color, 3 IR. */
int kinect2_stream_kind(void);

/* 1 = new depth frame (millimetres, 0 invalid), 0 = timeout, -1 = error.
 * z_mm has KINECT2_N floats. host_us is CLOCK_MONOTONIC at delivery. */
int kinect2_take(float *z_mm, uint64_t *host_us, int wait_ms);

/* 1 = new 640×480 RGB frame (nearest from 1920×1080 BGRX/RGBX), 0 = timeout, -1 = error. */
int kinect2_take_rgb(uint8_t *rgb640, int wait_ms);

/* 1 = new IR frame, KINECT2_N uint16 samples in [0, 65535], 0 = timeout, -1 = error. */
int kinect2_take_ir(uint16_t *ir, int wait_ms);

/* Factory undistort when the device is open. +Y up, same frame as the v1 cloud. */
int kinect2_cloud_from_depth(const float *z_mm, const cube_cloud_tune *t,
                             cube_cloud *out, int max_pts);
/* Project the undistorted frame already produced by kinect2_metric_depth.
 * Skips a second undistort of the same depth image. */
int kinect2_cloud_from_undistorted(const cube_cloud_tune *t, cube_cloud *out, int max_pts);
/* Undistorted depth in millimetres. The receiver projects this with the
 * intrinsics below. Fails when this process does not own the Kinect 2. */
int kinect2_metric_depth(const float *z_mm, float *out_mm);
int kinect2_ir_intrinsics(float *fx, float *fy, float *cx, float *cy);

#ifdef __cplusplus
}
#endif
#endif
