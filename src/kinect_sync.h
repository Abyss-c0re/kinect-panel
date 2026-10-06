/* Pair Kinect v1 and Kinect v2 clouds on the host clock, then one rigid. */
#ifndef KINECT_SYNC_H
#define KINECT_SYNC_H

#include "cube_kinect_cloud.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One depth frame is ~33 ms. A pair inside this window is in sync. */
#define KSYNC_PAIR_US 50000u
/* Still fuse a looser pair so one late frame does not blank the view. */
#define KSYNC_HOLD_US 100000u
#define KSYNC_STALE_US 200000u

typedef struct ksync_rigid {
  float R[9]; /* row-major. p_v1 = R * p_v2 + t */
  float t[3];
} ksync_rigid;

typedef struct ksync_align_result {
  int ok;
  float residual_m;
  int inliers;
  float yaw_deg;
  float pitch_deg;
} ksync_align_result;

void ksync_rigid_identity(ksync_rigid *r);
int ksync_rigid_is_identity(const ksync_rigid *r);
void ksync_rigid_apply(const ksync_rigid *r, float *x, float *y, float *z);
void ksync_rigid_ypr(const ksync_rigid *r, float *yaw_deg, float *pitch_deg);
int ksync_rigid_save(const ksync_rigid *r, const char *path);
int ksync_rigid_load(ksync_rigid *r, const char *path);
/* KINECT_EXTRINSIC or the tracker state file. */
int ksync_extrinsic_path(char *out, size_t n);

/* Raise stride until the grid fits in max_pts. Stride stays in 1..32. */
int ksync_cover_stride(int w, int h, int stride, int max_pts);

/* Best host-clock pair. Returns 0 when both rings have a fresh stamp.
 * i1/i2 index the arrays. skew_us is absolute. */
int ksync_best_pair(const uint64_t *t1, int n1, const uint64_t *t2, int n2,
                    uint64_t now_us, int *i1, int *i2, uint64_t *skew_us);

/* Kinect 2 points read blue so a fused cloud shows which sensor saw them. */
void ksync_tint_v2(cube_cloud *c);

int ksync_merge(cube_cloud *dst, const cube_cloud *v1, const cube_cloud *v2,
                const ksync_rigid *v2_to_v1, int max_pts);

/* Pinhole metres, +Y up, +Z forward. z_m == 0 is invalid. */
int ksync_project_depth(cube_cloud *out, const float *z_m, int w, int h,
                        float fx, float fy, float cx, float cy,
                        const cube_cloud_tune *t, int max_pts);

/* Map v2 camera points into the v1 camera. Fails closed when overlap is thin. */
int ksync_align(ksync_rigid *out, ksync_align_result *res, const cube_cloud *v1,
                const cube_cloud *v2);

#ifdef __cplusplus
}
#endif
#endif
