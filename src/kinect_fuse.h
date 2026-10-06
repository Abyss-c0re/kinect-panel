/* One depth owner: Kinect v1, Kinect v2, or both as one synchronised cloud. */
#ifndef KINECT_FUSE_H
#define KINECT_FUSE_H

#include "kinect_sync.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  KINECT_SRC_AUTO = 0,
  KINECT_SRC_V1 = 1,
  KINECT_SRC_V2 = 2,
  KINECT_SRC_BOTH = 3
} kinect_src;

typedef struct kinect_fuse_stats {
  kinect_src requested;
  kinect_src active;
  int running;
  int v1_open;
  int v2_open;
  int n1;
  int n2;
  int n;
  int skew_us; /* -1 when this frame is not a pair */
  int paired;
  int identity_rigid;
  uint32_t seq;
  char pipeline[16];
  char err[160];
} kinect_fuse_stats;

const char *kinect_src_name(kinect_src s);
kinect_src kinect_src_parse(const char *s);
/* 1 when s is auto/v1/v2/both or one of their aliases. Unknown tokens stay 0. */
int kinect_src_known(const char *s, kinect_src *out);
kinect_src kinect_src_from_env(void);

void kinect_fuse_set_source(kinect_src s);
kinect_src kinect_fuse_source(void);

/* Blocks until the devices are streaming or the open fails. */
int kinect_fuse_start(void);
void kinect_fuse_request_stop(void);
void kinect_fuse_stop(void);
int kinect_fuse_running(void);

void kinect_fuse_set_tune(const cube_cloud_tune *t);
void kinect_fuse_get_tune(cube_cloud_tune *t);
int kinect_fuse_set_tilt(double degrees);
int kinect_fuse_get_tilt(double *degrees, int *status);

int kinect_fuse_snapshot(cube_cloud *out);
int kinect_fuse_copy_v1_depth(uint16_t *depth /* 640*480 */);
void kinect_fuse_get_stats(kinect_fuse_stats *st);
const char *kinect_fuse_error(void);

int kinect_fuse_align(ksync_align_result *res);
void kinect_fuse_reset_rigid(void);
void kinect_fuse_get_rigid(ksync_rigid *r);

/* Enumerate only. Does not start a stream. */
int kinect_fuse_probe(int *v1, int *v2);

#ifdef __cplusplus
}
#endif
#endif
