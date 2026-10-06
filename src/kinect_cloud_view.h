#ifndef KINECT_CLOUD_VIEW_H
#define KINECT_CLOUD_VIEW_H

#include "cube_kinect_cloud.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int kinect_cloud_start(void);
void kinect_cloud_stop(void);
int kinect_cloud_running(void);
int kinect_cloud_live(void); /* live depth frames seen (not a stale file) */
const char *kinect_cloud_error(void);
int kinect_cloud_preview_rgb(uint8_t *rgb /* 640*480*3 */);
int kinect_cloud_points(void);
/* Latest Kinect-frame cloud (same points the preview was painted from). */
int kinect_cloud_snapshot(cube_cloud *out);
int kinect_cloud_copy_depth(uint16_t *depth /* 640*480 */);
void kinect_cloud_orbit(float elev, float azim, float dist);
void kinect_cloud_get_orbit(float *elev, float *azim, float *dist);
int kinect_cloud_set_tilt(double degrees);
int kinect_cloud_get_tilt(double *degrees, int *status);
void kinect_cloud_set_tune(const cube_cloud_tune *t);
void kinect_cloud_get_tune(cube_cloud_tune *t);
void kinect_cloud_set_solid(int on);
int kinect_cloud_get_solid(void);
void kinect_cloud_reset_view(void);
void kinect_cloud_set_fg_boost(int mid_frames);
int kinect_cloud_get_fg_boost(void);
const char *kinect_cloud_fg_backend(void);

#ifdef __cplusplus
}
#endif
#endif
