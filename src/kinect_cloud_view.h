#ifndef KINECT_CLOUD_VIEW_H
#define KINECT_CLOUD_VIEW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int kinect_cloud_start(void);
void kinect_cloud_stop(void);
int kinect_cloud_running(void);
const char *kinect_cloud_error(void);
int kinect_cloud_preview_rgb(uint8_t *rgb /* 640*480*3 */);
int kinect_cloud_points(void);
void kinect_cloud_orbit(float elev, float azim, float dist);
void kinect_cloud_get_orbit(float *elev, float *azim, float *dist);
int kinect_cloud_set_tilt(double degrees);
int kinect_cloud_get_tilt(double *degrees, int *status);

#ifdef __cplusplus
}
#endif
#endif
