/* Kinect depth → XYZ crimson cloud. FACE_BLUR_LAW: no RGB faces. Pure C. */
#ifndef CUBE_KINECT_CLOUD_H
#define CUBE_KINECT_CLOUD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CUBE_CLOUD_MAX 24000

typedef struct cube_cloud_pt {
  float x, y, z;
  uint8_t r, g, b;
} cube_cloud_pt;

typedef struct cube_cloud {
  cube_cloud_pt *pt;
  int n;
  int cap;
  int ok;
  uint32_t ts;
  char err[160];
  char path[320];
} cube_cloud;

void cube_cloud_init(cube_cloud *c);
void cube_cloud_clear(cube_cloud *c);
void cube_cloud_free(cube_cloud *c);

/* Stop cube_daemon / freenect viewers that hold the camera. */
void cube_cloud_free_holders(void);

typedef struct cube_cloud_tune {
  int stride;    /* 2..8 pixel step */
  float z_near;  /* metres */
  float z_far;
  int color;     /* 0=depth crimson 1=height 2=grey */
  int dot;       /* 1..4 splat radius */
} cube_cloud_tune;

void cube_cloud_tune_default(cube_cloud_tune *t);
void cube_cloud_color_point(cube_cloud_pt *p, const cube_cloud_tune *t);

/* Project a 640×480 FREENECT_DEPTH_11BIT buffer. */
int cube_cloud_from_depth(cube_cloud *c, const uint16_t *depth, int stride);
int cube_cloud_from_depth_ex(cube_cloud *c, const uint16_t *depth, const cube_cloud_tune *t);

/* One depth frame via libfreenect_sync. Does not stop the device. */
int cube_cloud_capture(cube_cloud *c, int stride);

/* Release sync handle (end of session). */
void cube_cloud_release(void);

int cube_cloud_load_xyz(cube_cloud *c, const char *path, int max_pts);
int cube_cloud_write_xyz(const cube_cloud *c, const char *path);
/* Persist latest + clouds/ copies. */
int cube_cloud_publish(const cube_cloud *c);
/* Simple software orbit render → P6 PPM (no GL). */
int cube_cloud_write_ppm(const cube_cloud *c, const char *path, int w, int h,
                         float elev_deg, float azim_deg, float dist);
/* Same orbit into packed RGB (w*h*3). */
int cube_cloud_render_rgb(const cube_cloud *c, uint8_t *rgb, int w, int h,
                          float elev_deg, float azim_deg, float dist);
int cube_cloud_render_rgb_ex(const cube_cloud *c, uint8_t *rgb, int w, int h,
                             float elev_deg, float azim_deg, float dist, int dot);

#ifdef __cplusplus
}
#endif
#endif
