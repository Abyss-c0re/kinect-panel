#ifndef KINECT_V4L_H
#define KINECT_V4L_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KINECT_RGB_W 640
#define KINECT_RGB_H 480
#define KINECT_IR_W 640
#define KINECT_IR_H 488

/* Real Xbox NUI / gspca node — never a loopback. Empty string if none. */
int kinect_find_source(char *out, size_t n);
/* Existing v4l2loopback (exact label, truncated "Kinect", or the only one). */
int kinect_find_vcam(char *out, size_t n);

int kinect_set_mode(const char *dev, const char *mode); /* "RGB" or "IR" */
int kinect_grab_rgb(const char *dev, uint8_t *rgb /* 640*480*3 */);
int kinect_grab_ir_grey(const char *dev, uint8_t *grey /* 640*488 */);
void kinect_grey_to_rgb(const uint8_t *grey, int w, int h, uint8_t *rgb);
void kinect_uyvy_to_yuyv(const uint8_t *uyvy, uint8_t *yuyv, int npix);
void kinect_grey_to_yuyv(const uint8_t *grey, int w, int h, uint8_t *yuyv /* 640*480*2 */);
int kinect_vcam_open(const char *dev);
int kinect_vcam_write_yuyv(int fd, const uint8_t *yuyv);

typedef struct kinect_cap {
  int fd;
  void *map[4];
  size_t map_len[4];
  int nmap;
  int streaming;
  uint32_t fourcc;
  int w, h;
} kinect_cap;

int kinect_cap_open(kinect_cap *c, const char *dev, uint32_t fourcc, int w, int h);
int kinect_cap_frame(kinect_cap *c, uint8_t *dst, size_t dst_len);
void kinect_cap_close(kinect_cap *c);

#ifdef __cplusplus
}
#endif
#endif
