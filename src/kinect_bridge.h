#ifndef KINECT_BRIDGE_H
#define KINECT_BRIDGE_H

#include "kinect_ir.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int kinect_bridge_start(const char *mode);
void kinect_bridge_stop(void);
int kinect_bridge_running(void);
/* 1 = Kinect gspca, 2 = Kinect 2 color/IR, 0 = not streaming. */
int kinect_bridge_sensor(void);
/* 1 when the preview is the peer's camera, not a local USB open. */
int kinect_bridge_net(void);
const char *kinect_bridge_error(void);
const char *kinect_bridge_vcam(void);
int kinect_bridge_preview_rgb(uint8_t *rgb /* 640*480*3 */);
void kinect_bridge_set_ir_tune(const kinect_ir_tune *t);
void kinect_bridge_get_ir_tune(kinect_ir_tune *t);
void kinect_bridge_set_fg_boost(int mid_frames);
int kinect_bridge_get_fg_boost(void);
const char *kinect_bridge_fg_backend(void);

#ifdef __cplusplus
}
#endif
#endif
