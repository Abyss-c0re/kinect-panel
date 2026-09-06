#ifndef KINECT_FG_HIP_H
#define KINECT_FG_HIP_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int fg_hip_available(void);
int fg_hip_init(int w, int h);
void fg_hip_shutdown(void);
/* a,b,out: host RGB packed w*h*3. t in (0,1). 0=ok */
int fg_hip_interp(const uint8_t *a, const uint8_t *b, float t, uint8_t *out, int w, int h);
#ifdef __cplusplus
}
#endif
#endif
