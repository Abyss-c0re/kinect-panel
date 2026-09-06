#ifndef KINECT_FG_H
#define KINECT_FG_H
/*
 * ROCm/HIP frame generation — in-memory only.
 * No disk, no network, no telemetry. At most two real frames + mids live.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FG_W 640
#define FG_H 480
#define FG_RGB (FG_W * FG_H * 3)
#define FG_MAX_MID 3

typedef struct fg_engine fg_engine;

fg_engine *fg_create(void);
void fg_destroy(fg_engine *e);

/* "hip" or "cpu". Never null. */
const char *fg_backend(const fg_engine *e);
int fg_ok(const fg_engine *e);

/* mid_frames: 0=off, 1=2x, 2=3x, 3=4x. */
void fg_set_boost(fg_engine *e, int mid_frames);
int fg_get_boost(const fg_engine *e);

/*
 * Push one real RGB 640×480 frame (not stored). After the second real
 * frame, generates mid_frames interpolated outputs then the current real.
 * Returns count of frames waiting in fg_copy (0 if first push).
 */
int fg_push_rgb(fg_engine *e, const uint8_t *rgb);
int fg_count(const fg_engine *e);
int fg_copy(const fg_engine *e, int i, uint8_t *rgb);

#ifdef __cplusplus
}
#endif
#endif
