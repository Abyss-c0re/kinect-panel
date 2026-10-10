/* One UDP peer. Latest frame wins. Depth is 8×8 block deltas, images are
 * 320×240 RGB565. Clocks are CLOCK_MONOTONIC plus a min-RTT offset.
 * Share does not open a camera. */
#ifndef KINECT_NET_H
#define KINECT_NET_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KNET_PORT_DEFAULT 8765

/* bit0 = Kinect v1, bit1 = Kinect 2. */
int kinect_net_listen(int port);
int kinect_net_connect(const char *hostport);
void kinect_net_stop(void);
int kinect_net_active(void);
int kinect_net_connected(void);
int kinect_net_listen_port(void);
int kinect_net_peer_sensors(void);
int kinect_net_local_sensors(void);
/* -1 until a PONG lands. */
int kinect_net_rtt_ms(void);
void kinect_net_set_local(int mask);
const char *kinect_net_error(void);

/* sensor 1: uint16 raw 11-bit. sensor 2: uint16 millimetres, or float
 * millimetres when is_float_mm is set. w and h are multiples of 8, ≤ 640×480.
 * ts_us is the capture stamp on this machine's CLOCK_MONOTONIC. */
void kinect_net_offer_depth(int sensor, const void *data, int w, int h, int is_float_mm,
                            uint64_t ts_us);
void kinect_net_offer_rgb(const uint8_t *rgb640, int sensor, uint64_t ts_us);
void kinect_net_offer_ir(const uint8_t *rgb640, int sensor, uint64_t ts_us);
/* fx,fy,cx,cy of the undistorted depth camera. Repeated until the peer has it. */
void kinect_net_offer_intrinsics(int sensor, float fx, float fy, float cx, float cy);

/* 1 = a newer frame was copied into dst (uint16, cap samples). */
int kinect_net_take_depth(int sensor, uint16_t *dst, int cap, int *w, int *h,
                          uint64_t *local_ts, uint32_t *seq);
/* 640×480 RGB. 1 = a new image. */
int kinect_net_take_rgb(uint8_t *rgb640, uint64_t *local_ts);
int kinect_net_take_intrinsics(int sensor, float *fx, float *fy, float *cx, float *cy);

/* 1 depth (default), 2 rgb, 3 ir. Idle stays on depth so a peer is not pulled
 * off the cloud stream. */
void kinect_net_want(int kind);

void kinect_net_counters(unsigned *tx_pkts, unsigned *rx_pkts, unsigned *tx_blocks);
/* Short label. Does not touch USB. */
void kinect_net_status(char *buf, size_t n);

#ifdef __cplusplus
}
#endif
#endif
