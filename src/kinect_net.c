#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* Latest-frame UDP. No TCP, no per-packet malloc, no frame queue.
 * Depth and images move as 8×8 blocks. A dirty block jumps the line.
 * A sweep cursor heals loss. The kernel socket buffer is small on purpose
 * so an overrun drops old datagrams instead of queueing them. */
#include "kinect_net.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define KNET_MAGIC 0x314E704Bu
#define KNET_VER 1
#define KNET_HDR 32
#define KNET_BLOCK 130
#define KNET_PKT_BLOCKS 8
#define KNET_DEPTH_MIN_US 50000u
#define KNET_IMG_MIN_US 125000u
#define KNET_BURST 400
#define KNET_STEADY 160
#define KNET_IMG_BURST 240
#define KNET_IMG_STEADY 80
#define KNET_MAX_W 640
#define KNET_MAX_H 480
#define KNET_MAX_PIX (KNET_MAX_W * KNET_MAX_H)
#define KNET_IMG_W 320
#define KNET_IMG_H 240
#define KNET_IMG_PIX (KNET_IMG_W * KNET_IMG_H)

enum {
  KNET_HELLO = 1,
  KNET_ACK = 2,
  KNET_PING = 3,
  KNET_PONG = 4,
  KNET_DEPTH = 5,
  KNET_RGB = 6,
  KNET_IR = 7,
  KNET_BYE = 8,
  KNET_WANT = 9,
  KNET_PARAMS = 10
};

#define KNET_F_KEY 1u
#define KNET_F_DEPTH 2u
#define KNET_F_RGB 4u
#define KNET_F_IR 8u

typedef struct {
  uint16_t *offer;
  uint16_t *enc;
  uint16_t *base;
  uint16_t *remote;
  int w, h;
  int enc_w, enc_h;
  uint64_t ts;
  uint32_t gen;
  uint32_t seen;
  uint16_t seq;
  uint16_t rx_seq;
  int have_rx;
  uint32_t remote_gen;
  uint32_t taken_gen;
  uint64_t taken_us;
  uint64_t remote_ts;
  int remote_w, remote_h;
  unsigned sent_ever;
  int dirty_cursor;
  int sweep;
  uint64_t last_tx_us;
} DepthSlot;

static pthread_t th;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t life_mu = PTHREAD_MUTEX_INITIALIZER;
static volatile int stop_flag;
static int started;
static int fd = -1;
static int bound_port;
static struct sockaddr_in peer;
static int peer_set;
static char peer_host[128];
static volatile int connected;
static volatile int local_mask;
static volatile int peer_mask;
static volatile int peer_want;
static volatile int want_kind = 1;
static volatile int want_dirty;
static uint64_t last_rx_us;
static uint64_t last_hello_us;
static uint64_t last_ping_us;
static uint64_t last_params_us;
static int64_t clock_off;
static int clock_ok;
static int64_t best_rtt;
static unsigned tx_pkts, rx_pkts, tx_blocks;
static char net_err[96];
static DepthSlot depth[2];
static uint16_t *img_offer, *img_enc, *img_base, *img_remote;
static int img_sensor;
static int img_kind;
static uint64_t img_ts;
static uint32_t img_gen, img_seen;
static uint16_t img_seq, img_rx_seq;
static int img_have_rx;
static uint32_t img_remote_gen, img_taken_gen;
static uint64_t img_taken_us, img_remote_ts;
static unsigned img_sent_ever;
static int img_dirty_cursor, img_sweep;
static uint64_t img_last_tx_us;
static int img_ready;
static int img_reset;
static float cal_fx[2], cal_fy[2], cal_cx[2], cal_cy[2];
static int cal_have[2];
static int cal_send[2];
static float rx_fx[2], rx_fy[2], rx_cx[2], rx_cy[2];
static int rx_cal[2];

static uint64_t mono_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void set_err(const char *s) {
  snprintf(net_err, sizeof net_err, "%s", s ? s : "");
}

static int env_port(void) {
  const char *e = getenv("KINECT_SHARE_PORT");
  int p;
  if (!e || !e[0]) return KNET_PORT_DEFAULT;
  p = atoi(e);
  if (p < 1 || p > 65535) return KNET_PORT_DEFAULT;
  return p;
}

static unsigned rd16(const uint8_t *p) { return (unsigned)p[0] | ((unsigned)p[1] << 8); }
static unsigned rd32(const uint8_t *p) {
  return (unsigned)p[0] | ((unsigned)p[1] << 8) | ((unsigned)p[2] << 16) | ((unsigned)p[3] << 24);
}
static uint64_t rd64(const uint8_t *p) {
  return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32);
}
static void wr16(uint8_t *p, unsigned v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}
static void wr32(uint8_t *p, unsigned v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}
static void wr64(uint8_t *p, uint64_t v) {
  wr32(p, (unsigned)v);
  wr32(p + 4, (unsigned)(v >> 32));
}

static uint8_t want_flags(void) {
  int k = want_kind;
  if (k == 2) return (uint8_t)KNET_F_RGB;
  if (k == 3) return (uint8_t)KNET_F_IR;
  return (uint8_t)KNET_F_DEPTH;
}

static int peer_wants(unsigned bit) {
  unsigned w = (unsigned)peer_want;
  if (w == 0) w = KNET_F_DEPTH;
  return (w & bit) != 0;
}

static int ensure_bufs(void) {
  int i;
  for (i = 0; i < 2; i++) {
    if (depth[i].offer) continue;
    depth[i].offer = (uint16_t *)calloc(KNET_MAX_PIX, sizeof(uint16_t));
    depth[i].enc = (uint16_t *)calloc(KNET_MAX_PIX, sizeof(uint16_t));
    depth[i].base = (uint16_t *)calloc(KNET_MAX_PIX, sizeof(uint16_t));
    depth[i].remote = (uint16_t *)calloc(KNET_MAX_PIX, sizeof(uint16_t));
    if (!depth[i].offer || !depth[i].enc || !depth[i].base || !depth[i].remote) return -1;
  }
  if (!img_offer) {
    img_offer = (uint16_t *)calloc(KNET_IMG_PIX, sizeof(uint16_t));
    img_enc = (uint16_t *)calloc(KNET_IMG_PIX, sizeof(uint16_t));
    img_base = (uint16_t *)calloc(KNET_IMG_PIX, sizeof(uint16_t));
    img_remote = (uint16_t *)calloc(KNET_IMG_PIX, sizeof(uint16_t));
    if (!img_offer || !img_enc || !img_base || !img_remote) return -1;
  }
  return 0;
}

static int open_bound(int port) {
  int s, yes = 1, buf = 64 * 1024, fl;
  struct sockaddr_in a;
  s = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (s < 0) return -1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
  setsockopt(s, SOL_SOCKET, SO_RCVBUF, &buf, sizeof buf);
  setsockopt(s, SOL_SOCKET, SO_SNDBUF, &buf, sizeof buf);
  memset(&a, 0, sizeof a);
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port = htons((unsigned)port);
  if (bind(s, (struct sockaddr *)&a, sizeof a) < 0) {
    close(s);
    return -1;
  }
  fl = fcntl(s, F_GETFL, 0);
  if (fl >= 0) fcntl(s, F_SETFL, fl | O_NONBLOCK);
  return s;
}

static int split_hp(const char *s, char *host, size_t nh, int *port) {
  const char *col;
  if (!s || !s[0] || !host || nh < 2 || !port) return -1;
  *port = env_port();
  col = strrchr(s, ':');
  if (col && col != s && strchr(s, ':') == col) {
    int p = atoi(col + 1);
    size_t n = (size_t)(col - s);
    if (p < 1 || p > 65535 || n + 1 > nh || n < 1) return -1;
    memcpy(host, s, n);
    host[n] = 0;
    *port = p;
    return 0;
  }
  snprintf(host, nh, "%s", s);
  return host[0] ? 0 : -1;
}

static int resolve_host(const char *host, int port, struct sockaddr_in *out) {
  char pstr[8];
  struct addrinfo hints, *res = NULL;
  snprintf(pstr, sizeof pstr, "%d", port);
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  if (getaddrinfo(host, pstr, &hints, &res) != 0 || !res) return -1;
  memset(out, 0, sizeof *out);
  memcpy(out, res->ai_addr, sizeof *out);
  freeaddrinfo(res);
  return out->sin_family == AF_INET ? 0 : -1;
}

static int send_raw(const struct sockaddr_in *to, const uint8_t *pkt, int n) {
  int s;
  ssize_t w;
  if (!to || fd < 0 || n < KNET_HDR) return -1;
  s = fd;
  w = sendto(s, pkt, (size_t)n, 0, (const struct sockaddr *)to, sizeof *to);
  if (w == n) {
    tx_pkts++;
    return 0;
  }
  return -1;
}

static void build_hdr(uint8_t *p, uint8_t type, uint8_t sensor, uint8_t flags, uint16_t seq,
                      uint16_t chunk, uint16_t nchunks, uint16_t w, uint16_t h, uint64_t ts,
                      uint16_t nbytes) {
  memset(p, 0, KNET_HDR);
  wr32(p, KNET_MAGIC);
  p[4] = KNET_VER;
  p[5] = type;
  p[6] = sensor;
  p[7] = (uint8_t)(flags | want_flags());
  wr16(p + 8, seq);
  wr16(p + 10, chunk);
  wr16(p + 12, nchunks);
  wr16(p + 14, w);
  wr16(p + 16, h);
  wr64(p + 20, ts);
  wr16(p + 28, nbytes);
}

static int send_msg(const struct sockaddr_in *to, uint8_t type, uint8_t sensor, uint8_t flags,
                    uint16_t seq, uint16_t w, uint16_t h, uint64_t ts, const void *payload,
                    int nbytes) {
  uint8_t pkt[1200];
  if (nbytes < 0 || nbytes > (int)sizeof pkt - KNET_HDR) return -1;
  build_hdr(pkt, type, sensor, flags, seq, 0, 1, w, h, ts, (uint16_t)nbytes);
  if (nbytes && payload) memcpy(pkt + KNET_HDR, payload, (size_t)nbytes);
  return send_raw(to, pkt, KNET_HDR + nbytes);
}

static int pix_changed(uint16_t a, uint16_t b, int gate, int v1) {
  int va = v1 ? (a > 0 && a < 2047) : (a > 0);
  int vb = v1 ? (b > 0 && b < 2047) : (b > 0);
  int d;
  if (va != vb) return 1;
  if (!va) return 0;
  d = (int)a - (int)b;
  if (d < 0) d = -d;
  return d >= gate;
}

static int block_dirty(const uint16_t *cur, const uint16_t *base, int w, int idx, int gate, int v1,
                       int min_ch) {
  int bw = w / 8;
  int bx = idx % bw;
  int by = idx / bw;
  int y, x, ch = 0;
  for (y = 0; y < 8; y++) {
    const uint16_t *a = base + ((by * 8 + y) * w + bx * 8);
    const uint16_t *b = cur + ((by * 8 + y) * w + bx * 8);
    for (x = 0; x < 8; x++) {
      if (pix_changed(a[x], b[x], gate, v1) && ++ch >= min_ch) return 1;
    }
  }
  return 0;
}

static void pack_block(uint8_t *dst, const uint16_t *src, int w, int idx) {
  int bw = w / 8;
  int bx = idx % bw;
  int by = idx / bw;
  int y, x, k = 0;
  wr16(dst, (unsigned)idx);
  for (y = 0; y < 8; y++) {
    const uint16_t *row = src + ((by * 8 + y) * w + bx * 8);
    for (x = 0; x < 8; x++, k++) wr16(dst + 2 + k * 2, row[x]);
  }
}

static void copy_block(uint16_t *dst, const uint16_t *src, int w, int idx) {
  int bw = w / 8;
  int bx = idx % bw;
  int by = idx / bw;
  int y;
  for (y = 0; y < 8; y++) {
    uint16_t *d = dst + ((by * 8 + y) * w + bx * 8);
    const uint16_t *s = src + ((by * 8 + y) * w + bx * 8);
    memcpy(d, s, 8 * sizeof(uint16_t));
  }
}

static void apply_block(uint16_t *dst, int w, int h, int idx, const uint8_t *pix) {
  int bw = w / 8;
  int nblocks = bw * (h / 8);
  int bx, by, y, x, k = 0;
  if (bw < 1 || idx < 0 || idx >= nblocks) return;
  bx = idx % bw;
  by = idx / bw;
  for (y = 0; y < 8; y++) {
    uint16_t *row = dst + ((by * 8 + y) * w + bx * 8);
    for (x = 0; x < 8; x++, k++) row[x] = (uint16_t)rd16(pix + k * 2);
  }
}

static int send_queued(const struct sockaddr_in *to, uint8_t type, uint8_t sensor, uint16_t seq,
                       uint16_t w, uint16_t h, uint64_t ts, uint8_t *payload, int *nblk,
                       int *chunk) {
  int nbytes;
  uint8_t pkt[KNET_HDR + KNET_PKT_BLOCKS * KNET_BLOCK];
  if (*nblk < 1) return 0;
  nbytes = *nblk * KNET_BLOCK;
  build_hdr(pkt, type, sensor, 0, seq, (uint16_t)(*chunk), 0, w, h, ts, (uint16_t)nbytes);
  memcpy(pkt + KNET_HDR, payload, (size_t)nbytes);
  *nblk = 0;
  (*chunk)++;
  tx_blocks += (unsigned)(nbytes / KNET_BLOCK);
  return send_raw(to, pkt, KNET_HDR + nbytes);
}

static void encode_plane(const struct sockaddr_in *to, uint8_t type, uint8_t sensor, uint16_t *enc,
                         uint16_t *base, int w, int h, uint16_t seq, uint64_t ts, int *cursor,
                         int *sweep, unsigned *sent_ever, int gate, int v1, int min_ch, int burst,
                         int steady) {
  int nblocks, budget, sent = 0, scanned = 0, nblk = 0, chunk = 0;
  int sw_guard;
  uint8_t payload[KNET_PKT_BLOCKS * KNET_BLOCK];
  uint8_t seen[(KNET_MAX_PIX / 64 + 7) / 8];
  if (w < 8 || h < 8 || (w % 8) || (h % 8)) return;
  nblocks = (w / 8) * (h / 8);
  if (nblocks < 1 || nblocks > KNET_MAX_PIX / 64) return;
  budget = (*sent_ever < (unsigned)nblocks) ? burst : steady;
  if (budget > nblocks) budget = nblocks;
  memset(seen, 0, (size_t)((nblocks + 7) / 8));
  while (sent < budget && scanned < nblocks) {
    int idx = *cursor;
    if (idx < 0 || idx >= nblocks) idx = 0;
    if (block_dirty(enc, base, w, idx, gate, v1, min_ch)) {
      if (nblk >= KNET_PKT_BLOCKS)
        send_queued(to, type, sensor, seq, (uint16_t)w, (uint16_t)h, ts, payload, &nblk, &chunk);
      pack_block(payload + nblk * KNET_BLOCK, enc, w, idx);
      nblk++;
      copy_block(base, enc, w, idx);
      seen[idx >> 3] = (uint8_t)(seen[idx >> 3] | (uint8_t)(1u << (idx & 7)));
      sent++;
    }
    *cursor = idx + 1;
    if (*cursor >= nblocks) *cursor = 0;
    scanned++;
  }
  sw_guard = nblocks;
  while (sent < budget && sw_guard-- > 0) {
    int idx = *sweep;
    if (idx < 0 || idx >= nblocks) idx = 0;
    *sweep = idx + 1;
    if (*sweep >= nblocks) *sweep = 0;
    if (seen[idx >> 3] & (uint8_t)(1u << (idx & 7))) continue;
    if (nblk >= KNET_PKT_BLOCKS)
      send_queued(to, type, sensor, seq, (uint16_t)w, (uint16_t)h, ts, payload, &nblk, &chunk);
    pack_block(payload + nblk * KNET_BLOCK, enc, w, idx);
    nblk++;
    seen[idx >> 3] = (uint8_t)(seen[idx >> 3] | (uint8_t)(1u << (idx & 7)));
    copy_block(base, enc, w, idx);
    sent++;
  }
  send_queued(to, type, sensor, seq, (uint16_t)w, (uint16_t)h, ts, payload, &nblk, &chunk);
  *sent_ever += (unsigned)sent;
}

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void rgb565_expand(uint8_t r5, uint8_t g6, uint8_t b5, uint8_t *rgb) {
  rgb[0] = (uint8_t)((r5 << 3) | (r5 >> 2));
  rgb[1] = (uint8_t)((g6 << 2) | (g6 >> 4));
  rgb[2] = (uint8_t)((b5 << 3) | (b5 >> 2));
}

static void downsample_rgb(const uint8_t *rgb, uint16_t *dst) {
  int y, x;
  for (y = 0; y < KNET_IMG_H; y++) {
    const uint8_t *row = rgb + (size_t)(y * 2) * KNET_MAX_W * 3;
    for (x = 0; x < KNET_IMG_W; x++) {
      const uint8_t *p = row + (size_t)(x * 2) * 3;
      dst[y * KNET_IMG_W + x] = rgb565(p[0], p[1], p[2]);
    }
  }
}

static uint64_t local_stamp(uint64_t remote_ts, uint64_t arrival) {
  int64_t est;
  if (!clock_ok) return arrival;
  est = (int64_t)remote_ts - clock_off;
  if (est < 0) return arrival;
  return (uint64_t)est;
}

static int from_peer(const struct sockaddr_in *from) {
  if (!peer_set) return 1;
  return from && peer.sin_addr.s_addr == from->sin_addr.s_addr;
}

static void learn_mask(const uint8_t *payload, int nbytes) {
  if (nbytes >= 1) peer_mask = payload[0] & 3;
}

static void on_clock(uint64_t t1, uint64_t t2, uint64_t t3) {
  int64_t rtt, off;
  if (t3 < t1) return;
  rtt = (int64_t)(t3 - t1);
  if (rtt < 0 || rtt > 2000000) return;
  off = (int64_t)t2 - (int64_t)((t1 + t3) / 2);
  if (!clock_ok || rtt <= best_rtt) {
    best_rtt = rtt;
    clock_off = off;
    clock_ok = 1;
  } else if (rtt < best_rtt * 2 + 1000) {
    clock_off = (clock_off * 7 + off) / 8;
  }
  if (best_rtt < 500000) best_rtt += best_rtt / 32 + 50;
}

static void apply_depth_packet(uint8_t sensor, uint16_t seq, uint16_t w, uint16_t h, uint64_t ts,
                               const uint8_t *payload, int nbytes) {
  DepthSlot *s;
  int i, n, si;
  uint64_t now;
  if (sensor != 1 && sensor != 2) return;
  if (w < 8 || h < 8 || (w % 8) || (h % 8) || w > KNET_MAX_W || h > KNET_MAX_H) return;
  if ((int)w * (int)h > KNET_MAX_PIX) return;
  si = sensor - 1;
  s = &depth[si];
  if (s->have_rx) {
    unsigned behind = (uint16_t)(s->rx_seq - seq);
    if (behind != 0 && behind < 32768u) return;
  }
  s->have_rx = 1;
  s->rx_seq = seq;
  if (s->remote_w != (int)w || s->remote_h != (int)h) {
    memset(s->remote, 0, (size_t)KNET_MAX_PIX * sizeof(uint16_t));
    s->remote_w = (int)w;
    s->remote_h = (int)h;
  }
  n = nbytes / KNET_BLOCK;
  for (i = 0; i < n; i++) {
    const uint8_t *b = payload + i * KNET_BLOCK;
    apply_block(s->remote, (int)w, (int)h, (int)rd16(b), b + 2);
  }
  now = mono_us();
  s->remote_ts = local_stamp(ts, now);
  s->remote_gen++;
  if (s->remote_gen == 0) s->remote_gen = 1;
}

static void apply_image_packet(uint8_t type, uint16_t seq, uint16_t w, uint16_t h, uint64_t ts,
                               const uint8_t *payload, int nbytes) {
  int i, n;
  if (w != KNET_IMG_W || h != KNET_IMG_H) return;
  if (img_have_rx) {
    unsigned behind = (uint16_t)(img_rx_seq - seq);
    if (behind != 0 && behind < 32768u) return;
  }
  img_have_rx = 1;
  img_rx_seq = seq;
  (void)type;
  n = nbytes / KNET_BLOCK;
  for (i = 0; i < n; i++) {
    const uint8_t *b = payload + i * KNET_BLOCK;
    apply_block(img_remote, KNET_IMG_W, KNET_IMG_H, (int)rd16(b), b + 2);
  }
  img_remote_ts = local_stamp(ts, mono_us());
  img_remote_gen++;
  if (img_remote_gen == 0) img_remote_gen = 1;
}

static void handle_packet(const uint8_t *pkt, int n, const struct sockaddr_in *from) {
  uint8_t type, sensor, flags;
  uint16_t seq, w, h, nbytes;
  uint64_t ts;
  const uint8_t *payload;
  struct sockaddr_in to;
  if (n < KNET_HDR) return;
  if (rd32(pkt) != KNET_MAGIC || pkt[4] != KNET_VER) return;
  if (!from_peer(from)) return;
  type = pkt[5];
  sensor = pkt[6];
  flags = pkt[7];
  seq = (uint16_t)rd16(pkt + 8);
  w = (uint16_t)rd16(pkt + 14);
  h = (uint16_t)rd16(pkt + 16);
  ts = rd64(pkt + 20);
  nbytes = (uint16_t)rd16(pkt + 28);
  if (KNET_HDR + (int)nbytes > n) return;
  payload = pkt + KNET_HDR;
  pthread_mutex_lock(&mu);
  if (!peer_set) {
    peer = *from;
    peer_set = 1;
  } else if (peer.sin_addr.s_addr != from->sin_addr.s_addr) {
    pthread_mutex_unlock(&mu);
    return;
  } else if (peer.sin_port != from->sin_port) {
    peer.sin_port = from->sin_port;
  }
  connected = 1;
  last_rx_us = mono_us();
  if (flags & (KNET_F_DEPTH | KNET_F_RGB | KNET_F_IR)) peer_want = flags & (KNET_F_DEPTH | KNET_F_RGB | KNET_F_IR);
  rx_pkts++;
  to = peer;
  pthread_mutex_unlock(&mu);

  if (type == KNET_HELLO || type == KNET_ACK || type == KNET_PING) learn_mask(payload, nbytes);
  if (type == KNET_HELLO) {
    uint8_t mask = (uint8_t)(local_mask & 3);
    send_msg(&to, KNET_ACK, 0, 0, 0, 0, 0, mono_us(), &mask, 1);
  } else if (type == KNET_PING) {
    uint8_t echo[8];
    wr64(echo, ts);
    send_msg(&to, KNET_PONG, 0, 0, seq, 0, 0, mono_us(), echo, 8);
  } else if (type == KNET_PONG && nbytes >= 8) {
    uint64_t t1 = rd64(payload);
    pthread_mutex_lock(&mu);
    on_clock(t1, ts, mono_us());
    pthread_mutex_unlock(&mu);
  } else if (type == KNET_DEPTH) {
    pthread_mutex_lock(&mu);
    apply_depth_packet(sensor, seq, w, h, ts, payload, nbytes);
    pthread_mutex_unlock(&mu);
  } else if (type == KNET_RGB || type == KNET_IR) {
    pthread_mutex_lock(&mu);
    apply_image_packet(type, seq, w, h, ts, payload, nbytes);
    pthread_mutex_unlock(&mu);
  } else if (type == KNET_PARAMS && nbytes >= 16 && (sensor == 1 || sensor == 2)) {
    float f[4];
    memcpy(f, payload, sizeof f);
    pthread_mutex_lock(&mu);
    rx_fx[sensor - 1] = f[0];
    rx_fy[sensor - 1] = f[1];
    rx_cx[sensor - 1] = f[2];
    rx_cy[sensor - 1] = f[3];
    rx_cal[sensor - 1] = (f[0] > 1.f && f[1] > 1.f);
    pthread_mutex_unlock(&mu);
  } else if (type == KNET_BYE) {
    pthread_mutex_lock(&mu);
    connected = 0;
    peer_mask = 0;
    pthread_mutex_unlock(&mu);
  }
}

static void service_tx(void) {
  struct sockaddr_in to;
  int have, i, send_want;
  uint64_t now;
  uint8_t mask;
  float csend[2][4];
  int do_cal[2];
  pthread_mutex_lock(&mu);
  now = mono_us();
  have = peer_set && fd >= 0;
  to = peer;
  if (connected && last_rx_us && now - last_rx_us > 3000000ull) {
    connected = 0;
    peer_mask = 0;
  }
  send_want = want_dirty;
  want_dirty = 0;
  for (i = 0; i < 2; i++) {
    DepthSlot *s = &depth[i];
    if (s->gen != s->seen && s->w >= 8 && s->h >= 8) {
      memcpy(s->enc, s->offer, (size_t)s->w * (size_t)s->h * sizeof(uint16_t));
      if (s->enc_w != s->w || s->enc_h != s->h) {
        memset(s->base, 0, (size_t)KNET_MAX_PIX * sizeof(uint16_t));
        s->enc_w = s->w;
        s->enc_h = s->h;
        s->sent_ever = 0;
        s->dirty_cursor = 0;
        s->sweep = 0;
      }
      s->seen = s->gen;
      s->seq++;
      if (s->seq == 0) s->seq = 1;
    }
    do_cal[i] = cal_send[i];
    csend[i][0] = cal_fx[i];
    csend[i][1] = cal_fy[i];
    csend[i][2] = cal_cx[i];
    csend[i][3] = cal_cy[i];
  }
  if (img_gen != img_seen && img_offer) {
    memcpy(img_enc, img_offer, (size_t)KNET_IMG_PIX * sizeof(uint16_t));
    if (!img_ready || img_reset) {
      memset(img_base, 0, (size_t)KNET_IMG_PIX * sizeof(uint16_t));
      img_sent_ever = 0;
      img_dirty_cursor = 0;
      img_sweep = 0;
      img_ready = 1;
      img_reset = 0;
    }
    img_seen = img_gen;
    img_seq++;
    if (img_seq == 0) img_seq = 1;
  }
  pthread_mutex_unlock(&mu);
  if (!have) return;

  mask = (uint8_t)(local_mask & 3);
  if (send_want) send_msg(&to, KNET_WANT, 0, 0, 0, 0, 0, now, &mask, 1);
  if (now - last_hello_us > 500000ull) {
    send_msg(&to, KNET_HELLO, 0, 0, 0, 0, 0, now, &mask, 1);
    last_hello_us = now;
  }
  if (now - last_ping_us > 200000ull) {
    send_msg(&to, KNET_PING, 0, 0, 0, 0, 0, now, &mask, 1);
    last_ping_us = now;
  }
  if (now - last_params_us > 500000ull) {
    for (i = 0; i < 2; i++) {
      uint8_t raw[16];
      if (!do_cal[i]) continue;
      memcpy(raw, csend[i], 16);
      send_msg(&to, KNET_PARAMS, (uint8_t)(i + 1), 0, 0, 0, 0, now, raw, 16);
    }
    last_params_us = now;
  }

  if (peer_wants(KNET_F_DEPTH)) {
    for (i = 0; i < 2; i++) {
      DepthSlot *s = &depth[i];
      int w, h, cursor, sweep;
      unsigned sent_ever;
      uint16_t seq;
      uint64_t ts;
      pthread_mutex_lock(&mu);
      w = s->enc_w;
      h = s->enc_h;
      if (s->seen == 0 || now - s->last_tx_us < KNET_DEPTH_MIN_US || w < 8) {
        pthread_mutex_unlock(&mu);
        continue;
      }
      s->last_tx_us = now;
      seq = s->seq;
      ts = s->ts;
      cursor = s->dirty_cursor;
      sweep = s->sweep;
      sent_ever = s->sent_ever;
      pthread_mutex_unlock(&mu);
      encode_plane(&to, KNET_DEPTH, (uint8_t)(i + 1), s->enc, s->base, w, h, seq, ts, &cursor,
                   &sweep, &sent_ever, i == 0 ? 12 : 30, i == 0, 4, KNET_BURST, KNET_STEADY);
      pthread_mutex_lock(&mu);
      s->dirty_cursor = cursor;
      s->sweep = sweep;
      s->sent_ever = sent_ever;
      pthread_mutex_unlock(&mu);
    }
  }

  {
    int kind, sensor, cursor, sweep;
    unsigned sent_ever;
    uint16_t seq;
    uint64_t ts;
    unsigned bit;
    pthread_mutex_lock(&mu);
    kind = img_kind;
    sensor = img_sensor;
    bit = (kind == 3) ? KNET_F_IR : KNET_F_RGB;
    if (!img_ready || img_seen == 0 || now - img_last_tx_us < KNET_IMG_MIN_US || !peer_wants(bit)) {
      pthread_mutex_unlock(&mu);
    } else {
      img_last_tx_us = now;
      seq = img_seq;
      ts = img_ts;
      cursor = img_dirty_cursor;
      sweep = img_sweep;
      sent_ever = img_sent_ever;
      pthread_mutex_unlock(&mu);
      encode_plane(&to, (uint8_t)(kind == 3 ? KNET_IR : KNET_RGB), (uint8_t)(sensor ? sensor : 1),
                   img_enc, img_base, KNET_IMG_W, KNET_IMG_H, seq, ts, &cursor, &sweep, &sent_ever,
                   1, 0, 4, KNET_IMG_BURST, KNET_IMG_STEADY);
      pthread_mutex_lock(&mu);
      img_dirty_cursor = cursor;
      img_sweep = sweep;
      img_sent_ever = sent_ever;
      pthread_mutex_unlock(&mu);
    }
  }
}

static void *net_main(void *arg) {
  (void)arg;
  while (!stop_flag) {
    struct pollfd p;
    uint8_t buf[2048];
    int nread = 0;
    p.fd = fd;
    p.events = POLLIN;
    p.revents = 0;
    if (fd >= 0) {
      int pr = poll(&p, 1, 5);
      if (pr < 0 && errno == EINTR) continue;
    } else {
      usleep(5000);
    }
    while (fd >= 0 && nread < 64 && !stop_flag) {
      struct sockaddr_in from;
      socklen_t fl = sizeof from;
      ssize_t n = recvfrom(fd, buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
      if (n < 0) break;
      nread++;
      handle_packet(buf, (int)n, &from);
    }
    service_tx();
  }
  if (peer_set && fd >= 0) {
    uint8_t mask = (uint8_t)(local_mask & 3);
    send_msg(&peer, KNET_BYE, 0, 0, 0, 0, 0, mono_us(), &mask, 1);
  }
  return NULL;
}

static int start_thread(void) {
  if (ensure_bufs() < 0) {
    set_err("oom");
    return -1;
  }
  stop_flag = 0;
  if (pthread_create(&th, NULL, net_main, NULL) != 0) {
    set_err("net thread");
    return -1;
  }
  started = 1;
  return 0;
}

int kinect_net_listen(int port) {
  char saved[128];
  int s, have_saved = 0;
  if (port <= 0) port = env_port();
  if (port < 1 || port > 65535) {
    set_err("bad port");
    return -1;
  }
  pthread_mutex_lock(&life_mu);
  if (started && bound_port == port) {
    pthread_mutex_unlock(&life_mu);
    return 0;
  }
  snprintf(saved, sizeof saved, "%s", peer_host);
  have_saved = saved[0] != 0;
  if (started) {
    pthread_mutex_unlock(&life_mu);
    kinect_net_stop();
    pthread_mutex_lock(&life_mu);
  }
  s = open_bound(port);
  if (s < 0) {
    set_err("bind failed");
    pthread_mutex_unlock(&life_mu);
    return -1;
  }
  fd = s;
  bound_port = port;
  if (start_thread() != 0) {
    close(fd);
    fd = -1;
    bound_port = 0;
    pthread_mutex_unlock(&life_mu);
    return -1;
  }
  set_err("");
  pthread_mutex_unlock(&life_mu);
  if (have_saved) kinect_net_connect(saved);
  return 0;
}

int kinect_net_connect(const char *hostport) {
  char host[128];
  int port = 0;
  struct sockaddr_in addr;
  const char *spec = hostport;
  if (!spec || !spec[0]) spec = getenv("KINECT_PEER");
  if (split_hp(spec, host, sizeof host, &port) != 0) {
    set_err("need host");
    return -1;
  }
  if (resolve_host(host, port, &addr) != 0) {
    set_err("peer resolve failed");
    return -1;
  }
  pthread_mutex_lock(&life_mu);
  if (!started) {
    int s = open_bound(0);
    if (s < 0) {
      set_err("socket failed");
      pthread_mutex_unlock(&life_mu);
      return -1;
    }
    fd = s;
    bound_port = 0;
    if (start_thread() != 0) {
      close(fd);
      fd = -1;
      pthread_mutex_unlock(&life_mu);
      return -1;
    }
  }
  pthread_mutex_lock(&mu);
  peer = addr;
  peer_set = 1;
  snprintf(peer_host, sizeof peer_host, "%s", spec);
  last_hello_us = 0;
  pthread_mutex_unlock(&mu);
  set_err("");
  pthread_mutex_unlock(&life_mu);
  return 0;
}

void kinect_net_stop(void) {
  pthread_t t;
  int was, s;
  pthread_mutex_lock(&life_mu);
  was = started;
  if (!was) {
    pthread_mutex_unlock(&life_mu);
    return;
  }
  stop_flag = 1;
  t = th;
  started = 0;
  pthread_mutex_unlock(&life_mu);
  pthread_join(t, NULL);
  pthread_mutex_lock(&life_mu);
  s = fd;
  fd = -1;
  bound_port = 0;
  pthread_mutex_unlock(&life_mu);
  if (s >= 0) close(s);
  pthread_mutex_lock(&mu);
  peer_set = 0;
  connected = 0;
  peer_mask = 0;
  peer_host[0] = 0;
  clock_ok = 0;
  best_rtt = 0;
  last_rx_us = 0;
  depth[0].remote_gen = depth[1].remote_gen = 0;
  depth[0].taken_gen = depth[1].taken_gen = 0;
  depth[0].have_rx = depth[1].have_rx = 0;
  img_remote_gen = 0;
  img_taken_gen = 0;
  img_have_rx = 0;
  rx_cal[0] = rx_cal[1] = 0;
  pthread_mutex_unlock(&mu);
}

int kinect_net_active(void) { return started; }
int kinect_net_connected(void) { return connected; }
int kinect_net_listen_port(void) { return started ? bound_port : 0; }
int kinect_net_peer_sensors(void) { return connected ? (peer_mask & 3) : 0; }
int kinect_net_local_sensors(void) { return local_mask & 3; }
int kinect_net_rtt_ms(void) {
  if (!clock_ok) return -1;
  return (int)(best_rtt / 1000);
}
const char *kinect_net_error(void) { return net_err; }

void kinect_net_set_local(int mask) {
  local_mask = mask & 3;
  want_dirty = 1;
}

void kinect_net_want(int kind) {
  if (kind < 1 || kind > 3) kind = 1;
  want_kind = kind;
  want_dirty = 1;
}

void kinect_net_offer_depth(int sensor, const void *data, int w, int h, int is_float_mm,
                            uint64_t ts_us) {
  DepthSlot *s;
  int i, n;
  if (!started || !data || sensor < 1 || sensor > 2) return;
  if (w < 8 || h < 8 || (w % 8) || (h % 8) || w > KNET_MAX_W || h > KNET_MAX_H) return;
  n = w * h;
  if (!peer_wants(KNET_F_DEPTH) && peer_want != 0) return;
  if (ensure_bufs() < 0) return;
  s = &depth[sensor - 1];
  pthread_mutex_lock(&mu);
  if (is_float_mm) {
    const float *z = (const float *)data;
    for (i = 0; i < n; i++) {
      float v = z[i];
      if (!isfinite(v) || v <= 0.f || v >= 65535.f)
        s->offer[i] = 0;
      else
        s->offer[i] = (uint16_t)(v + 0.5f);
    }
  } else {
    memcpy(s->offer, data, (size_t)n * sizeof(uint16_t));
  }
  if (n < KNET_MAX_PIX && (s->w != w || s->h != h))
    memset(s->offer + n, 0, (size_t)(KNET_MAX_PIX - n) * sizeof(uint16_t));
  s->w = w;
  s->h = h;
  s->ts = ts_us ? ts_us : mono_us();
  s->gen++;
  if (s->gen == 0) s->gen = 1;
  pthread_mutex_unlock(&mu);
}

static void offer_image(const uint8_t *rgb640, int sensor, int kind, uint64_t ts_us) {
  unsigned bit = (kind == 3) ? KNET_F_IR : KNET_F_RGB;
  if (!started || !rgb640) return;
  if (peer_want == 0 || !peer_wants(bit)) return;
  if (ensure_bufs() < 0) return;
  pthread_mutex_lock(&mu);
  downsample_rgb(rgb640, img_offer);
  img_sensor = (sensor == 2) ? 2 : 1;
  if (img_kind != kind) img_reset = 1;
  img_kind = kind;
  img_ts = ts_us ? ts_us : mono_us();
  img_gen++;
  if (img_gen == 0) img_gen = 1;
  pthread_mutex_unlock(&mu);
}

void kinect_net_offer_rgb(const uint8_t *rgb640, int sensor, uint64_t ts_us) {
  offer_image(rgb640, sensor, 2, ts_us);
}
void kinect_net_offer_ir(const uint8_t *rgb640, int sensor, uint64_t ts_us) {
  offer_image(rgb640, sensor, 3, ts_us);
}

void kinect_net_offer_intrinsics(int sensor, float fx, float fy, float cx, float cy) {
  int i;
  if (sensor < 1 || sensor > 2) return;
  if (!(fx > 1.f) || !(fy > 1.f)) return;
  i = sensor - 1;
  pthread_mutex_lock(&mu);
  cal_fx[i] = fx;
  cal_fy[i] = fy;
  cal_cx[i] = cx;
  cal_cy[i] = cy;
  cal_send[i] = 1;
  pthread_mutex_unlock(&mu);
}

int kinect_net_take_depth(int sensor, uint16_t *dst, int cap, int *w, int *h, uint64_t *local_ts,
                          uint32_t *seq) {
  DepthSlot *s;
  int n, si;
  uint64_t now;
  if (!dst || sensor < 1 || sensor > 2) return 0;
  si = sensor - 1;
  if (!depth[si].remote) return 0;
  s = &depth[si];
  now = mono_us();
  pthread_mutex_lock(&mu);
  if (s->remote_gen == 0 || s->remote_gen == s->taken_gen || s->remote_w < 8) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  if (s->taken_gen != 0 && now - s->taken_us < 30000ull) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  n = s->remote_w * s->remote_h;
  if (cap < n) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  memcpy(dst, s->remote, (size_t)n * sizeof(uint16_t));
  if (w) *w = s->remote_w;
  if (h) *h = s->remote_h;
  if (local_ts) *local_ts = s->remote_ts;
  if (seq) *seq = s->rx_seq;
  s->taken_gen = s->remote_gen;
  s->taken_us = now;
  pthread_mutex_unlock(&mu);
  return 1;
}

int kinect_net_take_rgb(uint8_t *rgb640, uint64_t *local_ts) {
  int y, x;
  uint64_t now, ts;
  if (!rgb640 || !img_remote) return 0;
  now = mono_us();
  pthread_mutex_lock(&mu);
  if (img_remote_gen == 0 || img_remote_gen == img_taken_gen) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  if (img_taken_gen != 0 && now - img_taken_us < 80000ull) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  for (y = 0; y < KNET_IMG_H; y++) {
    for (x = 0; x < KNET_IMG_W; x++) {
      uint16_t p = img_remote[y * KNET_IMG_W + x];
      uint8_t rgb[3];
      int oy, ox;
      rgb565_expand((uint8_t)((p >> 11) & 31), (uint8_t)((p >> 5) & 63), (uint8_t)(p & 31), rgb);
      for (oy = 0; oy < 2; oy++) {
        uint8_t *row = rgb640 + ((size_t)(y * 2 + oy) * KNET_MAX_W + (size_t)(x * 2)) * 3;
        for (ox = 0; ox < 2; ox++) {
          row[ox * 3] = rgb[0];
          row[ox * 3 + 1] = rgb[1];
          row[ox * 3 + 2] = rgb[2];
        }
      }
    }
  }
  ts = img_remote_ts;
  img_taken_gen = img_remote_gen;
  img_taken_us = now;
  pthread_mutex_unlock(&mu);
  if (local_ts) *local_ts = ts;
  return 1;
}

int kinect_net_take_intrinsics(int sensor, float *fx, float *fy, float *cx, float *cy) {
  int i;
  if (sensor < 1 || sensor > 2 || !fx || !fy || !cx || !cy) return 0;
  i = sensor - 1;
  pthread_mutex_lock(&mu);
  if (!rx_cal[i]) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  *fx = rx_fx[i];
  *fy = rx_fy[i];
  *cx = rx_cx[i];
  *cy = rx_cy[i];
  pthread_mutex_unlock(&mu);
  return 1;
}

void kinect_net_counters(unsigned *txp, unsigned *rxp, unsigned *txb) {
  if (txp) *txp = tx_pkts;
  if (rxp) *rxp = rx_pkts;
  if (txb) *txb = tx_blocks;
}

void kinect_net_status(char *buf, size_t n) {
  int mask, rtt, up, port;
  if (!buf || !n) return;
  if (!started) {
    snprintf(buf, n, "Net off");
    return;
  }
  up = connected;
  mask = peer_mask & 3;
  rtt = kinect_net_rtt_ms();
  port = bound_port;
  if (!up) {
    if (port > 0)
      snprintf(buf, n, "Sharing :%d", port);
    else
      snprintf(buf, n, "Connecting…");
    return;
  }
  snprintf(buf, n, "Peer %s%s %dms", (mask & 1) ? "Kinect " : "", (mask & 2) ? "Kinect2" : "",
           rtt < 0 ? 0 : rtt);
}
