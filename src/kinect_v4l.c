#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "kinect_v4l.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef V4L2_PIX_FMT_Y10BPACK
#define V4L2_PIX_FMT_Y10BPACK v4l2_fourcc('Y', '1', '0', 'B')
#endif

static const char *vcam_label(void) {
  const char *e = getenv("KINECT_VCAM_LABEL");
  return (e && e[0]) ? e : "Kinect Webcam";
}

static int read_trim(const char *path, char *out, size_t n) {
  FILE *f = fopen(path, "r");
  size_t L;
  if (!f) return -1;
  if (!fgets(out, (int)n, f)) {
    fclose(f);
    return -1;
  }
  fclose(f);
  L = strlen(out);
  while (L && (out[L - 1] == '\n' || out[L - 1] == '\r')) out[--L] = 0;
  return 0;
}

static void lower_copy(const char *in, char *out, size_t n) {
  size_t i;
  for (i = 0; i + 1 < n && in[i]; i++) out[i] = (char)tolower((unsigned char)in[i]);
  out[i] = 0;
}

typedef struct {
  char path[64];
  char name[128];
  char driver[64];
  char sys[256];
  int loopback;
} vdev;

static int is_loopback(const vdev *d) {
  char blob[256];
  snprintf(blob, sizeof blob, "%s %s %s", d->name, d->driver, d->sys);
  lower_copy(blob, blob, sizeof blob);
  if (strstr(d->sys, "/devices/virtual/")) return 1;
  if (strstr(blob, "loopback")) return 1;
  return 0;
}

static int collect(vdev *list, int max) {
  DIR *d = opendir("/dev");
  struct dirent *e;
  int n = 0;
  if (!d) return 0;
  while ((e = readdir(d)) && n < max) {
    char sys[256], tmp[256];
    vdev *v;
    if (strncmp(e->d_name, "video", 5) != 0) continue;
    if (e->d_name[5] < '0' || e->d_name[5] > '9') continue;
    v = &list[n];
    memset(v, 0, sizeof *v);
    snprintf(v->path, sizeof v->path, "/dev/%s", e->d_name);
    snprintf(sys, sizeof sys, "/sys/class/video4linux/%s", e->d_name);
    if (realpath(sys, v->sys) == NULL) v->sys[0] = 0;
    snprintf(tmp, sizeof tmp, "%s/name", sys);
    if (read_trim(tmp, v->name, sizeof v->name) < 0) v->name[0] = 0;
    snprintf(tmp, sizeof tmp, "%s/device/driver", sys);
    {
      char resolved[PATH_MAX];
      if (realpath(tmp, resolved) != NULL) {
        const char *base = strrchr(resolved, '/');
        snprintf(v->driver, sizeof v->driver, "%s", base ? base + 1 : resolved);
      }
    }
    v->loopback = is_loopback(v);
    n++;
  }
  closedir(d);
  /* sort by path */
  {
    int i, j;
    for (i = 0; i < n; i++)
      for (j = i + 1; j < n; j++)
        if (strcmp(list[j].path, list[i].path) < 0) {
          vdev t = list[i];
          list[i] = list[j];
          list[j] = t;
        }
  }
  return n;
}

int kinect_find_source(char *out, size_t n) {
  vdev list[32];
  int i, m = collect(list, 32);
  int best = -1, best_rank = 99;
  if (!out || n < 8) return -1;
  out[0] = 0;
  for (i = 0; i < m; i++) {
    char blob[256];
    int rank;
    if (list[i].loopback) continue;
    snprintf(blob, sizeof blob, "%s %s", list[i].name, list[i].driver);
    lower_copy(blob, blob, sizeof blob);
    if (strstr(blob, "lenovo") || strstr(blob, "uvcvideo")) continue;
    if (!strstr(blob, "nui") && !strstr(blob, "kinect") && !strstr(blob, "gspca") &&
        !strstr(blob, "xbox"))
      continue;
    rank = 1;
    if (strcmp(list[i].driver, "kinect") == 0 || strstr(list[i].driver, "gspca")) rank = 0;
    if (rank < best_rank) {
      best_rank = rank;
      best = i;
    }
  }
  if (best < 0) {
    for (i = 0; i < m; i++) {
      char blob[128];
      if (list[i].loopback) continue;
      lower_copy(list[i].driver, blob, sizeof blob);
      if (strstr(blob, "uvcvideo")) continue;
      best = i;
      break;
    }
  }
  if (best < 0) return -1;
  snprintf(out, n, "%s", list[best].path);
  return 0;
}

int kinect_find_vcam(char *out, size_t n) {
  vdev list[32], loops[16];
  int i, m = collect(list, 32), nl = 0;
  const char *want = vcam_label();
  char want_l[128], first[64];
  const char *sp;
  if (!out || n < 8) return -1;
  out[0] = 0;
  for (i = 0; i < m && nl < 16; i++)
    if (list[i].loopback) loops[nl++] = list[i];
  if (!nl) return -1;
  lower_copy(want, want_l, sizeof want_l);
  for (i = 0; i < nl; i++) {
    char nlbl[128];
    lower_copy(loops[i].name, nlbl, sizeof nlbl);
    if (strcmp(loops[i].name, want) == 0 || strcmp(nlbl, want_l) == 0) {
      snprintf(out, n, "%s", loops[i].path);
      return 0;
    }
  }
  first[0] = 0;
  sp = strchr(want, ' ');
  if (sp) {
    size_t L = (size_t)(sp - want);
    if (L >= sizeof first) L = sizeof first - 1;
    memcpy(first, want, L);
    first[L] = 0;
  } else
    snprintf(first, sizeof first, "%s", want);
  lower_copy(first, first, sizeof first);
  for (i = 0; i < nl; i++) {
    char nlbl[128];
    lower_copy(loops[i].name, nlbl, sizeof nlbl);
    if (strcmp(nlbl, first) == 0 || strncmp(nlbl, first, strlen(first)) == 0) {
      snprintf(out, n, "%s", loops[i].path);
      return 0;
    }
  }
  if (nl == 1) {
    snprintf(out, n, "%s", loops[0].path);
    return 0;
  }
  return -1;
}

static int xioctl(int fd, unsigned long req, void *arg) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r < 0 && errno == EINTR);
  return r;
}

int kinect_set_mode(const char *dev, const char *mode) {
  int fd, rgb;
  struct v4l2_format fmt;
  if (!dev || !mode) return -1;
  rgb = (mode[0] == 'R' || mode[0] == 'r');
  fd = open(dev, O_RDWR | O_NONBLOCK);
  if (fd < 0) return -1;
  memset(&fmt, 0, sizeof fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = rgb ? KINECT_RGB_W : KINECT_IR_W;
  fmt.fmt.pix.height = rgb ? KINECT_RGB_H : KINECT_IR_H;
  fmt.fmt.pix.pixelformat = rgb ? V4L2_PIX_FMT_UYVY : V4L2_PIX_FMT_Y10BPACK;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    close(fd);
    return -1;
  }
  close(fd);
  return 0;
}

typedef struct {
  void *start;
  size_t len;
} mapbuf;

static int grab_mmap(const char *dev, uint32_t fourcc, int w, int h, uint8_t *dst,
                     size_t dst_len) {
  int fd, i, type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  struct v4l2_format fmt;
  struct v4l2_requestbuffers req;
  struct v4l2_buffer buf;
  mapbuf maps[4];
  int nmap = 0, rc = -1;
  memset(maps, 0, sizeof maps);

  fd = open(dev, O_RDWR);
  if (fd < 0) return -1;
  memset(&fmt, 0, sizeof fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = (uint32_t)w;
  fmt.fmt.pix.height = (uint32_t)h;
  fmt.fmt.pix.pixelformat = fourcc;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) goto done;

  memset(&req, 0, sizeof req);
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) goto done;
  if (req.count > 4) req.count = 4;
  for (i = 0; i < (int)req.count; i++) {
    memset(&buf, 0, sizeof buf);
    buf.type = req.type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = (uint32_t)i;
    if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) goto done;
    maps[i].len = buf.length;
    maps[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (maps[i].start == MAP_FAILED) {
      maps[i].start = NULL;
      goto done;
    }
    nmap++;
    if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) goto done;
  }
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) goto done;
  memset(&buf, 0, sizeof buf);
  buf.type = req.type;
  buf.memory = V4L2_MEMORY_MMAP;
  /* a few tries — first gspca buffer can be short */
  for (i = 0; i < 6; i++) {
    fd_set fds;
    struct timeval tv = {2, 0};
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) continue;
    if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) continue;
    if (buf.bytesused >= dst_len / 2 && buf.index < (uint32_t)nmap) {
      size_t n = buf.bytesused < dst_len ? buf.bytesused : dst_len;
      memcpy(dst, maps[buf.index].start, n);
      if (n < dst_len) memset(dst + n, 0, dst_len - n);
      rc = 0;
      xioctl(fd, VIDIOC_QBUF, &buf);
      break;
    }
    xioctl(fd, VIDIOC_QBUF, &buf);
  }
  xioctl(fd, VIDIOC_STREAMOFF, &type);
done:
  for (i = 0; i < nmap; i++)
    if (maps[i].start) munmap(maps[i].start, maps[i].len);
  close(fd);
  return rc;
}

void kinect_uyvy_to_yuyv(const uint8_t *uyvy, uint8_t *yuyv, int npix) {
  int i;
  for (i = 0; i + 1 < npix; i += 2) {
    uint8_t u = uyvy[i * 2 + 0];
    uint8_t y0 = uyvy[i * 2 + 1];
    uint8_t v = uyvy[i * 2 + 2];
    uint8_t y1 = uyvy[i * 2 + 3];
    yuyv[i * 2 + 0] = y0;
    yuyv[i * 2 + 1] = u;
    yuyv[i * 2 + 2] = y1;
    yuyv[i * 2 + 3] = v;
  }
}

static void uyvy_to_rgb(const uint8_t *uyvy, uint8_t *rgb, int npix) {
  int i;
  for (i = 0; i + 1 < npix; i += 2) {
    int u = (int)uyvy[i * 2 + 0] - 128;
    int y0 = uyvy[i * 2 + 1];
    int v = (int)uyvy[i * 2 + 2] - 128;
    int y1 = uyvy[i * 2 + 3];
    int r, g, b, k;
    r = y0 + (int)(1.402f * v);
    g = y0 - (int)(0.344f * u + 0.714f * v);
    b = y0 + (int)(1.772f * u);
    if (r < 0) r = 0;
    if (r > 255) r = 255;
    if (g < 0) g = 0;
    if (g > 255) g = 255;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    k = i * 3;
    rgb[k] = (uint8_t)r;
    rgb[k + 1] = (uint8_t)g;
    rgb[k + 2] = (uint8_t)b;
    r = y1 + (int)(1.402f * v);
    g = y1 - (int)(0.344f * u + 0.714f * v);
    b = y1 + (int)(1.772f * u);
    if (r < 0) r = 0;
    if (r > 255) r = 255;
    if (g < 0) g = 0;
    if (g > 255) g = 255;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    rgb[k + 3] = (uint8_t)r;
    rgb[k + 4] = (uint8_t)g;
    rgb[k + 5] = (uint8_t)b;
  }
}

static void unpack_y10b(const uint8_t *buf, size_t blen, uint8_t *out, int npix) {
  size_t bit = 0, bits = blen * 8;
  int p;
  for (p = 0; p < npix; p++) {
    int v = 0, k;
    if (bit + 10 > bits) {
      out[p] = 0;
      continue;
    }
    for (k = 0; k < 10; k++) {
      unsigned byte = buf[bit / 8];
      v = (v << 1) | ((byte >> (7 - (bit % 8))) & 1);
      bit++;
    }
    out[p] = (uint8_t)(v >> 2);
  }
}

static void stretch_grey(uint8_t *g, int n) {
  int i, lo = 255, hi = 0;
  if (n <= 0) return;
  for (i = 0; i < n; i++) {
    if (g[i] < lo) lo = g[i];
    if (g[i] > hi) hi = g[i];
  }
  if (hi <= lo) return;
  for (i = 0; i < n; i++) {
    int v = (int)((g[i] - lo) * 255.0 / (hi - lo));
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    g[i] = (uint8_t)v;
  }
}

int kinect_grab_rgb(const char *dev, uint8_t *rgb) {
  uint8_t *uyvy;
  int rc;
  if (!dev || !rgb) return -1;
  uyvy = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 2);
  if (!uyvy) return -1;
  rc = grab_mmap(dev, V4L2_PIX_FMT_UYVY, KINECT_RGB_W, KINECT_RGB_H, uyvy,
                 (size_t)KINECT_RGB_W * KINECT_RGB_H * 2);
  if (rc == 0) uyvy_to_rgb(uyvy, rgb, KINECT_RGB_W * KINECT_RGB_H);
  free(uyvy);
  return rc;
}

int kinect_grab_ir_grey(const char *dev, uint8_t *grey) {
  const int w = KINECT_IR_W, h = KINECT_IR_H;
  size_t packed = (size_t)w * (size_t)h * 10 / 8;
  uint8_t *raw;
  int rc;
  if (!dev || !grey) return -1;
  raw = (uint8_t *)malloc(packed + 16);
  if (!raw) return -1;
  rc = grab_mmap(dev, V4L2_PIX_FMT_Y10BPACK, w, h, raw, packed);
  if (rc == 0) {
    unpack_y10b(raw, packed, grey, w * h);
    stretch_grey(grey, w * h);
  }
  free(raw);
  return rc;
}

void kinect_grey_to_rgb(const uint8_t *grey, int w, int h, uint8_t *rgb) {
  int i, n = w * h;
  for (i = 0; i < n; i++) {
    rgb[i * 3 + 0] = grey[i];
    rgb[i * 3 + 1] = grey[i];
    rgb[i * 3 + 2] = grey[i];
  }
}

void kinect_grey_to_yuyv(const uint8_t *grey, int w, int h, uint8_t *yuyv) {
  /* nearest-neighbor scale to 640x480 YUYV */
  int x, y;
  for (y = 0; y < KINECT_RGB_H; y++) {
    int sy = y * h / KINECT_RGB_H;
    for (x = 0; x < KINECT_RGB_W; x += 2) {
      int sx0 = x * w / KINECT_RGB_W;
      int sx1 = (x + 1) * w / KINECT_RGB_W;
      uint8_t y0 = grey[sy * w + sx0];
      uint8_t y1 = grey[sy * w + sx1];
      int o = (y * KINECT_RGB_W + x) * 2;
      yuyv[o + 0] = y0;
      yuyv[o + 1] = 128;
      yuyv[o + 2] = y1;
      yuyv[o + 3] = 128;
    }
  }
}

void kinect_rgb_to_yuyv(const uint8_t *rgb, uint8_t *yuyv) {
  int x, y;
  for (y = 0; y < KINECT_RGB_H; y++) {
    for (x = 0; x < KINECT_RGB_W; x += 2) {
      const uint8_t *p0 = rgb + (y * KINECT_RGB_W + x) * 3;
      const uint8_t *p1 = p0 + 3;
      int y0 = (77 * p0[0] + 150 * p0[1] + 29 * p0[2]) >> 8;
      int y1 = (77 * p1[0] + 150 * p1[1] + 29 * p1[2]) >> 8;
      int u = 128 + ((-43 * p0[0] - 85 * p0[1] + 128 * p0[2]) >> 8);
      int v = 128 + ((128 * p0[0] - 107 * p0[1] - 21 * p0[2]) >> 8);
      int o = (y * KINECT_RGB_W + x) * 2;
      if (u < 0) u = 0;
      if (u > 255) u = 255;
      if (v < 0) v = 0;
      if (v > 255) v = 255;
      yuyv[o + 0] = (uint8_t)y0;
      yuyv[o + 1] = (uint8_t)u;
      yuyv[o + 2] = (uint8_t)y1;
      yuyv[o + 3] = (uint8_t)v;
    }
  }
}

int kinect_vcam_open(const char *dev) {
  int fd;
  struct v4l2_format fmt;
  if (!dev) return -1;
  fd = open(dev, O_RDWR | O_NONBLOCK);
  if (fd < 0) return -1;
  memset(&fmt, 0, sizeof fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = KINECT_RGB_W;
  fmt.fmt.pix.height = KINECT_RGB_H;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.bytesperline = KINECT_RGB_W * 2;
  fmt.fmt.pix.sizeimage = KINECT_RGB_W * KINECT_RGB_H * 2;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

int kinect_vcam_write_yuyv(int fd, const uint8_t *yuyv) {
  size_t need = (size_t)KINECT_RGB_W * KINECT_RGB_H * 2;
  ssize_t w;
  if (fd < 0 || !yuyv) return -1;
  w = write(fd, yuyv, need);
  return (w == (ssize_t)need) ? 0 : -1;
}

int kinect_cap_open(kinect_cap *c, const char *dev, uint32_t fourcc, int w, int h) {
  struct v4l2_format fmt;
  struct v4l2_requestbuffers req;
  struct v4l2_buffer buf;
  int i, type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (!c || !dev) return -1;
  memset(c, 0, sizeof *c);
  c->fd = open(dev, O_RDWR);
  if (c->fd < 0) return -1;
  c->fourcc = fourcc;
  c->w = w;
  c->h = h;
  memset(&fmt, 0, sizeof fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = (uint32_t)w;
  fmt.fmt.pix.height = (uint32_t)h;
  fmt.fmt.pix.pixelformat = fourcc;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(c->fd, VIDIOC_S_FMT, &fmt) < 0) goto fail;
  memset(&req, 0, sizeof req);
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(c->fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) goto fail;
  if (req.count > 4) req.count = 4;
  for (i = 0; i < (int)req.count; i++) {
    memset(&buf, 0, sizeof buf);
    buf.type = req.type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = (uint32_t)i;
    if (xioctl(c->fd, VIDIOC_QUERYBUF, &buf) < 0) goto fail;
    c->map_len[i] = buf.length;
    c->map[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, c->fd, buf.m.offset);
    if (c->map[i] == MAP_FAILED) {
      c->map[i] = NULL;
      goto fail;
    }
    c->nmap++;
    if (xioctl(c->fd, VIDIOC_QBUF, &buf) < 0) goto fail;
  }
  if (xioctl(c->fd, VIDIOC_STREAMON, &type) < 0) goto fail;
  c->streaming = 1;
  return 0;
fail:
  kinect_cap_close(c);
  return -1;
}

int kinect_cap_frame(kinect_cap *c, uint8_t *dst, size_t dst_len) {
  struct v4l2_buffer buf;
  fd_set fds;
  struct timeval tv = {1, 500000};
  if (!c || c->fd < 0 || !dst) return -1;
  FD_ZERO(&fds);
  FD_SET(c->fd, &fds);
  if (select(c->fd + 1, &fds, NULL, NULL, &tv) <= 0) return -1;
  memset(&buf, 0, sizeof buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(c->fd, VIDIOC_DQBUF, &buf) < 0) return -1;
  if (buf.index < (uint32_t)c->nmap && c->map[buf.index]) {
    size_t n = buf.bytesused < dst_len ? buf.bytesused : dst_len;
    memcpy(dst, c->map[buf.index], n);
    if (n < dst_len) memset(dst + n, 0, dst_len - n);
  }
  xioctl(c->fd, VIDIOC_QBUF, &buf);
  return 0;
}

void kinect_cap_close(kinect_cap *c) {
  int i, type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (!c) return;
  if (c->streaming && c->fd >= 0) xioctl(c->fd, VIDIOC_STREAMOFF, &type);
  for (i = 0; i < c->nmap; i++)
    if (c->map[i]) munmap(c->map[i], c->map_len[i]);
  if (c->fd >= 0) close(c->fd);
  memset(c, 0, sizeof *c);
  c->fd = -1;
}
