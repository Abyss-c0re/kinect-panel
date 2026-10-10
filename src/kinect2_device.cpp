/* libfreenect2 is C++. Everything else in this tree stays C and calls this file. */
#include "kinect2_device.h"
#include "kinect_sync.h"

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static libfreenect2::Freenect2 *g_fn;
static libfreenect2::Freenect2Device *g_dev;
static libfreenect2::SyncMultiFrameListener *g_listener;
static libfreenect2::Registration *g_reg;
static libfreenect2::Frame *g_undistorted;
static char g_err[160];
static char g_pipe[16];
static int g_open;
/* 1 depth, 2 color, 3 IR. 0 when closed. */
static int g_kind;

static void set_err(const char *s) { snprintf(g_err, sizeof g_err, "%s", s ? s : ""); }

static uint64_t mono_us(void) {
  using namespace std::chrono;
  return (uint64_t)duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

/* Drops the expected VAAPI JPEG failure. Real device errors still print. */
struct Kinect2Log : libfreenect2::Logger {
  Level level() const { return Error; }
  void log(Level lv, const std::string &message) {
    /* Level rises with verbosity. Keep device errors, drop Info/Debug. */
    if (lv == None || lv > Error) return;
    if (message.find("vaInitialize") != std::string::npos) return;
    if (message.find("Vaapi") != std::string::npos) return;
    fprintf(stderr, "[kinect2] %s\n", message.c_str());
  }
};

static void quiet_log(void) {
  static int once;
  if (once) return;
  once = 1;
  libfreenect2::setGlobalLogger(new Kinect2Log());
}

/* VAAPI JPEG on the AMD libva driver decodes this sensor as a flat green
 * frame. Hide the driver so libfreenect2 falls back to TurboJPEG. */
struct VaapiHold {
  int on;
  char *saved;
  explicit VaapiHold(int color) : on(0), saved(0) {
    const char *rgb;
    const char *cur;
    if (!color) return;
    rgb = getenv("KINECT2_RGB");
    if (rgb && strcmp(rgb, "vaapi") == 0) return;
    cur = getenv("LIBVA_DRIVER_NAME");
    if (cur) saved = strdup(cur);
    setenv("LIBVA_DRIVER_NAME", "kinect2-no-vaapi", 1);
    on = 1;
  }
  ~VaapiHold() {
    if (!on) return;
    if (saved) {
      setenv("LIBVA_DRIVER_NAME", saved, 1);
      free(saved);
    } else {
      unsetenv("LIBVA_DRIVER_NAME");
    }
  }
};

static void led_set(int level) {
  libfreenect2::LedSettings s;
  if (!g_dev) return;
  if (level < 0) level = 0;
  if (level > 1000) level = 1000;
  memset(&s, 0, sizeof s);
  s.Mode = 0;
  s.StartLevel = (uint16_t)level;
  s.StopLevel = (uint16_t)level;
  s.LedId = 0;
  g_dev->setLedStatus(s);
  s.LedId = 1;
  g_dev->setLedStatus(s);
}

static libfreenect2::PacketPipeline *make_pipe(const char *name) {
  if (!name || !name[0]) return 0;
  if (strcmp(name, "cpu") == 0) return new libfreenect2::CpuPacketPipeline();
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  if (strcmp(name, "cl") == 0) return new libfreenect2::OpenCLPacketPipeline(-1);
  if (strcmp(name, "clkde") == 0) return new libfreenect2::OpenCLKdePacketPipeline(-1);
#endif
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  if (strcmp(name, "gl") == 0) return new libfreenect2::OpenGLPacketPipeline();
#endif
  return 0;
}

static void drop_device(void) {
  if (!g_dev) return;
  led_set(0);
  g_dev->stop();
  g_dev->close();
  delete g_dev;
  g_dev = 0;
}

int kinect2_count(void) {
  if (g_open) return 1;
  quiet_log();
  libfreenect2::Freenect2 fn;
  return (int)fn.enumerateDevices();
}

const char *kinect2_error(void) { return g_err; }
const char *kinect2_pipeline(void) { return g_pipe; }
int kinect2_is_open(void) { return g_open; }
int kinect2_stream_kind(void) { return g_open ? g_kind : 0; }

/* kind: 1 depth, 2 color, 3 IR. Color does not start the IR projector. */
static int open_device(int kind) {
  const char *pref;
  const char *order[4];
  int norder = 0, i;
  unsigned int ftype;
  if (kind != 1 && kind != 2 && kind != 3) {
    set_err("kinect 2 bad stream");
    return -1;
  }
  if (g_open && g_kind == kind) return 0;
  if (g_open) kinect2_close();
  quiet_log();
  VaapiHold vaapi_off(kind == 2);
  g_err[0] = 0;
  g_pipe[0] = 0;
  g_fn = new libfreenect2::Freenect2();
  if (g_fn->enumerateDevices() < 1) {
    set_err("no kinect 2");
    delete g_fn;
    g_fn = 0;
    return -1;
  }
  pref = getenv("KINECT2_PIPELINE");
  if (pref && pref[0]) order[norder++] = pref;
  /* CPU first. ROCm's OpenCL ICD often rejects these kernels. Color jpeg
   * decode is CPU either way. KINECT2_PIPELINE=cl selects OpenCL. */
  order[norder++] = "cpu";
  order[norder++] = "cl";
  order[norder++] = "gl";
  ftype = kind == 2 ? libfreenect2::Frame::Color
                    : kind == 3 ? libfreenect2::Frame::Ir
                                : libfreenect2::Frame::Depth;
  g_listener = new libfreenect2::SyncMultiFrameListener(ftype);
  for (i = 0; i < norder; i++) {
    libfreenect2::PacketPipeline *pipe = make_pipe(order[i]);
    if (!pipe) continue;
    g_dev = g_fn->openDefaultDevice(pipe);
    if (!g_dev) continue;
    if (kind == 2)
      g_dev->setColorFrameListener(g_listener);
    else
      g_dev->setIrAndDepthFrameListener(g_listener);
    if (!g_dev->startStreams(kind == 2, kind != 2)) {
      drop_device();
      continue;
    }
    snprintf(g_pipe, sizeof g_pipe, "%s", order[i]);
    break;
  }
  if (!g_dev) {
    set_err(kind == 2 ? "kinect 2 color failed" : kind == 3 ? "kinect 2 ir failed"
                                                             : "kinect 2 stream failed");
    delete g_listener;
    g_listener = 0;
    delete g_fn;
    g_fn = 0;
    return -1;
  }
  if (kind == 1) {
    g_reg = new libfreenect2::Registration(g_dev->getIrCameraParams(),
                                           g_dev->getColorCameraParams());
    g_undistorted = new libfreenect2::Frame(KINECT2_W, KINECT2_H, 4);
  }
  led_set(120);
  g_kind = kind;
  g_open = 1;
  return 0;
}

int kinect2_open(void) { return open_device(1); }
int kinect2_open_color(void) { return open_device(2); }
int kinect2_open_ir(void) { return open_device(3); }

void kinect2_close(void) {
  drop_device();
  delete g_reg;
  g_reg = 0;
  delete g_undistorted;
  g_undistorted = 0;
  delete g_listener;
  g_listener = 0;
  delete g_fn;
  g_fn = 0;
  g_open = 0;
  g_kind = 0;
  g_pipe[0] = 0;
}

int kinect2_take(float *z_mm, uint64_t *host_us, int wait_ms) {
  libfreenect2::FrameMap frames;
  libfreenect2::Frame *depth = 0;
  int rc = 0;
  if (!g_open || g_kind != 1 || !g_listener || !z_mm) return -1;
  if (wait_ms < 0) wait_ms = 0;
  if (wait_ms == 0 && !g_listener->hasNewFrame()) return 0;
  if (!g_listener->waitForNewFrame(frames, wait_ms == 0 ? 1 : wait_ms)) return 0;
  {
    libfreenect2::FrameMap::iterator it = frames.find(libfreenect2::Frame::Depth);
    if (it != frames.end()) depth = it->second;
  }
  if (depth && depth->data && depth->width == KINECT2_W && depth->height == KINECT2_H &&
      depth->bytes_per_pixel == 4) {
    memcpy(z_mm, depth->data, (size_t)KINECT2_N * sizeof(float));
    if (host_us) *host_us = mono_us();
    rc = 1;
  }
  g_listener->release(frames);
  return rc;
}

static int wait_frame(libfreenect2::FrameMap &frames, int wait_ms) {
  if (!g_open || !g_listener) return -1;
  if (wait_ms < 0) wait_ms = 0;
  if (wait_ms == 0 && !g_listener->hasNewFrame()) return 0;
  if (!g_listener->waitForNewFrame(frames, wait_ms == 0 ? 1 : wait_ms)) return 0;
  return 1;
}

static int blit_color(const libfreenect2::Frame *f, uint8_t *dst) {
  int sw, sh, bpp, y, x, bgr;
  if (!f || !f->data) return 0;
  sw = (int)f->width;
  sh = (int)f->height;
  bpp = (int)f->bytes_per_pixel;
  if (sw < 1 || sh < 1 || bpp < 3) return 0;
  bgr = f->format != libfreenect2::Frame::RGBX;
  for (y = 0; y < 480; y++) {
    int sy = y * sh / 480;
    const unsigned char *row = f->data + (size_t)sy * (size_t)sw * (size_t)bpp;
    for (x = 0; x < 640; x++) {
      int sx = x * sw / 640;
      const unsigned char *p = row + (size_t)sx * (size_t)bpp;
      uint8_t *o = dst + (y * 640 + x) * 3;
      if (bgr) {
        o[0] = p[2];
        o[1] = p[1];
        o[2] = p[0];
      } else {
        o[0] = p[0];
        o[1] = p[1];
        o[2] = p[2];
      }
    }
  }
  return 1;
}

int kinect2_take_rgb(uint8_t *rgb640, int wait_ms) {
  libfreenect2::FrameMap frames;
  libfreenect2::Frame *color = 0;
  int wr, rc = 0;
  if (!rgb640 || g_kind != 2) return -1;
  wr = wait_frame(frames, wait_ms);
  if (wr <= 0) return wr;
  {
    libfreenect2::FrameMap::iterator it = frames.find(libfreenect2::Frame::Color);
    if (it != frames.end()) color = it->second;
  }
  if (blit_color(color, rgb640)) rc = 1;
  g_listener->release(frames);
  return rc;
}

int kinect2_take_ir(uint16_t *ir, int wait_ms) {
  libfreenect2::FrameMap frames;
  libfreenect2::Frame *frame = 0;
  int wr, i, rc = 0;
  if (!ir || g_kind != 3) return -1;
  wr = wait_frame(frames, wait_ms);
  if (wr <= 0) return wr;
  {
    libfreenect2::FrameMap::iterator it = frames.find(libfreenect2::Frame::Ir);
    if (it != frames.end()) frame = it->second;
  }
  if (frame && frame->data && frame->width == KINECT2_W && frame->height == KINECT2_H &&
      frame->bytes_per_pixel == 4) {
    const float *src = (const float *)frame->data;
    for (i = 0; i < KINECT2_N; i++) {
      float v = src[i];
      if (!std::isfinite(v) || v <= 0.f)
        ir[i] = 0;
      else if (v >= 65535.f)
        ir[i] = 65535;
      else
        ir[i] = (uint16_t)(v + 0.5f);
    }
    rc = 1;
  }
  g_listener->release(frames);
  return rc;
}

int kinect2_cloud_from_depth(const float *z_mm, const cube_cloud_tune *t, cube_cloud *out,
                             int max_pts) {
  cube_cloud_tune def;
  libfreenect2::Frame depth(KINECT2_W, KINECT2_H, 4, (unsigned char *)z_mm);
  int stride, r, c, n = 0;
  float zn, zf;
  if (!out || !z_mm) return -1;
  if (!g_reg || !g_undistorted) {
    set_err("kinect 2 not open");
    return -1;
  }
  if (!t) {
    cube_cloud_tune_default(&def);
    t = &def;
  }
  if (max_pts < 1 || max_pts > CUBE_CLOUD_MAX) max_pts = CUBE_CLOUD_MAX;
  zn = t->z_near < 0.2f ? 0.2f : t->z_near;
  zf = t->z_far;
  if (zf < zn + 0.1f) zf = zn + 0.1f;
  stride = ksync_cover_stride(KINECT2_W, KINECT2_H, t->stride, max_pts);
  depth.format = libfreenect2::Frame::Float;
  g_reg->undistortDepth(&depth, g_undistorted);
  cube_cloud_clear(out);
  if (cube_cloud_reserve(out, max_pts) < 0) {
    snprintf(out->err, sizeof out->err, "oom");
    return -1;
  }
  for (r = 0; r < KINECT2_H && n < max_pts && n < out->cap; r += stride) {
    for (c = 0; c < KINECT2_W && n < max_pts && n < out->cap; c += stride) {
      float x, y, z;
      cube_cloud_pt *p;
      g_reg->getPointXYZ(g_undistorted, r, c, x, y, z);
      if (!std::isfinite(z) || z < zn || z > zf) continue;
      p = &out->pt[n++];
      p->x = x;
      p->y = -y; /* libfreenect2 is +Y down. The panel cloud is +Y up. */
      p->z = z;
      cube_cloud_color_point(p, t);
    }
  }
  out->n = n;
  out->ok = n > 0;
  if (!out->ok) snprintf(out->err, sizeof out->err, "empty kinect 2 cloud");
  return out->ok ? 0 : -1;
}

int kinect2_metric_depth(const float *z_mm, float *out_mm) {
  libfreenect2::Frame depth(KINECT2_W, KINECT2_H, 4, (unsigned char *)z_mm);
  if (!g_reg || !g_undistorted || !z_mm || !out_mm) return -1;
  depth.format = libfreenect2::Frame::Float;
  g_reg->undistortDepth(&depth, g_undistorted);
  if (!g_undistorted->data) return -1;
  memcpy(out_mm, g_undistorted->data, (size_t)KINECT2_N * sizeof(float));
  return 0;
}

int kinect2_ir_intrinsics(float *fx, float *fy, float *cx, float *cy) {
  libfreenect2::Freenect2Device::IrCameraParams p;
  if (!g_dev || !fx || !fy || !cx || !cy) return -1;
  p = g_dev->getIrCameraParams();
  if (!(p.fx > 1.f) || !(p.fy > 1.f)) return -1;
  *fx = p.fx;
  *fy = p.fy;
  *cx = p.cx;
  *cy = p.cy;
  return 0;
}
