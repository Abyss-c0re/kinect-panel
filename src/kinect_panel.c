#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* Kinect Panel — RGB / IR (boost) / point cloud. One view. Pure C. */
#include "kinect_bridge.h"
#include "kinect_cloud_view.h"
#include "kinect_fuse.h"
#include "kinect_ir.h"
#include "kinect_motor.h"
#include "kinect_net.h"
#include "kinect_session.h"
#include "kinect_v4l.h"
#include "kinect2_device.h"

#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtk.h>
#include <libfreenect.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define STEP 5.0
#define TILT_MIN -30.0
#define TILT_MAX 30.0

static GtkWidget *win, *status_lbl, *dev_lbl, *preview_img, *preview_ev;
static GtkWidget *root_box, *view_box, *full_tag;
static GtkWidget *btn_cam, *btn_preview, *tilt_scale, *tilt_lbl, *keep_above;
static GtkWidget *ir_frame, *cloud_frame, *tabs;
static GtkWidget *sc_boost, *sc_bright, *sc_contrast, *sc_gamma, *sc_black, *sc_white;
static GtkWidget *sc_near, *sc_far, *sc_stride, *sc_dot, *combo_color, *chk_solid;
static GtkWidget *btn_rgb_w, *btn_ir_w, *btn_cloud_w;
static GtkWidget *btn_src_auto, *btn_src_v1, *btn_src_v2, *btn_src_both;
static GtkWidget *chk_share, *peer_entry, *net_lbl;
static GtkWidget *sc_fg, *lbl_fg;
static int camera_on, preview_on, closing, snap_busy, bridge_starting;
static int src_suppress, mode_suppress, share_suppress;
static char align_note[96];
static volatile sig_atomic_t got_sig;
static char mode[8] = "RGB";
static char source[64], vcam[64];
static double target_tilt;
static kinect_ir_tune ir_tune;
static int ir_ui_guard;
static int drag_orbit;
static double drag_x, drag_y;
static float orbit_e = 18.f, orbit_a = -60.f, orbit_d = 3.2f;
static int view_full, painting;
static uint8_t *frame_rgb;
static int frame_w, frame_h, paint_w, paint_h;
static guint32 arrow_ms, f11_ms;
static int win_x, win_y, win_w, win_h;
static guint fs_watch, restore_watch;
static int fs_queued;

static int is_cloud(void) { return mode[0] == 'C'; }
static int is_ir(void) { return mode[0] == 'I'; }

static void set_status(const char *s) { gtk_label_set_text(GTK_LABEL(status_lbl), s); }

static const char *sensor_word(void) {
  switch (kinect_fuse_source()) {
    case KINECT_SRC_V1:
      return "Kinect";
    case KINECT_SRC_V2:
      return "Kinect 2";
    case KINECT_SRC_BOTH:
      return "Both";
    case KINECT_SRC_AUTO:
    default:
      return "Auto";
  }
}

static void refresh_dev(void) {
  char src[64], vc[64], buf[360];
  int v2cam = kinect_bridge_sensor() == 2;
  if (kinect_find_source(src, sizeof src) == 0)
    snprintf(source, sizeof source, "%s", src);
  if (kinect_find_vcam(vc, sizeof vc) == 0)
    snprintf(vcam, sizeof vcam, "%s", vc);
  else
    vcam[0] = 0;
  if (is_cloud() && kinect_cloud_running())
    snprintf(buf, sizeof buf, "Cloud LIVE  n=%d  ·  drag preview to orbit  ·  %s",
             kinect_cloud_points(), sensor_word());
  else if (camera_on && kinect_bridge_running() && v2cam)
    snprintf(buf, sizeof buf, "Kinect 2 %s live%s%s", mode, vcam[0] ? "  ·  Apps → " : "  ·  ",
             vcam[0] ? vcam : "loopback not loaded");
  else if (camera_on && kinect_bridge_running() && vcam[0])
    snprintf(buf, sizeof buf, "Apps → %s  “Kinect Webcam”  ·  %s  (%s)", vcam, mode, sensor_word());
  else if (vcam[0])
    snprintf(buf, sizeof buf, "Apps → %s  “Kinect Webcam”  (Camera OFF)  ·  %s", vcam, sensor_word());
  else
    snprintf(buf, sizeof buf, "Virtual cam not loaded · %s · run setup_loopback.sh --load",
             sensor_word());
  gtk_label_set_text(GTK_LABEL(dev_lbl), buf);
}

static void refresh_full_tag(void) {
  char buf[120];
  const char *mv = is_cloud() ? "Cloud" : is_ir() ? "IR" : "RGB";
  snprintf(buf, sizeof buf, "%s  ·  %s     ←→ camera    ↑↓ mode    F11", mv, sensor_word());
  if (full_tag) gtk_label_set_text(GTK_LABEL(full_tag), buf);
  if (win && view_full) {
    char title[160];
    snprintf(title, sizeof title, "Kinect Panel — %s · %s", mv, sensor_word());
    gtk_window_set_title(GTK_WINDOW(win), title);
  }
}

static gboolean on_img_draw(GtkWidget *w, cairo_t *cr, gpointer u) {
  GdkPixbuf *pb, *sc;
  uint8_t *pix;
  int row, y, aw, ah, dw, dh;
  double s, ox, oy;
  (void)u;
  /* Full screen draws here. A pixbuf on the image would become its minimum
     size and the panel could not shrink back after Escape. */
  if (!view_full) return FALSE;
  aw = gtk_widget_get_allocated_width(w);
  ah = gtk_widget_get_allocated_height(w);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_paint(cr);
  if (!frame_rgb || frame_w < 1 || frame_h < 1 || aw < 2 || ah < 2) return TRUE;
  s = (double)aw / (double)frame_w;
  if ((double)ah / (double)frame_h < s) s = (double)ah / (double)frame_h;
  dw = (int)(frame_w * s + 0.5);
  dh = (int)(frame_h * s + 0.5);
  if (dw < 1) dw = 1;
  if (dh < 1) dh = 1;
  if (dw > aw) dw = aw;
  if (dh > ah) dh = ah;
  pb = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, frame_w, frame_h);
  if (!pb) return TRUE;
  pix = gdk_pixbuf_get_pixels(pb);
  row = gdk_pixbuf_get_rowstride(pb);
  if (row == frame_w * 3)
    memcpy(pix, frame_rgb, (size_t)frame_w * (size_t)frame_h * 3);
  else {
    for (y = 0; y < frame_h; y++)
      memcpy(pix + (size_t)y * row, frame_rgb + (size_t)y * frame_w * 3, (size_t)frame_w * 3);
  }
  sc = gdk_pixbuf_scale_simple(pb, dw, dh, GDK_INTERP_NEAREST);
  g_object_unref(pb);
  if (!sc) return TRUE;
  ox = (aw - dw) / 2.0;
  oy = (ah - dh) / 2.0;
  gdk_cairo_set_source_pixbuf(cr, sc, ox, oy);
  cairo_paint(cr);
  g_object_unref(sc);
  return TRUE;
}

static void paint_preview(void) {
  GdkPixbuf *pb, *sc;
  uint8_t *pix;
  int row, y, dw, dh;
  if (painting || !preview_img || !frame_rgb || frame_w < 1 || frame_h < 1) return;
  if (view_full) {
    gtk_image_clear(GTK_IMAGE(preview_img));
    gtk_widget_set_size_request(preview_img, 460, 240);
    gtk_widget_queue_draw(preview_img);
    return;
  }
  painting = 1;
  dw = 460;
  dh = 240;
  if (view_full && preview_ev) {
    int aw = gtk_widget_get_allocated_width(preview_ev);
    int ah = gtk_widget_get_allocated_height(preview_ev);
    if (aw > 32 && ah > 32) {
      double s = (double)aw / (double)frame_w;
      double sy = (double)ah / (double)frame_h;
      if (sy < s) s = sy;
      dw = (int)(frame_w * s + 0.5);
      dh = (int)(frame_h * s + 0.5);
      if (dw < 1) dw = 1;
      if (dh < 1) dh = 1;
    }
  }
  pb = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, frame_w, frame_h);
  if (!pb) {
    painting = 0;
    return;
  }
  pix = gdk_pixbuf_get_pixels(pb);
  row = gdk_pixbuf_get_rowstride(pb);
  if (row == frame_w * 3)
    memcpy(pix, frame_rgb, (size_t)frame_w * (size_t)frame_h * 3);
  else {
    for (y = 0; y < frame_h; y++)
      memcpy(pix + y * row, frame_rgb + y * frame_w * 3, (size_t)frame_w * 3);
  }
  sc = gdk_pixbuf_scale_simple(pb, dw, dh, GDK_INTERP_NEAREST);
  paint_w = dw;
  paint_h = dh;
  if (sc) {
    gtk_image_set_from_pixbuf(GTK_IMAGE(preview_img), sc);
    g_object_unref(sc);
  }
  g_object_unref(pb);
  painting = 0;
}

static void show_rgb(const uint8_t *rgb, int w, int h) {
  size_t n;
  uint8_t *p;
  if (!rgb || w < 1 || h < 1) return;
  n = (size_t)w * (size_t)h * 3;
  p = (uint8_t *)realloc(frame_rgb, n);
  if (!p) return;
  frame_rgb = p;
  memcpy(frame_rgb, rgb, n);
  frame_w = w;
  frame_h = h;
  paint_w = 0;
  paint_h = 0;
  paint_preview();
}

static void enter_view_full(void);
static void leave_view_full(void);

static void on_preview_size(GtkWidget *w, GdkRectangle *a, gpointer u) {
  int dw, dh;
  (void)w;
  (void)u;
  if (painting || !view_full || !frame_rgb || !a || frame_w < 1 || frame_h < 1) return;
  dw = 460;
  dh = 240;
  if (a->width > 32 && a->height > 32) {
    double s = (double)a->width / (double)frame_w;
    double sy = (double)a->height / (double)frame_h;
    if (sy < s) s = sy;
    dw = (int)(frame_w * s + 0.5);
    dh = (int)(frame_h * s + 0.5);
  }
  if (dw == paint_w && dh == paint_h) return;
  paint_preview();
}

static gboolean apply_preview(gpointer data) {
  uint8_t *rgb = data;
  show_rgb(rgb, KINECT_RGB_W, KINECT_RGB_H);
  free(rgb);
  snap_busy = 0;
  return G_SOURCE_REMOVE;
}

static gboolean apply_status(gpointer data) {
  set_status((char *)data);
  g_free(data);
  return G_SOURCE_REMOVE;
}

static void tune_from_sliders(kinect_ir_tune *t) {
  if (!sc_boost) {
    kinect_ir_tune_default(t);
    return;
  }
  t->boost = (float)gtk_range_get_value(GTK_RANGE(sc_boost));
  t->brightness = (float)gtk_range_get_value(GTK_RANGE(sc_bright));
  t->contrast = (float)gtk_range_get_value(GTK_RANGE(sc_contrast));
  t->gamma = (float)gtk_range_get_value(GTK_RANGE(sc_gamma));
  t->black_pct = (int)gtk_range_get_value(GTK_RANGE(sc_black));
  t->white_pct = (int)gtk_range_get_value(GTK_RANGE(sc_white));
  t->invert = ir_tune.invert;
}

static void sliders_from_tune(const kinect_ir_tune *t) {
  if (!sc_boost) return;
  ir_ui_guard = 1;
  gtk_range_set_value(GTK_RANGE(sc_boost), t->boost);
  gtk_range_set_value(GTK_RANGE(sc_bright), t->brightness);
  gtk_range_set_value(GTK_RANGE(sc_contrast), t->contrast);
  gtk_range_set_value(GTK_RANGE(sc_gamma), t->gamma);
  gtk_range_set_value(GTK_RANGE(sc_black), t->black_pct);
  gtk_range_set_value(GTK_RANGE(sc_white), t->white_pct);
  ir_ui_guard = 0;
}

static void apply_ir_tune(void) {
  tune_from_sliders(&ir_tune);
  kinect_bridge_set_ir_tune(&ir_tune);
}

static void *snap_work(void *arg) {
  uint8_t *rgb = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
  int rc = -1;
  (void)arg;
  if (!rgb) {
    snap_busy = 0;
    return NULL;
  }
  if (is_cloud())
    rc = kinect_cloud_preview_rgb(rgb);
  else if (camera_on && kinect_bridge_running())
    rc = kinect_bridge_preview_rgb(rgb);
  else if (source[0] && is_ir()) {
    uint8_t *raw = NULL;
    uint16_t *ir10 = (uint16_t *)malloc((size_t)KINECT_IR_W * KINECT_IR_H * sizeof(uint16_t));
    uint8_t *grey = (uint8_t *)malloc((size_t)KINECT_IR_W * KINECT_IR_H);
    /* one-shot via grab then grade — grab_ir_grey already 8-bit; use raw path */
    if (grey && kinect_grab_ir_grey(source, grey) == 0) {
      /* grab_ir_grey already stretched; re-grade from 8-bit upsampled */
      int i, n = KINECT_IR_W * KINECT_IR_H;
      if (ir10) {
        for (i = 0; i < n; i++) ir10[i] = (uint16_t)grey[i] << 2;
        kinect_ir_grade(ir10, grey, n, &ir_tune);
      }
      kinect_ir_grey_to_rgb640(grey, KINECT_IR_W, KINECT_IR_H, rgb);
      rc = 0;
    }
    free(raw);
    free(ir10);
    free(grey);
  } else if (source[0])
    rc = kinect_grab_rgb(source, rgb);
  if (rc == 0)
    g_idle_add(apply_preview, rgb);
  else {
    g_idle_add(apply_status, g_strdup("Preview: no frame"));
    free(rgb);
    snap_busy = 0;
  }
  return NULL;
}

static void snap_async(void) {
  pthread_t t;
  if (snap_busy) return;
  snap_busy = 1;
  if (pthread_create(&t, NULL, snap_work, NULL) == 0)
    pthread_detach(t);
  else
    snap_busy = 0;
}

static void on_snap_clicked(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  snap_async();
}

static void cloud_status_line(char *buf, size_t n, const char *lead) {
  kinect_fuse_stats st;
  char skew[24];
  kinect_fuse_get_stats(&st);
  if (st.paired && st.skew_us >= 0)
    snprintf(skew, sizeof skew, "%d ms", st.skew_us / 1000);
  else
    snprintf(skew, sizeof skew, "-");
  snprintf(buf, n, "%s · n=%d · v1 %d · v2 %d · %s · skew %s%s%s", lead, st.n, st.n1, st.n2,
           kinect_src_name(st.active), skew, align_note[0] ? " · " : "", align_note);
}

static gboolean on_bridge_ok(gpointer data) {
  char buf[256];
  (void)data;
  bridge_starting = 0;
  refresh_dev();
  if (is_cloud()) {
    if (kinect_cloud_live())
      cloud_status_line(buf, sizeof buf, "Cloud LIVE");
    else
      snprintf(buf, sizeof buf, "Cloud · waiting for live depth…");
  } else if (kinect_bridge_sensor() == 2)
    snprintf(buf, sizeof buf, "Kinect 2 %s LIVE · “Kinect Webcam” (%s)", mode,
             vcam[0] ? vcam : "preview only");
  else
    snprintf(buf, sizeof buf, "Webcam LIVE · %s · “Kinect Webcam” (%s)", mode,
             vcam[0] ? vcam : "?");
  set_status(buf);
  if (!preview_on)
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_preview), TRUE);
  else
    snap_async();
  return G_SOURCE_REMOVE;
}

static gboolean on_bridge_fail(gpointer data) {
  char buf[256];
  snprintf(buf, sizeof buf, "Failed: %s", (char *)data);
  bridge_starting = 0;
  camera_on = 0;
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cam), FALSE);
  gtk_button_set_label(GTK_BUTTON(btn_cam), "Camera OFF");
  /* Kinect 2 owns its own lamp. A failed color/IR open must not light Kinect. */
  if (kinect_fuse_source() != KINECT_SRC_V2 && kinect_fuse_source() != KINECT_SRC_BOTH)
    kinect_motor_set_led(LED_RED);
  set_status(buf);
  refresh_dev();
  g_free(data);
  return G_SOURCE_REMOVE;
}

static void *arm_work(void *arg) {
  kinect_mode m = kinect_session_parse_mode(arg ? (const char *)arg : mode);
  int rc;
  if (m == KINECT_MODE_OFF) m = KINECT_MODE_RGB;
  kinect_bridge_set_ir_tune(&ir_tune);
  rc = kinect_session_start(m);
  if (rc == 0)
    g_idle_add(on_bridge_ok, NULL);
  else
    g_idle_add(on_bridge_fail, g_strdup(kinect_session_error()));
  return NULL;
}

static int image_is_v2(void) {
  kinect_src s = kinect_fuse_source();
  if (s == KINECT_SRC_V2 || s == KINECT_SRC_BOTH) return 1;
  if (s == KINECT_SRC_V1) return 0;
  /* Auto: Kinect webcam when gspca is there, otherwise Kinect 2. */
  return kinect_find_source(source, sizeof source) < 0;
}

static void arm_camera(void) {
  pthread_t t;
  kinect_src s = kinect_fuse_source();
  int gspca = kinect_find_source(source, sizeof source) == 0;
  if (!is_cloud() && s == KINECT_SRC_V1 && !gspca) {
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cam), FALSE);
    camera_on = 0;
    set_status("No Kinect video node. Plug Kinect.");
    return;
  }
  if (!is_cloud() && s == KINECT_SRC_AUTO && !gspca && kinect2_count() < 1) {
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cam), FALSE);
    camera_on = 0;
    set_status("No Kinect camera and no Kinect 2.");
    return;
  }
  if (is_cloud())
    set_status("Starting depth cloud…");
  else if (image_is_v2())
    set_status(is_ir() ? "Starting Kinect 2 IR…" : "Starting Kinect 2 RGB…");
  else
    set_status("Starting webcam…");
  bridge_starting = 1;
  if (pthread_create(&t, NULL, arm_work, mode) == 0) pthread_detach(t);
}

static void on_cam(GtkToggleButton *b, gpointer u) {
  int on = gtk_toggle_button_get_active(b);
  (void)u;
  camera_on = on;
  gtk_button_set_label(GTK_BUTTON(b), on ? "Camera ON" : "Camera OFF");
  if (on)
    arm_camera();
  else {
    bridge_starting = 0;
    kinect_session_off();
    set_status("Camera OFF");
    refresh_dev();
  }
}

static void on_preview(GtkToggleButton *b, gpointer u) {
  (void)u;
  preview_on = gtk_toggle_button_get_active(b);
  gtk_button_set_label(GTK_BUTTON(b), preview_on ? "Preview ON" : "Preview OFF");
  if (preview_on) snap_async();
}

static GtkWidget *src_radio_for(kinect_src s) {
  switch (s) {
    case KINECT_SRC_V1:
      return btn_src_v1;
    case KINECT_SRC_V2:
      return btn_src_v2;
    case KINECT_SRC_BOTH:
      return btn_src_both;
    case KINECT_SRC_AUTO:
    default:
      return btn_src_auto;
  }
}

static void status_mode_idle(void) {
  kinect_src s = kinect_fuse_source();
  if (is_cloud()) {
    set_status("Mode Cloud · Camera ON starts live depth");
    return;
  }
  if (s == KINECT_SRC_V2)
    set_status(is_ir() ? "Mode IR · Kinect 2 · Camera ON opens the IR camera"
                       : "Mode RGB · Kinect 2 · Camera ON opens the color camera");
  else if (s == KINECT_SRC_BOTH)
    set_status(is_ir() ? "Both · IR uses Kinect 2 · Camera ON opens it"
                       : "Both · RGB uses Kinect 2 · Camera ON opens it");
  else if (is_ir())
    set_status("Mode IR · grade sliders live · Camera ON publishes webcam");
  else
    set_status("Mode RGB · Camera ON publishes Kinect Webcam");
}

static void on_src_radio(GtkToggleButton *b, gpointer u) {
  kinect_src s;
  if (src_suppress || !gtk_toggle_button_get_active(b)) return;
  s = (kinect_src)GPOINTER_TO_INT(u);
  if (s < KINECT_SRC_AUTO || s > KINECT_SRC_BOTH) return;
  kinect_fuse_set_source(s);
  /* Sensor applies to RGB, IR, and Cloud. Do not switch the view. */
  if (camera_on)
    arm_camera();
  else
    status_mode_idle();
  refresh_dev();
}

static gboolean on_align_done(gpointer data) {
  snprintf(align_note, sizeof align_note, "%s", data ? (char *)data : "");
  set_status(align_note[0] ? align_note : "Align finished");
  g_free(data);
  return G_SOURCE_REMOVE;
}

static void *align_work(void *arg) {
  ksync_align_result res;
  char *msg;
  (void)arg;
  if (kinect_fuse_align(&res) != 0) {
    const char *e = kinect_fuse_error();
    msg = g_strdup(e && e[0] ? e : "align failed");
  } else {
    msg = g_strdup_printf("aligned yaw %.1f pitch %.1f · %.3f m · %d inliers", res.yaw_deg,
                          res.pitch_deg, res.residual_m, res.inliers);
  }
  g_idle_add(on_align_done, msg);
  return NULL;
}

static void on_align_clicked(GtkButton *b, gpointer u) {
  pthread_t t;
  (void)b;
  (void)u;
  set_status("Aligning Kinect 2 onto Kinect…");
  if (pthread_create(&t, NULL, align_work, NULL) == 0)
    pthread_detach(t);
  else
    set_status("Align thread failed");
}

static void on_align_reset(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  kinect_fuse_reset_rigid();
  snprintf(align_note, sizeof align_note, "identity");
  set_status("Align reset · identity");
}

static void on_share_toggled(GtkToggleButton *b, gpointer u) {
  (void)u;
  if (share_suppress) return;
  if (!gtk_toggle_button_get_active(b)) {
    kinect_net_stop();
    if (net_lbl) gtk_label_set_text(GTK_LABEL(net_lbl), "Net off");
    set_status("Share off");
    return;
  }
  if (kinect_net_listen(0) != 0) {
    const char *e = kinect_net_error();
    set_status(e && e[0] ? e : "Share failed");
    share_suppress = 1;
    gtk_toggle_button_set_active(b, FALSE);
    share_suppress = 0;
    return;
  }
  set_status("Sharing Kinect on UDP 8765");
}

static void on_net_connect(GtkButton *b, gpointer u) {
  const char *t = peer_entry ? gtk_entry_get_text(GTK_ENTRY(peer_entry)) : "";
  const char *peer = (t && t[0]) ? t : getenv("KINECT_PEER");
  (void)b;
  (void)u;
  if (!peer || !peer[0]) {
    set_status("Connect needs a host");
    return;
  }
  if (kinect_net_connect(peer) != 0) {
    const char *e = kinect_net_error();
    set_status(e && e[0] ? e : "Connect failed");
    return;
  }
  set_status("Connecting to peer…");
}

static gboolean net_tick(gpointer u) {
  char line[96];
  (void)u;
  if (closing || !net_lbl) return TRUE;
  kinect_net_status(line, sizeof line);
  gtk_label_set_text(GTK_LABEL(net_lbl), line);
  if (chk_share) {
    int on = kinect_net_listen_port() > 0;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_share)) != on) {
      share_suppress = 1;
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_share), on);
      share_suppress = 0;
    }
  }
  return TRUE;
}

static void on_mode(GtkToggleButton *b, gpointer u) {
  const char *m = u;
  if (!gtk_toggle_button_get_active(b)) return;
  snprintf(mode, sizeof mode, "%s", m);
  if (mode_suppress) {
    if (ir_frame) gtk_widget_set_sensitive(ir_frame, is_ir());
    if (cloud_frame) gtk_widget_set_sensitive(cloud_frame, is_cloud());
    return;
  }
  if (ir_frame) gtk_widget_set_sensitive(ir_frame, is_ir());
  if (cloud_frame) gtk_widget_set_sensitive(cloud_frame, is_cloud());
  /* Stay on Camera — Image tab is opt-in so tilt never disappears. */
  if (is_cloud()) {
    if (camera_on)
      arm_camera();
    else
      set_status("Mode Cloud · Camera ON starts live depth (headless: --headless --mode cloud)");
    if (!preview_on)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_preview), TRUE);
    return;
  }
  if (camera_on)
    arm_camera();
  if (!preview_on)
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_preview), TRUE);
  else
    snap_async();
  if (!camera_on) status_mode_idle();
}

static void on_ir_slider(GtkRange *r, gpointer u) {
  (void)r;
  (void)u;
  if (ir_ui_guard) return;
  apply_ir_tune();
}

static void on_see(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  kinect_ir_tune_see(&ir_tune);
  sliders_from_tune(&ir_tune);
  kinect_bridge_set_ir_tune(&ir_tune);
  set_status("IR SEE grade — percentile stretch + boost (the B/W that can see)");
  if (preview_on) snap_async();
}

static void apply_cloud_tune(void) {
  cube_cloud_tune t;
  if (!sc_near) return;
  kinect_cloud_get_tune(&t);
  t.z_near = (float)gtk_range_get_value(GTK_RANGE(sc_near));
  t.z_far = (float)gtk_range_get_value(GTK_RANGE(sc_far));
  t.stride = (int)gtk_range_get_value(GTK_RANGE(sc_stride));
  t.dot = (int)gtk_range_get_value(GTK_RANGE(sc_dot));
  if (combo_color)
    t.color = gtk_combo_box_get_active(GTK_COMBO_BOX(combo_color));
  if (t.z_far < t.z_near + 0.2f) t.z_far = t.z_near + 0.2f;
  kinect_cloud_set_tune(&t);
  if (chk_solid)
    kinect_cloud_set_solid(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_solid)));
}

static void sliders_from_cloud(const cube_cloud_tune *t) {
  if (!sc_near || !t) return;
  ir_ui_guard = 1;
  gtk_range_set_value(GTK_RANGE(sc_near), t->z_near);
  gtk_range_set_value(GTK_RANGE(sc_far), t->z_far);
  gtk_range_set_value(GTK_RANGE(sc_stride), t->stride);
  gtk_range_set_value(GTK_RANGE(sc_dot), t->dot);
  if (combo_color) gtk_combo_box_set_active(GTK_COMBO_BOX(combo_color), t->color);
  ir_ui_guard = 0;
}

static void on_cloud_slider(GtkRange *r, gpointer u) {
  (void)r;
  (void)u;
  if (ir_ui_guard) return;
  apply_cloud_tune();
}

static void on_cloud_color(GtkComboBox *c, gpointer u) {
  (void)c;
  (void)u;
  if (ir_ui_guard) return;
  apply_cloud_tune();
}

static void on_cloud_closeup(GtkButton *b, gpointer u) {
  cube_cloud_tune t;
  (void)b;
  (void)u;
  cube_cloud_tune_default(&t);
  t.z_near = 0.45f;
  t.z_far = 1.80f;
  t.stride = 2;
  t.dot = 3;
  t.color = 0;
  sliders_from_cloud(&t);
  kinect_cloud_set_tune(&t);
  set_status("Cloud Close-up — 0.45–1.8 m, dense");
}

static void on_cloud_room(GtkButton *b, gpointer u) {
  cube_cloud_tune t;
  (void)b;
  (void)u;
  cube_cloud_tune_default(&t);
  sliders_from_cloud(&t);
  kinect_cloud_set_tune(&t);
  set_status("Cloud Room — 0.5–3.5 m");
}

static void on_fg_boost(GtkRange *r, gpointer u) {
  int m;
  char buf[96];
  const char *be;
  (void)u;
  if (ir_ui_guard) return;
  m = (int)gtk_range_get_value(r);
  kinect_bridge_set_fg_boost(m);
  kinect_cloud_set_fg_boost(m);
  be = is_cloud() ? kinect_cloud_fg_backend() : kinect_bridge_fg_backend();
  if (lbl_fg) {
    snprintf(buf, sizeof buf, "FG %s · %dx (in-memory, no save)", be,
             m + 1);
    gtk_label_set_text(GTK_LABEL(lbl_fg), buf);
  }
}

static void on_cloud_reset_view(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  kinect_cloud_reset_view();
  orbit_e = 18.f;
  orbit_a = -60.f;
  orbit_d = 3.2f;
  set_status("Cloud view reset");
}

static void on_ir_reset(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  kinect_ir_tune_default(&ir_tune);
  sliders_from_tune(&ir_tune);
  kinect_bridge_set_ir_tune(&ir_tune);
  set_status("IR grade reset");
}

static void *tilt_work(void *arg) {
  double d = *(double *)arg;
  free(arg);
  {
    int rc = (is_cloud() && kinect_cloud_running()) ? kinect_cloud_set_tilt(d)
                                                    : kinect_motor_set_tilt(d);
    if (rc == 0)
      g_idle_add(apply_status, g_strdup_printf("Tilt → %.0f°", d));
    else
      g_idle_add(apply_status, g_strdup("Tilt error"));
  }
  return NULL;
}

static void set_tilt(double deg) {
  double *p;
  pthread_t t;
  if (deg < TILT_MIN) deg = TILT_MIN;
  if (deg > TILT_MAX) deg = TILT_MAX;
  target_tilt = deg;
  if (fabs(gtk_range_get_value(GTK_RANGE(tilt_scale)) - deg) > 0.05)
    gtk_range_set_value(GTK_RANGE(tilt_scale), deg);
  p = malloc(sizeof *p);
  if (!p) return;
  *p = deg;
  if (pthread_create(&t, NULL, tilt_work, p) == 0)
    pthread_detach(t);
  else
    free(p);
}

static void on_tilt_up(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  set_tilt(target_tilt + STEP);
}
static void on_tilt_dn(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  set_tilt(target_tilt - STEP);
}
static void on_tilt_lv(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  set_tilt(0);
}

static gboolean on_scale(GtkWidget *w, GdkEvent *e, gpointer u) {
  (void)w;
  (void)e;
  (void)u;
  set_tilt(gtk_range_get_value(GTK_RANGE(tilt_scale)));
  return FALSE;
}

static gboolean on_orbit_press(GtkWidget *w, GdkEventButton *e, gpointer u) {
  (void)w;
  (void)u;
  if (e->button != 1) return FALSE;
  drag_orbit = 1;
  drag_x = e->x;
  drag_y = e->y;
  kinect_cloud_get_orbit(&orbit_e, &orbit_a, &orbit_d);
  return TRUE;
}

static gboolean on_orbit_release(GtkWidget *w, GdkEventButton *e, gpointer u) {
  (void)w;
  (void)e;
  (void)u;
  drag_orbit = 0;
  return FALSE;
}

static gboolean on_orbit_motion(GtkWidget *w, GdkEventMotion *e, gpointer u) {
  (void)w;
  (void)u;
  if (!drag_orbit || !is_cloud()) return FALSE;
  orbit_a += (float)(e->x - drag_x) * 0.45f;
  orbit_e += (float)(e->y - drag_y) * 0.45f;
  if (orbit_e > 89.f) orbit_e = 89.f;
  if (orbit_e < -89.f) orbit_e = -89.f;
  drag_x = e->x;
  drag_y = e->y;
  kinect_cloud_orbit(orbit_e, orbit_a, orbit_d);
  return TRUE;
}

static gboolean on_orbit_scroll(GtkWidget *w, GdkEventScroll *e, gpointer u) {
  (void)w;
  (void)u;
  if (!is_cloud()) return FALSE;
  if (e->direction == GDK_SCROLL_UP) orbit_d *= 0.9f;
  else if (e->direction == GDK_SCROLL_DOWN) orbit_d *= 1.11f;
  if (orbit_d < 0.5f) orbit_d = 0.5f;
  if (orbit_d > 12.f) orbit_d = 12.f;
  kinect_cloud_orbit(orbit_e, orbit_a, orbit_d);
  return TRUE;
}

static gboolean poll_tilt(gpointer u) {
  double deg = 0;
  int st = 0;
  char buf[96];
  (void)u;
  if (closing) return FALSE;
  if (is_cloud()) {
    /* Never reopen motor-only freenect while Cloud owns USB. */
    if (kinect_cloud_get_tilt(&deg, &st) != 0) {
      gtk_label_set_text(GTK_LABEL(tilt_lbl), "Tilt: cloud");
      return TRUE;
    }
  } else {
    kinect_motor_pump();
    if (kinect_motor_get_tilt(&deg, &st) != 0) {
      gtk_label_set_text(GTK_LABEL(tilt_lbl), "Tilt: error");
      return TRUE;
    }
  }
  if (deg < -40) deg = target_tilt;
  snprintf(buf, sizeof buf, "Tilt: %.1f°  (target %.0f°)  [%s]", deg, target_tilt,
           st == 4 ? "moving" : "ok");
  gtk_label_set_text(GTK_LABEL(tilt_lbl), buf);
  return TRUE;
}

static int live_ok(void) { return kinect_session_running(); }

static gboolean preview_tick(gpointer u) {
  (void)u;
  if (closing) return FALSE;
  if (is_cloud()) {
    uint8_t *rgb = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
    if (rgb && kinect_cloud_preview_rgb(rgb) == 0)
      show_rgb(rgb, KINECT_RGB_W, KINECT_RGB_H);
    free(rgb);
    if (kinect_cloud_live()) {
      char buf[256];
      cloud_status_line(buf, sizeof buf, "Cloud LIVE");
      set_status(buf);
    }
    if (kinect_cloud_running() || kinect_cloud_live()) refresh_dev();
    return TRUE;
  }
  if (camera_on && kinect_bridge_running()) {
    uint8_t *rgb = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
    if (rgb && kinect_bridge_preview_rgb(rgb) == 0)
      show_rgb(rgb, KINECT_RGB_W, KINECT_RGB_H);
    free(rgb);
    return TRUE;
  }
  if (preview_on && !bridge_starting) snap_async();
  if (camera_on && !bridge_starting && !live_ok()) {
    const char *e = is_cloud() ? kinect_cloud_error() : kinect_bridge_error();
    if (e && e[0]) {
      char buf[200];
      snprintf(buf, sizeof buf, "Stopped: %s", e);
      set_status(buf);
    } else
      set_status("Stopped — toggle Camera ON again");
    camera_on = 0;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cam), FALSE);
    refresh_dev();
  } else if (camera_on && is_cloud() && kinect_cloud_running())
    refresh_dev();
  return TRUE;
}

static void cycle_sensor(int dir) {
  static const kinect_src order[] = {KINECT_SRC_AUTO, KINECT_SRC_V1, KINECT_SRC_V2,
                                     KINECT_SRC_BOTH};
  kinect_src cur = kinect_fuse_source();
  int at = 0, i;
  GtkWidget *b;
  for (i = 0; i < 4; i++)
    if (order[i] == cur) at = i;
  at = (at + (dir < 0 ? 3 : 1)) % 4;
  b = src_radio_for(order[at]);
  if (b) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(b), TRUE);
  refresh_full_tag();
}

static void cycle_mode(int dir) {
  GtkWidget *order[3];
  int at = 0;
  order[0] = btn_rgb_w;
  order[1] = btn_ir_w;
  order[2] = btn_cloud_w;
  if (is_ir()) at = 1;
  else if (is_cloud()) at = 2;
  at = (at + (dir < 0 ? 2 : 1)) % 3;
  if (order[at]) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(order[at]), TRUE);
  refresh_full_tag();
}

/* KWin ignores _NET_WM_STATE_FULLSCREEN on a utility window that stays on top.
   Withdraw, switch to a normal window, then ask again. If the atom still does
   not stick, cover the monitor so the view fills the screen anyway. */
static void cover_monitor(void) {
  GdkDisplay *d;
  GdkWindow *gw;
  GdkMonitor *mon = NULL;
  GdkRectangle geo;
  if (!win) return;
  d = gtk_widget_get_display(win);
  gw = gtk_widget_get_window(win);
  if (d && gw) mon = gdk_display_get_monitor_at_window(d, gw);
  if (!mon && d) mon = gdk_display_get_primary_monitor(d);
  if (!mon && d) mon = gdk_display_get_monitor(d, 0);
  if (!mon) return;
  gdk_monitor_get_geometry(mon, &geo);
  if (geo.width < 64 || geo.height < 64) return;
  gtk_window_move(GTK_WINDOW(win), geo.x, geo.y);
  gtk_window_resize(GTK_WINDOW(win), geo.width, geo.height);
}

static gboolean fullscreen_stuck(gpointer u) {
  GdkWindow *gw;
  (void)u;
  fs_watch = 0;
  if (!view_full || !win) return G_SOURCE_REMOVE;
  gw = gtk_widget_get_window(win);
  if (gw && (gdk_window_get_state(gw) & GDK_WINDOW_STATE_FULLSCREEN)) {
    paint_w = 0;
    paint_h = 0;
    if (preview_img) gtk_widget_queue_draw(preview_img);
    return G_SOURCE_REMOVE;
  }
  gtk_window_fullscreen(GTK_WINDOW(win));
  cover_monitor();
  paint_w = 0;
  paint_h = 0;
  paint_preview();
  return G_SOURCE_REMOVE;
}

static void enter_view_full(void) {
  if (view_full || !win || !preview_ev || !view_box || !root_box) return;
  gtk_window_get_position(GTK_WINDOW(win), &win_x, &win_y);
  gtk_window_get_size(GTK_WINDOW(win), &win_w, &win_h);
  g_object_ref(preview_ev);
  gtk_container_remove(GTK_CONTAINER(view_box), preview_ev);
  g_object_ref(root_box);
  gtk_container_remove(GTK_CONTAINER(win), root_box);
  gtk_container_set_border_width(GTK_CONTAINER(win), 0);
  gtk_container_add(GTK_CONTAINER(win), preview_ev);
  g_object_unref(preview_ev);
  view_full = 1;
  refresh_full_tag();
  if (full_tag) gtk_widget_show(full_tag);
  gtk_widget_set_can_focus(preview_ev, TRUE);
  gtk_widget_set_hexpand(preview_ev, TRUE);
  gtk_widget_set_vexpand(preview_ev, TRUE);
  gtk_widget_set_hexpand(preview_img, TRUE);
  gtk_widget_set_vexpand(preview_img, TRUE);
  gtk_widget_set_halign(preview_img, GTK_ALIGN_FILL);
  gtk_widget_set_valign(preview_img, GTK_ALIGN_FILL);
  if (restore_watch) {
    g_source_remove(restore_watch);
    restore_watch = 0;
  }
  gtk_window_set_keep_above(GTK_WINDOW(win), FALSE);
  gtk_window_set_decorated(GTK_WINDOW(win), FALSE);
  gtk_widget_hide(win);
  gtk_window_set_type_hint(GTK_WINDOW(win), GDK_WINDOW_TYPE_HINT_NORMAL);
  gtk_widget_show(win);
  gtk_window_fullscreen(GTK_WINDOW(win));
  gtk_window_present(GTK_WINDOW(win));
  gtk_widget_grab_focus(preview_ev);
  paint_w = 0;
  paint_h = 0;
  paint_preview();
  if (fs_watch) g_source_remove(fs_watch);
  fs_watch = g_timeout_add(400, fullscreen_stuck, NULL);
}

static void apply_windowed_geom(void) {
  if (!win || view_full || win_w < 1 || win_h < 1) return;
  gtk_window_unmaximize(GTK_WINDOW(win));
  gtk_window_move(GTK_WINDOW(win), win_x, win_y);
  gtk_window_resize(GTK_WINDOW(win), win_w, win_h);
}

static gboolean restore_windowed_size(gpointer u) {
  (void)u;
  restore_watch = 0;
  apply_windowed_geom();
  return G_SOURCE_REMOVE;
}

static void leave_view_full(void) {
  int above;
  if (!view_full || !win || !preview_ev || !view_box || !root_box) return;
  if (fs_watch) {
    g_source_remove(fs_watch);
    fs_watch = 0;
  }
  view_full = 0;
  gtk_window_unfullscreen(GTK_WINDOW(win));
  gtk_window_unmaximize(GTK_WINDOW(win));
  if (full_tag) gtk_widget_hide(full_tag);
  gtk_widget_set_can_focus(preview_ev, FALSE);
  gtk_widget_set_hexpand(preview_img, FALSE);
  gtk_widget_set_vexpand(preview_img, FALSE);
  gtk_widget_set_halign(preview_img, GTK_ALIGN_CENTER);
  gtk_widget_set_valign(preview_img, GTK_ALIGN_CENTER);
  gtk_widget_set_size_request(preview_img, 460, 240);
  g_object_ref(preview_ev);
  gtk_container_remove(GTK_CONTAINER(win), preview_ev);
  gtk_container_add(GTK_CONTAINER(win), root_box);
  g_object_unref(root_box);
  gtk_box_pack_start(GTK_BOX(view_box), preview_ev, TRUE, TRUE, 0);
  gtk_box_reorder_child(GTK_BOX(view_box), preview_ev, 0);
  g_object_unref(preview_ev);
  gtk_container_set_border_width(GTK_CONTAINER(win), 10);
  above = keep_above && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(keep_above));
  paint_w = 0;
  paint_h = 0;
  paint_preview();
  gtk_widget_hide(win);
  gtk_window_set_decorated(GTK_WINDOW(win), TRUE);
  gtk_window_set_type_hint(GTK_WINDOW(win), GDK_WINDOW_TYPE_HINT_UTILITY);
  gtk_widget_show_all(win);
  if (full_tag) gtk_widget_hide(full_tag);
  if (above) gtk_window_set_keep_above(GTK_WINDOW(win), TRUE);
  gtk_window_set_title(GTK_WINDOW(win), "Kinect Panel");
  apply_windowed_geom();
  if (restore_watch) g_source_remove(restore_watch);
  restore_watch = g_timeout_add(250, restore_windowed_size, NULL);
  if (btn_cam) gtk_widget_grab_focus(btn_cam);
}

static gboolean fs_idle(gpointer u) {
  int op = fs_queued;
  (void)u;
  fs_queued = 0;
  if (op == 1)
    enter_view_full();
  else if (op == 2)
    leave_view_full();
  return G_SOURCE_REMOVE;
}

/* Reparent after the key event returns. Doing it inside the handler removes
   the widget that is still on the emission stack. */
static void queue_fs(int enter) {
  if (fs_queued == 0) g_idle_add(fs_idle, NULL);
  fs_queued = enter ? 1 : 2;
}

static gboolean on_key(GtkWidget *w, GdkEventKey *e, gpointer u) {
  (void)w;
  (void)u;
  if (!e) return FALSE;
  if (e->keyval == GDK_KEY_F11) {
    if (e->time && f11_ms && e->time - f11_ms < 400) return TRUE;
    f11_ms = e->time;
    queue_fs(!view_full);
    return TRUE;
  }
  if (!view_full) return FALSE;
  if (e->keyval == GDK_KEY_Escape) {
    queue_fs(0);
    return TRUE;
  }
  if (e->keyval != GDK_KEY_Left && e->keyval != GDK_KEY_Right && e->keyval != GDK_KEY_Up &&
      e->keyval != GDK_KEY_Down)
    return FALSE;
  if (arrow_ms && e->time - arrow_ms < 220) return TRUE;
  arrow_ms = e->time;
  if (e->keyval == GDK_KEY_Left)
    cycle_sensor(-1);
  else if (e->keyval == GDK_KEY_Right)
    cycle_sensor(1);
  else if (e->keyval == GDK_KEY_Up)
    cycle_mode(-1);
  else
    cycle_mode(1);
  return TRUE;
}

static gboolean on_delete(GtkWidget *w, GdkEvent *e, gpointer u) {
  (void)w;
  (void)e;
  (void)u;
  /* X = quit. Hide is the explicit "keep process" path. */
  gtk_widget_destroy(win);
  return TRUE;
}

static void raise_panel(void) {
  if (!win) return;
  gtk_widget_show_all(win);
  gtk_window_present(GTK_WINDOW(win));
  if (keep_above)
    gtk_window_set_keep_above(GTK_WINDOW(win),
                              gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(keep_above)));
}

static void ui_from_session(void) {
  kinect_mode m = kinect_session_mode();
  int on = kinect_session_running();
  GtkWidget *radio = btn_rgb_w;
  camera_on = on;
  if (btn_cam) {
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cam), on);
    gtk_button_set_label(GTK_BUTTON(btn_cam), on ? "Camera ON" : "Camera OFF");
  }
  if (m == KINECT_MODE_IR) {
    snprintf(mode, sizeof mode, "IR");
    radio = btn_ir_w;
  } else if (m == KINECT_MODE_CLOUD) {
    snprintf(mode, sizeof mode, "CLOUD");
    radio = btn_cloud_w;
  } else if (m == KINECT_MODE_RGB) {
    snprintf(mode, sizeof mode, "RGB");
    radio = btn_rgb_w;
  }
  if (radio) {
    mode_suppress = 1;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(radio), TRUE);
    mode_suppress = 0;
  }
  if (src_radio_for(kinect_fuse_source())) {
    src_suppress = 1;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(src_radio_for(kinect_fuse_source())), TRUE);
    src_suppress = 0;
  }
  if (ir_frame) gtk_widget_set_sensitive(ir_frame, is_ir());
  if (cloud_frame) gtk_widget_set_sensitive(cloud_frame, is_cloud());
  refresh_dev();
}

static int gui_dispatch(const char *cmd, char *reply, size_t n) {
  char low[128];
  size_t i;
  snprintf(low, sizeof low, "%s", cmd ? cmd : "");
  for (i = 0; low[i]; i++)
    if (low[i] >= 'A' && low[i] <= 'Z') low[i] = (char)(low[i] - 'A' + 'a');
  if (strcmp(low, "raise") == 0) {
    raise_panel();
    snprintf(reply, n, "{\"ok\":true,\"action\":\"raise\"}\n");
    return 0;
  }
  {
    int q = kinect_session_dispatch(cmd, reply, n);
    ui_from_session();
    if (q) {
      closing = 1;
      gtk_main_quit();
    }
    return q;
  }
}

static gboolean on_sock(GIOChannel *ch, GIOCondition c, gpointer u) {
  (void)ch;
  (void)c;
  (void)u;
  kinect_ctl_pump(gui_dispatch);
  return TRUE;
}

static void cleanup(void) {
  if (closing && !kinect_session_running() && kinect_ctl_fd() < 0) {
    /* already torn down */
  }
  closing = 1;
  kinect_session_off();
  kinect_net_stop();
  kinect_ctl_close();
}

static void on_destroy(GtkWidget *w, gpointer u) {
  (void)w;
  (void)u;
  free(frame_rgb);
  frame_rgb = NULL;
  if (view_full && root_box) {
    g_object_unref(root_box);
    root_box = NULL;
    view_full = 0;
  }
  cleanup();
  gtk_main_quit();
}

static void on_sig(int s) {
  (void)s;
  got_sig = 1;
}

static gboolean sig_tick(gpointer u) {
  (void)u;
  if (!got_sig) return TRUE;
  cleanup();
  gtk_main_quit();
  return FALSE;
}

static void on_keep(GtkToggleButton *b, gpointer u) {
  (void)u;
  gtk_window_set_keep_above(GTK_WINDOW(win), gtk_toggle_button_get_active(b));
}

static void on_hide(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  gtk_widget_hide(win);
}

static void on_quit_clicked(GtkButton *b, gpointer u) {
  (void)b;
  (void)u;
  gtk_widget_destroy(win);
}

static gboolean boot(gpointer u) {
  double deg = 0;
  int st = 0;
  cube_cloud_tune ct;
  (void)u;
  kinect_ir_tune_default(&ir_tune);
  sliders_from_tune(&ir_tune);
  kinect_bridge_set_ir_tune(&ir_tune);
  cube_cloud_tune_default(&ct);
  sliders_from_cloud(&ct);
  kinect_cloud_set_tune(&ct);
  kinect_bridge_set_fg_boost(1);
  kinect_cloud_set_fg_boost(1);
  kinect_session_off(); /* GUI boot: sensor dark until Camera ON */
  if (kinect_motor_open() == 0 && kinect_motor_get_tilt(&deg, &st) == 0) {
    if (deg < TILT_MIN || deg > TILT_MAX) deg = 0;
    target_tilt = deg;
    gtk_range_set_value(GTK_RANGE(tilt_scale), deg);
    set_status("Ready. Kinect OFF. RGB / IR / Cloud → Camera ON.");
  } else
    set_status("Motor: not available (USB motor busy?)");
  refresh_dev();
  return FALSE;
}

static GtkWidget *mk_scale(const char *name, double lo, double hi, double step, double val,
                           GtkWidget *box, GCallback cb) {
  GtkWidget *row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
  GtkWidget *lab = gtk_label_new(name);
  GtkWidget *sc = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, lo, hi, step);
  gtk_widget_set_size_request(lab, 72, -1);
  gtk_label_set_xalign(GTK_LABEL(lab), 0);
  gtk_range_set_value(GTK_RANGE(sc), val);
  gtk_scale_set_draw_value(GTK_SCALE(sc), TRUE);
  gtk_scale_set_value_pos(GTK_SCALE(sc), GTK_POS_RIGHT);
  gtk_widget_set_hexpand(sc, TRUE);
  if (cb) g_signal_connect(sc, "value-changed", cb, NULL);
  gtk_box_pack_start(GTK_BOX(row), lab, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(row), sc, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box), row, FALSE, FALSE, 0);
  return sc;
}

static int take_mode_token(const char *tok, kinect_mode *mode_out, kinect_src *src_out,
                           int *src_set) {
  kinect_src s;
  if (!tok || !mode_out) return 0;
  /* cloud2 / v2 / kinect2 / both / sync / v1 select a cloud source.
   * auto is a --src value, not a mode name. */
  if (kinect_src_known(tok, &s) && s != KINECT_SRC_AUTO) {
    *mode_out = KINECT_MODE_CLOUD;
    if (src_out) *src_out = s;
    if (src_set) *src_set = 1;
    return 1;
  }
  *mode_out = kinect_session_parse_mode(tok);
  if (*mode_out == KINECT_MODE_OFF && tok[0] != 'o' && tok[0] != 'O') return 0;
  return 1;
}

static void usage(void) {
  fputs(
      "kinect-panel — Kinect and Kinect 2 (C)\n"
      "  (no args)                         GUI. Sensors OFF until Camera ON.\n"
      "  --headless [--mode rgb|ir|cloud]  stream, no window. Only auto-start.\n"
      "  --mode v1|v2|cloud2|kinect2|both|sync\n"
      "                                    cloud from that sensor (or both).\n"
      "  --src auto|v1|v2|both             sensor for RGB, IR, and Cloud.\n"
      "                                    v2/both + rgb|ir opens Kinect 2.\n"
      "  --off                             LED off, streams off, USB released.\n"
      "  --status                          JSON SoT line (does not touch USB)\n"
      "  --quit                            stop the owner process\n"
      "  --help\n"
      "GUI: F11 full screen of the RGB, IR, or cloud view. Esc or F11 leaves.\n"
      "     Full screen arrows: left/right camera, up/down mode.\n"
      "     Share listens on UDP 8765. Connect uses the peer as a local sensor.\n"
      "Socket: START cloud [v1|v2|both|auto]  |  ALIGN  |  ALIGN RESET\n"
      "        SHARE  |  CONNECT host[:port]  |  NET OFF\n"
      "Env: KINECT_SRC  KINECT_PEER  KINECT_SHARE_PORT  KINECT2_PIPELINE  KINECT_EXTRINSIC\n",
      stdout);
}

static int send_or_local(const char *cmd, int orphan_off) {
  char reply[512];
  if (kinect_ctl_send(cmd, reply, sizeof reply) == 0) {
    fputs(reply, stdout);
    if (!strchr(reply, '\n')) fputc('\n', stdout);
    return 0;
  }
  if (cmd && strncmp(cmd, "STATUS", 6) == 0) {
    /* No owner: bits only. Do not touch USB. */
    kinect_session_status_json(reply, sizeof reply);
    fputs(reply, stdout);
    return 0;
  }
  if (orphan_off) {
    kinect_session_off();
    kinect_session_status_json(reply, sizeof reply);
    fputs(reply, stdout);
    return 0;
  }
  fprintf(stderr, "kinect-panel: no owner process\n");
  return 1;
}

int main(int argc, char **argv) {
  GtkWidget *root, *frame, *box, *row, *cam_box, *tilt_f, *tilt, *foot, *ir_box;
  GtkWidget *up, *dn, *lv, *hide, *quitb, *snap, *tip;
  GtkWidget *see, *reset, *page_cam, *cam_page, *page_img, *scroll_img;
  GtkWidget *cloud_box, *cup, *room, *rview, *alignb, *alignr;
  int want_headless = 0, want_off = 0, want_status = 0, want_quit = 0, i;
  int src_explicit = 0, mode_given = 0;
  kinect_mode cli_mode = KINECT_MODE_RGB;
  kinect_src cli_src = KINECT_SRC_AUTO;
  GIOChannel *ch;
  char reply[512];

  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      usage();
      return 0;
    }
    if (strcmp(argv[i], "--headless") == 0) {
      want_headless = 1;
      continue;
    }
    if (strcmp(argv[i], "--off") == 0) {
      want_off = 1;
      continue;
    }
    if (strcmp(argv[i], "--status") == 0) {
      want_status = 1;
      continue;
    }
    if (strcmp(argv[i], "--quit") == 0) {
      want_quit = 1;
      continue;
    }
    if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
      const char *tok = argv[++i];
      if (!take_mode_token(tok, &cli_mode, &cli_src, &src_explicit)) {
        fprintf(stderr, "kinect-panel: unknown mode %s\n", tok);
        usage();
        return 2;
      }
      if (cli_mode == KINECT_MODE_OFF) cli_mode = KINECT_MODE_RGB;
      mode_given = 1;
      continue;
    }
    if (strncmp(argv[i], "--mode=", 7) == 0) {
      const char *tok = argv[i] + 7;
      if (!take_mode_token(tok, &cli_mode, &cli_src, &src_explicit)) {
        fprintf(stderr, "kinect-panel: unknown mode %s\n", tok);
        usage();
        return 2;
      }
      if (cli_mode == KINECT_MODE_OFF) cli_mode = KINECT_MODE_RGB;
      mode_given = 1;
      continue;
    }
    if ((strcmp(argv[i], "--src") == 0 && i + 1 < argc) || strncmp(argv[i], "--src=", 6) == 0) {
      const char *tok = argv[i][5] == '=' ? argv[i] + 6 : argv[++i];
      if (!kinect_src_known(tok, &cli_src)) {
        fprintf(stderr, "kinect-panel: unknown src %s\n", tok);
        usage();
        return 2;
      }
      src_explicit = 1;
      continue;
    }
    fprintf(stderr, "kinect-panel: unknown arg %s\n", argv[i]);
    usage();
    return 2;
  }

  if (want_status)
    return send_or_local("STATUS", 1);
  if (want_off)
    return send_or_local("OFF", 1);
  if (want_quit)
    return send_or_local("QUIT", 0);
  if (want_headless) {
    if (kinect_ctl_send("STATUS", reply, sizeof reply) == 0) {
      /* Forward a source only when this command named one. */
      if (src_explicit)
        snprintf(reply, sizeof reply, "START %s %s", kinect_session_mode_name(cli_mode),
                 kinect_src_name(cli_src));
      else
        snprintf(reply, sizeof reply, "START %s", kinect_session_mode_name(cli_mode));
      return send_or_local(reply, 0);
    }
    if (src_explicit) kinect_fuse_set_source(cli_src);
    return kinect_headless_run(cli_mode);
  }
  if (src_explicit) kinect_fuse_set_source(cli_src);

  if (kinect_ctl_send("RAISE", reply, sizeof reply) == 0) {
    fputs(reply, stdout);
    return 0;
  }

  gtk_init(&argc, &argv);
  if (kinect_ctl_listen() < 0) {
    fprintf(stderr, "kinect-panel: ctl bind failed\n");
    return 1;
  }
  ch = g_io_channel_unix_new(kinect_ctl_fd());
  g_io_add_watch(ch, G_IO_IN, on_sock, NULL);
  g_io_channel_unref(ch);
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);
  signal(SIGHUP, on_sig);
  atexit(cleanup);

  win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), "Kinect Panel");
  gtk_window_set_default_size(GTK_WINDOW(win), 520, 480);
  gtk_container_set_border_width(GTK_CONTAINER(win), 10);
  gtk_window_set_keep_above(GTK_WINDOW(win), TRUE);
  gtk_window_set_type_hint(GTK_WINDOW(win), GDK_WINDOW_TYPE_HINT_UTILITY);
  g_signal_connect(win, "delete-event", G_CALLBACK(on_delete), NULL);
  g_signal_connect(win, "destroy", G_CALLBACK(on_destroy), NULL);

  root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
  root_box = root;
  gtk_container_add(GTK_CONTAINER(win), root);
  g_signal_connect(win, "key-press-event", G_CALLBACK(on_key), NULL);

  status_lbl = gtk_label_new("Starting…");
  gtk_label_set_xalign(GTK_LABEL(status_lbl), 0);
  gtk_label_set_line_wrap(GTK_LABEL(status_lbl), TRUE);
  gtk_box_pack_start(GTK_BOX(root), status_lbl, FALSE, FALSE, 0);

  frame = gtk_frame_new("View  (RGB / IR / Cloud)   F11");
  box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
  view_box = box;
  gtk_container_set_border_width(GTK_CONTAINER(box), 4);
  preview_ev = gtk_event_box_new();
  gtk_widget_set_name(preview_ev, "kinect-preview");
  gtk_widget_add_events(preview_ev, GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK |
                                         GDK_POINTER_MOTION_MASK | GDK_SCROLL_MASK |
                                         GDK_KEY_PRESS_MASK);
  {
    GtkWidget *overlay = gtk_overlay_new();
    GtkCssProvider *css = gtk_css_provider_new();
    preview_img = gtk_image_new();
    gtk_widget_set_size_request(preview_img, 460, 240);
    gtk_widget_set_halign(preview_img, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(preview_img, GTK_ALIGN_CENTER);
    g_signal_connect(preview_img, "draw", G_CALLBACK(on_img_draw), NULL);
    gtk_container_add(GTK_CONTAINER(overlay), preview_img);
    full_tag = gtk_label_new("");
    gtk_widget_set_halign(full_tag, GTK_ALIGN_START);
    gtk_widget_set_valign(full_tag, GTK_ALIGN_START);
    gtk_widget_set_margin_start(full_tag, 16);
    gtk_widget_set_margin_top(full_tag, 12);
    gtk_style_context_add_class(gtk_widget_get_style_context(full_tag), "kinect-fulltag");
    gtk_widget_set_no_show_all(full_tag, TRUE);
    gtk_overlay_add_overlay(GTK_OVERLAY(overlay), full_tag);
    gtk_container_add(GTK_CONTAINER(preview_ev), overlay);
    gtk_css_provider_load_from_data(
        css,
        "#kinect-preview, #kinect-preview * { background-color: #000000; }\n"
        "label.kinect-fulltag { background-color: rgba(0,0,0,0.72); color: #ffffff;"
        " padding: 8px 12px; }\n",
        -1, NULL);
    gtk_style_context_add_provider(gtk_widget_get_style_context(preview_ev),
                                   GTK_STYLE_PROVIDER(css),
                                   GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    gtk_style_context_add_provider(gtk_widget_get_style_context(full_tag),
                                   GTK_STYLE_PROVIDER(css),
                                   GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(css);
  }
  gtk_widget_set_tooltip_text(preview_ev,
                              "F11: full screen of this view. There, left/right changes camera "
                              "and up/down changes RGB, IR, or Cloud.");
  g_signal_connect(preview_ev, "button-press-event", G_CALLBACK(on_orbit_press), NULL);
  g_signal_connect(preview_ev, "button-release-event", G_CALLBACK(on_orbit_release), NULL);
  g_signal_connect(preview_ev, "motion-notify-event", G_CALLBACK(on_orbit_motion), NULL);
  g_signal_connect(preview_ev, "scroll-event", G_CALLBACK(on_orbit_scroll), NULL);
  g_signal_connect(preview_ev, "key-press-event", G_CALLBACK(on_key), NULL);
  g_signal_connect(preview_ev, "size-allocate", G_CALLBACK(on_preview_size), NULL);
  gtk_box_pack_start(GTK_BOX(box), preview_ev, TRUE, TRUE, 0);
  row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
  btn_preview = gtk_toggle_button_new_with_label("Preview OFF");
  g_signal_connect(btn_preview, "toggled", G_CALLBACK(on_preview), NULL);
  snap = gtk_button_new_with_label("Snap once");
  g_signal_connect(snap, "clicked", G_CALLBACK(on_snap_clicked), NULL);
  gtk_box_pack_start(GTK_BOX(row), btn_preview, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), snap, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(box), row, FALSE, FALSE, 0);
  gtk_container_add(GTK_CONTAINER(frame), box);

  tabs = gtk_notebook_new();
  gtk_notebook_set_scrollable(GTK_NOTEBOOK(tabs), TRUE);
  page_cam = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
  cam_page = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
  page_img = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
  gtk_container_set_border_width(GTK_CONTAINER(page_cam), 6);
  gtk_container_set_border_width(GTK_CONTAINER(cam_page), 6);
  gtk_container_set_border_width(GTK_CONTAINER(page_img), 6);
  gtk_box_pack_start(GTK_BOX(root), tabs, TRUE, TRUE, 0);

  cam_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
  btn_cam = gtk_toggle_button_new_with_label("Camera OFF");
  gtk_style_context_add_class(gtk_widget_get_style_context(btn_cam), "suggested-action");
  g_signal_connect(btn_cam, "toggled", G_CALLBACK(on_cam), NULL);
  gtk_box_pack_start(GTK_BOX(cam_box), btn_cam, FALSE, FALSE, 0);
  {
    GtkWidget *mrow = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    btn_rgb_w = gtk_radio_button_new_with_label(NULL, "RGB");
    btn_ir_w = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_rgb_w), "IR");
    btn_cloud_w = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_rgb_w), "Cloud");
    g_signal_connect(btn_rgb_w, "toggled", G_CALLBACK(on_mode), (gpointer) "RGB");
    g_signal_connect(btn_ir_w, "toggled", G_CALLBACK(on_mode), (gpointer) "IR");
    g_signal_connect(btn_cloud_w, "toggled", G_CALLBACK(on_mode), (gpointer) "CLOUD");
    gtk_box_pack_start(GTK_BOX(mrow), btn_rgb_w, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(mrow), btn_ir_w, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(mrow), btn_cloud_w, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(cam_box), mrow, FALSE, FALSE, 0);
  }
  gtk_box_pack_start(GTK_BOX(cam_page), cam_box, FALSE, FALSE, 0);
  {
    GtkWidget *sf = gtk_frame_new("Sensor");
    GtkWidget *sb = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    GtkWidget *srow = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *srow2 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *arow = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *hint, *ab, *ar;
    kinect_src boot_src = kinect_fuse_source();
    gtk_container_set_border_width(GTK_CONTAINER(sb), 6);
    gtk_container_add(GTK_CONTAINER(sf), sb);
    btn_src_auto = gtk_radio_button_new_with_label(NULL, "Auto");
    btn_src_v1 = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_src_auto), "Kinect");
    btn_src_v2 =
        gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_src_auto), "Kinect 2");
    btn_src_both = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_src_auto), "Both");
    g_signal_connect(btn_src_auto, "toggled", G_CALLBACK(on_src_radio),
                     GINT_TO_POINTER(KINECT_SRC_AUTO));
    g_signal_connect(btn_src_v1, "toggled", G_CALLBACK(on_src_radio), GINT_TO_POINTER(KINECT_SRC_V1));
    g_signal_connect(btn_src_v2, "toggled", G_CALLBACK(on_src_radio), GINT_TO_POINTER(KINECT_SRC_V2));
    g_signal_connect(btn_src_both, "toggled", G_CALLBACK(on_src_radio),
                     GINT_TO_POINTER(KINECT_SRC_BOTH));
    src_suppress = 1;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(src_radio_for(boot_src)), TRUE);
    src_suppress = 0;
    gtk_box_pack_start(GTK_BOX(srow), btn_src_auto, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(srow), btn_src_v1, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(srow2), btn_src_v2, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(srow2), btn_src_both, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(sb), srow, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(sb), srow2, FALSE, FALSE, 0);
    ab = gtk_button_new_with_label("Align");
    ar = gtk_button_new_with_label("Reset align");
    g_signal_connect(ab, "clicked", G_CALLBACK(on_align_clicked), NULL);
    g_signal_connect(ar, "clicked", G_CALLBACK(on_align_reset), NULL);
    gtk_box_pack_start(GTK_BOX(arow), ab, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(arow), ar, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(sb), arow, FALSE, FALSE, 0);
    hint = gtk_label_new(
        "RGB, IR, and Cloud use this sensor. Both previews Kinect 2. "
        "Cloud Both fuses both. Blue points until Align.");
    gtk_label_set_xalign(GTK_LABEL(hint), 0);
    gtk_label_set_line_wrap(GTK_LABEL(hint), TRUE);
    gtk_box_pack_start(GTK_BOX(sb), hint, FALSE, FALSE, 0);
    {
      GtkWidget *nrow = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
      GtkWidget *go = gtk_button_new_with_label("Connect");
      const char *env_peer = getenv("KINECT_PEER");
      chk_share = gtk_check_button_new_with_label("Share");
      peer_entry = gtk_entry_new();
      gtk_entry_set_width_chars(GTK_ENTRY(peer_entry), 18);
      gtk_entry_set_placeholder_text(GTK_ENTRY(peer_entry), "host:8765");
      if (env_peer && env_peer[0]) gtk_entry_set_text(GTK_ENTRY(peer_entry), env_peer);
      net_lbl = gtk_label_new("Net off");
      gtk_label_set_xalign(GTK_LABEL(net_lbl), 0);
      g_signal_connect(chk_share, "toggled", G_CALLBACK(on_share_toggled), NULL);
      g_signal_connect(go, "clicked", G_CALLBACK(on_net_connect), NULL);
      gtk_widget_set_tooltip_text(chk_share, "Listen on UDP 8765. Does not open a camera.");
      gtk_widget_set_tooltip_text(go, "Use this host's Kinect as a local sensor.");
      gtk_box_pack_start(GTK_BOX(nrow), chk_share, FALSE, FALSE, 0);
      gtk_box_pack_start(GTK_BOX(nrow), peer_entry, TRUE, TRUE, 0);
      gtk_box_pack_start(GTK_BOX(nrow), go, FALSE, FALSE, 0);
      gtk_box_pack_start(GTK_BOX(sb), nrow, FALSE, FALSE, 0);
      gtk_box_pack_start(GTK_BOX(sb), net_lbl, FALSE, FALSE, 0);
    }
    gtk_box_pack_start(GTK_BOX(cam_page), sf, FALSE, FALSE, 0);
  }
  /* Preview stays under the mode buttons so a live frame cannot cover them. */
  gtk_box_pack_start(GTK_BOX(page_cam), frame, FALSE, FALSE, 0);
  if (mode_given && cli_mode == KINECT_MODE_CLOUD)
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cloud_w), TRUE);

  dev_lbl = gtk_label_new("");
  gtk_label_set_xalign(GTK_LABEL(dev_lbl), 0);
  gtk_label_set_selectable(GTK_LABEL(dev_lbl), TRUE);
  gtk_label_set_line_wrap(GTK_LABEL(dev_lbl), TRUE);
  gtk_box_pack_start(GTK_BOX(page_cam), dev_lbl, FALSE, FALSE, 0);

  ir_frame = gtk_frame_new("IR vision grade");
  ir_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
  gtk_container_set_border_width(GTK_CONTAINER(ir_box), 6);
  gtk_container_add(GTK_CONTAINER(ir_frame), ir_box);
  sc_boost = mk_scale("Boost", 0.5, 8.0, 0.1, 2.2, ir_box, G_CALLBACK(on_ir_slider));
  sc_bright = mk_scale("Bright", -80, 80, 1, 8, ir_box, G_CALLBACK(on_ir_slider));
  sc_contrast = mk_scale("Contrast", 0.4, 3.0, 0.05, 1.35, ir_box, G_CALLBACK(on_ir_slider));
  sc_gamma = mk_scale("Gamma", 0.35, 2.2, 0.05, 0.70, ir_box, G_CALLBACK(on_ir_slider));
  sc_black = mk_scale("Black %", 0, 25, 1, 4, ir_box, G_CALLBACK(on_ir_slider));
  sc_white = mk_scale("White %", 70, 100, 1, 96, ir_box, G_CALLBACK(on_ir_slider));
  row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
  see = gtk_button_new_with_label("SEE (boost B/W)");
  reset = gtk_button_new_with_label("Reset grade");
  g_signal_connect(see, "clicked", G_CALLBACK(on_see), NULL);
  g_signal_connect(reset, "clicked", G_CALLBACK(on_ir_reset), NULL);
  gtk_box_pack_start(GTK_BOX(row), see, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), reset, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(ir_box), row, FALSE, FALSE, 0);
  gtk_widget_set_sensitive(ir_frame, FALSE);
  gtk_box_pack_start(GTK_BOX(page_img), ir_frame, FALSE, FALSE, 0);

  cloud_frame = gtk_frame_new("Cloud");
  cloud_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
  gtk_container_set_border_width(GTK_CONTAINER(cloud_box), 6);
  gtk_container_add(GTK_CONTAINER(cloud_frame), cloud_box);
  sc_near = mk_scale("Near m", 0.35, 2.0, 0.05, 0.50, cloud_box, G_CALLBACK(on_cloud_slider));
  sc_far = mk_scale("Far m", 1.0, 5.0, 0.05, 3.50, cloud_box, G_CALLBACK(on_cloud_slider));
  sc_stride = mk_scale("Stride", 2, 8, 1, 3, cloud_box, G_CALLBACK(on_cloud_slider));
  sc_dot = mk_scale("Dot", 1, 4, 1, 2, cloud_box, G_CALLBACK(on_cloud_slider));
  {
    GtkWidget *crow = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *clab = gtk_label_new("Color");
    combo_color = gtk_combo_box_text_new();
    gtk_widget_set_size_request(clab, 72, -1);
    gtk_label_set_xalign(GTK_LABEL(clab), 0);
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_color), "Depth");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_color), "Height");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_color), "Grey");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo_color), "Objects");
    gtk_combo_box_set_active(GTK_COMBO_BOX(combo_color), 3);
    g_signal_connect(combo_color, "changed", G_CALLBACK(on_cloud_color), NULL);
    gtk_box_pack_start(GTK_BOX(crow), clab, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(crow), combo_color, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(cloud_box), crow, FALSE, FALSE, 0);
    chk_solid = gtk_check_button_new_with_label("Solid 3D map (objects)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_solid), TRUE);
    g_signal_connect(chk_solid, "toggled", G_CALLBACK(on_cloud_slider), NULL);
    gtk_box_pack_start(GTK_BOX(cloud_box), chk_solid, FALSE, FALSE, 0);
  }
  row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
  cup = gtk_button_new_with_label("Close-up");
  room = gtk_button_new_with_label("Room");
  rview = gtk_button_new_with_label("Reset view");
  g_signal_connect(cup, "clicked", G_CALLBACK(on_cloud_closeup), NULL);
  g_signal_connect(room, "clicked", G_CALLBACK(on_cloud_room), NULL);
  g_signal_connect(rview, "clicked", G_CALLBACK(on_cloud_reset_view), NULL);
  gtk_box_pack_start(GTK_BOX(row), cup, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), room, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), rview, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(cloud_box), row, FALSE, FALSE, 0);
  row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
  alignb = gtk_button_new_with_label("Align");
  alignr = gtk_button_new_with_label("Reset align");
  g_signal_connect(alignb, "clicked", G_CALLBACK(on_align_clicked), NULL);
  g_signal_connect(alignr, "clicked", G_CALLBACK(on_align_reset), NULL);
  gtk_box_pack_start(GTK_BOX(row), alignb, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), alignr, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(cloud_box), row, FALSE, FALSE, 0);
  gtk_widget_set_sensitive(cloud_frame, FALSE);
  gtk_box_pack_start(GTK_BOX(page_img), cloud_frame, FALSE, FALSE, 0);

  {
    GtkWidget *fgf = gtk_frame_new("Frame gen (ROCm)");
    GtkWidget *fgb = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_set_border_width(GTK_CONTAINER(fgb), 6);
    gtk_container_add(GTK_CONTAINER(fgf), fgb);
    sc_fg = mk_scale("Boost", 0, 3, 1, 1, fgb, G_CALLBACK(on_fg_boost));
    lbl_fg = gtk_label_new("FG · off until Camera/Cloud");
    gtk_label_set_xalign(GTK_LABEL(lbl_fg), 0);
    gtk_label_set_line_wrap(GTK_LABEL(lbl_fg), TRUE);
    gtk_box_pack_start(GTK_BOX(fgb), lbl_fg, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(page_img), fgf, FALSE, FALSE, 0);
  }

  tip = gtk_label_new(
      "Image tab: IR grade or Cloud clip/density/color. Camera tab keeps tilt and sensor.\n"
      "Cloud: Near/Far clip metres. Align fits Kinect 2 onto Kinect. Drag preview to orbit.");
  gtk_label_set_xalign(GTK_LABEL(tip), 0);
  gtk_label_set_line_wrap(GTK_LABEL(tip), TRUE);
  gtk_box_pack_start(GTK_BOX(page_img), tip, FALSE, FALSE, 0);
  scroll_img = gtk_scrolled_window_new(NULL, NULL);
  gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll_img), GTK_POLICY_NEVER,
                                 GTK_POLICY_AUTOMATIC);
  gtk_container_add(GTK_CONTAINER(scroll_img), page_img);

  tilt_f = gtk_frame_new("Tilt");
  tilt = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
  gtk_container_set_border_width(GTK_CONTAINER(tilt), 6);
  gtk_container_add(GTK_CONTAINER(tilt_f), tilt);
  row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
  up = gtk_button_new_with_label("▲ Up");
  lv = gtk_button_new_with_label("Level 0°");
  dn = gtk_button_new_with_label("▼ Down");
  g_signal_connect(up, "clicked", G_CALLBACK(on_tilt_up), NULL);
  g_signal_connect(lv, "clicked", G_CALLBACK(on_tilt_lv), NULL);
  g_signal_connect(dn, "clicked", G_CALLBACK(on_tilt_dn), NULL);
  gtk_box_pack_start(GTK_BOX(row), up, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), lv, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(row), dn, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(tilt), row, FALSE, FALSE, 0);
  tilt_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, TILT_MIN, TILT_MAX, 1);
  gtk_range_set_value(GTK_RANGE(tilt_scale), 0);
  gtk_scale_set_digits(GTK_SCALE(tilt_scale), 0);
  gtk_scale_set_draw_value(GTK_SCALE(tilt_scale), TRUE);
  g_signal_connect(tilt_scale, "button-release-event", G_CALLBACK(on_scale), NULL);
  g_signal_connect(tilt_scale, "key-release-event", G_CALLBACK(on_scale), NULL);
  gtk_box_pack_start(GTK_BOX(tilt), tilt_scale, FALSE, FALSE, 0);
  tilt_lbl = gtk_label_new("Tilt: —");
  gtk_box_pack_start(GTK_BOX(tilt), tilt_lbl, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page_cam), tilt_f, FALSE, FALSE, 0);
  {
    GtkWidget *scroll_cam = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll_cam), GTK_POLICY_NEVER,
                                   GTK_POLICY_AUTOMATIC);
    gtk_container_add(GTK_CONTAINER(scroll_cam), page_cam);
    gtk_box_pack_start(GTK_BOX(cam_page), scroll_cam, TRUE, TRUE, 0);
    gtk_notebook_append_page(GTK_NOTEBOOK(tabs), cam_page, gtk_label_new("Camera"));
  }
  gtk_notebook_append_page(GTK_NOTEBOOK(tabs), scroll_img, gtk_label_new("Image"));

  foot = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
  keep_above = gtk_check_button_new_with_label("Always on top");
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(keep_above), TRUE);
  g_signal_connect(keep_above, "toggled", G_CALLBACK(on_keep), NULL);
  hide = gtk_button_new_with_label("Hide");
  g_signal_connect(hide, "clicked", G_CALLBACK(on_hide), NULL);
  quitb = gtk_button_new_with_label("Quit");
  g_signal_connect(quitb, "clicked", G_CALLBACK(on_quit_clicked), NULL);
  gtk_box_pack_start(GTK_BOX(foot), keep_above, TRUE, TRUE, 0);
  gtk_box_pack_end(GTK_BOX(foot), quitb, FALSE, FALSE, 0);
  gtk_box_pack_end(GTK_BOX(foot), hide, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(root), foot, FALSE, FALSE, 0);

  g_timeout_add(250, poll_tilt, NULL);
  g_timeout_add(200, preview_tick, NULL);
  g_timeout_add(200, sig_tick, NULL);
  g_timeout_add(500, net_tick, NULL);
  g_idle_add(boot, NULL);
  {
    /* Logical screen is often half the pixel size. A fixed 860px window falls off it. */
    GdkDisplay *d = gdk_display_get_default();
    GdkMonitor *mon = d ? gdk_display_get_primary_monitor(d) : NULL;
    GdkRectangle work;
    int w = 520, h = 480;
    if (!mon && d) mon = gdk_display_get_monitor(d, 0);
    if (mon) {
      gdk_monitor_get_workarea(mon, &work);
      w = work.width > 80 ? work.width - 48 : 520;
      if (w > 720) w = 720;
      h = work.height > 80 ? work.height - 48 : 480;
      if (h > 640) h = 640;
    }
    gtk_window_set_default_size(GTK_WINDOW(win), w, h);
    gtk_widget_show_all(win);
    gtk_window_resize(GTK_WINDOW(win), w, h);
  }
  gtk_main();
  cleanup();
  return 0;
}
