#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
/* Kinect Panel — RGB / IR (boost) / point cloud. One view. Pure C. */
#include "kinect_bridge.h"
#include "kinect_cloud_view.h"
#include "kinect_ir.h"
#include "kinect_motor.h"
#include "kinect_v4l.h"

#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtk.h>
#include <libfreenect.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define STEP 5.0
#define TILT_MIN -30.0
#define TILT_MAX 30.0

static GtkWidget *win, *status_lbl, *dev_lbl, *preview_img, *preview_ev;
static GtkWidget *btn_cam, *btn_preview, *tilt_scale, *tilt_lbl, *keep_above;
static GtkWidget *ir_frame, *cloud_frame, *tabs;
static GtkWidget *sc_boost, *sc_bright, *sc_contrast, *sc_gamma, *sc_black, *sc_white;
static GtkWidget *sc_near, *sc_far, *sc_stride, *sc_dot, *combo_color;
static GtkWidget *sc_fg, *lbl_fg;
static int camera_on, preview_on, closing, snap_busy, bridge_starting;
static int sock_fd = -1;
static char sock_path[256];
static char mode[8] = "RGB";
static char source[64], vcam[64];
static double target_tilt;
static kinect_ir_tune ir_tune;
static int ir_ui_guard;
static int drag_orbit;
static double drag_x, drag_y;
static float orbit_e = 18.f, orbit_a = -60.f, orbit_d = 3.2f;

static int is_cloud(void) { return mode[0] == 'C'; }
static int is_ir(void) { return mode[0] == 'I'; }

static void set_status(const char *s) { gtk_label_set_text(GTK_LABEL(status_lbl), s); }

static void refresh_dev(void) {
  char src[64], vc[64], buf[360];
  if (kinect_find_source(src, sizeof src) == 0)
    snprintf(source, sizeof source, "%s", src);
  if (kinect_find_vcam(vc, sizeof vc) == 0)
    snprintf(vcam, sizeof vcam, "%s", vc);
  else
    vcam[0] = 0;
  if (is_cloud() && kinect_cloud_running())
    snprintf(buf, sizeof buf, "Cloud LIVE  n=%d  ·  drag preview to orbit  ·  src depth",
             kinect_cloud_points());
  else if (camera_on && kinect_bridge_running() && vcam[0])
    snprintf(buf, sizeof buf, "Apps → %s  “Kinect Webcam”  ·  %s  (src %s)", vcam, mode,
             source[0] ? source : "—");
  else if (vcam[0])
    snprintf(buf, sizeof buf, "Apps → %s  “Kinect Webcam”  (Camera OFF)  ·  src %s", vcam,
             source[0] ? source : "—");
  else
    snprintf(buf, sizeof buf, "Virtual cam not loaded · src %s · run setup_loopback.sh --load",
             source[0] ? source : "—");
  gtk_label_set_text(GTK_LABEL(dev_lbl), buf);
}

static void show_rgb(const uint8_t *rgb, int w, int h) {
  GdkPixbuf *pb, *sc;
  uint8_t *pix;
  int row;
  pb = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, w, h);
  if (!pb) return;
  pix = gdk_pixbuf_get_pixels(pb);
  row = gdk_pixbuf_get_rowstride(pb);
  if (row == w * 3)
    memcpy(pix, rgb, (size_t)w * (size_t)h * 3);
  else {
    int y;
    for (y = 0; y < h; y++) memcpy(pix + y * row, rgb + y * w * 3, (size_t)w * 3);
  }
  sc = gdk_pixbuf_scale_simple(pb, 460, 240, GDK_INTERP_NEAREST);
  gtk_image_set_from_pixbuf(GTK_IMAGE(preview_img), sc);
  g_object_unref(sc);
  g_object_unref(pb);
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

static gboolean on_bridge_ok(gpointer data) {
  char buf[192];
  (void)data;
  bridge_starting = 0;
  refresh_dev();
  if (is_cloud())
    snprintf(buf, sizeof buf,
             kinect_cloud_live() ? "Cloud LIVE · n=%d · drag preview to orbit"
                                 : "Cloud · waiting for live depth…",
             kinect_cloud_points());
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
  kinect_motor_set_led(LED_RED);
  set_status(buf);
  refresh_dev();
  g_free(data);
  return G_SOURCE_REMOVE;
}

static void *arm_work(void *arg) {
  const char *m = arg;
  int rc;
  if (m && m[0] == 'C') {
    kinect_bridge_stop();
    rc = kinect_cloud_start();
    if (rc == 0)
      g_idle_add(on_bridge_ok, NULL);
    else
      g_idle_add(on_bridge_fail, g_strdup(kinect_cloud_error()));
    return NULL;
  }
  kinect_cloud_stop();
  usleep(200000); /* let freenect drop the USB camera so gspca can reopen */
  kinect_bridge_set_ir_tune(&ir_tune);
  rc = kinect_bridge_start(m);
  if (rc == 0) {
    kinect_motor_set_led(m && m[0] == 'I' ? LED_GREEN : LED_BLINK_GREEN);
    g_idle_add(on_bridge_ok, NULL);
  } else
    g_idle_add(on_bridge_fail, g_strdup(kinect_bridge_error()));
  return NULL;
}

static void arm_camera(void) {
  pthread_t t;
  if (!is_cloud() && kinect_find_source(source, sizeof source) < 0) {
    set_status("No Kinect video node. Plug Kinect.");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_cam), FALSE);
    camera_on = 0;
    return;
  }
  set_status(is_cloud() ? "Starting depth cloud…" : "Starting webcam…");
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
    kinect_bridge_stop();
    kinect_cloud_stop();
    kinect_motor_set_led(LED_OFF);
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

static void on_mode(GtkToggleButton *b, gpointer u) {
  const char *m = u;
  if (!gtk_toggle_button_get_active(b)) return;
  snprintf(mode, sizeof mode, "%s", m);
  if (ir_frame) gtk_widget_set_sensitive(ir_frame, is_ir());
  if (cloud_frame) gtk_widget_set_sensitive(cloud_frame, is_cloud());
  /* Stay on Camera — Image tab is opt-in so tilt never disappears. */
  if (is_cloud()) {
    /* Python viewer started on launch — do not wait for Camera ON. */
    set_status("Cloud — capturing depth into this preview…");
    if (!preview_on)
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_preview), TRUE);
    if (!kinect_cloud_running() && !bridge_starting) {
      pthread_t t;
      bridge_starting = 1;
      if (pthread_create(&t, NULL, arm_work, (gpointer) "CLOUD") == 0)
        pthread_detach(t);
      else
        bridge_starting = 0;
    } else
      snap_async();
    return;
  }
  kinect_cloud_stop();
  if (camera_on)
    arm_camera();
  if (!preview_on)
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(btn_preview), TRUE);
  else
    snap_async();
  if (!camera_on) {
    if (is_ir())
      set_status("Mode IR · grade sliders live · Camera ON publishes webcam");
    else
      set_status("Mode RGB · Camera ON publishes “Kinect Webcam”");
  }
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

static int live_ok(void) {
  if (is_cloud()) return kinect_cloud_running();
  return kinect_bridge_running();
}

static gboolean preview_tick(gpointer u) {
  (void)u;
  if (closing) return FALSE;
  if (is_cloud()) {
    uint8_t *rgb = (uint8_t *)malloc((size_t)KINECT_RGB_W * KINECT_RGB_H * 3);
    if (rgb && kinect_cloud_preview_rgb(rgb) == 0)
      show_rgb(rgb, KINECT_RGB_W, KINECT_RGB_H);
    free(rgb);
    if (kinect_cloud_live()) {
      char buf[192];
      snprintf(buf, sizeof buf, "Cloud LIVE · n=%d · drag preview to orbit",
               kinect_cloud_points());
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

static gboolean on_delete(GtkWidget *w, GdkEvent *e, gpointer u) {
  (void)w;
  (void)e;
  (void)u;
  gtk_widget_hide(win);
  return TRUE;
}

static void raise_panel(void) {
  gtk_widget_show_all(win);
  gtk_window_present(GTK_WINDOW(win));
  gtk_window_set_keep_above(GTK_WINDOW(win),
                            gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(keep_above)));
}

static gboolean on_sock(GIOChannel *ch, GIOCondition c, gpointer u) {
  int fd;
  (void)ch;
  (void)c;
  (void)u;
  fd = accept(sock_fd, NULL, NULL);
  if (fd >= 0) {
    char b[8];
    (void)read(fd, b, sizeof b);
    close(fd);
  }
  raise_panel();
  return TRUE;
}

static int acquire_instance(void) {
  struct sockaddr_un a;
  const char *rt = getenv("XDG_RUNTIME_DIR");
  GIOChannel *ch;
  snprintf(sock_path, sizeof sock_path, "%s/kinect-panel.sock", rt && rt[0] ? rt : "/tmp");
  sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock_fd < 0) return 1;
  memset(&a, 0, sizeof a);
  a.sun_family = AF_UNIX;
  snprintf(a.sun_path, sizeof a.sun_path, "%s", sock_path);
  if (connect(sock_fd, (struct sockaddr *)&a, sizeof a) == 0) {
    const char *msg = "RAISE\n";
    (void)write(sock_fd, msg, 6);
    close(sock_fd);
    sock_fd = -1;
    return 0;
  }
  close(sock_fd);
  unlink(sock_path);
  sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (bind(sock_fd, (struct sockaddr *)&a, sizeof a) < 0) return 1;
  listen(sock_fd, 3);
  ch = g_io_channel_unix_new(sock_fd);
  g_io_add_watch(ch, G_IO_IN, on_sock, NULL);
  g_io_channel_unref(ch);
  return 1;
}

static void cleanup(void) {
  closing = 1;
  kinect_bridge_stop();
  kinect_cloud_stop();
  kinect_motor_close();
  if (sock_fd >= 0) {
    close(sock_fd);
    unlink(sock_path);
    sock_fd = -1;
  }
}

static void on_destroy(GtkWidget *w, gpointer u) {
  (void)w;
  (void)u;
  cleanup();
  gtk_main_quit();
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
  if (kinect_motor_open() == 0 && kinect_motor_get_tilt(&deg, &st) == 0) {
    kinect_motor_set_led(LED_OFF);
    if (deg < TILT_MIN || deg > TILT_MAX) deg = 0;
    target_tilt = deg;
    gtk_range_set_value(GTK_RANGE(tilt_scale), deg);
    set_status("Ready. RGB / IR / Cloud → Camera ON. IR sliders boost vision.");
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

int main(int argc, char **argv) {
  GtkWidget *root, *frame, *box, *row, *cam_box, *tilt_f, *tilt, *foot, *ir_box;
  GtkWidget *btn_rgb, *btn_ir, *btn_cloud, *up, *dn, *lv, *hide, *snap, *tip;
  GtkWidget *see, *reset, *page_cam, *page_img, *scroll_img;
  GtkWidget *cloud_box, *cup, *room, *rview;

  gtk_init(&argc, &argv);
  if (!acquire_instance()) {
    fprintf(stderr, "Kinect panel already running — raised existing window.\n");
    return 0;
  }

  win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), "Kinect Panel");
  gtk_window_set_default_size(GTK_WINDOW(win), 500, 620);
  gtk_container_set_border_width(GTK_CONTAINER(win), 10);
  gtk_window_set_keep_above(GTK_WINDOW(win), TRUE);
  gtk_window_set_type_hint(GTK_WINDOW(win), GDK_WINDOW_TYPE_HINT_UTILITY);
  g_signal_connect(win, "delete-event", G_CALLBACK(on_delete), NULL);
  g_signal_connect(win, "destroy", G_CALLBACK(on_destroy), NULL);

  root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
  gtk_container_add(GTK_CONTAINER(win), root);

  status_lbl = gtk_label_new("Starting…");
  gtk_label_set_xalign(GTK_LABEL(status_lbl), 0);
  gtk_label_set_line_wrap(GTK_LABEL(status_lbl), TRUE);
  gtk_box_pack_start(GTK_BOX(root), status_lbl, FALSE, FALSE, 0);

  frame = gtk_frame_new("View  (RGB / IR / Cloud)");
  box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
  gtk_container_set_border_width(GTK_CONTAINER(box), 4);
  preview_ev = gtk_event_box_new();
  gtk_widget_add_events(preview_ev, GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK |
                                         GDK_POINTER_MOTION_MASK | GDK_SCROLL_MASK);
  preview_img = gtk_image_new();
  gtk_widget_set_size_request(preview_img, 460, 240);
  gtk_container_add(GTK_CONTAINER(preview_ev), preview_img);
  g_signal_connect(preview_ev, "button-press-event", G_CALLBACK(on_orbit_press), NULL);
  g_signal_connect(preview_ev, "button-release-event", G_CALLBACK(on_orbit_release), NULL);
  g_signal_connect(preview_ev, "motion-notify-event", G_CALLBACK(on_orbit_motion), NULL);
  g_signal_connect(preview_ev, "scroll-event", G_CALLBACK(on_orbit_scroll), NULL);
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
  page_img = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
  gtk_container_set_border_width(GTK_CONTAINER(page_cam), 6);
  gtk_container_set_border_width(GTK_CONTAINER(page_img), 6);
  gtk_box_pack_start(GTK_BOX(page_cam), frame, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(root), tabs, TRUE, TRUE, 0);

  cam_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
  btn_cam = gtk_toggle_button_new_with_label("Camera OFF");
  gtk_style_context_add_class(gtk_widget_get_style_context(btn_cam), "suggested-action");
  g_signal_connect(btn_cam, "toggled", G_CALLBACK(on_cam), NULL);
  gtk_box_pack_start(GTK_BOX(cam_box), btn_cam, TRUE, TRUE, 0);
  btn_rgb = gtk_radio_button_new_with_label(NULL, "RGB");
  btn_ir = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_rgb), "IR");
  btn_cloud = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(btn_rgb), "Cloud");
  g_signal_connect(btn_rgb, "toggled", G_CALLBACK(on_mode), (gpointer) "RGB");
  g_signal_connect(btn_ir, "toggled", G_CALLBACK(on_mode), (gpointer) "IR");
  g_signal_connect(btn_cloud, "toggled", G_CALLBACK(on_mode), (gpointer) "CLOUD");
  gtk_box_pack_start(GTK_BOX(cam_box), btn_rgb, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(cam_box), btn_ir, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(cam_box), btn_cloud, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page_cam), cam_box, FALSE, FALSE, 0);

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
    gtk_combo_box_set_active(GTK_COMBO_BOX(combo_color), 0);
    g_signal_connect(combo_color, "changed", G_CALLBACK(on_cloud_color), NULL);
    gtk_box_pack_start(GTK_BOX(crow), clab, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(crow), combo_color, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(cloud_box), crow, FALSE, FALSE, 0);
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
      "Image tab: IR grade or Cloud clip/density/color. Camera tab keeps tilt.\n"
      "Cloud: Near/Far clip metres. Stride = sparser. Drag preview to orbit.");
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
  gtk_notebook_append_page(GTK_NOTEBOOK(tabs), page_cam, gtk_label_new("Camera"));
  gtk_notebook_append_page(GTK_NOTEBOOK(tabs), scroll_img, gtk_label_new("Image"));

  foot = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
  keep_above = gtk_check_button_new_with_label("Always on top");
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(keep_above), TRUE);
  g_signal_connect(keep_above, "toggled", G_CALLBACK(on_keep), NULL);
  hide = gtk_button_new_with_label("Hide");
  g_signal_connect(hide, "clicked", G_CALLBACK(on_hide), NULL);
  gtk_box_pack_start(GTK_BOX(foot), keep_above, TRUE, TRUE, 0);
  gtk_box_pack_end(GTK_BOX(foot), hide, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(root), foot, FALSE, FALSE, 0);

  g_timeout_add(250, poll_tilt, NULL);
  g_timeout_add(200, preview_tick, NULL);
  g_idle_add(boot, NULL);
  gtk_widget_show_all(win);
  gtk_main();
  cleanup();
  return 0;
}
