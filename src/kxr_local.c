#define _POSIX_C_SOURCE 200809L
/* Local native view: Kinect cloud from the tracked head. No WiVRn, no stream. */
#include "xr/kinect_xr.h"
#include "xr/kinect_xr_gl.h"

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile int g_quit, g_flip, g_freeze, g_relock;

static void on_key(GLFWwindow *w, int key, int sc, int action, int mods) {
  (void)w;
  (void)sc;
  (void)mods;
  if (action != GLFW_PRESS) return;
  if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) g_quit = 1;
  else if (key == GLFW_KEY_F) g_flip = !g_flip;
  else if (key == GLFW_KEY_SPACE) g_freeze = !g_freeze;
  else if (key == GLFW_KEY_L) g_relock = 1;
}

static kxr_quat quat_look(kxr_vec3 eye, kxr_vec3 target) {
  kxr_vec3 f, z, x, y, up = {0, 1, 0};
  float ln, tr;
  kxr_quat q;
  f.x = target.x - eye.x;
  f.y = target.y - eye.y;
  f.z = target.z - eye.z;
  ln = sqrtf(f.x * f.x + f.y * f.y + f.z * f.z);
  if (ln < 1e-5f) {
    q.x = q.y = q.z = 0;
    q.w = 1;
    return q;
  }
  f.x /= ln;
  f.y /= ln;
  f.z /= ln;
  /* OpenXR camera looks down -Z, so camera +Z = -forward */
  z.x = -f.x;
  z.y = -f.y;
  z.z = -f.z;
  x.x = up.y * z.z - up.z * z.y;
  x.y = up.z * z.x - up.x * z.z;
  x.z = up.x * z.y - up.y * z.x;
  ln = sqrtf(x.x * x.x + x.y * x.y + x.z * x.z);
  if (ln < 1e-5f) {
    up.x = 0;
    up.y = 0;
    up.z = 1;
    x.x = up.y * z.z - up.z * z.y;
    x.y = up.z * z.x - up.x * z.z;
    x.z = up.x * z.y - up.y * z.x;
    ln = sqrtf(x.x * x.x + x.y * x.y + x.z * x.z);
    if (ln < 1e-5f) {
      q.x = q.y = q.z = 0;
      q.w = 1;
      return q;
    }
  }
  x.x /= ln;
  x.y /= ln;
  x.z /= ln;
  y.x = z.y * x.z - z.z * x.y;
  y.y = z.z * x.x - z.x * x.z;
  y.z = z.x * x.y - z.y * x.x;
  /* rotation matrix columns = camera axes; to quat */
  tr = x.x + y.y + z.z;
  if (tr > 0) {
    float s = sqrtf(tr + 1.f) * 2.f;
    q.w = 0.25f * s;
    q.x = (y.z - z.y) / s;
    q.y = (z.x - x.z) / s;
    q.z = (x.y - y.x) / s;
  } else if (x.x > y.y && x.x > z.z) {
    float s = sqrtf(1.f + x.x - y.y - z.z) * 2.f;
    q.w = (y.z - z.y) / s;
    q.x = 0.25f * s;
    q.y = (y.x + x.y) / s;
    q.z = (z.x + x.z) / s;
  } else if (y.y > z.z) {
    float s = sqrtf(1.f + y.y - x.x - z.z) * 2.f;
    q.w = (z.x - x.z) / s;
    q.x = (y.x + x.y) / s;
    q.y = 0.25f * s;
    q.z = (z.y + y.z) / s;
  } else {
    float s = sqrtf(1.f + z.z - x.x - y.y) * 2.f;
    q.w = (x.y - y.x) / s;
    q.x = (z.x + x.z) / s;
    q.y = (z.y + y.z) / s;
    q.z = 0.25f * s;
  }
  return q;
}

static void title_of(GLFWwindow *win, kxr_session *s, const kxr_head *h, int live) {
  char t[192];
  snprintf(t, sizeof t, "Kinect local  n=%d  track=%s conf=%.2f  %s  [F]lip [L]ock [Q]uit",
           kxr_points(s), h->ok ? "head" : "none", h->conf, live ? "live" : "xyz");
  glfwSetWindowTitle(win, t);
}

int main(int argc, char **argv) {
  const char *xyz = "/home/voldemar/Dev/lab/prophecy_cube/state/trackers/kinect_latest.xyz";
  const char *src_name = NULL;
  int live = 0, i;
  kxr_session *kx;
  kxr_gl *gl;
  GLFWwindow *win;
  float *verts;
  const int vmax = CUBE_CLOUD_MAX;
  kxr_head last;
  kxr_pose head;
  kxr_fov fov = {-0.85f, 0.85f, 0.80f, -0.80f};

  memset(&last, 0, sizeof last);
  kxr_pose_identity(&head);
  head.p.y = 1.6f;

  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--xyz") == 0 && i + 1 < argc)
      xyz = argv[++i];
    else if (strcmp(argv[i], "--live") == 0)
      live = 1;
    else if (strcmp(argv[i], "--src") == 0 && i + 1 < argc)
      src_name = argv[++i];
    else if (strncmp(argv[i], "--src=", 6) == 0)
      src_name = argv[i] + 6;
    else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      fprintf(stderr,
              "kxr_local — Kinect cloud from tracked head. No VR stream.\n"
              "  --live       own the depth USB (Kinect, Kinect 2, or both)\n"
              "  --src auto|v1|v2|both\n"
              "  --xyz PATH   published cloud (default: panel latest)\n"
              "  F flip look  L relock  Space freeze  Q quit\n");
      return 0;
    }
  }

  kx = kxr_create();
  if (!kx) return 1;
  if (src_name) kxr_set_source(kx, src_name);
  kxr_set_hide_radius(kx, 0.22f);
  if (live) {
    if (kxr_live_start(kx) != 0) {
      fprintf(stderr, "live failed (%s) — xyz fallback\n", kxr_error(kx));
      live = 0;
    }
  }
  if (kxr_points(kx) < 1 && xyz && kxr_load_xyz(kx, xyz) != 0)
    fprintf(stderr, "xyz: %s\n", kxr_error(kx));

  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  win = glfwCreateWindow(1280, 720, "Kinect local", NULL, NULL);
  if (!win) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSetKeyCallback(win, on_key);
  glfwSwapInterval(1);
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK && !glCreateShader) {
    fprintf(stderr, "glewInit\n");
    return 1;
  }
  (void)glGetError();

  gl = kxr_gl_create();
  verts = (float *)malloc((size_t)vmax * 6 * sizeof(float));
  if (!gl || !verts) return 1;

  while (!g_quit && !glfwWindowShouldClose(win)) {
    kxr_head blob;
    kxr_view view;
    kxr_vec3 mapped, target;
    float mvp[16];
    int nvert = 0, ww, wh;

    glfwPollEvents();
    if (live) kxr_live_poll(kx);

    if (!g_freeze && kxr_track_head(kx, &blob) == 0 && blob.ok) {
      last = blob;
      kxr_map_kinect(blob.p_kinect, &mapped);
      /* Kinect sits at XR origin after identity rig. Head is in the room. */
      head.p = mapped;
      /* Face the sensor (horizontal). F looks into the room instead. */
      target.x = 0;
      target.y = mapped.y;
      target.z = g_flip ? mapped.z - 1.f : 0.f;
      if (g_flip && fabsf(mapped.z) < 0.05f) target.z = -1.f;
      head.q = quat_look(head.p, target);
      kxr_set_head(kx, &head);
      if (g_relock) {
        kxr_rig id;
        kxr_rig_identity(&id);
        id.locked = 1;
        kxr_set_rig(kx, &id);
        g_relock = 0;
      }
    }

    view.pose = head;
    view.fov = fov;
    kxr_set_views(kx, &view, 1);
    if (verts && kxr_fill_vertices(kx, verts, vmax, &nvert) == 0)
      kxr_gl_upload(gl, verts, nvert);

    glfwGetFramebufferSize(win, &ww, &wh);
    glViewport(0, 0, ww, wh);
    glClearColor(0.02f, 0.0f, 0.03f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    kxr_mvp(kx, 0, 0.08f, 8.f, mvp);
    kxr_gl_draw(gl, mvp, 0.016f, (float)wh);
    glfwSwapBuffers(win);
    title_of(win, kx, &last, live);
  }

  free(verts);
  kxr_gl_destroy(gl);
  kxr_destroy(kx);
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
