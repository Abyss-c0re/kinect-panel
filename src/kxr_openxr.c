#define _POSIX_C_SOURCE 200809L
#define XR_USE_GRAPHICS_API_OPENGL
#define XR_USE_PLATFORM_XLIB
#include <X11/Xlib.h>
#include <GL/glew.h>
#include <GL/glx.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_GLX
#include <GLFW/glfw3native.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include "xr/kinect_xr.h"
#include "xr/kinect_xr_gl.h"
#include "xr/kxr_menu.h"
#include "cube_kinect_mesh.h"
#include "kinect_fuse.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_V 4
#define MAX_IMG 16

static void die(const char *m) {
  fprintf(stderr, "kxr_openxr: %s\n", m);
  exit(1);
}

static const char *xrerr(XrInstance inst, XrResult r) {
  static char buf[128];
  if (inst && XR_SUCCEEDED(xrResultToString(inst, r, buf))) return buf;
  snprintf(buf, sizeof buf, "XrResult(%d)", (int)r);
  return buf;
}

typedef struct {
  XrSwapchain handle;
  uint32_t w, h, n;
  XrSwapchainImageOpenGLKHR img[MAX_IMG];
  GLuint fbo[MAX_IMG];
  GLuint depth[MAX_IMG];
} kxr_swap;

static int64_t pick_format(XrSession sess) {
  int64_t fmts[32];
  uint32_t n = 0, i;
  if (XR_FAILED(xrEnumerateSwapchainFormats(sess, 32, &n, fmts)) || n < 1)
    return (int64_t)GL_RGBA8;
  for (i = 0; i < n; i++)
    if (fmts[i] == (int64_t)GL_RGBA8) return fmts[i];
  for (i = 0; i < n; i++)
    if (fmts[i] == (int64_t)GL_SRGB8_ALPHA8) return fmts[i];
  return fmts[0];
}

static int swap_init(XrSession sess, const XrViewConfigurationView *vc, kxr_swap *s) {
  XrSwapchainCreateInfo ci = {.type = XR_TYPE_SWAPCHAIN_CREATE_INFO};
  XrSwapchainImageBaseHeader *base;
  uint32_t n = 0, i;
  XrResult r;
  memset(s, 0, sizeof *s);
  /* CubeUI: RGBA8 + sampleCount 1. MSAA / sRGB here made WiVRn xrEndFrame SEGV. */
  ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
  ci.format = pick_format(sess);
  ci.sampleCount = 1;
  ci.width = vc->recommendedImageRectWidth;
  ci.height = vc->recommendedImageRectHeight;
  ci.faceCount = 1;
  ci.arraySize = 1;
  ci.mipCount = 1;
  r = xrCreateSwapchain(sess, &ci, &s->handle);
  if (XR_FAILED(r)) return -1;
  s->w = ci.width;
  s->h = ci.height;
  xrEnumerateSwapchainImages(s->handle, 0, &n, NULL);
  if (n > MAX_IMG) n = MAX_IMG;
  s->n = n;
  for (i = 0; i < n; i++) s->img[i].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
  base = (XrSwapchainImageBaseHeader *)s->img;
  if (XR_FAILED(xrEnumerateSwapchainImages(s->handle, n, &n, base))) return -1;
  s->n = n;
  for (i = 0; i < n; i++) {
    glGenFramebuffers(1, &s->fbo[i]);
    glGenRenderbuffers(1, &s->depth[i]);
    glBindRenderbuffer(GL_RENDERBUFFER, s->depth[i]);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, (GLsizei)s->w,
                          (GLsizei)s->h);
    glBindFramebuffer(GL_FRAMEBUFFER, s->fbo[i]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           s->img[i].image, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, s->depth[i]);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      return -1;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  return 0;
}

static void swap_free(kxr_swap *s) {
  uint32_t i;
  if (!s) return;
  for (i = 0; i < s->n; i++) {
    if (s->fbo[i]) glDeleteFramebuffers(1, &s->fbo[i]);
    if (s->depth[i]) glDeleteRenderbuffers(1, &s->depth[i]);
  }
  if (s->handle) xrDestroySwapchain(s->handle);
  memset(s, 0, sizeof *s);
}

static volatile int g_lock, g_sample, g_solve, g_yaw, g_quit;
static volatile int g_scale_up, g_scale_dn, g_scale_rst, g_menu;
static volatile int g_toggle_solid, g_toggle_map;

static void on_key(GLFWwindow *w, int key, int sc, int action, int mods) {
  (void)w;
  (void)sc;
  (void)mods;
  if (action != GLFW_PRESS) return;
  if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) g_quit = 1;
  else if (key == GLFW_KEY_L) g_lock = 1;
  else if (key == GLFW_KEY_S) g_sample = 1;
  else if (key == GLFW_KEY_R) g_solve = 1;
  else if (key == GLFW_KEY_Y) g_yaw = 1;
  else if (key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD) g_scale_up = 1;
  else if (key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT) g_scale_dn = 1;
  else if (key == GLFW_KEY_0) g_scale_rst = 1;
  else if (key == GLFW_KEY_M) g_menu = 1;
  else if (key == GLFW_KEY_G) g_toggle_solid = 1;
  else if (key == GLFW_KEY_TAB) g_toggle_map = 1;
}

static void usage(void) {
  fprintf(stderr,
          "kxr_openxr — Kinect cloud in the player's OpenXR view\n"
          "  --live         own the depth USB and start the cloud (default)\n"
          "  --no-live      do not open the camera; only watch --xyz\n"
          "  --src auto|v1|v2|both   which depth camera (default auto / KINECT_SRC)\n"
          "  --xyz PATH     fallback / watch this published cloud\n"
          "  --place X Y Z YAW PITCH   sensor pose in XR metres/degrees\n"
          "  --no-autolock  do not lock the human head to the HMD pose\n"
          "  --stage        prefer STAGE space (else LOCAL)\n"
          "\n"
          "Keys: L lock  +/- scale  0 reset scale  S sample  R solve  Y yaw  Q quit\n"
          "Controllers:\n"
          "  both grips  twist = rotate cloud, pinch/spread = zoom (about the grab)\n"
          "  one grip    drag around you = orbit yaw/pitch\n"
          "  left stick  move   right stick  look (camera)\n"
          "  A / X       lock head   B sample   Y solve\n"
          "  right menu  panel   G solid 3D   Tab map-fly   M menu\n"
          "Runtime: XR_RUNTIME_JSON=/usr/share/openxr/1/openxr_wivrn.json\n");
}

/* ---- OpenXR controller actions ----------------------------------------- */

typedef struct {
  XrActionSet set;
  XrAction grip_pose;
  XrAction squeeze;
  XrAction squeeze_click;
  XrAction lock;
  XrAction sample;
  XrAction solve;
  XrAction yaw;
  XrAction panel;
  XrAction stick_click;
  XrAction trigger;
  XrAction stick;
  XrAction aim_pose;
  XrPath hand[2];
  XrSpace grip_space[2];
  XrSpace aim_space[2];
  int ready;
} kxr_xrinput;

typedef struct {
  int active;
  int dual;
  kxr_rig rig;
  float scale;
  kxr_vec3 left, right, mid;
  float dist, yaw, pitch;
} kxr_grab;

static XrPath xp(XrInstance inst, const char *s) {
  XrPath p = XR_NULL_PATH;
  if (!s || XR_FAILED(xrStringToPath(inst, s, &p))) return XR_NULL_PATH;
  return p;
}

static void bind_one(XrActionSuggestedBinding *b, uint32_t *n, uint32_t max, XrAction act,
                     XrInstance inst, const char *path) {
  XrPath p;
  if (!act || *n >= max) return;
  p = xp(inst, path);
  if (!p) return;
  b[*n].action = act;
  b[*n].binding = p;
  (*n)++;
}

static void suggest_profile(XrInstance inst, const char *profile,
                            XrActionSuggestedBinding *b, uint32_t n) {
  XrInteractionProfileSuggestedBinding sb = {
      .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
  if (n < 1) return;
  sb.interactionProfile = xp(inst, profile);
  if (!sb.interactionProfile) return;
  sb.suggestedBindings = b;
  sb.countSuggestedBindings = n;
  if (XR_FAILED(xrSuggestInteractionProfileBindings(inst, &sb)))
    fprintf(stderr, "kxr_openxr: skip bindings %s\n", profile);
}

static int make_action(XrActionSet set, XrActionType ty, const char *name, const char *loc,
                       const XrPath *sub, uint32_t nsub, XrAction *out) {
  XrActionCreateInfo ci = {.type = XR_TYPE_ACTION_CREATE_INFO};
  ci.actionType = ty;
  ci.countSubactionPaths = nsub;
  ci.subactionPaths = sub;
  snprintf(ci.actionName, sizeof ci.actionName, "%s", name);
  snprintf(ci.localizedActionName, sizeof ci.localizedActionName, "%s", loc);
  return XR_FAILED(xrCreateAction(set, &ci, out)) ? -1 : 0;
}

static int input_init(kxr_xrinput *in, XrInstance inst, XrSession sess) {
  XrActionSetCreateInfo sci = {.type = XR_TYPE_ACTION_SET_CREATE_INFO};
  XrSessionActionSetsAttachInfo ai = {.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
  XrActionSuggestedBinding b[32];
  uint32_t n;
  int i;
  memset(in, 0, sizeof *in);
  in->hand[0] = xp(inst, "/user/hand/left");
  in->hand[1] = xp(inst, "/user/hand/right");
  if (!in->hand[0] || !in->hand[1]) return -1;

  snprintf(sci.actionSetName, sizeof sci.actionSetName, "kxr");
  snprintf(sci.localizedActionSetName, sizeof sci.localizedActionSetName, "Kinect XR");
  sci.priority = 0;
  if (XR_FAILED(xrCreateActionSet(inst, &sci, &in->set))) return -1;

  if (make_action(in->set, XR_ACTION_TYPE_POSE_INPUT, "grip_pose", "Grip Pose", in->hand, 2,
                  &in->grip_pose) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_FLOAT_INPUT, "squeeze", "Squeeze", in->hand, 2,
                  &in->squeeze) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "squeeze_click", "Squeeze Click",
                  in->hand, 2, &in->squeeze_click) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "lock", "Lock Head", NULL, 0,
                  &in->lock) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "sample", "Lock Sample", NULL, 0,
                  &in->sample) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "solve", "Lock Solve", NULL, 0,
                  &in->solve) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "yaw", "Yaw From Walk", NULL, 0,
                  &in->yaw) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "panel", "Open Panel", NULL, 0,
                  &in->panel) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_BOOLEAN_INPUT, "stick_click", "Stick Click",
                  in->hand, 2, &in->stick_click) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_FLOAT_INPUT, "trigger", "Trigger", in->hand, 2,
                  &in->trigger) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_VECTOR2F_INPUT, "stick", "Thumbstick", in->hand, 2,
                  &in->stick) != 0)
    return -1;
  if (make_action(in->set, XR_ACTION_TYPE_POSE_INPUT, "aim_pose", "Aim Pose", in->hand, 2,
                  &in->aim_pose) != 0)
    return -1;

  /* khr simple — no squeeze; select = lock, menu = sample */
  n = 0;
  bind_one(b, &n, 32, in->grip_pose, inst, "/user/hand/left/input/grip/pose");
  bind_one(b, &n, 32, in->grip_pose, inst, "/user/hand/right/input/grip/pose");
  bind_one(b, &n, 32, in->aim_pose, inst, "/user/hand/left/input/aim/pose");
  bind_one(b, &n, 32, in->aim_pose, inst, "/user/hand/right/input/aim/pose");
  bind_one(b, &n, 32, in->lock, inst, "/user/hand/left/input/select/click");
  bind_one(b, &n, 32, in->lock, inst, "/user/hand/right/input/select/click");
  bind_one(b, &n, 32, in->panel, inst, "/user/hand/right/input/menu/click");
  suggest_profile(inst, "/interaction_profiles/khr/simple_controller", b, n);

  /* Quest / Meta Touch (WiVRn) */
  n = 0;
  bind_one(b, &n, 32, in->grip_pose, inst, "/user/hand/left/input/grip/pose");
  bind_one(b, &n, 32, in->grip_pose, inst, "/user/hand/right/input/grip/pose");
  bind_one(b, &n, 32, in->aim_pose, inst, "/user/hand/left/input/aim/pose");
  bind_one(b, &n, 32, in->aim_pose, inst, "/user/hand/right/input/aim/pose");
  bind_one(b, &n, 32, in->squeeze, inst, "/user/hand/left/input/squeeze/value");
  bind_one(b, &n, 32, in->squeeze, inst, "/user/hand/right/input/squeeze/value");
  bind_one(b, &n, 32, in->trigger, inst, "/user/hand/left/input/trigger/value");
  bind_one(b, &n, 32, in->trigger, inst, "/user/hand/right/input/trigger/value");
  bind_one(b, &n, 32, in->lock, inst, "/user/hand/left/input/x/click");
  bind_one(b, &n, 32, in->lock, inst, "/user/hand/right/input/a/click");
  bind_one(b, &n, 32, in->sample, inst, "/user/hand/right/input/b/click");
  bind_one(b, &n, 32, in->solve, inst, "/user/hand/left/input/y/click");
  /* Quest Touch has no right menu path — left ☰ is the app menu button. */
  bind_one(b, &n, 32, in->panel, inst, "/user/hand/left/input/menu/click");
  bind_one(b, &n, 32, in->stick_click, inst, "/user/hand/left/input/thumbstick/click");
  bind_one(b, &n, 32, in->stick_click, inst, "/user/hand/right/input/thumbstick/click");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/left/input/thumbstick");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/right/input/thumbstick");
  suggest_profile(inst, "/interaction_profiles/oculus/touch_controller", b, n);

  /* Valve Index */
  n = 0;
  bind_one(b, &n, 24, in->grip_pose, inst, "/user/hand/left/input/grip/pose");
  bind_one(b, &n, 24, in->grip_pose, inst, "/user/hand/right/input/grip/pose");
  bind_one(b, &n, 24, in->squeeze, inst, "/user/hand/left/input/squeeze/value");
  bind_one(b, &n, 24, in->squeeze, inst, "/user/hand/right/input/squeeze/value");
  bind_one(b, &n, 24, in->lock, inst, "/user/hand/left/input/a/click");
  bind_one(b, &n, 24, in->lock, inst, "/user/hand/right/input/a/click");
  bind_one(b, &n, 24, in->sample, inst, "/user/hand/right/input/b/click");
  bind_one(b, &n, 24, in->solve, inst, "/user/hand/left/input/b/click");
  bind_one(b, &n, 32, in->stick_click, inst, "/user/hand/left/input/thumbstick/click");
  bind_one(b, &n, 32, in->stick_click, inst, "/user/hand/right/input/thumbstick/click");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/left/input/thumbstick");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/right/input/thumbstick");
  suggest_profile(inst, "/interaction_profiles/valve/index_controller", b, n);

  /* Vive wand */
  n = 0;
  bind_one(b, &n, 24, in->grip_pose, inst, "/user/hand/left/input/grip/pose");
  bind_one(b, &n, 24, in->grip_pose, inst, "/user/hand/right/input/grip/pose");
  bind_one(b, &n, 24, in->squeeze_click, inst, "/user/hand/left/input/squeeze/click");
  bind_one(b, &n, 24, in->squeeze_click, inst, "/user/hand/right/input/squeeze/click");
  bind_one(b, &n, 24, in->lock, inst, "/user/hand/left/input/trigger/click");
  bind_one(b, &n, 24, in->lock, inst, "/user/hand/right/input/trigger/click");
  bind_one(b, &n, 24, in->sample, inst, "/user/hand/right/input/trackpad/click");
  bind_one(b, &n, 24, in->solve, inst, "/user/hand/left/input/trackpad/click");
  bind_one(b, &n, 24, in->panel, inst, "/user/hand/right/input/menu/click");
  suggest_profile(inst, "/interaction_profiles/htc/vive_controller", b, n);

  /* WMR */
  n = 0;
  bind_one(b, &n, 24, in->grip_pose, inst, "/user/hand/left/input/grip/pose");
  bind_one(b, &n, 24, in->grip_pose, inst, "/user/hand/right/input/grip/pose");
  bind_one(b, &n, 24, in->squeeze_click, inst, "/user/hand/left/input/squeeze/click");
  bind_one(b, &n, 24, in->squeeze_click, inst, "/user/hand/right/input/squeeze/click");
  bind_one(b, &n, 24, in->lock, inst, "/user/hand/left/input/trigger/click");
  bind_one(b, &n, 24, in->lock, inst, "/user/hand/right/input/trigger/click");
  bind_one(b, &n, 24, in->panel, inst, "/user/hand/right/input/menu/click");
  bind_one(b, &n, 32, in->stick_click, inst, "/user/hand/left/input/thumbstick/click");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/left/input/thumbstick");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/right/input/thumbstick");
  suggest_profile(inst, "/interaction_profiles/microsoft/motion_controller", b, n);

  /* Pico 4 / Neo 3 — right menu is the app button (WiVRn). */
  n = 0;
  bind_one(b, &n, 32, in->grip_pose, inst, "/user/hand/left/input/grip/pose");
  bind_one(b, &n, 32, in->grip_pose, inst, "/user/hand/right/input/grip/pose");
  bind_one(b, &n, 32, in->aim_pose, inst, "/user/hand/left/input/aim/pose");
  bind_one(b, &n, 32, in->aim_pose, inst, "/user/hand/right/input/aim/pose");
  bind_one(b, &n, 32, in->squeeze, inst, "/user/hand/left/input/squeeze/value");
  bind_one(b, &n, 32, in->squeeze, inst, "/user/hand/right/input/squeeze/value");
  bind_one(b, &n, 32, in->trigger, inst, "/user/hand/left/input/trigger/value");
  bind_one(b, &n, 32, in->trigger, inst, "/user/hand/right/input/trigger/value");
  bind_one(b, &n, 32, in->lock, inst, "/user/hand/right/input/a/click");
  bind_one(b, &n, 32, in->sample, inst, "/user/hand/right/input/b/click");
  bind_one(b, &n, 32, in->panel, inst, "/user/hand/right/input/menu/click");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/left/input/thumbstick");
  bind_one(b, &n, 32, in->stick, inst, "/user/hand/right/input/thumbstick");
  suggest_profile(inst, "/interaction_profiles/bytedance/pico4_controller", b, n);
  suggest_profile(inst, "/interaction_profiles/pico/neo3_controller", b, n);

  for (i = 0; i < 2; i++) {
    XrActionSpaceCreateInfo asci = {.type = XR_TYPE_ACTION_SPACE_CREATE_INFO};
    asci.action = in->grip_pose;
    asci.subactionPath = in->hand[i];
    asci.poseInActionSpace.orientation.w = 1;
    if (XR_FAILED(xrCreateActionSpace(sess, &asci, &in->grip_space[i])))
      return -1;
    asci.action = in->aim_pose;
    if (XR_FAILED(xrCreateActionSpace(sess, &asci, &in->aim_space[i])))
      in->aim_space[i] = XR_NULL_HANDLE;
  }

  ai.countActionSets = 1;
  ai.actionSets = &in->set;
  if (XR_FAILED(xrAttachSessionActionSets(sess, &ai))) {
    fprintf(stderr, "kxr_openxr: xrAttachSessionActionSets failed\n");
    return -1;
  }
  in->ready = 1;
  return 0;
}

static void input_free(kxr_xrinput *in) {
  int i;
  if (!in) return;
  for (i = 0; i < 2; i++) {
    if (in->grip_space[i]) xrDestroySpace(in->grip_space[i]);
    if (in->aim_space[i]) xrDestroySpace(in->aim_space[i]);
    in->grip_space[i] = XR_NULL_HANDLE;
    in->aim_space[i] = XR_NULL_HANDLE;
  }
  if (in->set) xrDestroyActionSet(in->set);
  memset(in, 0, sizeof *in);
}

static int input_sync(kxr_xrinput *in, XrSession sess) {
  XrActiveActionSet aas;
  XrActionsSyncInfo si = {.type = XR_TYPE_ACTIONS_SYNC_INFO};
  if (!in || !in->ready) return -1;
  aas.actionSet = in->set;
  aas.subactionPath = XR_NULL_PATH;
  si.countActiveActionSets = 1;
  si.activeActionSets = &aas;
  return XR_FAILED(xrSyncActions(sess, &si)) ? -1 : 0;
}

static int bool_edge(XrSession sess, XrAction act) {
  XrActionStateBoolean st = {.type = XR_TYPE_ACTION_STATE_BOOLEAN};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO, .action = act};
  if (!act || XR_FAILED(xrGetActionStateBoolean(sess, &gi, &st))) return 0;
  return st.isActive && st.currentState && st.changedSinceLastSync;
}

static int bool_down_hand(kxr_xrinput *in, XrSession sess, XrAction act, int hand) {
  XrActionStateBoolean st = {.type = XR_TYPE_ACTION_STATE_BOOLEAN};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
  gi.action = act;
  gi.subactionPath = in->hand[hand];
  if (!act || XR_FAILED(xrGetActionStateBoolean(sess, &gi, &st))) return 0;
  return st.isActive && st.currentState;
}

static int bool_edge_hand(kxr_xrinput *in, XrSession sess, XrAction act, int hand) {
  XrActionStateBoolean st = {.type = XR_TYPE_ACTION_STATE_BOOLEAN};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
  gi.action = act;
  gi.subactionPath = in->hand[hand];
  if (!act || XR_FAILED(xrGetActionStateBoolean(sess, &gi, &st))) return 0;
  return st.isActive && st.currentState && st.changedSinceLastSync;
}

static int trigger_edge(kxr_xrinput *in, XrSession sess, int *was) {
  int hand, down = 0, edge;
  for (hand = 0; hand < 2; hand++) {
    XrActionStateFloat st = {.type = XR_TYPE_ACTION_STATE_FLOAT};
    XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
    gi.action = in->trigger;
    gi.subactionPath = in->hand[hand];
    if (XR_SUCCEEDED(xrGetActionStateFloat(sess, &gi, &st)) && st.isActive &&
        st.currentState >= 0.70f)
      down = 1;
  }
  edge = down && !*was;
  *was = down;
  return edge;
}

static int locate_pose(kxr_xrinput *in, XrSession sess, XrSpace local, XrTime t, int hand,
                       int aim, kxr_pose *out) {
  XrAction act = aim ? in->aim_pose : in->grip_pose;
  XrSpace sp = aim ? in->aim_space[hand] : in->grip_space[hand];
  XrActionStatePose ps = {.type = XR_TYPE_ACTION_STATE_POSE};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
  XrSpaceLocation loc = {.type = XR_TYPE_SPACE_LOCATION};
  gi.action = act;
  gi.subactionPath = in->hand[hand];
  if (!sp || XR_FAILED(xrGetActionStatePose(sess, &gi, &ps)) || !ps.isActive) return 0;
  if (XR_FAILED(xrLocateSpace(sp, local, t, &loc))) return 0;
  if (!(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)) return 0;
  *out = kxr_pose_from_xr(&loc.pose);
  return 1;
}

static kxr_quat qmul(kxr_quat a, kxr_quat b) {
  kxr_quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

static kxr_vec3 xfm_pt(const float m[16], kxr_vec3 p) {
  kxr_vec3 o;
  o.x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
  o.y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
  o.z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
  return o;
}

static kxr_vec3 xfm_dir(const float m[16], kxr_vec3 p) {
  kxr_vec3 o;
  o.x = m[0] * p.x + m[4] * p.y + m[8] * p.z;
  o.y = m[1] * p.x + m[5] * p.y + m[9] * p.z;
  o.z = m[2] * p.x + m[6] * p.y + m[10] * p.z;
  return o;
}

static kxr_vec3 qrot3(kxr_quat q, kxr_vec3 v) {
  kxr_vec3 t, u, o;
  float n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n > 1e-8f) {
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
  }
  t.x = 2.f * (q.y * v.z - q.z * v.y);
  t.y = 2.f * (q.z * v.x - q.x * v.z);
  t.z = 2.f * (q.x * v.y - q.y * v.x);
  u.x = q.y * t.z - q.z * t.y;
  u.y = q.z * t.x - q.x * t.z;
  u.z = q.x * t.y - q.y * t.x;
  o.x = v.x + q.w * t.x + u.x;
  o.y = v.y + q.w * t.y + u.y;
  o.z = v.z + q.w * t.z + u.z;
  return o;
}

static void pose_model(const kxr_pose *p, float m[16]) {
  kxr_vec3 x = {1, 0, 0}, y = {0, 1, 0}, z = {0, 0, 1};
  memset(m, 0, 16 * sizeof(float));
  x = qrot3(p->q, x);
  y = qrot3(p->q, y);
  z = qrot3(p->q, z);
  m[0] = x.x;
  m[1] = x.y;
  m[2] = x.z;
  m[4] = y.x;
  m[5] = y.y;
  m[6] = y.z;
  m[8] = z.x;
  m[9] = z.y;
  m[10] = z.z;
  m[12] = p->p.x;
  m[13] = p->p.y;
  m[14] = p->p.z;
  m[15] = 1.f;
}

static void apply_menu_act(kxr_menu_act a, kxr_session *kx, kxr_menu *menu, int *autolock,
                           float *tilt, const kxr_pose *head, int *solid, int *map) {
  cube_cloud_tune t;
  if (!kx || !menu || a == KXR_ACT_NONE) return;
  kxr_get_tune(kx, &t);
  switch (a) {
  case KXR_ACT_NEAR_N:
    t.z_near -= 0.05f;
    if (t.z_near < 0.35f) t.z_near = 0.35f;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_NEAR_P:
    t.z_near += 0.05f;
    if (t.z_near > 2.f) t.z_near = 2.f;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_FAR_N:
    t.z_far -= 0.10f;
    if (t.z_far < 1.f) t.z_far = 1.f;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_FAR_P:
    t.z_far += 0.10f;
    if (t.z_far > 5.f) t.z_far = 5.f;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_STRIDE_N:
    if (t.stride > 2) t.stride--;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_STRIDE_P:
    if (t.stride < 8) t.stride++;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_DOT_N:
    if (t.dot > 1) t.dot--;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_DOT_P:
    if (t.dot < 4) t.dot++;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_COLOR:
    t.color = (t.color + 1) % 4;
    kxr_set_tune(kx, &t);
    break;
  case KXR_ACT_TILT_N:
    *tilt = kxr_get_tilt(kx) - 5.f;
    if (*tilt < -30.f) *tilt = -30.f;
    kxr_set_tilt(kx, (double)*tilt);
    break;
  case KXR_ACT_TILT_P:
    *tilt = kxr_get_tilt(kx) + 5.f;
    if (*tilt > 30.f) *tilt = 30.f;
    kxr_set_tilt(kx, (double)*tilt);
    break;
  case KXR_ACT_SCALE_N:
    kxr_set_scale(kx, kxr_get_scale(kx) / 1.08f);
    break;
  case KXR_ACT_SCALE_P:
    kxr_set_scale(kx, kxr_get_scale(kx) * 1.08f);
    break;
  case KXR_ACT_HIDE_N:
    kxr_set_hide_radius(kx, kxr_get_hide_radius(kx) - 0.04f);
    break;
  case KXR_ACT_HIDE_P:
    kxr_set_hide_radius(kx, kxr_get_hide_radius(kx) + 0.04f);
    break;
  case KXR_ACT_AUTOLOCK:
    *autolock = !*autolock;
    break;
  case KXR_ACT_LOCK:
    kxr_lock_head(kx);
    break;
  case KXR_ACT_SAMPLE:
    kxr_lock_sample(kx);
    break;
  case KXR_ACT_SOLVE:
    kxr_lock_solve(kx);
    break;
  case KXR_ACT_YAW:
    kxr_lock_yaw_walk(kx);
    break;
  case KXR_ACT_RESET:
    kxr_lock_reset(kx);
    kxr_set_scale(kx, 1.f);
    break;
  case KXR_ACT_SOLID:
    if (solid) {
      *solid = !*solid;
      if (*solid) {
        t.color = 3;
        kxr_set_tune(kx, &t);
      }
    }
    break;
  case KXR_ACT_MAP:
    if (map) *map = !*map;
    break;
  case KXR_ACT_SRC_N:
  case KXR_ACT_SRC_P: {
    static const char *tok[] = {"auto", "v1", "v2", "both"};
    int i = (int)kinect_fuse_source();
    if (i < 0 || i > 3) i = 0;
    i = (a == KXR_ACT_SRC_N) ? (i + 3) % 4 : (i + 1) % 4;
    kxr_set_source(kx, tok[i]);
    break;
  }
  case KXR_ACT_ALIGN_V2:
    kxr_align_v2(kx);
    break;
  case KXR_ACT_ALIGN_RESET:
    kxr_align_reset(kx);
    break;
  case KXR_ACT_CLOSE:
    kxr_menu_close(menu);
    break;
  default:
    break;
  }
  menu->dirty = 1;
}

static void fill_menu_snap(kxr_menu_snap *sn, kxr_session *kx, int autolock, float tilt,
                           int solid, int map) {
  cube_cloud_tune t;
  kxr_rig rig;
  kxr_head blob;
  memset(sn, 0, sizeof *sn);
  kxr_get_tune(kx, &t);
  kxr_get_rig(kx, &rig);
  memset(&blob, 0, sizeof blob);
  kxr_track_head(kx, &blob);
  sn->z_near = t.z_near;
  sn->z_far = t.z_far;
  sn->stride = t.stride;
  sn->dot = t.dot;
  sn->color = t.color;
  sn->tilt_deg = kxr_get_tilt(kx);
  (void)tilt;
  sn->scale = kxr_get_scale(kx);
  sn->hide = kxr_get_hide_radius(kx);
  sn->autolock = autolock;
  sn->locked = rig.locked;
  sn->live = kxr_live_running(kx);
  sn->points = kxr_points(kx);
  sn->body_h = kxr_get_body_h(kx);
  sn->floor_k = kxr_get_floor_k(kx);
  sn->have_xr_floor = kxr_have_xr_floor(kx);
  sn->xr_floor = kxr_get_xr_floor(kx);
  sn->solid = solid;
  sn->map = map;
  sn->nobj = kxr_mesh_objects(kx);
  sn->conf = blob.conf;
  {
    ksync_rigid rigid;
    kinect_fuse_get_rigid(&rigid);
    sn->src = (int)kinect_fuse_source();
    sn->identity_rigid = ksync_rigid_is_identity(&rigid);
    ksync_rigid_ypr(&rigid, &sn->align_yaw, &sn->align_pitch);
    sn->switching = kxr_source_switching(kx);
  }
  snprintf(sn->err, sizeof sn->err, "%s", kxr_error(kx));
}

static float squeeze_of(kxr_xrinput *in, XrSession sess, int hand) {
  XrActionStateFloat fl = {.type = XR_TYPE_ACTION_STATE_FLOAT};
  XrActionStateBoolean cl = {.type = XR_TYPE_ACTION_STATE_BOOLEAN};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
  float v = 0.f;
  gi.subactionPath = in->hand[hand];
  gi.action = in->squeeze;
  if (XR_SUCCEEDED(xrGetActionStateFloat(sess, &gi, &fl)) && fl.isActive)
    v = fl.currentState;
  gi.action = in->squeeze_click;
  if (XR_SUCCEEDED(xrGetActionStateBoolean(sess, &gi, &cl)) && cl.isActive &&
      cl.currentState && v < 1.f)
    v = 1.f;
  return v;
}

static int locate_grip(kxr_xrinput *in, XrSession sess, XrSpace local, XrTime t, int hand,
                       kxr_vec3 *out) {
  XrActionStatePose ps = {.type = XR_TYPE_ACTION_STATE_POSE};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
  XrSpaceLocation loc = {.type = XR_TYPE_SPACE_LOCATION};
  gi.action = in->grip_pose;
  gi.subactionPath = in->hand[hand];
  if (XR_FAILED(xrGetActionStatePose(sess, &gi, &ps)) || !ps.isActive) return 0;
  if (XR_FAILED(xrLocateSpace(in->grip_space[hand], local, t, &loc))) return 0;
  if (!(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)) return 0;
  out->x = loc.pose.position.x;
  out->y = loc.pose.position.y;
  out->z = loc.pose.position.z;
  return 1;
}

static int stick_hand(kxr_xrinput *in, XrSession sess, int hand, float *x, float *y) {
  XrActionStateVector2f st = {.type = XR_TYPE_ACTION_STATE_VECTOR2F};
  XrActionStateGetInfo gi = {.type = XR_TYPE_ACTION_STATE_GET_INFO};
  *x = *y = 0.f;
  if (!in) return 0;
  gi.action = in->stick;
  gi.subactionPath = in->hand[hand];
  if (XR_FAILED(xrGetActionStateVector2f(sess, &gi, &st)) || !st.isActive) return 0;
  *x = st.currentState.x;
  *y = st.currentState.y;
  return 1;
}

static int stick_of(kxr_xrinput *in, XrSession sess, float *x, float *y) {
  return stick_hand(in, sess, 0, x, y) || stick_hand(in, sess, 1, x, y);
}

static float horiz_yaw_deg(kxr_vec3 a, kxr_vec3 b) {
  return atan2f(b.x - a.x, -(b.z - a.z)) * 180.f / (float)M_PI;
}

static float elev_pitch_deg(kxr_vec3 a, kxr_vec3 b) {
  float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  float h = sqrtf(dx * dx + dz * dz);
  return atan2f(dy, h > 1e-4f ? h : 1e-4f) * 180.f / (float)M_PI;
}

static kxr_vec3 vmid(kxr_vec3 a, kxr_vec3 b) {
  kxr_vec3 m = {0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.5f * (a.z + b.z)};
  return m;
}

static float vdist(kxr_vec3 a, kxr_vec3 b) {
  float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

static void grab_begin_dual(kxr_grab *g, kxr_session *kx, kxr_vec3 L, kxr_vec3 R) {
  memset(g, 0, sizeof *g);
  kxr_get_rig(kx, &g->rig);
  g->scale = kxr_get_scale(kx);
  g->left = L;
  g->right = R;
  g->mid = vmid(L, R);
  g->dist = vdist(L, R);
  if (g->dist < 0.04f) g->dist = 0.04f;
  g->yaw = horiz_yaw_deg(L, R);
  g->pitch = elev_pitch_deg(L, R);
  g->dual = 1;
  g->active = 1;
}

static void grab_begin_one(kxr_grab *g, kxr_session *kx, kxr_vec3 hand) {
  memset(g, 0, sizeof *g);
  kxr_get_rig(kx, &g->rig);
  g->scale = kxr_get_scale(kx);
  g->mid = hand;
  g->left = hand;
  g->dual = 0;
  g->active = 1;
}

static void grab_apply_dual(kxr_grab *g, kxr_session *kx, kxr_vec3 L, kxr_vec3 R) {
  kxr_vec3 mid = vmid(L, R), d;
  float dist = vdist(L, R);
  float yaw = horiz_yaw_deg(L, R);
  float pitch = elev_pitch_deg(L, R);
  float factor;
  if (dist < 0.04f) dist = 0.04f;
  factor = dist / g->dist;
  kxr_set_rig_scale(kx, &g->rig, g->scale);
  kxr_orbit_about(kx, g->mid, yaw - g->yaw, pitch - g->pitch);
  kxr_scale_about(kx, g->mid, factor);
  d.x = mid.x - g->mid.x;
  d.y = mid.y - g->mid.y;
  d.z = mid.z - g->mid.z;
  kxr_translate(kx, d);
}

static void grab_apply_one(kxr_grab *g, kxr_session *kx, kxr_vec3 hand, kxr_vec3 head) {
  float yaw0 = horiz_yaw_deg(head, g->mid);
  float yaw1 = horiz_yaw_deg(head, hand);
  float pitch0 = elev_pitch_deg(head, g->mid);
  float pitch1 = elev_pitch_deg(head, hand);
  kxr_set_rig_scale(kx, &g->rig, g->scale);
  kxr_orbit_about(kx, head, yaw1 - yaw0, pitch1 - pitch0);
}

int main(int argc, char **argv) {
  const char *xyz = "/home/voldemar/Dev/lab/prophecy_cube/state/trackers/kinect_latest.xyz";
  const char *src_name = NULL;
  int live = 1, autolock = 1, want_stage = 0, placed = 0;
  unsigned note_seen = 0;
  kxr_vec3 place_p = {0, 0, 0};
  float place_yaw = 0, place_pitch = 0;
  int i;

  GLFWwindow *win = NULL;
  XrInstance instance = XR_NULL_HANDLE;
  XrSystemId system = XR_NULL_SYSTEM_ID;
  XrSession session = XR_NULL_HANDLE;
  XrSpace local_space = XR_NULL_HANDLE;
  XrSpace view_space = XR_NULL_HANDLE;
  XrViewConfigurationType vtype = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  XrViewConfigurationView vc[MAX_V];
  XrView views[MAX_V];
  kxr_swap swaps[MAX_V] = {{0}};
  uint32_t nv = 0;
  XrSessionState st = XR_SESSION_STATE_UNKNOWN;
  int running = 0, exit_rc = 1;

  kxr_session *kx = NULL;
  kxr_gl *gl = NULL;
  float *verts = NULL;
  const int vmax = CUBE_CLOUD_MAX;
  kxr_xrinput xin;
  kxr_grab grab;
  kxr_menu menu;
  int user_adjusted = 0;
  int trig_was = 0, solid = 1, map_on = 0, map_seeded = 0;
  float tilt_deg = 0.f, nav_cd = 0.f;
  float map_yaw = 0.f, map_pitch = -0.25f;
  kxr_vec3 map_pos = {0, 1.4f, 1.6f};
  kxr_vec3 map_hmd0 = {0, 0, 0};
  kxr_vec3 loco_pos = {0, 0, 0};
  float loco_yaw = 0.f;
  float *meshv = NULL;
  unsigned *meshi = NULL;
  XrTime prev_t = 0;
  memset(&xin, 0, sizeof xin);
  memset(&grab, 0, sizeof grab);
  kxr_menu_init(&menu);

  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--xyz") == 0 && i + 1 < argc)
      xyz = argv[++i];
    else if (strcmp(argv[i], "--live") == 0)
      live = 1;
    else if (strcmp(argv[i], "--no-live") == 0)
      live = 0;
    else if (strcmp(argv[i], "--src") == 0 && i + 1 < argc)
      src_name = argv[++i];
    else if (strncmp(argv[i], "--src=", 6) == 0)
      src_name = argv[i] + 6;
    else if (strcmp(argv[i], "--no-autolock") == 0)
      autolock = 0;
    else if (strcmp(argv[i], "--stage") == 0)
      want_stage = 1;
    else if (strcmp(argv[i], "--place") == 0 && i + 5 < argc) {
      place_p.x = (float)atof(argv[++i]);
      place_p.y = (float)atof(argv[++i]);
      place_p.z = (float)atof(argv[++i]);
      place_yaw = (float)atof(argv[++i]);
      place_pitch = (float)atof(argv[++i]);
      placed = 1;
    } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      usage();
      return 0;
    } else {
      usage();
      return 2;
    }
  }

#if defined(GLFW_PLATFORM_X11)
  glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
  if (!glfwInit()) die("glfwInit");
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_FALSE);
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  win = glfwCreateWindow(960, 540, "kxr_openxr · Kinect in player view", NULL, NULL);
  if (!win) die("glfw window (need GL 3.3)");
  glfwMakeContextCurrent(win);
  glfwSetKeyCallback(win, on_key);
  glewExperimental = GL_TRUE;
  {
    GLenum ge = glewInit();
    (void)glGetError();
    if (ge != GLEW_OK && !glCreateShader)
      die("glewInit");
    if (ge != GLEW_OK)
      fprintf(stderr, "glew: %s (core context, continuing)\n", glewGetErrorString(ge));
  }
  glDisable(GL_FRAMEBUFFER_SRGB);

  {
    const char *exts[] = {XR_KHR_OPENGL_ENABLE_EXTENSION_NAME};
    XrInstanceCreateInfo ici = {.type = XR_TYPE_INSTANCE_CREATE_INFO};
    XrResult r;
    strncpy(ici.applicationInfo.applicationName, "kxr_openxr",
            XR_MAX_APPLICATION_NAME_SIZE - 1);
    strncpy(ici.applicationInfo.engineName, "kinect_xr", XR_MAX_ENGINE_NAME_SIZE - 1);
    ici.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    ici.enabledExtensionCount = 1;
    ici.enabledExtensionNames = exts;
    r = xrCreateInstance(&ici, &instance);
    if (XR_FAILED(r)) {
      fprintf(stderr, "xrCreateInstance: %s\n", xrerr(0, r));
      fprintf(stderr, "set XR_RUNTIME_JSON=/usr/share/openxr/1/openxr_wivrn.json\n");
      goto done;
    }
  }
  {
    XrSystemGetInfo gi = {.type = XR_TYPE_SYSTEM_GET_INFO,
                          .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    if (XR_FAILED(xrGetSystem(instance, &gi, &system))) {
      fprintf(stderr, "no HMD system (is WiVRn connected?)\n");
      goto done;
    }
  }
  {
    PFN_xrGetOpenGLGraphicsRequirementsKHR pfn = NULL;
    XrGraphicsRequirementsOpenGLKHR req = {.type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
    xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR",
                          (PFN_xrVoidFunction *)&pfn);
    if (!pfn || XR_FAILED(pfn(instance, system, &req))) {
      fprintf(stderr, "OpenGL graphics requirements missing\n");
      goto done;
    }
  }
  {
    XrGraphicsBindingOpenGLXlibKHR bind = {.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR};
    XrSessionCreateInfo sci = {.type = XR_TYPE_SESSION_CREATE_INFO};
    Display *dpy = glfwGetX11Display();
    Window xw = glfwGetX11Window(win);
    XWindowAttributes wa;
    if (!dpy) {
      fprintf(stderr, "glfw X11 display is NULL (need X11, not Wayland)\n");
      goto done;
    }
    bind.xDisplay = dpy;
    bind.glxContext = glfwGetGLXContext(win);
    bind.glxDrawable = glfwGetGLXWindow(win);
    bind.visualid = 0;
    bind.glxFBConfig = 0;
    if (dpy && xw && XGetWindowAttributes(dpy, xw, &wa) && wa.visual) {
      int ncfg = 0, i;
      GLXFBConfig *cfgs;
      bind.visualid = (uint32_t)XVisualIDFromVisual(wa.visual);
      cfgs = glXGetFBConfigs(dpy, DefaultScreen(dpy), &ncfg);
      for (i = 0; i < ncfg; i++) {
        int vid = 0;
        glXGetFBConfigAttrib(dpy, cfgs[i], GLX_VISUAL_ID, &vid);
        if ((uint32_t)vid == bind.visualid) {
          bind.glxFBConfig = cfgs[i];
          break;
        }
      }
      if (cfgs) XFree(cfgs);
    }
    sci.next = &bind;
    sci.systemId = system;
    if (XR_FAILED(xrCreateSession(instance, &sci, &session))) {
      fprintf(stderr, "xrCreateSession failed (GLX binding)\n");
      goto done;
    }
  }
  {
    XrReferenceSpaceCreateInfo rsi = {.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    XrResult r;
    const char *space_name = "LOCAL";
    rsi.poseInReferenceSpace.orientation.w = 1;
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    r = xrCreateReferenceSpace(session, &rsi, &local_space);
    if (XR_FAILED(r)) {
      fprintf(stderr, "reference space: %s\n", xrerr(instance, r));
      goto done;
    }
    /* CubeUI: STAGE when the runtime has a floor origin — real metres. */
    if (want_stage) {
      XrSpace stage = XR_NULL_HANDLE;
      rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
      if (XR_SUCCEEDED(xrCreateReferenceSpace(session, &rsi, &stage))) {
        xrDestroySpace(local_space);
        local_space = stage;
        space_name = "STAGE";
      }
    }
    fprintf(stderr, "content space=%s\n", space_name);
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    if (XR_FAILED(xrCreateReferenceSpace(session, &rsi, &view_space))) {
      fprintf(stderr, "VIEW space failed\n");
      goto done;
    }
  }
  {
    uint32_t n = 0;
    xrEnumerateViewConfigurationViews(instance, system, vtype, 0, &n, NULL);
    if (n > MAX_V) n = MAX_V;
    if (n < 1) {
      fprintf(stderr, "no views\n");
      goto done;
    }
    for (i = 0; i < (int)n; i++) vc[i].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
    xrEnumerateViewConfigurationViews(instance, system, vtype, n, &nv, vc);
    memset(swaps, 0, sizeof swaps);
    for (i = 0; i < (int)nv; i++) {
      if (swap_init(session, &vc[i], &swaps[i]) != 0) {
        fprintf(stderr, "swapchain %d failed\n", i);
        goto done;
      }
      views[i].type = XR_TYPE_VIEW;
    }
  }

  if (input_init(&xin, instance, session) != 0)
    fprintf(stderr, "kxr_openxr: controllers unavailable (keyboard still works)\n");
  else
    fprintf(stderr, "controllers: grips twist+pinch, A/X lock, B sample, Y solve\n");

  kx = kxr_create();
  if (!kx) die("kxr_create");
  if (src_name) kxr_set_source(kx, src_name);
  {
    cube_cloud_tune t0;
    kxr_get_tune(kx, &t0);
    t0.color = 3;
    kxr_set_tune(kx, &t0);
  }
  if (placed) {
    kxr_rig rig;
    kxr_rig_place(&rig, place_p, place_yaw, place_pitch);
    kxr_set_rig(kx, &rig);
    autolock = 0;
  }
  if (live) {
    if (kxr_live_start(kx) != 0)
      fprintf(stderr, "live failed (%s) — falling back to xyz\n", kxr_error(kx));
  }
  if (kxr_points(kx) < 1 && xyz) {
    if (kxr_load_xyz(kx, xyz) != 0)
      fprintf(stderr, "xyz load: %s\n", kxr_error(kx));
  }
  if (kxr_points(kx) < 1)
    fprintf(stderr, "no cloud yet — lock will wait for points\n");

  gl = kxr_gl_create();
  if (!gl || kxr_gl_error(gl)[0])
    fprintf(stderr, "gl: %s\n", gl ? kxr_gl_error(gl) : "null");
  verts = (float *)malloc((size_t)vmax * 6 * sizeof(float));
  meshv = (float *)malloc((size_t)CUBE_MESH_MAX_V * 9 * sizeof(float));
  meshi = (unsigned *)malloc((size_t)CUBE_MESH_MAX_I * sizeof(unsigned));
  if (!verts) die("oom verts");

  fprintf(stderr, "kxr_openxr ready views=%u %ux%u  keys: L/S/R/Y/Q  grips: rotate+zoom\n",
          nv, swaps[0].w, swaps[0].h);

  while (!g_quit && !glfwWindowShouldClose(win)) {
    XrFrameWaitInfo fwi = {.type = XR_TYPE_FRAME_WAIT_INFO};
    XrFrameState fs = {.type = XR_TYPE_FRAME_STATE};
    XrFrameBeginInfo fbi = {.type = XR_TYPE_FRAME_BEGIN_INFO};
    XrEventDataBuffer ev = {.type = XR_TYPE_EVENT_DATA_BUFFER};
    XrCompositionLayerProjection layer = {.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    XrCompositionLayerProjectionView pv[MAX_V];
    const XrCompositionLayerBaseHeader *layers[1];
    memset(pv, 0, sizeof pv);
    XrFrameEndInfo fei = {.type = XR_TYPE_FRAME_END_INFO};
    int nlayer = 0;
    static XrEnvironmentBlendMode blend = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    static int blend_inited;

    glfwPollEvents();

    while (xrPollEvent(instance, &ev) == XR_SUCCESS) {
      if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
        XrEventDataSessionStateChanged *sc = (XrEventDataSessionStateChanged *)&ev;
        st = sc->state;
        if (st == XR_SESSION_STATE_READY) {
          XrSessionBeginInfo bi = {.type = XR_TYPE_SESSION_BEGIN_INFO,
                                   .primaryViewConfigurationType = vtype};
          xrBeginSession(session, &bi);
          running = 1;
        } else if (st == XR_SESSION_STATE_STOPPING) {
          xrEndSession(session);
          running = 0;
        } else if (st == XR_SESSION_STATE_EXITING ||
                   st == XR_SESSION_STATE_LOSS_PENDING) {
          g_quit = 1;
        }
      }
      ev.type = XR_TYPE_EVENT_DATA_BUFFER;
    }

    if (kxr_live_running(kx)) {
      kxr_live_poll(kx);
    } else if (xyz) {
      struct stat st;
      static time_t xyz_m;
      static off_t xyz_sz;
      if (stat(xyz, &st) == 0 && (st.st_mtime != xyz_m || st.st_size != xyz_sz)) {
        xyz_m = st.st_mtime;
        xyz_sz = st.st_size;
        kxr_load_xyz(kx, xyz);
      }
    }

    if (!running) {
      struct timespec ts = {.tv_nsec = 20000000};
      nanosleep(&ts, NULL);
      continue;
    }

    if (!blend_inited) {
      XrEnvironmentBlendMode modes[4];
      uint32_t nm = 0;
      if (XR_SUCCEEDED(xrEnumerateEnvironmentBlendModes(instance, system, vtype, 4, &nm,
                                                        modes)) &&
          nm > 0)
        blend = modes[0];
      blend_inited = 1;
    }

    if (XR_FAILED(xrWaitFrame(session, &fwi, &fs))) break;
    xrBeginFrame(session, &fbi);

    if (xin.ready) {
      input_sync(&xin, session);
      if (bool_edge(session, xin.panel)) g_menu = 1;
      if (!menu.open) {
        if (bool_edge(session, xin.lock)) g_lock = 1;
        if (bool_edge(session, xin.sample)) g_sample = 1;
        if (bool_edge(session, xin.solve)) g_solve = 1;
        if (bool_edge(session, xin.yaw)) g_yaw = 1;
        if (bool_edge_hand(&xin, session, xin.stick_click, 0) ||
            bool_edge_hand(&xin, session, xin.stick_click, 1))
          g_scale_rst = 1;
      }
    }

    if (fs.shouldRender) {
      XrViewLocateInfo vli = {.type = XR_TYPE_VIEW_LOCATE_INFO};
      XrViewState vs = {.type = XR_TYPE_VIEW_STATE};
      XrSpaceLocation headloc = {.type = XR_TYPE_SPACE_LOCATION};
      uint32_t got = 0;
      kxr_view kv[KXR_MAX_VIEWS];
      kxr_rig rig;
      kxr_pose xr_head;
      int nvert = 0;
      int views_ok;
      int have_xr_head = 0;
      kxr_vec3 laser_o = {0, 0, 0}, laser_d = {0, 0, -1};
      int laser_ok = 0, have_mesh = 0, nmesh_v = 0, nmesh_i = 0;
      int menu_activate = 0;
      kxr_pose_identity(&xr_head);

      vli.viewConfigurationType = vtype;
      vli.displayTime = fs.predictedDisplayTime;
      vli.space = local_space;
      for (i = 0; i < (int)nv; i++) {
        memset(&views[i], 0, sizeof views[i]);
        views[i].type = XR_TYPE_VIEW;
      }
      xrLocateViews(session, &vli, &vs, nv, &got, views);
      xrLocateSpace(view_space, local_space, fs.predictedDisplayTime, &headloc);
      views_ok = (int)got > 0 &&
                 (vs.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) &&
                 (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);

      if (headloc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) {
        xr_head = kxr_pose_from_xr(&headloc.pose);
        have_xr_head = 1;
        kxr_set_head(kx, &xr_head);
      }
      if (got > KXR_MAX_VIEWS) got = KXR_MAX_VIEWS;
      for (i = 0; i < (int)got; i++) kv[i] = kxr_view_from_xr(&views[i]);
      kxr_set_views(kx, kv, (int)got);

      if (g_menu) {
        g_menu = 0;
        if (have_xr_head)
          kxr_menu_toggle(&menu, &xr_head);
        else
          menu.open = !menu.open;
        menu.dirty = 1;
      }
      if (g_toggle_solid) {
        g_toggle_solid = 0;
        solid = !solid;
      }
      if (g_toggle_map) {
        g_toggle_map = 0;
        map_on = !map_on;
        map_seeded = 0;
      }
      if (!map_on) map_seeded = 0;
      if (map_on && !map_seeded && have_xr_head) {
        kxr_vec3 back = {0, 0.25f, 1.8f};
        back = qrot3(xr_head.q, back);
        map_hmd0 = xr_head.p;
        map_pos.x = xr_head.p.x + back.x;
        map_pos.y = xr_head.p.y + back.y;
        map_pos.z = xr_head.p.z + back.z;
        map_yaw = 0.f;
        map_pitch = 0.f;
        map_seeded = 1;
      }

      if (xin.ready) {
        kxr_pose hp[2];
        kxr_vec3 hand[2], aim_o, aim_d, fwd = {0, 0, -1};
        int got_hand[2], got_aim = 0;
        float sq[2], dt = 1.f / 72.f;
        int hold[2], dual, one;
        memset(hp, 0, sizeof hp);
        if (prev_t > 0 && fs.predictedDisplayTime > prev_t) {
          dt = (float)(fs.predictedDisplayTime - prev_t) * 1e-9f;
          if (dt < 0.004f) dt = 0.004f;
          if (dt > 0.05f) dt = 0.05f;
        }
        got_hand[0] = locate_grip(&xin, session, local_space, fs.predictedDisplayTime, 0,
                                  &hand[0]);
        got_hand[1] = locate_grip(&xin, session, local_space, fs.predictedDisplayTime, 1,
                                  &hand[1]);
        sq[0] = squeeze_of(&xin, session, 0);
        sq[1] = squeeze_of(&xin, session, 1);
        hold[0] = got_hand[0] && sq[0] >= 0.62f;
        hold[1] = got_hand[1] && sq[1] >= 0.62f;
        dual = hold[0] && hold[1];
        one = (hold[0] ^ hold[1]);
        got_aim = locate_pose(&xin, session, local_space, fs.predictedDisplayTime, 1, 1,
                              &hp[1]);
        if (!got_aim)
          got_aim = locate_pose(&xin, session, local_space, fs.predictedDisplayTime, 1, 0,
                                &hp[1]);
        if (!got_aim)
          got_aim = locate_pose(&xin, session, local_space, fs.predictedDisplayTime, 0, 1,
                                &hp[0]);
        if (got_aim) {
          kxr_pose *ap = (hp[1].q.w != 0.f || hp[1].p.x != 0.f || hp[1].p.y != 0.f)
                             ? &hp[1]
                             : &hp[0];
          aim_o = ap->p;
          aim_d = qrot3(ap->q, fwd);
          laser_o = aim_o;
          laser_d = aim_d;
          laser_ok = 1;
        }

        if (menu.open) {
          if (bool_edge(session, xin.lock) || trigger_edge(&xin, session, &trig_was))
            menu_activate = 1;
          if (bool_edge(session, xin.sample)) {
            apply_menu_act(kxr_menu_nav(&menu, -1, 0, 0), kx, &menu, &autolock, &tilt_deg,
                           have_xr_head ? &xr_head : NULL, &solid, &map_on);
          }
          if (nav_cd > 0.f)
            nav_cd -= dt;
          else {
            float sx = 0, sy = 0;
            int dx = 0, dy = 0;
            stick_of(&xin, session, &sx, &sy);
            if (sy > 0.55f)
              dy = -1;
            else if (sy < -0.55f)
              dy = 1;
            if (sx > 0.55f)
              dx = 1;
            else if (sx < -0.55f)
              dx = -1;
            if (dx || dy) {
              apply_menu_act(kxr_menu_nav(&menu, dx, dy, 0), kx, &menu, &autolock,
                             &tilt_deg, have_xr_head ? &xr_head : NULL, &solid, &map_on);
              nav_cd = 0.18f;
            }
          }
          grab.active = 0;
        } else if (dual) {
          if (!grab.active || !grab.dual)
            grab_begin_dual(&grab, kx, hand[0], hand[1]);
          grab_apply_dual(&grab, kx, hand[0], hand[1]);
          user_adjusted = 1;
        } else if (one && have_xr_head) {
          int h = hold[1] ? 1 : 0;
          if (!grab.active || grab.dual)
            grab_begin_one(&grab, kx, hand[h]);
          grab_apply_one(&grab, kx, hand[h], xr_head.p);
          user_adjusted = 1;
        } else {
          grab.active = 0;
        }
        if (!menu.open && !grab.active) {
          float lx = 0, ly = 0, rx = 0, ry = 0;
          stick_hand(&xin, session, 0, &lx, &ly);
          stick_hand(&xin, session, 1, &rx, &ry);
          if (fabsf(lx) < 0.15f) lx = 0;
          if (fabsf(ly) < 0.15f) ly = 0;
          if (fabsf(rx) < 0.15f) rx = 0;
          if (fabsf(ry) < 0.15f) ry = 0;
          ly = -ly; /* Quest / WiVRn stick Y is inverted vs look-forward */
          if (map_on && have_xr_head) {
            kxr_vec3 fwd = {0, 0, -1}, rgt = {1, 0, 0};
            kxr_quat qlook;
            float ln, spd = 1.7f * dt;
            qlook = xr_head.q;
            {
              kxr_quat qy = {0, sinf(map_yaw * 0.5f), 0, cosf(map_yaw * 0.5f)};
              qlook = qmul(qy, xr_head.q);
            }
            fwd = qrot3(qlook, fwd);
            rgt = qrot3(qlook, rgt);
            ln = sqrtf(fwd.x * fwd.x + fwd.y * fwd.y + fwd.z * fwd.z);
            if (ln > 1e-4f) {
              fwd.x /= ln;
              fwd.y /= ln;
              fwd.z /= ln;
            }
            ln = sqrtf(rgt.x * rgt.x + rgt.z * rgt.z);
            if (ln > 1e-4f) {
              rgt.x /= ln;
              rgt.z /= ln;
            }
            map_pos.x += (fwd.x * ly + rgt.x * lx) * spd;
            map_pos.y += (fwd.y * ly) * spd;
            map_pos.z += (fwd.z * ly + rgt.z * lx) * spd;
            map_yaw -= rx * 1.9f * dt;
            map_pitch -= ry * 1.4f * dt;
            if (map_pitch < -1.2f) map_pitch = -1.2f;
            if (map_pitch > 1.2f) map_pitch = 1.2f;
          } else if (have_xr_head) {
            kxr_vec3 fwd = {0, 0, -1}, rgt = {1, 0, 0};
            float ln;
            fwd = qrot3(xr_head.q, fwd);
            rgt = qrot3(xr_head.q, rgt);
            fwd.y = 0;
            rgt.y = 0;
            ln = sqrtf(fwd.x * fwd.x + fwd.z * fwd.z);
            if (ln > 1e-4f) {
              fwd.x /= ln;
              fwd.z /= ln;
            }
            ln = sqrtf(rgt.x * rgt.x + rgt.z * rgt.z);
            if (ln > 1e-4f) {
              rgt.x /= ln;
              rgt.z /= ln;
            }
            loco_pos.x += (fwd.x * ly + rgt.x * lx) * 1.5f * dt;
            loco_pos.z += (fwd.z * ly + rgt.z * lx) * 1.5f * dt;
            loco_pos.y += ry * 1.1f * dt;
            loco_yaw -= rx * 1.7f * dt;
          }
        }
      }

      /* Left stick walks / right stick looks — apply to the eyes. */
      if ((map_on || loco_yaw != 0.f || loco_pos.x != 0.f || loco_pos.z != 0.f ||
           loco_pos.y != 0.f) &&
          (int)got > 0) {
        kxr_quat qy, qp, qbody;
        kxr_vec3 body_p;
        int e;
        float hy = map_on ? map_yaw : loco_yaw;
        float hp = map_on ? map_pitch : 0.f;
        qy.x = 0;
        qy.y = sinf(hy * 0.5f);
        qy.z = 0;
        qy.w = cosf(hy * 0.5f);
        qp.x = sinf(hp * 0.5f);
        qp.y = 0;
        qp.z = 0;
        qp.w = cosf(hp * 0.5f);
        qbody.w = qy.w * qp.w - qy.x * qp.x - qy.y * qp.y - qy.z * qp.z;
        qbody.x = qy.w * qp.x + qy.x * qp.w + qy.y * qp.z - qy.z * qp.y;
        qbody.y = qy.w * qp.y - qy.x * qp.z + qy.y * qp.w + qy.z * qp.x;
        qbody.z = qy.w * qp.z + qy.x * qp.y - qy.y * qp.x + qy.z * qp.w;
        if (map_on && have_xr_head) {
          /* Stick body + live HMD offset so the headset still aims the map camera. */
          body_p.x = map_pos.x + (xr_head.p.x - map_hmd0.x);
          body_p.y = map_pos.y + (xr_head.p.y - map_hmd0.y);
          body_p.z = map_pos.z + (xr_head.p.z - map_hmd0.z);
          qbody = qmul(qbody, xr_head.q);
        } else {
          body_p.x = xr_head.p.x + loco_pos.x;
          body_p.y = xr_head.p.y + loco_pos.y;
          body_p.z = xr_head.p.z + loco_pos.z;
        }
        for (e = 0; e < (int)got && e < KXR_MAX_VIEWS; e++) {
          kxr_vec3 off, local = {0, 0, 0};
          if (have_xr_head) {
            off.x = kv[e].pose.p.x - xr_head.p.x;
            off.y = kv[e].pose.p.y - xr_head.p.y;
            off.z = kv[e].pose.p.z - xr_head.p.z;
            {
              kxr_quat qc = xr_head.q;
              qc.x = -qc.x;
              qc.y = -qc.y;
              qc.z = -qc.z;
              local = qrot3(qc, off);
            }
          }
          off = qrot3(qbody, local);
          kv[e].pose.p.x = body_p.x + off.x;
          kv[e].pose.p.y = body_p.y + off.y;
          kv[e].pose.p.z = body_p.z + off.z;
          if (map_on) {
            kv[e].pose.q = qbody;
          } else {
            kv[e].pose.q = qmul(qy, kv[e].pose.q);
          }
        }
        kxr_set_views(kx, kv, (int)got);
      }

      /* Menu is a view-space HUD — always in front of the camera you see through. */
      if (menu.open && (int)got > 0) {
        kxr_pose cam = kv[0].pose;
        menu.hover_ok = 0;
        menu.hover_row = -1;
        float viewm[16];
        if ((int)got > 1) {
          cam.p.x = 0.5f * (kv[0].pose.p.x + kv[1].pose.p.x);
          cam.p.y = 0.5f * (kv[0].pose.p.y + kv[1].pose.p.y);
          cam.p.z = 0.5f * (kv[0].pose.p.z + kv[1].pose.p.z);
        }
        kxr_menu_recenter(&menu, &cam);
        if (laser_ok) {
          kxr_vec3 ov, dv;
          float t, px, py, u, v;
          kxr_view_matrix(&cam, viewm);
          ov = xfm_pt(viewm, laser_o);
          dv = xfm_dir(viewm, laser_d);
          if (fabsf(dv.z) > 1e-5f) {
            t = (-1.05f - ov.z) / dv.z;
            if (t > 0.02f && t < 8.f) {
              px = ov.x + t * dv.x;
              py = ov.y + t * dv.y;
              u = px / menu.width_m + 0.5f;
              v = 0.5f - py / menu.height_m;
              if (u >= 0.f && u <= 1.f && v >= 0.f && v <= 1.f) {
                int ir = (int)((v * (float)KXR_MENU_HGT - (float)KXR_MENU_Y0) /
                               (float)KXR_MENU_PITCH);
                menu.hover_ok = 1;
                menu.hover_row = (ir >= 0 && ir < KXR_MENU_ROWS) ? ir : -1;
                if (menu_activate && ir >= 0)
                  apply_menu_act(kxr_menu_click_row(&menu, ir, u < 0.5f ? -1 : 1), kx,
                                 &menu, &autolock, &tilt_deg,
                                 have_xr_head ? &xr_head : NULL, &solid, &map_on);
              }
            }
          }
        }
        if (menu_activate && !(menu.hover_ok && menu.hover_row >= 0))
          apply_menu_act(kxr_menu_nav(&menu, 0, 0, 1), kx, &menu, &autolock, &tilt_deg,
                         have_xr_head ? &xr_head : NULL, &solid, &map_on);
      }

      if (g_lock) {
        g_lock = 0;
        if (kxr_lock_head(kx) != 0)
          fprintf(stderr, "lock: %s\n", kxr_error(kx));
        else
          fprintf(stderr, "lock ok\n");
      }
      if (g_sample) {
        g_sample = 0;
        if (kxr_lock_sample(kx) != 0)
          fprintf(stderr, "sample: %s\n", kxr_error(kx));
        else
          fprintf(stderr, "sample ok\n");
      }
      if (g_solve) {
        g_solve = 0;
        if (kxr_lock_solve(kx) != 0)
          fprintf(stderr, "solve: %s\n", kxr_error(kx));
        else
          fprintf(stderr, "solve ok\n");
      }
      if (g_yaw) {
        g_yaw = 0;
        if (kxr_lock_yaw_walk(kx) != 0)
          fprintf(stderr, "yaw: %s\n", kxr_error(kx));
        else
          fprintf(stderr, "yaw ok\n");
      }
      if (g_scale_up) {
        g_scale_up = 0;
        kxr_set_scale(kx, kxr_get_scale(kx) * 1.08f);
        fprintf(stderr, "scale=%.3f\n", kxr_get_scale(kx));
      }
      if (g_scale_dn) {
        g_scale_dn = 0;
        kxr_set_scale(kx, kxr_get_scale(kx) / 1.08f);
        fprintf(stderr, "scale=%.3f\n", kxr_get_scale(kx));
      }
      if (g_scale_rst) {
        g_scale_rst = 0;
        kxr_set_scale(kx, 1.f);
        fprintf(stderr, "scale=1\n");
      }
      kxr_get_rig(kx, &rig);
      if (autolock && !user_adjusted && !grab.active && !menu.open) {
        kxr_head blob;
        int need_lock = !rig.locked;
        if (kxr_track_head(kx, &blob) == 0 && blob.conf >= 0.35f) {
          if (rig.locked && have_xr_head) {
            kxr_vec3 pred;
            if (kxr_predict_head(kx, &pred) == 0) {
              float dx = pred.x - xr_head.p.x;
              float dy = pred.y - xr_head.p.y;
              float dz = pred.z - xr_head.p.z;
              if (dx * dx + dy * dy + dz * dz > 0.40f * 0.40f) need_lock = 1;
            }
          }
          if (need_lock && kxr_lock_head(kx) == 0) {
            if (kxr_autoscale_height(kx) == 0)
              fprintf(stderr, "autoscale=%.3f (floor→head → XR eye height)\n",
                      kxr_get_scale(kx));
            kxr_get_rig(kx, &rig);
            fprintf(stderr, "autolock residual=%.3fm scale=%.3f conf=%.2f\n",
                    rig.residual_m, kxr_get_scale(kx), blob.conf);
          }
        }
      }

      if (verts && kxr_fill_vertices(kx, verts, vmax, &nvert) == 0) {
        if (gl) kxr_gl_upload(gl, verts, nvert);
        if (solid && meshv && meshi &&
            kxr_fill_mesh(kx, meshv, CUBE_MESH_MAX_V, &nmesh_v, meshi, CUBE_MESH_MAX_I,
                          &nmesh_i) == 0 &&
            gl) {
          kxr_gl_upload_mesh(gl, meshv, nmesh_v, meshi, nmesh_i);
          have_mesh = 1;
        }
      }

      if (menu.open && gl) {
        kxr_menu_snap sn;
        char note[160];
        if (kxr_poll_note(kx, &note_seen, note, sizeof note)) {
          snprintf(menu.status, sizeof menu.status, "%s", note);
          menu.dirty = 1;
        }
        fill_menu_snap(&sn, kx, autolock, tilt_deg, solid, map_on);
        kxr_menu_paint(&menu, &sn);
        kxr_gl_hud_upload(gl, menu.rgba, KXR_MENU_W, KXR_MENU_HGT);
      }

      if (!views_ok) got = 0;
      for (i = 0; i < (int)got; i++) {
        XrSwapchainImageAcquireInfo ai = {.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        XrSwapchainImageWaitInfo wi = {.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
                                       .timeout = XR_INFINITE_DURATION};
        XrSwapchainImageReleaseInfo ri = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        uint32_t idx = 0;
        float mvp[16];
        xrAcquireSwapchainImage(swaps[i].handle, &ai, &idx);
        xrWaitSwapchainImage(swaps[i].handle, &wi);
        if (idx >= swaps[i].n) idx = 0;
        /* CubeUI: bind the acquired color tex each frame (don't trust stale FBO). */
        glBindFramebuffer(GL_FRAMEBUFFER, swaps[i].fbo[idx]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                               swaps[i].img[idx].image, 0);
        glViewport(0, 0, (GLsizei)swaps[i].w, (GLsizei)swaps[i].h);
        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        kxr_mvp(kx, i, 0.05f, 12.f, mvp);
        if (gl) {
          if (have_mesh)
            kxr_gl_draw_mesh(gl, mvp);
          else
            kxr_gl_draw(gl, mvp, 0.016f, (float)swaps[i].h);
        }
        if (gl && menu.open) {
          float proj[16], model[16];
          memset(model, 0, sizeof model);
          model[0] = model[5] = model[10] = model[15] = 1.f;
          model[14] = -1.05f;
          kxr_proj_matrix(&kv[i].fov, 0.08f, 8.f, proj);
          kxr_gl_hud_draw(gl, proj, model, menu.width_m, menu.height_m);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        xrReleaseSwapchainImage(swaps[i].handle, &ri);

        memset(&pv[i], 0, sizeof pv[i]);
        pv[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
        pv[i].pose = views[i].pose;
        pv[i].fov = views[i].fov;
        pv[i].subImage.swapchain = swaps[i].handle;
        pv[i].subImage.imageRect.offset.x = 0;
        pv[i].subImage.imageRect.offset.y = 0;
        pv[i].subImage.imageRect.extent.width = (int32_t)swaps[i].w;
        pv[i].subImage.imageRect.extent.height = (int32_t)swaps[i].h;
      }

      /* WiVRn stereo: submit both eyes or nothing (CubeUI always viewCount=2). */
      if (got == nv && nv > 0) {
        layer.space = local_space;
        layer.viewCount = got;
        layer.views = pv;
        layers[0] = (const XrCompositionLayerBaseHeader *)&layer;
        nlayer = 1;
      }
    }

    fei.displayTime = fs.predictedDisplayTime;
    fei.environmentBlendMode = blend;
    fei.layerCount = (uint32_t)nlayer;
    fei.layers = nlayer ? layers : NULL;
    {
      XrResult er = xrEndFrame(session, &fei);
      if (XR_FAILED(er))
        fprintf(stderr, "xrEndFrame: %s\n", xrerr(instance, er));
    }

    prev_t = fs.predictedDisplayTime;

    /* Desktop mirror — default framebuffer, never the OpenXR swapchain. */
    if (gl && running) {
      int ww, wh;
      float mvp[16];
      glfwGetFramebufferSize(win, &ww, &wh);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glViewport(0, 0, ww, wh);
      glClearColor(0.02f, 0.0f, 0.03f, 1.f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      kxr_mvp(kx, 0, 0.05f, 12.f, mvp);
      if (solid)
        kxr_gl_draw_mesh(gl, mvp);
      else
        kxr_gl_draw(gl, mvp, 0.016f, (float)wh);
      glfwSwapBuffers(win);
    }
  }
  exit_rc = 0;

done:
  free(verts);
  free(meshv);
  free(meshi);
  kxr_menu_free(&menu);
  kxr_gl_destroy(gl);
  kxr_destroy(kx);
  for (i = 0; i < MAX_V; i++) swap_free(&swaps[i]);
  input_free(&xin);
  if (view_space) xrDestroySpace(view_space);
  if (local_space) xrDestroySpace(local_space);
  if (session) xrDestroySession(session);
  if (instance) xrDestroyInstance(instance);
  if (win) glfwDestroyWindow(win);
  glfwTerminate();
  return exit_rc;
}
