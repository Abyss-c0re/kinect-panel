/* Head-locked Kinect XR menu — always in front of the HMD.
 * Open with the right controller menu button. */
#ifndef KXR_MENU_H
#define KXR_MENU_H

#include "kinect_xr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KXR_MENU_W 720
#define KXR_MENU_HGT 520
#define KXR_MENU_ROWS 20
/* Row bands. height_m must stay width_m * HGT / W or the laser misses rows. */
#define KXR_MENU_Y0 58
#define KXR_MENU_PITCH 20

typedef enum {
  KXR_ACT_NONE = 0,
  KXR_ACT_NEAR_N,
  KXR_ACT_NEAR_P,
  KXR_ACT_FAR_N,
  KXR_ACT_FAR_P,
  KXR_ACT_STRIDE_N,
  KXR_ACT_STRIDE_P,
  KXR_ACT_DOT_N,
  KXR_ACT_DOT_P,
  KXR_ACT_COLOR,
  KXR_ACT_TILT_N,
  KXR_ACT_TILT_P,
  KXR_ACT_SCALE_N,
  KXR_ACT_SCALE_P,
  KXR_ACT_HIDE_N,
  KXR_ACT_HIDE_P,
  KXR_ACT_AUTOLOCK,
  KXR_ACT_LOCK,
  KXR_ACT_SAMPLE,
  KXR_ACT_SOLVE,
  KXR_ACT_YAW,
  KXR_ACT_RESET,
  KXR_ACT_SOLID,
  KXR_ACT_MAP,
  KXR_ACT_SRC_N,
  KXR_ACT_SRC_P,
  KXR_ACT_ALIGN_V2,
  KXR_ACT_ALIGN_RESET,
  KXR_ACT_CLOSE
} kxr_menu_act;

typedef struct kxr_menu_snap {
  float z_near, z_far;
  int stride, dot, color;
  float tilt_deg;
  float scale, hide;
  int autolock, locked, live, points;
  float body_h, floor_k, xr_floor, conf;
  int have_xr_floor;
  int solid, map, nobj;
  /* 0 auto, 1 kinect, 2 kinect 2, 3 both. identity_rigid is the v2 fit. */
  int src;
  int identity_rigid;
  float align_yaw, align_pitch;
  int switching;
  char err[80];
} kxr_menu_snap;

typedef struct kxr_menu {
  int open;
  int row;
  int dirty;
  char status[160];
  uint8_t *rgba;
  kxr_pose pose;
  float width_m, height_m;
  int have_pose;
  int hover_row;
  int hover_ok;
} kxr_menu;

void kxr_menu_init(kxr_menu *m);
void kxr_menu_free(kxr_menu *m);
void kxr_menu_open(kxr_menu *m, const kxr_pose *head);
void kxr_menu_close(kxr_menu *m);
void kxr_menu_toggle(kxr_menu *m, const kxr_pose *head);
void kxr_menu_recenter(kxr_menu *m, const kxr_pose *head);
void kxr_menu_set_center(kxr_menu *m, kxr_vec3 c);

void kxr_menu_paint(kxr_menu *m, const kxr_menu_snap *snap);
kxr_menu_act kxr_menu_nav(kxr_menu *m, int dx, int dy, int activate);

/* Ray in XR: origin + dir (unit-ish). Returns 1 and row on hit. */
int kxr_menu_ray(const kxr_menu *m, kxr_vec3 origin, kxr_vec3 dir, int *row,
                 float *hitx, float *hity, float *hitz, float *hitu);
kxr_menu_act kxr_menu_click_row(kxr_menu *m, int row, int dir);

#ifdef __cplusplus
}
#endif
#endif
