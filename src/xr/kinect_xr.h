/* Kinect point cloud in the player's OpenXR space.
 *
 * Kinect camera K: +X right, +Y up, +Z into the room (panel cloud).
 * OpenXR local X:  +X right, +Y up, -Z forward (OpenGL).
 * Default axis map: (x,y,z)_K → (x,y,-z)_X.
 *
 * Lock T so the Kinect-seen human head sits on the OpenXR head pose.
 * Head is the crown of a standing/sitting body column, not the tallest point.
 * Draw the cloud with the compositor's per-eye view+FOV — not an orbit camera.
 *
 * This header does not include <openxr/openxr.h>. If you include OpenXR first,
 * kxr_pose_from_xr / kxr_fov_from_xr / kxr_view_from_xr become available.
 */
#ifndef KINECT_XR_H
#define KINECT_XR_H

#include "cube_kinect_cloud.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KXR_MAX_VIEWS 2
#define KXR_MAX_SAMP 32

typedef struct kxr_vec3 {
  float x, y, z;
} kxr_vec3;

typedef struct kxr_quat {
  float x, y, z, w;
} kxr_quat;

typedef struct kxr_pose {
  kxr_vec3 p;
  kxr_quat q;
} kxr_pose;

/* Same convention as XrFovf: angles in radians, signed (left/down negative). */
typedef struct kxr_fov {
  float left, right, up, down;
} kxr_fov;

typedef struct kxr_view {
  kxr_pose pose;
  kxr_fov fov;
} kxr_view;

typedef struct kxr_rig {
  kxr_pose kinect_in_xr; /* Kinect optical frame in XR local, after axis map */
  int locked;
  float residual_m;
  int nsamples;
} kxr_rig;

typedef struct kxr_head {
  int ok;
  kxr_vec3 p_kinect; /* HMD blob, Kinect camera frame */
  float conf;
  int n;
} kxr_head;

typedef struct kxr_session kxr_session;

kxr_session *kxr_create(void);
void kxr_destroy(kxr_session *s);

void kxr_set_tune(kxr_session *s, const cube_cloud_tune *t);
void kxr_get_tune(kxr_session *s, cube_cloud_tune *t);
void kxr_set_hide_radius(kxr_session *s, float metres);
float kxr_get_hide_radius(const kxr_session *s);
/* Uniform scale of Kinect metres → XR metres (1 = real). Re-solves lock t. */
void kxr_set_scale(kxr_session *s, float scale);
float kxr_get_scale(const kxr_session *s);
/* Replace rig + scale without re-solving the head lock (grab restore). */
void kxr_set_rig_scale(kxr_session *s, const kxr_rig *r, float scale);
/* Rotate the cloud about an XR-space pivot (degrees). Does not pin the head. */
void kxr_orbit_about(kxr_session *s, kxr_vec3 pivot, float yaw_deg, float pitch_deg);
/* Uniform scale about an XR-space pivot. Does not pin the head. */
void kxr_scale_about(kxr_session *s, kxr_vec3 pivot, float factor);
/* Translate the cloud in XR metres. */
void kxr_translate(kxr_session *s, kxr_vec3 delta);
/* Match floor→blob height in the cloud to the OpenXR head height (STAGE y). */
int kxr_autoscale_height(kxr_session *s);
/* WayVR-style: XR floor Y from a controller laid on the ground.
 * Sets scale so Kinect floor→head matches XR floor→head, then pins the head.
 * That puts the wearer's feet on the real floor in the cloud. */
int kxr_calibrate_floor(kxr_session *s, float xr_floor_y);
void kxr_clear_xr_floor(kxr_session *s);
int kxr_have_xr_floor(const kxr_session *s);
float kxr_get_xr_floor(const kxr_session *s);
float kxr_get_floor_k(const kxr_session *s);
float kxr_get_body_h(const kxr_session *s);
const char *kxr_error(const kxr_session *s);

/* Cloud in Kinect camera frame. */
int kxr_set_cloud(kxr_session *s, const cube_cloud *c);
int kxr_push_depth(kxr_session *s, const uint16_t *depth, uint32_t ts);
int kxr_load_xyz(kxr_session *s, const char *path);
int kxr_get_cloud(kxr_session *s, cube_cloud *out);
int kxr_points(const kxr_session *s);

/* "auto", "v1", "v2", or "both" (aliases accepted). Stored for the next
 * live start. A running live session reopens on a worker so the app thread
 * does not block on USB. Call from the app thread; kxr_destroy joins it. */
void kxr_set_source(kxr_session *s, const char *name);
/* 1 while that worker is reopening depth. */
int kxr_source_switching(const kxr_session *s);
/* Grid-fit Kinect 2 onto Kinect (same search as the panel). Off the app thread. */
void kxr_align_v2(kxr_session *s);
void kxr_align_reset(kxr_session *s);
/* 1 and a copy when a status line is newer than *seen. */
int kxr_poll_note(kxr_session *s, unsigned *seen, char *buf, size_t n);

/* Own the depth USB (v1, v2, or both). Fails if another owner holds the camera. */
int kxr_live_start(kxr_session *s);
void kxr_live_stop(kxr_session *s);
int kxr_live_running(const kxr_session *s);
int kxr_live_poll(kxr_session *s);
int kxr_set_tilt(kxr_session *s, double degrees);
float kxr_get_tilt(const kxr_session *s);

/* Frames. */
void kxr_map_kinect(kxr_vec3 k, kxr_vec3 *x);
void kxr_unmap_kinect(kxr_vec3 x, kxr_vec3 *k);

void kxr_pose_identity(kxr_pose *p);
void kxr_rig_identity(kxr_rig *r);
/* Place the sensor in XR: position of the depth camera, yaw (Y), pitch (X) degrees.
 * Pitch includes motor tilt (positive = look up). Applied after the axis map. */
void kxr_rig_place(kxr_rig *r, kxr_vec3 pos, float yaw_deg, float pitch_deg);
void kxr_set_rig(kxr_session *s, const kxr_rig *r);
void kxr_get_rig(const kxr_session *s, kxr_rig *r);

/* OpenXR VIEW space located in LOCAL/STAGE — the player's head, not an eye. */
void kxr_set_head(kxr_session *s, const kxr_pose *xr_head);
void kxr_get_head(const kxr_session *s, kxr_pose *xr_head);
void kxr_set_views(kxr_session *s, const kxr_view *views, int n);
int kxr_nviews(const kxr_session *s);

/* Human head from the current Kinect-frame cloud (body-shaped cluster).
 * After a lock, prefers the person whose mapped head sits on the OpenXR VIEW. */
int kxr_track_head(kxr_session *s, kxr_head *out);

/* Translation lock: keep current rotation, solve t so T*head_k == xr_head. */
int kxr_lock_head(kxr_session *s);
/* Walk samples (mapped Kinect ↔ XR head). ≥3 → full rigid via Umeyama. */
int kxr_lock_sample(kxr_session *s);
int kxr_lock_solve(kxr_session *s);
/* Two-sample yaw from a walk (horizontal deltas). */
int kxr_lock_yaw_walk(kxr_session *s);
void kxr_lock_reset(kxr_session *s);

/* Predicted XR head from the Kinect blob (after T). */
int kxr_predict_head(const kxr_session *s, kxr_vec3 *out);

/* Cloud in XR local. Drops points inside hide_radius of the XR head. */
int kxr_world_cloud(const kxr_session *s, cube_cloud *out);
/* Interleaved xyz + rgb01, n points. Ready for a GL VBO. */
int kxr_fill_vertices(const kxr_session *s, float *xyzrgb, int max_pts, int *n);
/* Solid room mesh: interleaved xyz+nrm+rgb01 (9 floats) + triangle indices. */
int kxr_fill_mesh(const kxr_session *s, float *xyznrgb, int max_v, int *nv,
                  unsigned *idx, int max_i, int *ni);
int kxr_mesh_objects(const kxr_session *s);

/* Column-major OpenGL matrices. */
void kxr_view_matrix(const kxr_pose *eye, float out16[16]);
void kxr_proj_matrix(const kxr_fov *fov, float znear, float zfar, float out16[16]);
void kxr_mvp(const kxr_session *s, int eye, float znear, float zfar, float out16[16]);

/* CPU raster through the player's eye (not orbit). Proof of real perspective. */
int kxr_render_eye(const kxr_session *s, int eye, uint8_t *rgb, int w, int h,
                   float znear, float zfar);

void kxr_status_json(const kxr_session *s, char *buf, size_t n);

#if defined(XR_TYPE_POSE_F) || defined(XR_CURRENT_API_VERSION)
static inline kxr_pose kxr_pose_from_xr(const XrPosef *p) {
  kxr_pose o;
  o.p.x = p->position.x;
  o.p.y = p->position.y;
  o.p.z = p->position.z;
  o.q.x = p->orientation.x;
  o.q.y = p->orientation.y;
  o.q.z = p->orientation.z;
  o.q.w = p->orientation.w;
  return o;
}
static inline kxr_fov kxr_fov_from_xr(const XrFovf *f) {
  kxr_fov o;
  o.left = f->angleLeft;
  o.right = f->angleRight;
  o.up = f->angleUp;
  o.down = f->angleDown;
  return o;
}
static inline kxr_view kxr_view_from_xr(const XrView *v) {
  kxr_view o;
  o.pose = kxr_pose_from_xr(&v->pose);
  o.fov = kxr_fov_from_xr(&v->fov);
  return o;
}
#endif

#ifdef __cplusplus
}
#endif
#endif
