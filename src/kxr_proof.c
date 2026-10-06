#define _POSIX_C_SOURCE 200809L
#include "xr/kinect_xr.h"
#include "xr/kxr_menu.h"
#include "cube_kinect_mesh.h"
#include "kinect_fuse.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int fails;

static void expect(int ok, const char *what) {
  if (ok) {
    printf("  ok  %s\n", what);
  } else {
    printf("  FAIL %s\n", what);
    fails++;
  }
}

static void add_pt(cube_cloud *c, float x, float y, float z, uint8_t r, uint8_t g,
                   uint8_t b) {
  if (c->n >= c->cap) return;
  c->pt[c->n].x = x;
  c->pt[c->n].y = y;
  c->pt[c->n].z = z;
  c->pt[c->n].r = r;
  c->pt[c->n].g = g;
  c->pt[c->n].b = b;
  c->n++;
  c->ok = 1;
}

static int reserve(cube_cloud *c, int n) {
  cube_cloud_pt *p = (cube_cloud_pt *)realloc(c->pt, (size_t)n * sizeof *p);
  if (!p) return -1;
  c->pt = p;
  c->cap = n;
  return 0;
}

static kxr_quat quat_yaw(float deg) {
  float a = deg * (float)M_PI / 180.f;
  kxr_quat q;
  q.x = 0;
  q.y = sinf(a * 0.5f);
  q.z = 0;
  q.w = cosf(a * 0.5f);
  return q;
}

static kxr_view make_view(kxr_pose head, float yaw_deg) {
  kxr_view v;
  v.pose = head;
  if (fabsf(yaw_deg) > 0.01f) {
    kxr_quat qy = quat_yaw(yaw_deg);
    /* qy * head.q */
    kxr_quat a = qy, b = head.q, r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    v.pose.q = r;
  }
  v.fov.left = -0.85f;
  v.fov.right = 0.85f;
  v.fov.up = 0.80f;
  v.fov.down = -0.80f;
  return v;
}

static int project_pt(const float mvp[16], float x, float y, float z, float *ndc_x,
                      float *ndc_y) {
  float cx = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
  float cy = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
  float cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
  if (cw <= 1e-5f) return -1;
  *ndc_x = cx / cw;
  *ndc_y = cy / cw;
  return 0;
}

static int write_ppm(const char *path, const uint8_t *rgb, int w, int h) {
  FILE *f = fopen(path, "wb");
  if (!f) return -1;
  fprintf(f, "P6\n%d %d\n255\n", w, h);
  fwrite(rgb, 3, (size_t)w * (size_t)h, f);
  fclose(f);
  return 0;
}

/* A sitting/standing body + a bright marker 1.5 m in front of the head. */
static void synth_room(cube_cloud *c, kxr_vec3 head_k, kxr_vec3 marker_k) {
  int ix, iz, i;
  cube_cloud_init(c);
  reserve(c, 8000);
  /* floor in front of Kinect */
  for (ix = -12; ix <= 12; ix++) {
    for (iz = 4; iz <= 28; iz++) {
      add_pt(c, ix * 0.08f, -0.85f, iz * 0.10f, 70, 20, 30);
    }
  }
  /* HMD / head blob — top of a body column */
  for (i = 0; i < 40; i++) {
    float a = (float)i / 40.f * 6.28f;
    add_pt(c, head_k.x + 0.06f * cosf(a), head_k.y + 0.02f * sinf(a * 2.f),
           head_k.z + 0.06f * sinf(a), 240, 40, 70);
  }
  /* shoulders + torso + legs down to the floor (human column, not a lamp) */
  for (i = 0; i < 56; i++) {
    float t = (float)i / 55.f;
    float y = head_k.y - t * (head_k.y + 0.82f);
    float w = t < 0.18f ? 0.16f : (t < 0.55f ? 0.20f : 0.10f);
    add_pt(c, head_k.x - w * 0.5f, y, head_k.z, 180, 30, 50);
    add_pt(c, head_k.x + w * 0.5f, y, head_k.z, 180, 30, 50);
    add_pt(c, head_k.x, y, head_k.z + 0.04f, 170, 28, 48);
  }
  /* marker the player should see dead-ahead */
  for (ix = -2; ix <= 2; ix++) {
    for (iz = -2; iz <= 2; iz++) {
      add_pt(c, marker_k.x + ix * 0.02f, marker_k.y, marker_k.z + iz * 0.02f, 40,
             220, 90);
    }
  }
}

/* Taller lamp / shelf crown — old tracker locked this instead of the person. */
static void synth_lamp(cube_cloud *c, float x, float y, float z) {
  int i;
  for (i = 0; i < 36; i++) {
    float a = (float)i / 36.f * 6.28f;
    add_pt(c, x + 0.05f * cosf(a), y + 0.03f * sinf(a * 3.f), z + 0.05f * sinf(a),
           40, 40, 200);
  }
}

static int test_lock_and_perspective(void) {
  kxr_session *s = kxr_create();
  cube_cloud cloud;
  kxr_vec3 head_k = {0.15f, 0.55f, 1.70f};
  kxr_vec3 marker_k;
  kxr_pose xr_head;
  kxr_view view;
  kxr_head blob;
  kxr_vec3 pred;
  kxr_rig rig;
  float mvp[16], nx = 0, ny = 0;
  uint8_t *pix;
  int rc = 0;

  printf("lock + player-eye perspective\n");
  /* Marker 1.5 m further into the room than the head, same x/y. */
  marker_k.x = head_k.x;
  marker_k.y = head_k.y;
  marker_k.z = head_k.z + 1.50f;

  synth_room(&cloud, head_k, marker_k);
  expect(kxr_set_cloud(s, &cloud) == 0, "set synthetic cloud");
  expect(kxr_track_head(s, &blob) == 0 && blob.ok, "track HMD blob");
  expect(fabsf(blob.p_kinect.z - head_k.z) < 0.15f, "blob depth near planted head");

  kxr_pose_identity(&xr_head);
  xr_head.p.x = 0.02f;
  xr_head.p.y = 1.62f;
  xr_head.p.z = -0.05f;
  kxr_set_head(s, &xr_head);
  expect(kxr_lock_head(s) == 0, "lock translation to OpenXR head");
  kxr_get_rig(s, &rig);
  expect(rig.locked, "rig locked");
  expect(rig.residual_m < 0.03f, "lock residual < 3 cm");
  expect(kxr_predict_head(s, &pred) == 0, "predict head");
  expect(fabsf(pred.x - xr_head.p.x) < 0.03f && fabsf(pred.y - xr_head.p.y) < 0.03f &&
             fabsf(pred.z - xr_head.p.z) < 0.03f,
         "predicted head == OpenXR head");

  view = make_view(xr_head, 0);
  kxr_set_views(s, &view, 1);
  kxr_mvp(s, 0, 0.05f, 20.f, mvp);

  /* World position of the marker after T */
  {
    cube_cloud world;
    int i, hit = 0;
    float mx = 0, my = 0, mz = 0;
    cube_cloud_init(&world);
    expect(kxr_world_cloud(s, &world) == 0, "world cloud");
    for (i = 0; i < world.n; i++) {
      if (world.pt[i].g > 180) {
        mx += world.pt[i].x;
        my += world.pt[i].y;
        mz += world.pt[i].z;
        hit++;
      }
    }
    expect(hit > 4, "green marker survived world xform");
    if (hit > 0) {
      mx /= (float)hit;
      my /= (float)hit;
      mz /= (float)hit;
      expect(project_pt(mvp, mx, my, mz, &nx, &ny) == 0, "project marker");
      printf("    marker_world=(%.3f,%.3f,%.3f) xr_head=(%.3f,%.3f,%.3f) ndc=(%.3f,%.3f)\n",
             mx, my, mz, xr_head.p.x, xr_head.p.y, xr_head.p.z, nx, ny);
      expect(fabsf(nx) < 0.12f && fabsf(ny) < 0.12f,
             "looking forward: marker at image centre (real perspective)");
    }
    cube_cloud_free(&world);
  }

  /* Yaw 30° — same marker must leave the centre. Orbit would keep it centred. */
  view = make_view(xr_head, 30.f);
  kxr_set_views(s, &view, 1);
  kxr_mvp(s, 0, 0.05f, 20.f, mvp);
  {
    cube_cloud world;
    int i, hit = 0;
    float mx = 0, my = 0, mz = 0;
    cube_cloud_init(&world);
    kxr_world_cloud(s, &world);
    for (i = 0; i < world.n; i++) {
      if (world.pt[i].g > 180) {
        mx += world.pt[i].x;
        my += world.pt[i].y;
        mz += world.pt[i].z;
        hit++;
      }
    }
    if (hit > 0) {
      mx /= (float)hit;
      my /= (float)hit;
      mz /= (float)hit;
      project_pt(mvp, mx, my, mz, &nx, &ny);
      printf("    yaw30 ndc=(%.3f,%.3f)\n", nx, ny);
      expect(fabsf(nx) > 0.20f, "yaw 30°: marker leaves centre (not orbit)");
    }
    cube_cloud_free(&world);
  }

  pix = (uint8_t *)malloc(640 * 480 * 3);
  if (pix) {
    view = make_view(xr_head, 0);
    kxr_set_views(s, &view, 1);
    if (kxr_render_eye(s, 0, pix, 640, 480, 0.05f, 20.f) == 0)
      write_ppm("/tmp/kxr_eye_forward.ppm", pix, 640, 480);
    view = make_view(xr_head, 90.f);
    kxr_set_views(s, &view, 1);
    if (kxr_render_eye(s, 0, pix, 640, 480, 0.05f, 20.f) == 0)
      write_ppm("/tmp/kxr_eye_yaw30.ppm", pix, 640, 480);
    free(pix);
    printf("  wrote /tmp/kxr_eye_forward.ppm /tmp/kxr_eye_yaw30.ppm\n");
  }

  cube_cloud_free(&cloud);
  kxr_destroy(s);
  return rc;
}

static int test_orbit_scale_about_head(void) {
  kxr_session *s = kxr_create();
  cube_cloud cloud;
  kxr_vec3 head_k = {0.12f, 0.55f, 1.80f};
  kxr_vec3 marker_k = {0.12f, 0.55f, 3.20f};
  kxr_pose xr;
  kxr_vec3 pred, pivot;
  kxr_rig rig;
  float sc0;
  printf("orbit / scale about XR head (grip grab)\n");
  synth_room(&cloud, head_k, marker_k);
  kxr_set_cloud(s, &cloud);
  kxr_pose_identity(&xr);
  xr.p.x = 0.01f;
  xr.p.y = 1.60f;
  xr.p.z = -0.02f;
  kxr_set_head(s, &xr);
  expect(kxr_lock_head(s) == 0, "lock");
  expect(kxr_predict_head(s, &pred) == 0, "predict before grab");
  expect(fabsf(pred.x - xr.p.x) < 0.03f && fabsf(pred.z - xr.p.z) < 0.03f,
         "head pinned before grab");
  pivot = xr.p;
  kxr_orbit_about(s, pivot, 35.f, 0.f);
  expect(kxr_predict_head(s, &pred) == 0, "predict after yaw");
  expect(fabsf(pred.x - xr.p.x) < 0.04f && fabsf(pred.y - xr.p.y) < 0.04f &&
             fabsf(pred.z - xr.p.z) < 0.04f,
         "yaw about head keeps the wearer pinned");
  sc0 = kxr_get_scale(s);
  kxr_scale_about(s, pivot, 1.25f);
  expect(fabsf(kxr_get_scale(s) - sc0 * 1.25f) < 0.02f, "scale factor applied");
  expect(kxr_predict_head(s, &pred) == 0, "predict after zoom");
  expect(fabsf(pred.x - xr.p.x) < 0.05f && fabsf(pred.z - xr.p.z) < 0.05f,
         "zoom about head keeps the wearer pinned");
  {
    kxr_vec3 d = {0.10f, 0, 0};
    kxr_translate(s, d);
    expect(kxr_predict_head(s, &pred) == 0, "predict after translate");
    expect(fabsf(pred.x - (xr.p.x + 0.10f)) < 0.05f, "translate moves the cloud");
  }
  kxr_get_rig(s, &rig);
  expect(rig.locked, "still marked locked");
  cube_cloud_free(&cloud);
  kxr_destroy(s);
  return 0;
}

static int test_mesh_from_plane(void) {
  cube_mesh mesh;
  uint16_t *dep;
  int i;
  cube_cloud_tune t;
  printf("solid 3D mesh from depth plane\n");
  cube_mesh_init(&mesh);
  cube_cloud_tune_default(&t);
  t.color = 3;
  t.z_near = 0.4f;
  t.z_far = 4.f;
  dep = (uint16_t *)calloc(640 * 480, sizeof *dep);
  expect(dep != NULL, "alloc depth");
  if (dep) {
    for (i = 0; i < 640 * 480; i++) dep[i] = 900;
    expect(cube_mesh_from_depth(&mesh, dep, &t) == 0, "mesh from plane");
    expect(mesh.nv > 100 && mesh.ni > 100, "plane has verts and triangles");
    printf("    nv=%d ni=%d nobj=%d\n", mesh.nv, mesh.ni, mesh.nobj);
    free(dep);
  }
  cube_mesh_free(&mesh);
  return 0;
}

static int test_floor_calib(void) {
  kxr_session *s = kxr_create();
  cube_cloud cloud;
  kxr_vec3 head_k = {0.10f, 0.58f, 1.80f};
  kxr_vec3 marker_k = {0.10f, 0.58f, 3.10f};
  kxr_pose xr;
  kxr_vec3 pred;
  float sc;
  printf("floor calib (controller on the floor)\n");
  synth_room(&cloud, head_k, marker_k);
  kxr_set_cloud(s, &cloud);
  kxr_pose_identity(&xr);
  xr.p.y = 1.64f;
  kxr_set_head(s, &xr);
  expect(kxr_track_head(s, NULL) == 0, "track before floor");
  expect(kxr_calibrate_floor(s, 0.02f) == 0, "calibrate floor at y=0.02");
  expect(kxr_have_xr_floor(s), "xr floor stored");
  sc = kxr_get_scale(s);
  printf("    scale=%.3f body_h=%.3f floor_k=%.3f\n", sc, kxr_get_body_h(s),
         kxr_get_floor_k(s));
  expect(sc > 1.05f && sc < 1.40f, "scale matches XR eye / Kinect body");
  expect(kxr_predict_head(s, &pred) == 0, "predict after floor");
  expect(fabsf(pred.y - xr.p.y) < 0.05f, "head still pinned after floor scale");
  cube_cloud_free(&cloud);
  kxr_destroy(s);
  return 0;
}

static int test_human_not_lamp(void) {
  kxr_session *s = kxr_create();
  cube_cloud cloud;
  kxr_vec3 head_k = {0.10f, 0.58f, 1.85f};
  kxr_vec3 marker_k = {0.10f, 0.58f, 3.20f};
  kxr_head blob;
  printf("human shape vs taller lamp\n");
  synth_room(&cloud, head_k, marker_k);
  synth_lamp(&cloud, -0.70f, 1.25f, 2.10f);
  expect(kxr_set_cloud(s, &cloud) == 0, "set room + lamp");
  expect(kxr_track_head(s, &blob) == 0 && blob.ok, "track human");
  printf("    blob=(%.3f,%.3f,%.3f) want=(%.3f,%.3f,%.3f) conf=%.2f n=%d\n",
         blob.p_kinect.x, blob.p_kinect.y, blob.p_kinect.z, head_k.x, head_k.y,
         head_k.z, blob.conf, blob.n);
  expect(fabsf(blob.p_kinect.x - head_k.x) < 0.20f &&
             fabsf(blob.p_kinect.z - head_k.z) < 0.22f,
         "picked the person, not the taller lamp");
  expect(blob.p_kinect.y < 1.00f, "head height is the person crown, not the lamp");
  cube_cloud_free(&cloud);
  kxr_destroy(s);
  return 0;
}

static int test_umeyama(void) {
  kxr_session *s = kxr_create();
  cube_cloud cloud;
  kxr_vec3 heads_k[4] = {
      {0.00f, 0.50f, 1.60f},
      {0.40f, 0.52f, 1.70f},
      {-0.30f, 0.48f, 2.00f},
      {0.20f, 0.55f, 2.20f},
  };
  kxr_pose xr;
  kxr_rig rig;
  int i;
  printf("umeyama multi-sample rigid\n");
  kxr_pose_identity(&xr);
  xr.p.y = 1.6f;
  /* Plant a blob at each sample, lock_sample, then solve. */
  for (i = 0; i < 4; i++) {
    kxr_vec3 mapped, want;
    synth_room(&cloud, heads_k[i], heads_k[i]);
    kxr_set_cloud(s, &cloud);
    /* Known extra yaw 20° + translation after axis map */
    kxr_map_kinect(heads_k[i], &mapped);
    {
      float a = 20.f * (float)M_PI / 180.f;
      float c = cosf(a), s0 = sinf(a);
      want.x = c * mapped.x + s0 * mapped.z + 0.30f;
      want.y = mapped.y + 1.10f;
      want.z = -s0 * mapped.x + c * mapped.z - 0.20f;
    }
    xr.p = want;
    kxr_set_head(s, &xr);
    expect(kxr_lock_sample(s) == 0, "sample");
    cube_cloud_free(&cloud);
  }
  expect(kxr_lock_solve(s) == 0, "solve rigid");
  kxr_get_rig(s, &rig);
  expect(rig.locked && rig.nsamples >= 3, "solved with ≥3 samples");
  printf("    umeyama residual=%.4fm samples=%d\n", rig.residual_m, rig.nsamples);
  expect(rig.residual_m < 0.08f, "umeyama residual < 8 cm");
  kxr_destroy(s);
  return 0;
}

static int test_real_xyz(const char *path) {
  kxr_session *s = kxr_create();
  kxr_head blob;
  kxr_pose xr;
  kxr_view view;
  uint8_t *pix;
  char js[512];
  printf("real xyz %s\n", path);
  if (kxr_load_xyz(s, path) != 0) {
    printf("  skip (%s)\n", kxr_error(s));
    kxr_destroy(s);
    return 0;
  }
  expect(kxr_points(s) > 100, "loaded points");
  if (kxr_track_head(s, &blob) != 0) {
    printf("  no blob (%s) — still render from identity rig\n", kxr_error(s));
  } else {
    kxr_pose_identity(&xr);
    xr.p.y = 1.6f;
    kxr_set_head(s, &xr);
    expect(kxr_lock_head(s) == 0, "lock real cloud to seated origin");
  }
  kxr_pose_identity(&xr);
  if (blob.ok) {
    /* stay at locked head */
  } else {
    xr.p.y = 1.2f;
    xr.p.z = 0.0f;
    kxr_set_head(s, &xr);
  }
  kxr_get_head(s, &xr);
  view = make_view(xr, 0);
  kxr_set_views(s, &view, 1);
  pix = (uint8_t *)malloc(960 * 540 * 3);
  if (pix && kxr_render_eye(s, 0, pix, 960, 540, 0.08f, 8.f) == 0) {
    write_ppm("/tmp/kxr_real_eye.ppm", pix, 960, 540);
    printf("  wrote /tmp/kxr_real_eye.ppm n=%d\n", kxr_points(s));
  }
  free(pix);
  kxr_status_json(s, js, sizeof js);
  printf("  %s", js);
  kxr_destroy(s);
  return 0;
}

static float menu_band_y(const kxr_menu *m, int row) {
  float v = ((float)KXR_MENU_Y0 + (float)row * (float)KXR_MENU_PITCH + 10.f) /
            (float)KXR_MENU_HGT;
  return (0.5f - v) * m->height_m;
}

static int menu_ray_row(const kxr_menu *m, int want, int *got) {
  kxr_vec3 o, d;
  o.x = 0.f;
  o.y = menu_band_y(m, want);
  o.z = 0.8f;
  d.x = 0.f;
  d.y = 0.f;
  d.z = -1.f;
  return kxr_menu_ray(m, o, d, got, NULL, NULL, NULL, NULL);
}

static int test_menu_follows_panel(void) {
  kxr_menu menu;
  kxr_menu_snap snap;
  kxr_session *s;
  ksync_rigid rig;
  char note[160];
  unsigned seen = 0;
  int row = -1, i, pixels = 0, got = 0;
  struct timespec t0, t1;
  const char *old;
  char saved[512];
  double dt;
  printf("headset menu follows panel\n");
  clock_gettime(CLOCK_MONOTONIC, &t0);

  kxr_menu_init(&menu);
  expect(menu.rgba != NULL, "menu bitmap");
  expect(fabsf(menu.height_m - menu.width_m * (float)KXR_MENU_HGT / (float)KXR_MENU_W) <
             1e-5f,
         "menu height matches texture");
  expect(KXR_MENU_ROWS == 20, "sensor rows added");
  menu.open = 1;
  menu.have_pose = 1;
  kxr_pose_identity(&menu.pose);
  expect(menu_ray_row(&menu, 0, &row) == 1 && row == 0, "ray hits near");
  expect(menu_ray_row(&menu, 16, &row) == 1 && row == 16, "ray hits sensor");
  expect(menu_ray_row(&menu, 17, &row) == 1 && row == 17, "ray hits align v2");
  expect(menu_ray_row(&menu, 18, &row) == 1 && row == 18, "ray hits reset align");
  expect(menu_ray_row(&menu, 19, &row) == 1 && row == 19, "ray hits close");
  expect(kxr_menu_click_row(&menu, 16, 1) == KXR_ACT_SRC_P, "sensor cycles forward");
  expect(kxr_menu_click_row(&menu, 16, -1) == KXR_ACT_SRC_N, "sensor cycles back");
  expect(kxr_menu_click_row(&menu, 17, 1) == KXR_ACT_ALIGN_V2, "align v2 action");
  expect(kxr_menu_click_row(&menu, 18, 1) == KXR_ACT_ALIGN_RESET, "reset align action");
  expect(kxr_menu_click_row(&menu, 19, 1) == KXR_ACT_CLOSE, "close stays last");
  expect(kxr_menu_click_row(&menu, 14, 1) == KXR_ACT_SOLID, "solid row unchanged");
  menu.open = 1;
  menu.row = 0;
  kxr_menu_nav(&menu, 0, -1, 0);
  expect(menu.row == KXR_MENU_ROWS - 1, "stick wraps onto close");

  memset(&snap, 0, sizeof snap);
  snap.src = 2;
  snap.identity_rigid = 1;
  snap.points = 12;
  kxr_menu_paint(&menu, &snap);
  for (i = 0; i < KXR_MENU_W * KXR_MENU_HGT; i++) {
    const uint8_t *p = menu.rgba + (size_t)i * 4;
    if (p[0] == 242 && p[1] == 80 && p[2] == 100) pixels++;
  }
  expect(pixels > 40, "sensor value painted");
  kxr_menu_free(&menu);

  s = kxr_create();
  expect(s != NULL, "session");
  kxr_set_source(s, "v2");
  expect(kinect_fuse_source() == KINECT_SRC_V2, "source stored as kinect 2");
  expect(!kxr_live_running(s), "setting source does not open usb");
  expect(!kxr_source_switching(s), "dark session does not reopen");
  expect(kxr_poll_note(s, &seen, note, sizeof note) == 1, "sensor note");
  expect(strcmp(note, "sensor KINECT 2") == 0, "note names kinect 2");
  kxr_set_source(s, "both");
  expect(kinect_fuse_source() == KINECT_SRC_BOTH, "source stored as both");
  kxr_set_source(s, "not-a-kinect");
  expect(kinect_fuse_source() == KINECT_SRC_BOTH, "unknown source kept");
  for (;;) {
    if (!kxr_poll_note(s, &seen, note, sizeof note)) break;
  }

  old = getenv("KINECT_EXTRINSIC");
  saved[0] = 0;
  if (old && old[0]) snprintf(saved, sizeof saved, "%s", old);
  if (setenv("KINECT_EXTRINSIC", "/tmp/kxr-proof-extrinsic.txt", 1) == 0) {
    kxr_align_reset(s);
    kinect_fuse_get_rigid(&rig);
    expect(ksync_rigid_is_identity(&rig), "reset align is identity");
  } else {
    expect(0, "redirect extrinsic away from the saved fit");
  }
  kxr_align_v2(s);
  for (i = 0; i < 100 && !got; i++) {
    if (kxr_poll_note(s, &seen, note, sizeof note) && strstr(note, "need both")) got = 1;
    else {
      struct timespec req;
      req.tv_sec = 0;
      req.tv_nsec = 10000000L;
      nanosleep(&req, NULL);
    }
  }
  expect(got, "align v2 asks for both clouds");
  kxr_destroy(s);
  if (saved[0]) setenv("KINECT_EXTRINSIC", saved, 1);
  else unsetenv("KINECT_EXTRINSIC");
  remove("/tmp/kxr-proof-extrinsic.txt");

  clock_gettime(CLOCK_MONOTONIC, &t1);
  dt = (double)(t1.tv_sec - t0.tv_sec) + (double)(t1.tv_nsec - t0.tv_nsec) / 1e9;
  expect(dt < 3.0, "menu test did not open usb");
  return 0;
}

int main(int argc, char **argv) {
  const char *xyz = NULL;
  int i;
  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--xyz") == 0 && i + 1 < argc)
      xyz = argv[++i];
    else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      fprintf(stderr, "kxr_proof — kinect_xr selftest\n  kxr_proof [--xyz PATH]\n");
      return 0;
    }
  }
  printf("kxr_proof\n");
  test_lock_and_perspective();
  test_orbit_scale_about_head();
  test_mesh_from_plane();
  test_floor_calib();
  test_human_not_lamp();
  test_umeyama();
  test_menu_follows_panel();
  if (xyz)
    test_real_xyz(xyz);
  else
    test_real_xyz("/home/voldemar/Dev/lab/prophecy_cube/state/trackers/kinect_latest.xyz");
  printf("{\"ok\":%s,\"fails\":%d,\"lib\":\"kinect_xr\"}\n", fails ? "false" : "true",
         fails);
  return fails ? 1 : 0;
}
