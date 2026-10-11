/* Offline checks for Kinect v1+v2 pairing, rigid align, and the shared pinhole. */
#include "kinect_sync.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;

static void expect(int cond, const char *msg) {
  if (cond) return;
  fprintf(stderr, "FAIL %s\n", msg);
  fails++;
}

static void fill_room(cube_cloud *c) {
  float x, y, z;
  cube_cloud_reserve(c, 4000);
  for (x = -1.f; x <= 1.f; x += 0.1f) {
    for (z = 1.f; z <= 2.6f; z += 0.1f) {
      cube_cloud_pt *p = &c->pt[c->n++];
      p->x = x;
      p->y = 0.f;
      p->z = z;
      p->r = 200;
      p->g = 40;
      p->b = 40;
    }
  }
  for (x = -1.f; x <= 1.f; x += 0.1f) {
    for (y = 0.f; y <= 1.4f; y += 0.14f) {
      cube_cloud_pt *p = &c->pt[c->n++];
      p->x = x;
      p->y = y;
      p->z = 2.6f;
      p->r = 200;
      p->g = 80;
      p->b = 40;
    }
  }
  for (z = 1.f; z <= 2.6f; z += 0.1f) {
    for (y = 0.f; y <= 1.4f; y += 0.14f) {
      cube_cloud_pt *p = &c->pt[c->n++];
      p->x = -1.f;
      p->y = y;
      p->z = z;
      p->r = 180;
      p->g = 60;
      p->b = 50;
    }
  }
  c->ok = c->n > 0;
}

static void rot_y(float deg, float x, float y, float z, float *ox, float *oy, float *oz) {
  float a = deg * (float)M_PI / 180.f;
  float c = cosf(a), s = sinf(a);
  *ox = c * x + s * z;
  *oy = y;
  *oz = -s * x + c * z;
}

static float nearest(const cube_cloud *c, float x, float y, float z) {
  int i;
  float best = 1e30f;
  for (i = 0; i < c->n; i++) {
    float dx = c->pt[i].x - x, dy = c->pt[i].y - y, dz = c->pt[i].z - z;
    float d = dx * dx + dy * dy + dz * dz;
    if (d < best) best = d;
  }
  return best;
}

int main(void) {
  uint64_t t1[3] = {1000, 34000, 67000};
  uint64_t t2[3] = {8000, 42000, 75000};
  int i1 = -1, i2 = -1;
  uint64_t skew = 0;
  ksync_rigid id, rig;
  cube_cloud room, moved, merged, proj;
  ksync_align_result ar;
  memset(&ar, 0, sizeof ar);
  cube_cloud_tune tune;
  float zmap[16];
  float x = 1, y = 2, z = 3;
  char path[] = "/tmp/kinect2_extrinsic_proof.txt";
  double acc = 0;
  int i;

  expect(ksync_best_pair(t1, 3, t2, 3, 80000, &i1, &i2, &skew) == 0, "pair exists");
  expect(i1 == 0 && i2 == 0 && skew == 7000, "closest stamps");
  expect(ksync_best_pair(t1, 3, t2, 3, 900000, &i1, &i2, &skew) != 0, "stale pair dropped");

  ksync_rigid_identity(&id);
  ksync_rigid_apply(&id, &x, &y, &z);
  expect(x == 1.f && y == 2.f && z == 3.f, "identity apply");
  expect(ksync_rigid_is_identity(&id), "identity detect");
  id.t[0] = 0.25f;
  ksync_rigid_apply(&id, &x, &y, &z);
  expect(fabsf(x - 1.25f) < 1e-5f, "translate x");

  expect(ksync_cover_stride(640, 480, 2, 100000) == 2, "stride stays");
  expect(ksync_cover_stride(640, 480, 2, 1000) > 2, "stride grows to fit");

  cube_cloud_init(&room);
  cube_cloud_init(&moved);
  cube_cloud_init(&merged);
  cube_cloud_init(&proj);
  fill_room(&room);
  cube_cloud_reserve(&moved, room.n);
  for (i = 0; i < room.n; i++) {
    cube_cloud_pt *p = &moved.pt[moved.n++];
    *p = room.pt[i];
    rot_y(12.f, room.pt[i].x, room.pt[i].y, room.pt[i].z, &p->x, &p->y, &p->z);
    p->x += 0.22f;
    p->y += -0.03f;
    p->z += 0.08f;
  }
  moved.ok = 1;
  ksync_rigid_identity(&rig);
  expect(ksync_align(&rig, &ar, &room, &moved) == 0, "align converges");
  if (ar.ok) {
    fprintf(stderr, "align residual=%.4f inliers=%d yaw=%.2f pitch=%.2f\n", ar.residual_m,
            ar.inliers, ar.yaw_deg, ar.pitch_deg);
    expect(ar.residual_m < 0.03f, "align residual");
    expect(fabsf(ar.yaw_deg + 12.f) < 3.f, "align yaw");
  }
  for (i = 0; i < moved.n; i += 5) {
    float px = moved.pt[i].x, py = moved.pt[i].y, pz = moved.pt[i].z;
    ksync_rigid_apply(&rig, &px, &py, &pz);
    acc += sqrt((double)nearest(&room, px, py, pz));
  }
  acc /= (double)(moved.n / 5);
  fprintf(stderr, "cloud residual=%.4f m\n", acc);
  expect(acc < 0.03, "mapped cloud sits on the kinect");

  {
    cube_cloud face;
    ksync_rigid face_rig;
    ksync_align_result face_ar;
    cube_cloud_init(&face);
    cube_cloud_reserve(&face, room.n);
    for (i = 0; i < room.n; i++) {
      cube_cloud_pt *p = &face.pt[face.n++];
      *p = room.pt[i];
      rot_y(180.f, room.pt[i].x, room.pt[i].y, room.pt[i].z, &p->x, &p->y, &p->z);
      p->x += 0.35f;
      p->y += -0.04f;
      p->z += 0.15f;
    }
    face.ok = 1;
    ksync_rigid_identity(&face_rig);
    memset(&face_ar, 0, sizeof face_ar);
    expect(ksync_align(&face_rig, &face_ar, &room, &face) == 0, "facing align converges");
    if (face_ar.ok) {
      fprintf(stderr, "facing residual=%.4f inliers=%d yaw=%.2f pitch=%.2f\n", face_ar.residual_m,
              face_ar.inliers, face_ar.yaw_deg, face_ar.pitch_deg);
      expect(face_ar.residual_m < 0.05f, "facing residual");
      expect(fabsf(fabsf(face_ar.yaw_deg) - 180.f) < 8.f, "facing yaw");
    }
    cube_cloud_free(&face);
  }

  /* Two views of one corridor from opposite ends. Each keeps its own near
   * half plus the shared middle, so the centroids are not the same point. */
  {
    cube_cloud near_end, far_end;
    ksync_rigid got;
    ksync_align_result room_ar;
    float x, y, z;
    cube_cloud_init(&near_end);
    cube_cloud_init(&far_end);
    cube_cloud_reserve(&near_end, 4000);
    cube_cloud_reserve(&far_end, 4000);
    for (x = -1.0f; x <= 1.01f; x += 0.2f) {
      for (z = 0.8f; z <= 5.21f; z += 0.2f) {
        cube_cloud_pt p;
        p.x = x;
        p.y = 0.f;
        p.z = z;
        p.r = p.g = p.b = 0;
        if (z <= 3.6f) near_end.pt[near_end.n++] = p;
        if (z >= 2.2f) {
          cube_cloud_pt q = p;
          rot_y(180.f, p.x - 0.12f, p.y - 0.2f, p.z - 5.0f, &q.x, &q.y, &q.z);
          if (q.z >= 0.6f && q.z <= 3.8f) far_end.pt[far_end.n++] = q;
        }
      }
    }
    for (y = 0.2f; y <= 1.41f; y += 0.2f) {
      for (z = 0.8f; z <= 5.21f; z += 0.2f) {
        cube_cloud_pt p;
        p.x = -1.f;
        p.y = y;
        p.z = z;
        p.r = p.g = p.b = 0;
        if (z <= 3.6f) near_end.pt[near_end.n++] = p;
        if (z >= 2.2f) {
          cube_cloud_pt q = p;
          rot_y(180.f, p.x - 0.12f, p.y - 0.2f, p.z - 5.0f, &q.x, &q.y, &q.z);
          if (q.z >= 0.6f && q.z <= 3.8f) far_end.pt[far_end.n++] = q;
        }
      }
    }
    for (x = -1.f; x <= 1.01f; x += 0.25f) {
      for (y = 0.f; y <= 1.21f; y += 0.25f) {
        cube_cloud_pt p;
        p.x = x;
        p.y = y;
        p.z = 0.7f;
        p.r = p.g = p.b = 40;
        near_end.pt[near_end.n++] = p;
        p.z = 5.1f;
        rot_y(180.f, p.x - 0.12f, p.y - 0.2f, p.z - 5.0f, &p.x, &p.y, &p.z);
        far_end.pt[far_end.n++] = p;
      }
    }
    near_end.ok = near_end.n > 30;
    far_end.ok = far_end.n > 30;
    ksync_rigid_identity(&got);
    memset(&room_ar, 0, sizeof room_ar);
    expect(ksync_align(&got, &room_ar, &near_end, &far_end) == 0, "corridor align");
    if (room_ar.ok) {
      fprintf(stderr, "corridor residual=%.4f inliers=%d yaw=%.2f pitch=%.2f t=%.2f %.2f %.2f\n",
              room_ar.residual_m, room_ar.inliers, room_ar.yaw_deg, room_ar.pitch_deg, got.t[0],
              got.t[1], got.t[2]);
      expect(fabsf(fabsf(room_ar.yaw_deg) - 180.f) < 10.f, "corridor yaw");
      expect(room_ar.residual_m < 0.08f, "corridor residual");
      /* A flat corridor is unchanged along its length, so the fit may sit
       * a metre off the true baseline. It still has to face the other way
       * and keep the cameras metres apart. */
      expect(fabsf(got.t[2]) > 2.5f, "corridor baseline");
    }
    cube_cloud_free(&near_end);
    cube_cloud_free(&far_end);
  }

  expect(ksync_rigid_save(&rig, path) == 0, "save rigid");
  ksync_rigid_identity(&id);
  expect(ksync_rigid_load(&id, path) == 0, "load rigid");
  expect(fabsf(id.t[0] - rig.t[0]) < 1e-5f && fabsf(id.R[0] - rig.R[0]) < 1e-5f, "roundtrip");

  ksync_tint_v2(&moved);
  expect(moved.pt[0].b > moved.pt[0].r, "v2 tint is blue");
  expect(ksync_merge(&merged, &room, &moved, &rig, 50) == 0, "merge");
  expect(merged.n > 0 && merged.n <= 50, "merge cap");

  memset(zmap, 0, sizeof zmap);
  zmap[2 * 4 + 2] = 2.f;
  cube_cloud_tune_default(&tune);
  tune.stride = 1;
  tune.z_near = 0.4f;
  tune.z_far = 4.f;
  expect(ksync_project_depth(&proj, zmap, 4, 4, 100.f, 100.f, 1.5f, 1.5f, &tune, 10) == 0,
         "project");
  expect(proj.n == 1, "one point");
  expect(fabsf(proj.pt[0].x - 0.01f) < 1e-4f, "pinhole x");
  expect(fabsf(proj.pt[0].y + 0.01f) < 1e-4f, "y up");
  expect(fabsf(proj.pt[0].z - 2.f) < 1e-4f, "range");

  cube_cloud_free(&room);
  cube_cloud_free(&moved);
  cube_cloud_free(&merged);
  cube_cloud_free(&proj);
  printf("{\"ok\":%s,\"fails\":%d,\"lib\":\"kinect_sync\"}\n", fails ? "false" : "true", fails);
  return fails ? 1 : 0;
}
