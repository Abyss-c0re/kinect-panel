/* Depth → lit triangle mesh + object colours. FACE_BLUR_LAW: no RGB faces. */
#ifndef CUBE_KINECT_MESH_H
#define CUBE_KINECT_MESH_H

#include "cube_kinect_cloud.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CUBE_MESH_MAX_V 20000
#define CUBE_MESH_MAX_I 120000
#define CUBE_MESH_MAX_OBJ 24

typedef struct cube_mesh_v {
  float x, y, z;
  float nx, ny, nz;
  uint8_t r, g, b;
  int obj;
} cube_mesh_v;

typedef struct cube_mesh {
  cube_mesh_v *v;
  unsigned *idx;
  int nv, ni, nobj;
  int ok;
  char err[160];
} cube_mesh;

void cube_mesh_init(cube_mesh *m);
void cube_mesh_clear(cube_mesh *m);
void cube_mesh_free(cube_mesh *m);

/* 640×480 FREENECT_DEPTH_11BIT → surface + connected objects. */
int cube_mesh_from_depth(cube_mesh *m, const uint16_t *depth, const cube_cloud_tune *t);

/* Orbit raster (filled tris, lit). Same camera as cube_cloud_render_rgb_ex. */
int cube_mesh_render_rgb(const cube_mesh *m, uint8_t *rgb, int w, int h, float elev_deg,
                         float azim_deg, float dist);

#ifdef __cplusplus
}
#endif
#endif
