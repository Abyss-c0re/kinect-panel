#ifndef KINECT_XR_GL_H
#define KINECT_XR_GL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct kxr_gl kxr_gl;

kxr_gl *kxr_gl_create(void);
void kxr_gl_destroy(kxr_gl *g);
int kxr_gl_upload(kxr_gl *g, const float *xyzrgb, int n);
/* point_m = splat diameter in metres; view_h = eye framebuffer height in px. */
void kxr_gl_draw(kxr_gl *g, const float mvp[16], float point_m, float view_h);
/* Solid room: xyz+nrm+rgb (9 floats/vert), triangle indices. */
int kxr_gl_upload_mesh(kxr_gl *g, const float *xyznrgb, int nv, const unsigned *idx, int ni);
void kxr_gl_draw_mesh(kxr_gl *g, const float mvp[16]);
/* World-locked RGBA panel (CubeUI-style quad). model is column-major. */
int kxr_gl_hud_upload(kxr_gl *g, const unsigned char *rgba, int w, int h);
void kxr_gl_hud_draw(kxr_gl *g, const float mvp[16], const float model[16], float wm,
                     float hm);
void kxr_gl_line(kxr_gl *g, const float mvp[16], const float a[3], const float b[3],
                 float r, float gc, float bcol);
int kxr_gl_count(const kxr_gl *g);
const char *kxr_gl_error(const kxr_gl *g);

#ifdef __cplusplus
}
#endif
#endif
