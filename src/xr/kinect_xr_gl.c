#include "kinect_xr_gl.h"

#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct kxr_gl {
  GLuint prog, vao, vbo;
  GLint u_mvp, u_pm, u_vh;
  int n;
  GLuint hud_prog, hud_vao, hud_vbo, hud_tex;
  GLint hud_mvp, hud_samp;
  int hud_w, hud_h;
  GLuint line_prog, line_vao, line_vbo;
  GLint line_mvp;
  GLuint mesh_prog, mesh_vao, mesh_vbo, mesh_ebo;
  GLint mesh_mvp;
  int mesh_nv, mesh_ni;
  char err[160];
};

static const char *VS =
    "#version 330 core\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec3 a_col;\n"
    "uniform mat4 u_mvp;\n"
    "uniform float u_point_m;\n"
    "uniform float u_view_h;\n"
    "out vec3 v_col;\n"
    "void main(){\n"
    "  gl_Position = u_mvp * vec4(a_pos, 1.0);\n"
    "  float w = max(gl_Position.w, 0.08);\n"
    "  gl_PointSize = clamp(u_point_m * u_view_h * 0.55 / w, 2.0, 48.0);\n"
    "  v_col = a_col;\n"
    "}\n";

static const char *MESH_VS =
    "#version 330 core\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec3 a_nrm;\n"
    "layout(location=2) in vec3 a_col;\n"
    "uniform mat4 u_mvp;\n"
    "out vec3 v_col;\n"
    "out vec3 v_nrm;\n"
    "void main(){\n"
    "  gl_Position = u_mvp * vec4(a_pos, 1.0);\n"
    "  v_nrm = a_nrm;\n"
    "  v_col = a_col;\n"
    "}\n";

static const char *MESH_FS =
    "#version 330 core\n"
    "in vec3 v_col;\n"
    "in vec3 v_nrm;\n"
    "out vec4 o;\n"
    "void main(){\n"
    "  vec3 L = normalize(vec3(0.35, 0.82, 0.45));\n"
    "  float d = max(dot(normalize(v_nrm), L), 0.0);\n"
    "  float amb = 0.32;\n"
    "  o = vec4(v_col * (amb + 0.68 * d), 1.0);\n"
    "}\n";

static const char *HUD_VS =
    "#version 330 core\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec2 a_uv;\n"
    "uniform mat4 u_mvp;\n"
    "out vec2 v_uv;\n"
    "void main(){\n"
    "  gl_Position = u_mvp * vec4(a_pos, 1.0);\n"
    "  v_uv = a_uv;\n"
    "}\n";

static const char *HUD_FS =
    "#version 330 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_tex;\n"
    "out vec4 o;\n"
    "void main(){ o = texture(u_tex, v_uv); }\n";

static const char *LINE_VS =
    "#version 330 core\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec3 a_col;\n"
    "uniform mat4 u_mvp;\n"
    "out vec3 v_col;\n"
    "void main(){ gl_Position = u_mvp * vec4(a_pos, 1.0); v_col = a_col; }\n";

static const char *LINE_FS =
    "#version 330 core\n"
    "in vec3 v_col;\n"
    "out vec4 o;\n"
    "void main(){ o = vec4(v_col, 1.0); }\n";

static const char *FS =
    "#version 330 core\n"
    "in vec3 v_col;\n"
    "out vec4 o;\n"
    "void main(){\n"
    "  vec2 p = gl_PointCoord * 2.0 - 1.0;\n"
    "  if (dot(p,p) > 1.0) discard;\n"
    "  o = vec4(v_col, 1.0);\n"
    "}\n";

static GLuint compile(GLenum type, const char *src, char *err, size_t errn) {
  GLuint sh = glCreateShader(type);
  GLint ok = 0;
  glShaderSource(sh, 1, &src, NULL);
  glCompileShader(sh);
  glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    glGetShaderInfoLog(sh, (GLsizei)errn, NULL, err);
    glDeleteShader(sh);
    return 0;
  }
  return sh;
}

kxr_gl *kxr_gl_create(void) {
  kxr_gl *g = (kxr_gl *)calloc(1, sizeof *g);
  GLuint vs, fs;
  GLint ok = 0;
  if (!g) return NULL;
  vs = compile(GL_VERTEX_SHADER, VS, g->err, sizeof g->err);
  fs = compile(GL_FRAGMENT_SHADER, FS, g->err, sizeof g->err);
  if (!vs || !fs) {
    if (vs) glDeleteShader(vs);
    if (fs) glDeleteShader(fs);
    return g;
  }
  g->prog = glCreateProgram();
  glAttachShader(g->prog, vs);
  glAttachShader(g->prog, fs);
  glLinkProgram(g->prog);
  glDeleteShader(vs);
  glDeleteShader(fs);
  glGetProgramiv(g->prog, GL_LINK_STATUS, &ok);
  if (!ok) {
    glGetProgramInfoLog(g->prog, (GLsizei)sizeof g->err, NULL, g->err);
    return g;
  }
  g->u_mvp = glGetUniformLocation(g->prog, "u_mvp");
  g->u_pm = glGetUniformLocation(g->prog, "u_point_m");
  g->u_vh = glGetUniformLocation(g->prog, "u_view_h");
  glGenVertexArrays(1, &g->vao);
  glGenBuffers(1, &g->vbo);
  glBindVertexArray(g->vao);
  glBindBuffer(GL_ARRAY_BUFFER, g->vbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glBindVertexArray(0);

  vs = compile(GL_VERTEX_SHADER, HUD_VS, g->err, sizeof g->err);
  fs = compile(GL_FRAGMENT_SHADER, HUD_FS, g->err, sizeof g->err);
  if (vs && fs) {
    g->hud_prog = glCreateProgram();
    glAttachShader(g->hud_prog, vs);
    glAttachShader(g->hud_prog, fs);
    glLinkProgram(g->hud_prog);
    g->hud_mvp = glGetUniformLocation(g->hud_prog, "u_mvp");
    g->hud_samp = glGetUniformLocation(g->hud_prog, "u_tex");
    glGenVertexArrays(1, &g->hud_vao);
    glGenBuffers(1, &g->hud_vbo);
    glGenTextures(1, &g->hud_tex);
    glBindTexture(GL_TEXTURE_2D, g->hud_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }
  if (vs) glDeleteShader(vs);
  if (fs) glDeleteShader(fs);

  vs = compile(GL_VERTEX_SHADER, LINE_VS, g->err, sizeof g->err);
  fs = compile(GL_FRAGMENT_SHADER, LINE_FS, g->err, sizeof g->err);
  if (vs && fs) {
    g->line_prog = glCreateProgram();
    glAttachShader(g->line_prog, vs);
    glAttachShader(g->line_prog, fs);
    glLinkProgram(g->line_prog);
    g->line_mvp = glGetUniformLocation(g->line_prog, "u_mvp");
    glGenVertexArrays(1, &g->line_vao);
    glGenBuffers(1, &g->line_vbo);
  }
  if (vs) glDeleteShader(vs);
  if (fs) glDeleteShader(fs);

  vs = compile(GL_VERTEX_SHADER, MESH_VS, g->err, sizeof g->err);
  fs = compile(GL_FRAGMENT_SHADER, MESH_FS, g->err, sizeof g->err);
  if (vs && fs) {
    g->mesh_prog = glCreateProgram();
    glAttachShader(g->mesh_prog, vs);
    glAttachShader(g->mesh_prog, fs);
    glLinkProgram(g->mesh_prog);
    g->mesh_mvp = glGetUniformLocation(g->mesh_prog, "u_mvp");
    glGenVertexArrays(1, &g->mesh_vao);
    glGenBuffers(1, &g->mesh_vbo);
    glGenBuffers(1, &g->mesh_ebo);
  }
  if (vs) glDeleteShader(vs);
  if (fs) glDeleteShader(fs);
  return g;
}

void kxr_gl_destroy(kxr_gl *g) {
  if (!g) return;
  if (g->vbo) glDeleteBuffers(1, &g->vbo);
  if (g->vao) glDeleteVertexArrays(1, &g->vao);
  if (g->prog) glDeleteProgram(g->prog);
  if (g->hud_vbo) glDeleteBuffers(1, &g->hud_vbo);
  if (g->hud_vao) glDeleteVertexArrays(1, &g->hud_vao);
  if (g->hud_tex) glDeleteTextures(1, &g->hud_tex);
  if (g->hud_prog) glDeleteProgram(g->hud_prog);
  if (g->line_vbo) glDeleteBuffers(1, &g->line_vbo);
  if (g->line_vao) glDeleteVertexArrays(1, &g->line_vao);
  if (g->line_prog) glDeleteProgram(g->line_prog);
  if (g->mesh_ebo) glDeleteBuffers(1, &g->mesh_ebo);
  if (g->mesh_vbo) glDeleteBuffers(1, &g->mesh_vbo);
  if (g->mesh_vao) glDeleteVertexArrays(1, &g->mesh_vao);
  if (g->mesh_prog) glDeleteProgram(g->mesh_prog);
  free(g);
}

int kxr_gl_upload(kxr_gl *g, const float *xyzrgb, int n) {
  if (!g || !g->vbo) return -1;
  if (n < 0) n = 0;
  g->n = n;
  glBindBuffer(GL_ARRAY_BUFFER, g->vbo);
  glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)n * 6 * (GLsizeiptr)sizeof(float),
               xyzrgb, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  return 0;
}

void kxr_gl_draw(kxr_gl *g, const float mvp[16], float point_m, float view_h) {
  if (!g || !g->prog || g->n < 1) return;
  if (point_m < 0.004f) point_m = 0.004f;
  if (view_h < 64.f) view_h = 64.f;
  glUseProgram(g->prog);
  glUniformMatrix4fv(g->u_mvp, 1, GL_FALSE, mvp);
  glUniform1f(g->u_pm, point_m);
  glUniform1f(g->u_vh, view_h);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glBindVertexArray(g->vao);
  glDrawArrays(GL_POINTS, 0, g->n);
  glBindVertexArray(0);
  glUseProgram(0);
}

int kxr_gl_hud_upload(kxr_gl *g, const unsigned char *rgba, int w, int h) {
  if (!g || !g->hud_tex || !rgba || w < 1 || h < 1) return -1;
  glBindTexture(GL_TEXTURE_2D, g->hud_tex);
  if (g->hud_w != w || g->hud_h != h) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    g->hud_w = w;
    g->hud_h = h;
  } else {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
  }
  glBindTexture(GL_TEXTURE_2D, 0);
  return 0;
}

static void mul4(const float a[16], const float b[16], float o[16]) {
  int c, r, k;
  float t[16];
  for (c = 0; c < 4; c++) {
    for (r = 0; r < 4; r++) {
      float s = 0;
      for (k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      t[c * 4 + r] = s;
    }
  }
  memcpy(o, t, sizeof t);
}

void kxr_gl_hud_draw(kxr_gl *g, const float mvp[16], const float model[16], float wm,
                     float hm) {
  float hw, hh, vp[16];
  float vtx[6 * 5];
  if (!g || !g->hud_prog || !g->hud_tex || wm < 0.05f || hm < 0.05f) return;
  hw = wm * 0.5f;
  hh = hm * 0.5f;
  /* two triangles, XY plane, +Z toward user */
  vtx[0] = -hw;
  vtx[1] = -hh;
  vtx[2] = 0;
  vtx[3] = 0;
  vtx[4] = 1;
  vtx[5] = hw;
  vtx[6] = -hh;
  vtx[7] = 0;
  vtx[8] = 1;
  vtx[9] = 1;
  vtx[10] = hw;
  vtx[11] = hh;
  vtx[12] = 0;
  vtx[13] = 1;
  vtx[14] = 0;
  vtx[15] = -hw;
  vtx[16] = -hh;
  vtx[17] = 0;
  vtx[18] = 0;
  vtx[19] = 1;
  vtx[20] = hw;
  vtx[21] = hh;
  vtx[22] = 0;
  vtx[23] = 1;
  vtx[24] = 0;
  vtx[25] = -hw;
  vtx[26] = hh;
  vtx[27] = 0;
  vtx[28] = 0;
  vtx[29] = 0;
  mul4(mvp, model, vp);
  glUseProgram(g->hud_prog);
  glUniformMatrix4fv(g->hud_mvp, 1, GL_FALSE, vp);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, g->hud_tex);
  if (g->hud_samp >= 0) glUniform1i(g->hud_samp, 0);
  glBindVertexArray(g->hud_vao);
  glBindBuffer(GL_ARRAY_BUFFER, g->hud_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof vtx, vtx, GL_STREAM_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glEnable(GL_DEPTH_TEST);
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glUseProgram(0);
}

void kxr_gl_line(kxr_gl *g, const float mvp[16], const float a[3], const float b[3],
                 float r, float gc, float bcol) {
  float vtx[12];
  if (!g || !g->line_prog || !a || !b) return;
  vtx[0] = a[0];
  vtx[1] = a[1];
  vtx[2] = a[2];
  vtx[3] = r;
  vtx[4] = gc;
  vtx[5] = bcol;
  vtx[6] = b[0];
  vtx[7] = b[1];
  vtx[8] = b[2];
  vtx[9] = r;
  vtx[10] = gc;
  vtx[11] = bcol;
  glUseProgram(g->line_prog);
  glUniformMatrix4fv(g->line_mvp, 1, GL_FALSE, mvp);
  glBindVertexArray(g->line_vao);
  glBindBuffer(GL_ARRAY_BUFFER, g->line_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof vtx, vtx, GL_STREAM_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glLineWidth(2.f);
  glDrawArrays(GL_LINES, 0, 2);
  glBindVertexArray(0);
  glUseProgram(0);
}

int kxr_gl_upload_mesh(kxr_gl *g, const float *xyznrgb, int nv, const unsigned *idx,
                       int ni) {
  if (!g || !g->mesh_vbo || !xyznrgb || !idx || nv < 3 || ni < 3) {
    if (g) {
      g->mesh_nv = 0;
      g->mesh_ni = 0;
    }
    return -1;
  }
  g->mesh_nv = nv;
  g->mesh_ni = ni;
  glBindVertexArray(g->mesh_vao);
  glBindBuffer(GL_ARRAY_BUFFER, g->mesh_vbo);
  glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)nv * 9 * (GLsizeiptr)sizeof(float), xyznrgb,
               GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g->mesh_ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)ni * (GLsizeiptr)sizeof(unsigned), idx,
               GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float),
                        (void *)(6 * sizeof(float)));
  glBindVertexArray(0);
  return 0;
}

void kxr_gl_draw_mesh(kxr_gl *g, const float mvp[16]) {
  if (!g || !g->mesh_prog || g->mesh_ni < 3) return;
  glUseProgram(g->mesh_prog);
  glUniformMatrix4fv(g->mesh_mvp, 1, GL_FALSE, mvp);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glBindVertexArray(g->mesh_vao);
  glDrawElements(GL_TRIANGLES, g->mesh_ni, GL_UNSIGNED_INT, (void *)0);
  glBindVertexArray(0);
  glUseProgram(0);
}

int kxr_gl_count(const kxr_gl *g) { return g ? g->n : 0; }
const char *kxr_gl_error(const kxr_gl *g) { return g ? g->err : "null"; }
