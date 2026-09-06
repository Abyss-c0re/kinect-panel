#ifndef KINECT_IR_GRADE_H
#define KINECT_IR_GRADE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Software IR grade. gspca Kinect has no V4L gain/exposure. */
typedef struct kinect_ir_tune {
  float boost;      /* 0.5 .. 8  linear gain after stretch */
  float brightness; /* -80 .. 80 */
  float contrast;   /* 0.4 .. 3 */
  float gamma;      /* 0.35 .. 2.4  <1 lifts shadows */
  int black_pct;    /* 0 .. 25  low percentile clip */
  int white_pct;    /* 70 .. 100 high percentile clip */
  int invert;
} kinect_ir_tune;

void kinect_ir_tune_default(kinect_ir_tune *t);
void kinect_ir_tune_see(kinect_ir_tune *t); /* the “suddenly B/W and it can see” grade */

void kinect_unpack_y10b_u16(const uint8_t *buf, size_t blen, uint16_t *out, int npix);
void kinect_ir_grade(const uint16_t *in, uint8_t *out, int n, const kinect_ir_tune *t);
void kinect_ir_grey_to_rgb640(const uint8_t *grey, int w, int h, uint8_t *rgb);

#ifdef __cplusplus
}
#endif
#endif /* KINECT_IR_GRADE_H */
