#define _DEFAULT_SOURCE
/* Prints backend + a checksum. Does not write frames. */
#include "fg.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

int main(void) {
  fg_engine *e = fg_create();
  uint8_t a[FG_RGB], b[FG_RGB], o[FG_RGB];
  unsigned sum = 0;
  int i, n;
  if (!e) {
    printf("{\"ok\":false,\"err\":\"create\"}\n");
    return 1;
  }
  memset(a, 20, sizeof a);
  memset(b, 200, sizeof b);
  for (i = 0; i < FG_W * 40; i++) {
    int x = i % FG_W;
    a[(100 * FG_W + x) * 3] = 255;
    b[(140 * FG_W + x) * 3] = 255;
  }
  fg_set_boost(e, 1);
  fg_push_rgb(e, a);
  n = fg_push_rgb(e, b);
  if (n > 0 && fg_copy(e, 0, o) == 0)
    for (i = 0; i < FG_RGB; i++) sum += o[i];
  printf("{\"ok\":true,\"backend\":\"%s\",\"outs\":%d,\"checksum\":%u,\"collect\":false}\n",
         fg_backend(e), n, sum);
  fg_destroy(e);
  return 0;
}
