/* Loopback proof: two processes, no USB. Each offers a tiny depth frame and
 * an RGB preview. The other side must rebuild both, including a one-block
 * delta and the Kinect 2 intrinsics packet. */
#include "kinect_net.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define DW 64
#define DH 48

static uint64_t mono_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void fill_depth(uint16_t *d, int sensor, int phase) {
  int x, y;
  memset(d, 0, (size_t)DW * DH * sizeof(uint16_t));
  if (sensor == 1) {
    for (y = 8; y < 16; y++)
      for (x = 8; x < 16; x++) d[y * DW + x] = (uint16_t)(500 + x);
    for (y = 24; y < 32; y++)
      for (x = 32; x < 40; x++) d[y * DW + x] = (uint16_t)(phase ? 1500 : 900);
  } else {
    for (y = 16; y < 24; y++)
      for (x = 16; x < 24; x++) d[y * DW + x] = (uint16_t)(2000 + x);
  }
}

static void fill_rgb(uint8_t *rgb, int who) {
  memset(rgb, 0, 640 * 480 * 3);
  if (who == 1) {
    rgb[(40 * 640 + 32) * 3] = 220;
    rgb[(40 * 640 + 32) * 3 + 1] = 40;
    rgb[(40 * 640 + 32) * 3 + 2] = 80;
  } else {
    rgb[(60 * 640 + 48) * 3] = 10;
    rgb[(60 * 640 + 48) * 3 + 1] = 200;
    rgb[(60 * 640 + 48) * 3 + 2] = 90;
  }
}

static int near_u8(int got, int want) {
  int d = got - want;
  if (d < 0) d = -d;
  return d <= 12;
}

static int run_side(int child, int out_fd) {
  uint16_t *depth = (uint16_t *)calloc((size_t)DW * DH, sizeof(uint16_t));
  uint16_t *got = (uint16_t *)calloc(640 * 480, sizeof(uint16_t));
  uint8_t *rgb = (uint8_t *)calloc(640 * 480 * 3, 1);
  uint8_t *rgb_got = (uint8_t *)calloc(640 * 480 * 3, 1);
  uint64_t t0 = mono_us();
  int depth_ok = 0, delta_ok = 0, rgb_ok = 0, cal_ok = 0, peer_ok = 0, rtt_ok = 0;
  int rc = 1;
  char msg[180];
  if (!depth || !got || !rgb || !rgb_got) {
    snprintf(msg, sizeof msg, "oom\n");
    goto done;
  }
  if (child) {
    kinect_net_set_local(1);
    if (kinect_net_listen(18765) != 0) {
      snprintf(msg, sizeof msg, "listen: %s\n", kinect_net_error());
      goto done;
    }
  } else {
    usleep(80000);
    kinect_net_set_local(2);
    if (kinect_net_connect("127.0.0.1:18765") != 0) {
      snprintf(msg, sizeof msg, "connect: %s\n", kinect_net_error());
      goto done;
    }
  }
  while (mono_us() - t0 < 4000000ull) {
    uint64_t now = mono_us();
    uint64_t age = now - t0;
    int phase = age > 700000ull;
    int w = 0, h = 0;
    uint64_t ts = 0;
    uint32_t seq = 0;
    int other = child ? 2 : 1;
    fill_depth(depth, child ? 1 : 2, phase);
    fill_rgb(rgb, child ? 1 : 2);
    if (age < 1500000ull) {
      kinect_net_want(1);
      kinect_net_offer_depth(child ? 1 : 2, depth, DW, DH, 0, now);
    } else {
      kinect_net_want(2);
      kinect_net_offer_rgb(rgb, child ? 1 : 2, now);
    }
    if (!child) kinect_net_offer_intrinsics(2, 366.5f, 366.25f, 256.f, 212.f);
    if (kinect_net_take_depth(other, got, 640 * 480, &w, &h, &ts, &seq) == 1 && w == DW && h == DH) {
      if (child) {
        if (got[16 * DW + 16] == 2016) depth_ok = 1;
      } else {
        if (got[8 * DW + 8] == 508) depth_ok = 1;
        if (got[24 * DW + 32] == 1500) delta_ok = 1;
      }
    }
    if (age > 1500000ull && kinect_net_take_rgb(rgb_got, &ts) == 1) {
      if (child) {
        uint8_t *p = rgb_got + (60 * 640 + 48) * 3;
        if (near_u8(p[0], 10) && near_u8(p[1], 200) && near_u8(p[2], 90)) rgb_ok = 1;
      } else {
        uint8_t *p = rgb_got + (40 * 640 + 32) * 3;
        if (near_u8(p[0], 220) && near_u8(p[1], 40) && near_u8(p[2], 80)) rgb_ok = 1;
      }
    }
    if (child) {
      float fx = 0, fy = 0, cx = 0, cy = 0;
      if (kinect_net_take_intrinsics(2, &fx, &fy, &cx, &cy) && fx > 366.f && fx < 367.f &&
          fy > 366.f && fy < 367.f && cx > 255.f && cx < 257.f && cy > 211.f && cy < 213.f)
        cal_ok = 1;
    } else {
      cal_ok = 1;
    }
    if (kinect_net_connected() && (kinect_net_peer_sensors() & (child ? 2 : 1))) peer_ok = 1;
    if (kinect_net_rtt_ms() >= 0) rtt_ok = 1;
    if (depth_ok && (child || delta_ok) && rgb_ok && cal_ok && peer_ok && rtt_ok && age > 500000ull)
      break;
    usleep(15000);
  }
  if (depth_ok && (child || delta_ok) && rgb_ok && cal_ok && peer_ok && rtt_ok) {
    snprintf(msg, sizeof msg, "OK rtt=%d peer=%d\n", kinect_net_rtt_ms(), kinect_net_peer_sensors());
    rc = 0;
  } else {
    snprintf(msg, sizeof msg,
             "FAIL depth=%d delta=%d rgb=%d cal=%d peer=%d rtt=%d sensors=%d conn=%d\n", depth_ok,
             delta_ok, rgb_ok, cal_ok, peer_ok, rtt_ok, kinect_net_peer_sensors(),
             kinect_net_connected());
    rc = 1;
  }
done:
  kinect_net_stop();
  if (out_fd >= 0) {
    ssize_t w = write(out_fd, msg, strlen(msg));
    (void)w;
  } else {
    fputs(msg, stdout);
  }
  free(depth);
  free(got);
  free(rgb);
  free(rgb_got);
  return rc;
}

int main(void) {
  int fds[2];
  pid_t pid;
  char line[200];
  ssize_t n;
  int st = 0, parent_rc, child_rc;
  if (pipe(fds) != 0) {
    perror("pipe");
    return 1;
  }
  pid = fork();
  if (pid < 0) {
    perror("fork");
    return 1;
  }
  if (pid == 0) {
    int rc;
    close(fds[0]);
    rc = run_side(1, fds[1]);
    close(fds[1]);
    _exit(rc);
  }
  close(fds[1]);
  parent_rc = run_side(0, -1);
  n = read(fds[0], line, sizeof line - 1);
  if (n < 0) n = 0;
  line[n] = 0;
  close(fds[0]);
  if (waitpid(pid, &st, 0) < 0) st = 1;
  child_rc = WIFEXITED(st) ? WEXITSTATUS(st) : 1;
  printf("child %s", line[0] ? line : "FAIL no report\n");
  printf("parent %s", parent_rc ? "FAIL\n" : "OK\n");
  return (parent_rc || child_rc) ? 1 : 0;
}
