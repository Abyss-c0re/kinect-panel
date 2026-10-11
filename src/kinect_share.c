/* Headless peer. Listens and/or connects, then runs the fused cloud.
 * The local Kinect is published. A sensor this machine does not have is
 * filled from the peer. No GTK. SIGTERM releases the camera and the socket. */
#include "kinect_fuse.h"
#include "kinect_net.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop_flag;

static void on_sig(int s) {
  (void)s;
  stop_flag = 1;
}

static uint64_t mono_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void bbox(const char *name, const cube_cloud *c) {
  float minx = 1e9f, miny = 1e9f, minz = 1e9f;
  float maxx = -1e9f, maxy = -1e9f, maxz = -1e9f;
  int i;
  if (!c || c->n < 1 || !c->pt) {
    printf("  %s empty\n", name);
    return;
  }
  for (i = 0; i < c->n; i++) {
    float x = c->pt[i].x, y = c->pt[i].y, z = c->pt[i].z;
    if (x < minx) minx = x;
    if (y < miny) miny = y;
    if (z < minz) minz = z;
    if (x > maxx) maxx = x;
    if (y > maxy) maxy = y;
    if (z > maxz) maxz = z;
  }
  printf("  %s n=%d  x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f\n", name, c->n, minx, maxx, miny,
         maxy, minz, maxz);
}

static int dump_clouds(const char *path) {
  cube_cloud merged, a, b;
  char p1[512], p2[512];
  int rc;
  cube_cloud_init(&merged);
  cube_cloud_init(&a);
  cube_cloud_init(&b);
  rc = kinect_fuse_snapshot_parts(&merged, &a, &b);
  if (rc != 0 || merged.n < 1) {
    printf("dump: no cloud yet\n");
    cube_cloud_free(&merged);
    cube_cloud_free(&a);
    cube_cloud_free(&b);
    return -1;
  }
  if (cube_cloud_write_xyz(&merged, path) != 0) {
    printf("dump failed %s\n", path);
    rc = -1;
  } else {
    printf("dump %s\n", path);
    bbox("fused", &merged);
    bbox("kinect", &a);
    bbox("kinect2", &b);
    snprintf(p1, sizeof p1, "%s.v1.xyz", path);
    snprintf(p2, sizeof p2, "%s.v2.xyz", path);
    if (a.n > 0) cube_cloud_write_xyz(&a, p1);
    if (b.n > 0) cube_cloud_write_xyz(&b, p2);
    rc = 0;
  }
  cube_cloud_free(&merged);
  cube_cloud_free(&a);
  cube_cloud_free(&b);
  return rc;
}

static void usage(void) {
  fputs("kinect_share — publish this Kinect and use the peer's as a local sensor\n"
        "  --listen [port]          bind UDP (default 8765, or KINECT_SHARE_PORT)\n"
        "  --connect host[:port]    peer (or KINECT_PEER)\n"
        "  --src auto|v1|v2|both    default both\n"
        "  --seconds N              stop after N seconds (default: until signal)\n"
        "  --align-after N          fit Kinect 2 onto Kinect once both clouds exist\n"
        "  --near M --far M         depth window in metres (default 0.50 .. 3.50)\n"
        "  --dump PATH              write the fused xyz (plus .v1.xyz and .v2.xyz)\n",
        stdout);
}

int main(int argc, char **argv) {
  int i, do_listen = 0, do_connect = 0, port = 0, seconds = 0, align_after = 0;
  int did_align = 0, did_dump = 0, have_near = 0, have_far = 0;
  float z_near = 0.f, z_far = 0.f;
  kinect_src src = KINECT_SRC_BOTH;
  const char *host = NULL;
  const char *dump = NULL;
  uint64_t t0, next_stat, next_align;
  int v1 = 0, v2 = 0;
  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      usage();
      return 0;
    }
    if (strcmp(argv[i], "--listen") == 0) {
      do_listen = 1;
      if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9') port = atoi(argv[++i]);
      continue;
    }
    if (strcmp(argv[i], "--connect") == 0 && i + 1 < argc) {
      do_connect = 1;
      host = argv[++i];
      continue;
    }
    if (strcmp(argv[i], "--src") == 0 && i + 1 < argc) {
      if (!kinect_src_known(argv[++i], &src)) {
        fprintf(stderr, "bad --src\n");
        return 1;
      }
      continue;
    }
    if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
      seconds = atoi(argv[++i]);
      continue;
    }
    if (strcmp(argv[i], "--align-after") == 0 && i + 1 < argc) {
      align_after = atoi(argv[++i]);
      continue;
    }
    if (strcmp(argv[i], "--dump") == 0 && i + 1 < argc) {
      dump = argv[++i];
      continue;
    }
    if (strcmp(argv[i], "--near") == 0 && i + 1 < argc) {
      z_near = strtof(argv[++i], NULL);
      have_near = 1;
      continue;
    }
    if (strcmp(argv[i], "--far") == 0 && i + 1 < argc) {
      z_far = strtof(argv[++i], NULL);
      have_far = 1;
      continue;
    }
    fprintf(stderr, "unknown arg %s\n", argv[i]);
    usage();
    return 1;
  }
  if (!do_listen && !do_connect) {
    if (getenv("KINECT_PEER") && getenv("KINECT_PEER")[0]) {
      do_connect = 1;
      host = getenv("KINECT_PEER");
    }
    do_listen = 1;
  }
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);
  signal(SIGHUP, on_sig);

  kinect_fuse_set_source(src);
  if (have_near || have_far) {
    cube_cloud_tune tune;
    kinect_fuse_get_tune(&tune);
    if (have_near) tune.z_near = z_near;
    if (have_far) tune.z_far = z_far;
    kinect_fuse_set_tune(&tune);
  }
  kinect_fuse_probe(&v1, &v2);
  kinect_net_set_local((v1 ? 1 : 0) | (v2 ? 2 : 0));
  kinect_net_want(1);
  if (do_listen && kinect_net_listen(port) != 0) {
    fprintf(stderr, "listen failed: %s\n", kinect_net_error());
    return 1;
  }
  if (do_connect && kinect_net_connect(host) != 0) {
    fprintf(stderr, "connect failed: %s\n", kinect_net_error());
    kinect_net_stop();
    return 1;
  }
  if (do_connect) {
    int spins;
    for (spins = 0; spins < 50 && !stop_flag && !kinect_net_connected(); spins++) usleep(100000);
  }
  if (kinect_fuse_start() != 0) {
    fprintf(stderr, "fuse: %s\n", kinect_fuse_error());
    kinect_fuse_stop();
    kinect_net_stop();
    return 1;
  }
  t0 = mono_us();
  next_stat = t0;
  next_align = t0 + (align_after > 0 ? (uint64_t)align_after * 1000000ull : 0);
  printf("share up local=%d peer=%s src=%s\n", kinect_net_local_sensors(),
         kinect_net_connected() ? "yes" : "waiting", kinect_src_name(src));
  fflush(stdout);
  while (!stop_flag) {
    kinect_fuse_stats st;
    ksync_rigid rig;
    uint64_t now = mono_us();
    unsigned txp = 0, rxp = 0, txb = 0;
    if (seconds > 0 && now - t0 >= (uint64_t)seconds * 1000000ull) break;
    if (now >= next_stat) {
      float yaw = 0, pitch = 0;
      kinect_fuse_get_stats(&st);
      kinect_fuse_get_rigid(&rig);
      ksync_rigid_ypr(&rig, &yaw, &pitch);
      kinect_net_counters(&txp, &rxp, &txb);
      printf("t=%.1f src=%s n1=%d n2=%d n=%d paired=%d skew_ms=%d ident=%d yaw=%.1f "
             "t=%.2f,%.2f,%.2f net=%d rtt=%d peer=%d tx=%u rx=%u blocks=%u %s\n",
             (now - t0) / 1e6, kinect_src_name(st.active), st.n1, st.n2, st.n, st.paired,
             st.skew_us >= 0 ? st.skew_us / 1000 : -1, st.identity_rigid, yaw, rig.t[0], rig.t[1],
             rig.t[2], kinect_net_connected(), kinect_net_rtt_ms(), kinect_net_peer_sensors(), txp,
             rxp, txb, st.err);
      fflush(stdout);
      next_stat = now + 1000000ull;
    }
    if (align_after > 0 && !did_align && now >= next_align) {
      ksync_align_result ar;
      kinect_fuse_get_stats(&st);
      if (st.n1 > 30 && st.n2 > 30 && kinect_fuse_align(&ar) == 0) {
        printf("align ok residual=%.4f inliers=%d yaw=%.2f pitch=%.2f\n", ar.residual_m, ar.inliers,
               ar.yaw_deg, ar.pitch_deg);
        fflush(stdout);
        did_align = 1;
        /* The fuse thread rebuilds the merged cloud on its next frame. */
        usleep(400000);
        if (dump && !did_dump && dump_clouds(dump) == 0) did_dump = 1;
      } else {
        printf("align waiting: %s\n", kinect_fuse_error());
        fflush(stdout);
        next_align = now + 5000000ull;
      }
    }
    usleep(200000);
  }
  if (dump && !did_dump) dump_clouds(dump);
  kinect_fuse_stop();
  kinect_net_stop();
  printf("share stop\n");
  return 0;
}
