#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "kinect_session.h"
#include "kinect_bridge.h"
#include "kinect_cloud_view.h"
#include "kinect_fuse.h"
#include "kinect_motor.h"
#include "kinect_net.h"
#include "kinect2_device.h"

#include <errno.h>
#include <libfreenect.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static kinect_mode cur = KINECT_MODE_OFF;
static int headless;
static char err[160];
static int listen_fd = -1;
static char sock_path[256];
static volatile sig_atomic_t hl_stop;

static void set_err(const char *s) {
  snprintf(err, sizeof err, "%s", s ? s : "");
}

const char *kinect_session_mode_name(kinect_mode m) {
  switch (m) {
    case KINECT_MODE_RGB:
      return "rgb";
    case KINECT_MODE_IR:
      return "ir";
    case KINECT_MODE_CLOUD:
      return "cloud";
    case KINECT_MODE_OFF:
    default:
      return "off";
  }
}

kinect_mode kinect_session_parse_mode(const char *s) {
  if (!s || !s[0]) return KINECT_MODE_OFF;
  if (s[0] == 'R' || s[0] == 'r') return KINECT_MODE_RGB;
  if (s[0] == 'I' || s[0] == 'i') return KINECT_MODE_IR;
  if (s[0] == 'C' || s[0] == 'c') return KINECT_MODE_CLOUD;
  if (s[0] == 'O' || s[0] == 'o') return KINECT_MODE_OFF;
  return KINECT_MODE_OFF;
}

static void live_led(kinect_mode m) {
  switch (m) {
    case KINECT_MODE_IR:
      kinect_motor_set_led(LED_GREEN);
      break;
    case KINECT_MODE_CLOUD:
      kinect_motor_set_led(LED_YELLOW);
      break;
    case KINECT_MODE_RGB:
      kinect_motor_set_led(LED_BLINK_GREEN);
      break;
    case KINECT_MODE_OFF:
    default:
      kinect_motor_set_led(LED_OFF);
      break;
  }
}

static void net_want_for(kinect_mode mode) {
  if (!kinect_net_active()) return;
  if (mode == KINECT_MODE_IR)
    kinect_net_want(3);
  else if (mode == KINECT_MODE_RGB)
    kinect_net_want(2);
  else
    kinect_net_want(1);
}

int kinect_session_off(void) {
  kinect_bridge_stop();
  kinect_cloud_stop();
  kinect_motor_inhibit(0);
  if (kinect_motor_open() == 0)
    kinect_motor_set_led(LED_OFF);
  kinect_motor_close();
  net_want_for(KINECT_MODE_OFF);
  pthread_mutex_lock(&mu);
  cur = KINECT_MODE_OFF;
  err[0] = 0;
  pthread_mutex_unlock(&mu);
  return 0;
}

int kinect_session_start(kinect_mode mode) {
  int rc;

  if (mode == KINECT_MODE_OFF)
    return kinect_session_off();

  kinect_session_off();
  set_err("");

  if (mode == KINECT_MODE_CLOUD) {
    char why[160];
    rc = kinect_cloud_start();
    if (rc != 0) {
      snprintf(why, sizeof why, "%s", kinect_cloud_error());
      /* cloud_start can return while a dump fallback or a slow open is still up. */
      kinect_cloud_stop();
      set_err(why[0] ? why : "cloud start failed");
      pthread_mutex_lock(&mu);
      cur = KINECT_MODE_OFF;
      pthread_mutex_unlock(&mu);
      return -1;
    }
    {
      kinect_fuse_stats st;
      int skip_led = 0;
      kinect_fuse_get_stats(&st);
      if (st.v1_open && !kinect_fuse_local_v1()) skip_led = 1;
      if (!skip_led) live_led(KINECT_MODE_CLOUD);
    }
  } else {
    const char *m = (mode == KINECT_MODE_IR) ? "IR" : "RGB";
    usleep(200000);
    rc = kinect_bridge_start(m);
    if (rc != 0) {
      set_err(kinect_bridge_error());
      kinect_bridge_stop();
      pthread_mutex_lock(&mu);
      cur = KINECT_MODE_OFF;
      pthread_mutex_unlock(&mu);
      return -1;
    }
    /* Kinect 2 lights its own lamp. A remote Kinect is not this USB motor. */
    if (kinect_bridge_sensor() != 2 && !kinect_bridge_net()) live_led(mode);
  }
  net_want_for(mode);

  pthread_mutex_lock(&mu);
  cur = mode;
  pthread_mutex_unlock(&mu);
  return 0;
}

int kinect_session_running(void) {
  kinect_mode m;
  pthread_mutex_lock(&mu);
  m = cur;
  pthread_mutex_unlock(&mu);
  if (m == KINECT_MODE_CLOUD) return kinect_cloud_running();
  if (m == KINECT_MODE_RGB || m == KINECT_MODE_IR) return kinect_bridge_running();
  return 0;
}

int kinect_session_headless(void) { return headless; }
void kinect_session_set_headless(int on) { headless = on ? 1 : 0; }

kinect_mode kinect_session_mode(void) {
  kinect_mode m;
  pthread_mutex_lock(&mu);
  m = cur;
  pthread_mutex_unlock(&mu);
  return m;
}

const char *kinect_session_error(void) { return err; }

void kinect_session_status_json(char *buf, size_t n) {
  kinect_mode m;
  kinect_fuse_stats st;
  kinect_src sel;
  int run, hl, v1, v2, skew, paired;
  const char *src;
  const char *pipe;
  if (!buf || !n) return;
  pthread_mutex_lock(&mu);
  m = cur;
  hl = headless;
  pthread_mutex_unlock(&mu);
  run = kinect_session_running();
  /* Source is a setting. Stats are the last session, not a USB probe. */
  sel = kinect_fuse_source();
  kinect_fuse_get_stats(&st);
  v1 = 0;
  v2 = 0;
  skew = -1;
  paired = 0;
  pipe = "";
  src = kinect_src_name(sel);
  if (run && m == KINECT_MODE_CLOUD) {
    src = kinect_src_name(st.active);
    v1 = st.v1_open ? 1 : 0;
    v2 = st.v2_open ? 1 : 0;
    paired = st.paired ? 1 : 0;
    if (paired && st.skew_us >= 0) skew = st.skew_us / 1000;
    if (v2 && st.pipeline[0]) pipe = st.pipeline;
  } else if (run && (m == KINECT_MODE_RGB || m == KINECT_MODE_IR)) {
    int sens = kinect_bridge_sensor();
    if (sens == 2) {
      v2 = 1;
      if (kinect2_pipeline()[0]) pipe = kinect2_pipeline();
    } else if (sens == 1) {
      v1 = 1;
    }
  }
  {
    int net = kinect_net_connected();
    int rtt = net ? kinect_net_rtt_ms() : -1;
    int peer = kinect_net_peer_sensors();
    snprintf(buf, n,
             "{\"ok\":true,\"running\":%s,\"mode\":\"%s\",\"headless\":%s,\"collect\":false,"
             "\"src\":\"%s\",\"v1\":%s,\"v2\":%s,\"skew_ms\":%d,\"paired\":%s,\"pipe\":\"%s\","
             "\"net\":%s,\"rtt\":%d,\"peer\":%d}\n",
             run ? "true" : "false", kinect_session_mode_name(m), hl ? "true" : "false", src,
             v1 ? "true" : "false", v2 ? "true" : "false", skew, paired ? "true" : "false", pipe,
             net ? "true" : "false", rtt, peer);
  }
}

int kinect_ctl_path(char *out, size_t n) {
  const char *rt = getenv("XDG_RUNTIME_DIR");
  if (!out || !n) return -1;
  snprintf(out, n, "%s/kinect-panel.sock", rt && rt[0] ? rt : "/tmp");
  return 0;
}

int kinect_ctl_listen(void) {
  struct sockaddr_un a;
  kinect_ctl_path(sock_path, sizeof sock_path);
  unlink(sock_path);
  listen_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listen_fd < 0) return -1;
  memset(&a, 0, sizeof a);
  a.sun_family = AF_UNIX;
  snprintf(a.sun_path, sizeof a.sun_path, "%s", sock_path);
  if (bind(listen_fd, (struct sockaddr *)&a, sizeof a) < 0) {
    close(listen_fd);
    listen_fd = -1;
    return -1;
  }
  if (listen(listen_fd, 4) < 0) {
    close(listen_fd);
    listen_fd = -1;
    unlink(sock_path);
    return -1;
  }
  return 0;
}

int kinect_ctl_fd(void) { return listen_fd; }

int kinect_ctl_send(const char *cmd, char *reply, size_t n) {
  struct sockaddr_un a;
  int fd, w, r;
  char path[256];
  char line[128];

  if (!cmd) return -1;
  kinect_ctl_path(path, sizeof path);
  fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) return -1;
  memset(&a, 0, sizeof a);
  a.sun_family = AF_UNIX;
  snprintf(a.sun_path, sizeof a.sun_path, "%s", path);
  if (connect(fd, (struct sockaddr *)&a, sizeof a) < 0) {
    close(fd);
    return -1;
  }
  snprintf(line, sizeof line, "%s\n", cmd);
  w = (int)write(fd, line, strlen(line));
  (void)w;
  shutdown(fd, SHUT_WR);
  if (reply && n) {
    r = (int)read(fd, reply, n - 1);
    if (r < 0) r = 0;
    reply[r] = 0;
  }
  close(fd);
  return 0;
}

void kinect_ctl_pump(int (*dispatch)(const char *cmd, char *reply, size_t n)) {
  int cfd;
  char cmd[128], reply[512];
  ssize_t r;
  size_t L;
  struct timeval tv;

  if (listen_fd < 0 || !dispatch) return;
  cfd = accept(listen_fd, NULL, NULL);
  if (cfd < 0) return;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  r = read(cfd, cmd, sizeof cmd - 1);
  if (r <= 0) {
    close(cfd);
    return;
  }
  cmd[r] = 0;
  L = strlen(cmd);
  while (L && (cmd[L - 1] == '\n' || cmd[L - 1] == '\r' || cmd[L - 1] == ' '))
    cmd[--L] = 0;
  reply[0] = 0;
  dispatch(cmd, reply, sizeof reply);
  if (!reply[0])
    snprintf(reply, sizeof reply, "{\"ok\":false,\"error\":\"empty\"}\n");
  (void)write(cfd, reply, strlen(reply));
  close(cfd);
}

void kinect_ctl_close(void) {
  if (listen_fd >= 0) {
    close(listen_fd);
    listen_fd = -1;
  }
  if (sock_path[0]) {
    unlink(sock_path);
    sock_path[0] = 0;
  }
}

static void trim_cmd(char *s) {
  char *p;
  for (p = s; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p = (char)(*p - 'A' + 'a');
  }
}

static void split_two(const char *arg, char *a, size_t na, char *b, size_t nb) {
  size_t i = 0;
  if (!arg) arg = "";
  if (na) a[0] = 0;
  if (nb) b[0] = 0;
  while (*arg == ' ' || *arg == '\t') arg++;
  while (*arg && *arg != ' ' && *arg != '\t' && i + 1 < na) a[i++] = *arg++;
  if (na) a[i] = 0;
  while (*arg == ' ' || *arg == '\t') arg++;
  i = 0;
  while (*arg && *arg != ' ' && *arg != '\t' && i + 1 < nb) b[i++] = *arg++;
  if (nb) b[i] = 0;
}

int kinect_session_dispatch(const char *cmd, char *reply, size_t n) {
  char buf[128];
  if (!cmd || !reply || !n) return 0;
  snprintf(buf, sizeof buf, "%s", cmd);
  trim_cmd(buf);

  if (strcmp(buf, "off") == 0) {
    kinect_session_off();
    kinect_session_status_json(reply, n);
    if (headless) return 1;
    return 0;
  }
  if (strcmp(buf, "quit") == 0) {
    kinect_session_off();
    snprintf(reply, n, "{\"ok\":true,\"action\":\"quit\",\"mode\":\"off\"}\n");
    return 1;
  }
  if (strcmp(buf, "status") == 0) {
    kinect_session_status_json(reply, n);
    return 0;
  }
  if (strcmp(buf, "align reset") == 0) {
    kinect_fuse_reset_rigid();
    kinect_session_status_json(reply, n);
    return 0;
  }
  if (strcmp(buf, "align") == 0) {
    ksync_align_result res;
    if (kinect_fuse_align(&res) != 0) {
      const char *e = kinect_fuse_error();
      snprintf(reply, n, "{\"ok\":false,\"error\":\"%s\"}\n",
               e && e[0] ? e : "align failed");
    } else {
      snprintf(reply, n,
               "{\"ok\":true,\"action\":\"align\",\"residual_m\":%.4f,\"inliers\":%d,"
               "\"yaw_deg\":%.2f,\"pitch_deg\":%.2f}\n",
               res.residual_m, res.inliers, res.yaw_deg, res.pitch_deg);
    }
    return 0;
  }
  if (strncmp(buf, "start", 5) == 0) {
    const char *arg = buf + 5;
    char tok[32], extra[32];
    kinect_mode m;
    kinect_src src;
    split_two(arg, tok, sizeof tok, extra, sizeof extra);
    if (!tok[0]) {
      m = KINECT_MODE_RGB;
    } else if (strcmp(tok, "cloud") == 0) {
      m = KINECT_MODE_CLOUD;
      /* No suffix keeps the current sensor. A known suffix selects it. */
      if (extra[0] && kinect_src_known(extra, &src)) kinect_fuse_set_source(src);
    } else if (kinect_src_known(tok, &src) && src != KINECT_SRC_AUTO) {
      m = KINECT_MODE_CLOUD;
      kinect_fuse_set_source(src);
    } else {
      m = kinect_session_parse_mode(tok);
      if (m == KINECT_MODE_OFF) m = KINECT_MODE_RGB;
      /* START rgb v2 / START ir both — suffix selects the camera too. */
      if (extra[0] && kinect_src_known(extra, &src)) kinect_fuse_set_source(src);
    }
    if (kinect_session_start(m) != 0)
      snprintf(reply, n, "{\"ok\":false,\"error\":\"%s\"}\n",
               err[0] ? err : "start failed");
    else
      kinect_session_status_json(reply, n);
    return 0;
  }
  if (strcmp(buf, "share") == 0) {
    int port = 0;
    const char *e = getenv("KINECT_SHARE_PORT");
    if (e && atoi(e) > 0) port = atoi(e);
    if (kinect_net_listen(port) != 0)
      snprintf(reply, n, "{\"ok\":false,\"error\":\"%s\"}\n",
               kinect_net_error()[0] ? kinect_net_error() : "share failed");
    else
      kinect_session_status_json(reply, n);
    return 0;
  }
  if (strncmp(buf, "connect", 7) == 0 && (buf[7] == 0 || buf[7] == ' ')) {
    const char *arg = buf + 7;
    while (*arg == ' ') arg++;
    if (!arg[0]) arg = getenv("KINECT_PEER");
    if (!arg || !arg[0])
      snprintf(reply, n, "{\"ok\":false,\"error\":\"need host\"}\n");
    else if (kinect_net_connect(arg) != 0)
      snprintf(reply, n, "{\"ok\":false,\"error\":\"%s\"}\n",
               kinect_net_error()[0] ? kinect_net_error() : "connect failed");
    else
      kinect_session_status_json(reply, n);
    return 0;
  }
  if (strcmp(buf, "net off") == 0) {
    kinect_net_stop();
    kinect_session_status_json(reply, n);
    return 0;
  }
  if (strcmp(buf, "raise") == 0) {
    snprintf(reply, n, "{\"ok\":false,\"error\":\"headless\",\"mode\":\"%s\"}\n",
             kinect_session_mode_name(kinect_session_mode()));
    return 0;
  }
  snprintf(reply, n, "{\"ok\":false,\"error\":\"unknown\"}\n");
  return 0;
}

static void hl_sig(int s) {
  (void)s;
  hl_stop = 1;
}

static int hl_dispatch(const char *cmd, char *reply, size_t n) {
  int q = kinect_session_dispatch(cmd, reply, n);
  if (q) hl_stop = 1;
  return q;
}

static void hl_atexit(void) {
  kinect_session_off();
  kinect_net_stop();
}

int kinect_headless_run(kinect_mode mode) {
  if (mode == KINECT_MODE_OFF) mode = KINECT_MODE_RGB;
  headless = 1;
  hl_stop = 0;
  signal(SIGINT, hl_sig);
  signal(SIGTERM, hl_sig);
  signal(SIGHUP, hl_sig);
  atexit(hl_atexit);

  if (kinect_ctl_listen() < 0) {
    fprintf(stderr, "kinect-panel: ctl bind failed (%s)\n", strerror(errno));
    return 1;
  }
  if (kinect_session_start(mode) != 0) {
    fprintf(stderr, "kinect-panel: headless start failed: %s\n",
            err[0] ? err : "unknown");
    kinect_ctl_close();
    return 1;
  }
  fprintf(stderr, "kinect-panel: headless %s live  ctl=%s  (--off to stop)\n",
          kinect_session_mode_name(mode), sock_path);

  while (!hl_stop) {
    fd_set rfds;
    struct timeval tv;
    int fd = listen_fd;
    FD_ZERO(&rfds);
    if (fd >= 0) FD_SET(fd, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 250000;
    if (fd >= 0 && select(fd + 1, &rfds, NULL, NULL, &tv) > 0 && FD_ISSET(fd, &rfds))
      kinect_ctl_pump(hl_dispatch);
  }

  kinect_session_off();
  kinect_ctl_close();
  return 0;
}
