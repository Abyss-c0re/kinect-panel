#ifndef KINECT_SESSION_H
#define KINECT_SESSION_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Session is the SoT for Kinect energy.
 * Sensor streams / IR projector / LED spend power only while a session is live.
 * GUI default is OFF. --headless is the only auto-start. */

typedef enum {
  KINECT_MODE_OFF = 0,
  KINECT_MODE_RGB,
  KINECT_MODE_IR,
  KINECT_MODE_CLOUD
} kinect_mode;

int kinect_session_off(void);
int kinect_session_start(kinect_mode mode);
int kinect_session_running(void);
int kinect_session_headless(void);
void kinect_session_set_headless(int on);
kinect_mode kinect_session_mode(void);
const char *kinect_session_mode_name(kinect_mode m);
kinect_mode kinect_session_parse_mode(const char *s);
const char *kinect_session_error(void);
/* One JSON line. Cube wins over HUD prose. */
void kinect_session_status_json(char *buf, size_t n);

/* Control socket — IN command / OUT JSON. One owner process.
 * Commands: OFF | START rgb|ir|cloud | STATUS | QUIT | RAISE */
int kinect_ctl_path(char *out, size_t n);
int kinect_ctl_listen(void);
int kinect_ctl_fd(void);
int kinect_ctl_send(const char *cmd, char *reply, size_t n);
void kinect_ctl_pump(int (*dispatch)(const char *cmd, char *reply, size_t n));
void kinect_ctl_close(void);

/* Built-in dispatch for OFF/START/STATUS/QUIT. Returns 1 if process should exit. */
int kinect_session_dispatch(const char *cmd, char *reply, size_t n);

/* No window. Stream until SIGTERM/SIGINT/SIGHUP or socket OFF/QUIT. */
int kinect_headless_run(kinect_mode mode);

#ifdef __cplusplus
}
#endif
#endif
