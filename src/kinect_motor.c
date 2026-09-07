#define _POSIX_C_SOURCE 200809L
#include "kinect_motor.h"

#include <libfreenect.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static freenect_context *ctx;
static freenect_device *dev;
static int opened;
static int inhibited;

void kinect_motor_inhibit(int on) { inhibited = on ? 1 : 0; }
int kinect_motor_inhibited(void) { return inhibited; }

int kinect_motor_open(void) {
  int n;
  if (inhibited) return -1;
  pthread_mutex_lock(&mu);
  if (opened) {
    pthread_mutex_unlock(&mu);
    return 0;
  }
  if (freenect_init(&ctx, NULL) < 0) {
    pthread_mutex_unlock(&mu);
    return -1;
  }
  freenect_set_log_level(ctx, FREENECT_LOG_WARNING);
  freenect_select_subdevices(ctx, FREENECT_DEVICE_MOTOR);
  n = freenect_num_devices(ctx);
  if (n < 1 || freenect_open_device(ctx, &dev, 0) < 0) {
    freenect_shutdown(ctx);
    ctx = NULL;
    dev = NULL;
    pthread_mutex_unlock(&mu);
    return -1;
  }
  opened = 1;
  /* Fresh motor open must not spend LED energy. Caller turns it on for a live session. */
  freenect_set_led(dev, LED_OFF);
  pthread_mutex_unlock(&mu);
  return 0;
}

void kinect_motor_close(void) {
  pthread_mutex_lock(&mu);
  if (!opened) {
    pthread_mutex_unlock(&mu);
    return;
  }
  if (dev) {
    freenect_set_led(dev, LED_OFF);
    freenect_close_device(dev);
  }
  if (ctx) freenect_shutdown(ctx);
  ctx = NULL;
  dev = NULL;
  opened = 0;
  pthread_mutex_unlock(&mu);
}

static void pump_n(int n, int usec) {
  struct timeval tv;
  int i;
  tv.tv_sec = 0;
  tv.tv_usec = usec;
  for (i = 0; i < n; i++) freenect_process_events_timeout(ctx, &tv);
}

int kinect_motor_set_tilt(double degrees) {
  int r;
  if (inhibited) return -1;
  if (degrees < -30) degrees = -30;
  if (degrees > 30) degrees = 30;
  if (kinect_motor_open() < 0) return -1;
  pthread_mutex_lock(&mu);
  r = freenect_set_tilt_degs(dev, degrees);
  pump_n(4, 50000);
  pthread_mutex_unlock(&mu);
  return r < 0 ? -1 : 0;
}

int kinect_motor_get_tilt(double *degrees, int *status) {
  freenect_raw_tilt_state *st;
  if (inhibited) return -1;
  if (kinect_motor_open() < 0) return -1;
  pthread_mutex_lock(&mu);
  pump_n(1, 30000);
  freenect_update_tilt_state(dev);
  st = freenect_get_tilt_state(dev);
  if (!st) {
    pthread_mutex_unlock(&mu);
    return -1;
  }
  if (degrees) *degrees = freenect_get_tilt_degs(st);
  if (status) *status = (int)freenect_get_tilt_status(st);
  pthread_mutex_unlock(&mu);
  return 0;
}

int kinect_motor_set_led(int led) {
  int r;
  if (inhibited) return 0;
  if (kinect_motor_open() < 0) return -1;
  pthread_mutex_lock(&mu);
  r = freenect_set_led(dev, (freenect_led_options)led);
  pthread_mutex_unlock(&mu);
  return r < 0 ? -1 : 0;
}

void kinect_motor_pump(void) {
  pthread_mutex_lock(&mu);
  if (opened) pump_n(1, 20000);
  pthread_mutex_unlock(&mu);
}
