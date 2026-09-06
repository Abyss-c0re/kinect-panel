#ifndef KINECT_MOTOR_H
#define KINECT_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

int kinect_motor_open(void);
void kinect_motor_close(void);
int kinect_motor_set_tilt(double degrees);
int kinect_motor_get_tilt(double *degrees, int *status);
int kinect_motor_set_led(int led);
void kinect_motor_pump(void);

#ifdef __cplusplus
}
#endif
#endif
