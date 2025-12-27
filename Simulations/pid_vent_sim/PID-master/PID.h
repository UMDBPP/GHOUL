#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

  /* Controller gains */
  int Kp;
  int Ki;
  int Kd;

  /* Derivative low-pass filter time constant */
  int tau;

  /* Output limits */
  int limMin;
  int limMax;

  /* Integrator limits */
  int limMinInt;
  int limMaxInt;

  /* Sample time (in seconds) */
  int T;

  /* Controller "memory" */
  int integrator;
  int prevError; /* Required for integrator */
  int differentiator;
  int prevMeasurement; /* Required for differentiator */

  /* Controller output */
  int out;

  int error;

} PIDController;

void PIDController_Init(PIDController *pid);
int PIDController_Update(PIDController *pid, int setpoint, int measurement);

#endif
