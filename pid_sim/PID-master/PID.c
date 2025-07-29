#include "PID.h"

void PIDController_Init(PIDController *pid) {

  /* Clear controller variables */
  pid->integrator = 0;
  pid->prevError = 0;

  pid->differentiator = 0;
  pid->prevMeasurement = 0;

  pid->out = 0;
}

int PIDController_Update(PIDController *pid, int setpoint, int measurement) {

  /*
   * Error signal
   */
  int error = setpoint - measurement;

  /*
   * Proportional
   */
  int proportional = (pid->Kp * error);

  /*
   * Integral
   */
  pid->integrator =
      (pid->integrator + ((pid->Ki * pid->T * (error + pid->prevError)) / 2)) /
      2000;

  /* Anti-wind-up via integrator clamping */
  if (pid->integrator > pid->limMaxInt) {

    pid->integrator = pid->limMaxInt;

  } else if (pid->integrator < pid->limMinInt) {

    pid->integrator = pid->limMinInt;
  }

  /*
   * Derivative (band-limited differentiator)
   */

  pid->differentiator =
      -(2 * pid->Kd *
            (measurement -
             pid->prevMeasurement) /* Note: derivative on measurement, therefore
                                      minus sign in front of equation! */
        + (2 * pid->tau - pid->T) * pid->differentiator) /
      (2 * pid->tau + pid->T);

  /*
   * Compute output and apply limits
   */
  pid->out = proportional + pid->integrator + pid->differentiator;

  if (pid->out > pid->limMax) {

    pid->out = pid->limMax;

  } else if (pid->out < pid->limMin) {

    pid->out = pid->limMin;
  }

  /* Store error and measurement for later use */
  pid->prevError = error;
  pid->prevMeasurement = measurement;

  /* Return controller output */
  return pid->out;
}
