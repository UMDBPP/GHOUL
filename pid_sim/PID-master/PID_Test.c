#include <stdio.h>
#include <stdlib.h>

#include "PID.h"

/* Controller parameters */
#define PID_KP -1
#define PID_KI -1
#define PID_KD 0

#define PID_TAU 2

#define PID_LIM_MIN 0
#define PID_LIM_MAX 60

#define PID_LIM_MIN_INT 0
#define PID_LIM_MAX_INT 60

#define SAMPLE_TIME_S 1

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 400

/* Simulated dynamical system (first order) */
int TestSystem_Update(int inp);

int main() {
  /* Initialise PID controller */
  PIDController pid = {PID_KP,          PID_KI,          PID_KD,
                       PID_TAU,         PID_LIM_MIN,     PID_LIM_MAX,
                       PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S};

  PIDController_Init(&pid);

  /* Simulate response using test system */
  int setpoint = 100;

  printf("Time ()\tSystem Output\tControllerOutput\r\n");
  for (int t = 0; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S) {

    /* Get measurement from system */
    int measurement = TestSystem_Update(pid.out);

    /* Compute new control signal */
    PIDController_Update(&pid, setpoint, measurement);

    printf("%d\t%d\t%d\r\n", t, measurement, pid.out);
  }

  return 0;
}

int TestSystem_Update(int inp) {

  static int output = 600;
  // static const int alpha = 2;

  // output = (SAMPLE_TIME_S * inp + output) / (alpha * SAMPLE_TIME_S);

  output = output - ((inp * 100) / 30);

  return output;
}
