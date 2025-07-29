#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../IIRFirstOrder.h"
#include "PID.h"

/* Controller parameters */
#define PID_KP 0
#define PID_KI -1
#define PID_KD 0

#define PID_TAU 2

#define PID_LIM_MIN 0
#define PID_LIM_MAX 10

#define PID_LIM_MIN_INT 0
#define PID_LIM_MAX_INT 10

#define SAMPLE_TIME_S 5 // sample time in seconds

/* Maximum run-time of simulation in seconds */
#define SIMULATION_TIME_MAX (5 * 60 * 60)

/* Simulated dynamical system (first order) */
int TestSystem_Update(int inp);

int ascent_changes[] = {-35, -12, -22};
IIRFirstOrder iir;

int main() {

  PIDController pid = {PID_KP,          PID_KI,          PID_KD,
                       PID_TAU,         PID_LIM_MIN,     PID_LIM_MAX,
                       PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S};
  int setpoint = 1000;
  unsigned int delay = 0;

  srand(time(NULL)); // set random seed

  PIDController_Init(&pid);
  IIRFirstOrder_Init(&iir, 900);

  for (int i = 0; i < 100; i++)
    IIRFirstOrder_Update(&iir, 6000);

  FILE *f = fopen("pid_output.csv", "w");
  if (f == NULL) {
    printf("Error opening file!\n");
    exit(1);
  }

  fprintf(f, "Time (s),System Output,ControllerOutput\n");
  for (int t = 0; t <= SIMULATION_TIME_MAX; t++) {

    /* Get measurement from system */
    int measurement = TestSystem_Update(pid.out);

    pid.out = 0;

    if (t >= 30) {
      if (delay <= 0) {
        /* Compute new control signal */
        PIDController_Update(&pid, setpoint,
                             IIRFirstOrder_Update(&iir, measurement));
        delay = pid.out + (1 * 30);
      }
      delay--;
    }

    fprintf(f, "%d,%d,%d\n", t, measurement, pid.out);
  }

  return 0;
}

int TestSystem_Update(int inp) {

  static int output = 6000;
  int32_t delta = ascent_changes[rand() % ((sizeof(ascent_changes) / 4))];
  static int venting = 0;
  // static const int alpha = 2;

  // output = (SAMPLE_TIME_S * inp + output) / (alpha * SAMPLE_TIME_S);

  venting = venting + inp;

  output = output + ((rand() % 101) - 50);

  if (venting > 0) {
    output = output + delta;
    venting--;
  }

  return output;
}
