#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "IIRFirstOrder.h"

/* Maximum run-time of simulation in seconds */
#define SIMULATION_TIME_MAX (5 * 60 * 60)
#define INITIAL_ASCENT_RATE 6500

/* Accepts control system signal (vent time in seconds) and time in seconds and
 * returns an integer representing balloon altitude */
void TestSystem_Update(int inp, int t);

/* Variables used by simulation, global for logging */
int vel = INITIAL_ASCENT_RATE;
int pos = 0;

/* Variables used by control system */
IIRFirstOrder iir;
int setpoint = 23000 * 1000;
unsigned int delay = 0;
int altitude_control = 0;
int ascent_rate_control = 0;
int prev_altitude = 0;
int ascent_rate = 0;
int filtered_ascent_rate = 0;
int altitude = 0;

/* Initializes Control System, accepts a filter parameter used to filter ascent
 * rate from 0-1000 (990 recommended) */
void ControlSystem_Init(int32_t filter);

/* */
int ControlSystem_Update(int altitude, int ascent_rate);

int main() {

  srand(time(NULL)); // set random seed

  // -- START of application code in setup --
  ControlSystem_Init(990);
  // -- END of application code in setup --

  FILE *f = fopen("sim_output.csv", "w");
  if (f == NULL) {
    printf("Error opening file!\n");
    exit(1);
  }

  fprintf(f, "Time (s),Altitude,Controller Output,Ascent Rate (m/s),Filtered "
             "Ascent Rate (m/s)\n");

  for (int t = 0; t <= SIMULATION_TIME_MAX; t++) {

    TestSystem_Update(ascent_rate_control, t);

    // -- START of application code in main() loop --

    /* Get measurement from system, calculate ascent rate, filter ascent_rate */
    altitude = pos;
    ascent_rate = altitude - prev_altitude;
    prev_altitude = altitude;
    filtered_ascent_rate = IIRFirstOrder_Update(&iir, ascent_rate);

    altitude_control = 0;
    ascent_rate_control = 0;

    ControlSystem_Update(altitude, ascent_rate);

    // -- END of application code in main() loop --

    fprintf(f, "%d,%d,%d,%d,%d\n", t, altitude, ascent_rate_control,
            ascent_rate, filtered_ascent_rate);

    if (altitude < 0) {
      printf("Altitude less than 0 meters\n");
      fclose(f);
      return 0;
    }

    if (altitude > 35000 * 1000) {
      printf("Altitude limit reached\n");
      fclose(f);
      return 0;
    }
  }

  fclose(f);

  return 0;
}

void TestSystem_Update(int inp, int t) {

  static int venting = 0;
  static int inner_ascent_rate = INITIAL_ASCENT_RATE;

  int delta = (-1) * (rand() % 41);

  /* simulate venting of helium by applying random negative ascent rate change
   * for given number of seconds */
  venting = venting + inp;

  if (venting > 0) {
    inner_ascent_rate = inner_ascent_rate + delta;
    venting--;
  }

  /* Generates random system noise to hopefully
   * simulate updrafs and downdrafts
   */
  double a = ((rand() % 4001) - 2000);
  double b = ((double)((rand() % 1001))) / 1000.0;
  double y = a * sin(b * t);
  int noise = (int)(y);

  /* Add small random noise that accumulates */
  inner_ascent_rate = inner_ascent_rate + ((rand() % 21) - 10);

  /* Calculate final simulated ascent rate for iteration */
  vel = inner_ascent_rate + noise;

  pos = pos + vel;
}

void ControlSystem_Init(int32_t filter) {
  IIRFirstOrder_Init(&iir, filter); // force initial ascent rate

  for (int i = 0; i < 100; i++)
    IIRFirstOrder_Update(&iir, INITIAL_ASCENT_RATE);
}

int ControlSystem_Update(int altitude, int ascent_rate) {

  /* Don't update control until above 3km */
  if (altitude < 3000 * 1000)
    return 0;

  /* Don't update control if still waiting for previous
   * vent command to complete
   */
  if (delay > 0) {
    delay--;
    return 0;
  }

  /* Update position controller */
  altitude_control = (setpoint - altitude) / 1000;

  if (altitude_control < 0)
    altitude_control = 0;

  ascent_rate_control =
      ((altitude_control - filtered_ascent_rate) * (-1)) / 100;

  if (ascent_rate_control < 0)
    ascent_rate_control = 0;

  // if ascent rate is within desired range do not vent
  if (filtered_ascent_rate < 500)
    ascent_rate_control = 0;

  // delay next controller update by the vent time
  delay = ascent_rate_control + 1;

  return ascent_rate_control;
}
