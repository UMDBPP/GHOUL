#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "IIRFirstOrder.h"

/* Maximum run-time of simulation in seconds */
#define SIMULATION_TIME_MAX (5 * 60 * 60)
#define INITIAL_ASCENT_RATE 6000

/* Accepts control system signal (vent time in seconds) and time in seconds and
 * returns an integer representing balloon altitude */
void TestSystem_Update(int inp, int t);

/* Variables used by simulation, global for logging */
int vel = INITIAL_ASCENT_RATE;
int pos = 0;
int total_vent_time = 0;
int target_altitude_reached = 0;

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

    /*
if (altitude < 0) {
printf("Altitude less than 0 meters\n");
break;
}
    */

    if (altitude > 35000 * 1000) {
      printf("Altitude limit reached\n");
      break;
    }

    if (altitude >= setpoint && target_altitude_reached == 0) {
      printf("Target altitude of %d m reached at %d min\n", setpoint / 1000,
             t / 60);
      target_altitude_reached = 1;
    }

    if (t == 1 * 60 * 60)
      printf("Altitude at 1 hour: %d m\n", altitude / 1000);
    if (t == 2 * 60 * 60)
      printf("Altitude at 2 hours: %d m\n", altitude / 1000);
    if (t == 3 * 60 * 60)
      printf("Altitude at 3 hours: %d m\n", altitude / 1000);
    if (t == 4 * 60 * 60)
      printf("Altitude at 4 hours: %d m\n", altitude / 1000);
    if (t == 5 * 60 * 60)
      printf("Altitude at 5 hours: %d m\n", altitude / 1000);
  }

  printf("Total vent time: %d s\n", total_vent_time);

  fclose(f);

  return 0;
}

void TestSystem_Update(int inp, int t) {

  static int venting = 0;
  static int inner_ascent_rate = INITIAL_ASCENT_RATE;

  /* Select random delta */
  int delta = (-1) * ((rand() % 101));

  /* Simulate ascent rate increase during ascent */
  int curve = ((-1) * (int)(pow(((pos / 1000) - 10000) / 5000, 2))) + 2;

  /* simulate venting of helium by applying random negative ascent rate change
   * for given number of seconds */
  venting = venting + inp;
  total_vent_time = total_vent_time + inp;

  if (venting > 0) {
    inner_ascent_rate = inner_ascent_rate + delta;
    venting--;
  }

  /* Add small random noise that accumulates */
  inner_ascent_rate = inner_ascent_rate + ((rand() % 11) - 5);

  if (curve < 0)
    curve = 0;

  inner_ascent_rate = inner_ascent_rate + curve;

  /* Generates random system noise to hopefully
   * simulate updrafs and downdrafts
   */
  double a = ((rand() % 4001) - 2000);
  double b = ((double)((rand() % 1001))) / 1000.0;
  double y = a * sin(b * t);
  int noise = (int)(y);

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
  altitude_control = (setpoint - altitude) / 500;

  if (altitude_control < 0)
    altitude_control = 0;

  ascent_rate_control =
      ((-1) * (altitude_control - filtered_ascent_rate)) / 100;

  if (ascent_rate_control < 0)
    ascent_rate_control = 0;
  if (ascent_rate_control > 0)
    ascent_rate_control = 1;

  // if ascent rate is within desired range do not vent
  if (filtered_ascent_rate < 500)
    ascent_rate_control = 0;

  // delay next controller update by the vent time
  delay = ascent_rate_control;

  return ascent_rate_control;
}
