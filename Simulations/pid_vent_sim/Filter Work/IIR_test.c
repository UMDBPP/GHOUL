#include <stdio.h>
#include <stdlib.h>

#include "IIRFirstOrder.h"

#include "input_ascent_100.h"

IIRFirstOrder iir;

int main() {

  int32_t output = 0;

  /* Initialise IIR Filter  */
  IIRFirstOrder_Init(&iir, 990);

  FILE *f = fopen("iir_output.csv", "w");
  if (f == NULL) {
    printf("Error opening file!\n");
    exit(1);
  }

  for (int i = 0; i < (sizeof(input) / 4); i++) {
    output = IIRFirstOrder_Update(&iir, input[i] * 10);

    fprintf(f, "%d\n", output);
  }

  fclose(f);

  return 0;
}
