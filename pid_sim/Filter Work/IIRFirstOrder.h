/*
 *
 * IIR (Infinite Impulse Response) First-Order Filter
 *
 * Implements a discrete-time first-order IIR filter.
 * Constant term alpha sets the amount of filtering. 0 = no filtering, 1 = max.
 * filtering
 *
 * Written by: Philip M. Salmony @ philsal.co.uk
 * Last changed: 01 Dec 2019
 *
 */

#ifndef IIR_FIRST_ORDER_H
#define IIR_FIRST_ORDER_H

typedef struct {
  int32_t alpha;
  int32_t out;
} IIRFirstOrder;

void IIRFirstOrder_Init(IIRFirstOrder *filt, int32_t alpha) {
  filt->alpha = alpha;
  filt->out = 0;
}

int32_t IIRFirstOrder_Update(IIRFirstOrder *filt, int32_t in) {
  filt->out =
      ((filt->alpha * filt->out) / 1000) + (((1000 - filt->alpha) * in) / 1000);
  return filt->out;
}

#endif
