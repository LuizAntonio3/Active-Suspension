#include "prbs.h"

void prbs(uint16_t startValue, uint16_t period, uint16_t* sequence, uint16_t* binarySequence) {
  
    uint16_t lfsr = startValue;
    uint16_t bit;

    sequence[0] = startValue;
    binarySequence[0] = startValue & 0x0001;

    for (uint16_t i = 1; i <= period; i++) {

        bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
        lfsr = (lfsr >> 1) | (bit << 15);

        sequence[i] = lfsr;
        binarySequence[i] = bit;
    }

  return;
}
