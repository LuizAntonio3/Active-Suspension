#pragma once
#include <stdint.h>

/* 
  PRBS generator function.
*/

const uint16_t prbsPeriod = 65535;

void prbs(uint16_t startValue, uint16_t period, uint16_t* sequence, uint16_t* binarySequence);
