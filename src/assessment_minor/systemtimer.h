#ifndef SYSTEMTIMER_H
#define SYSTEMTIMER_H

#include "stdint.h"

uint64_t millisecondsElasped();
void addMillisToSystemCounter(uint64_t millis);

#endif //SYSTEMTIMER_H