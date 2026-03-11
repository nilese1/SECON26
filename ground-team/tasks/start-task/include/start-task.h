#include <time.h>

#include "../../core/include/robot-core.h"

#define PHOTORESISTOR_GPIO 6 // unused pin
#define MAX_CYCLE_TO_START_MILLIS 100


// blocks thread until light from photoresistor is detected
void wait_for_light(); 
