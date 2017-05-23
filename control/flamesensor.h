/* flame sensor library header file written by Micael Monteiro


/*Standard Library Dependencies*/
#include <stdint.h>
#include <stdlib.h>
#ifndef FLAMESENSOR
#define FLAMESENSOR
#endif

/*Exponential Moving Average - Alpha parameter */
#ifndef EMA_ALPHA
#define EMA_ALPHA 10 /*This is in percentage. Should be between 0-99* - Corresponds to amount of smoothing.*/



/*Function Prototypes*/
int ema_filter(int current_value);
int ema_filter1(int current_value); 
int ema_filter2(int current_value);
int getFS1values();
int getFS2values();
int getFS3values();
char flameposition(int sensor1, int sensor2, int sensor3);

#endif