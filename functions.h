/*
 * functions.h
 *
 *  Created on: Mar 4, 2023
 *      Author: albtr
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

/* Variable Declarations */

char rxResponse[32];    //The purpose of this is to store the response from UCA3RXBUF so that multichar commands can be recieved and acted on

int rx_i;

/* Stepper Motor Data
Step Frequency (Steps/S) = (v (rpm) * 360 (deg/rot)) / (theta step (deg/step) * n (steps/microstep) * 60 (s/min))

Full Step Increment: 1.8deg
linear Distance/Step: 0.0005in
max linear Speed = 0.8 in/s

v (rpm) = max linear speed_in/s * (full step_deg / line distance/step_deg/in)

microstepping up to 1/256 steps/microstep

Formula:
f_step = v / fullStepIncrement

f_mstep = v / (fullStepIncrement * microstepping)



fullStepDeg = 1.8;
distancePerStep = 0.0005;
maxLinearSpeed = 0.8;
microstep = 1/256;

float v = maxLinearSpeed * (fullStepDeg / distancePerStep);
float freq_step = v / fullStepDeg;
float freq_mStep = v / (fullStepDeg * microstep);
*/

/* Function Declarations */

void init_gpio();
void init_uart();
void init_bt();
void init_DRV8424EVM();
int set_blink_rate();

void send_byte(unsigned char txByte);
void send_AT_NAME();

#endif /* FUNCTIONS_H_ */
