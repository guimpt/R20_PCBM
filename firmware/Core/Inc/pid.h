/*
 * pid.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Guim
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

/*
 * DEFINES
 */

#define MULTIPLIER 1024

/*
 * STRUCTS & ENUMS
 */

typedef struct{
	// Interrupt flag
	uint8_t update_flag;

	// Input, output and feedback
	int32_t x_k;
	int32_t y_k;
	int32_t u_k; // Not saturated
	int32_t h_k;

	// PID gains
	float kp;
	float ki;
	float kd;
	int32_t kp_int;
	int32_t ki_int;
	int32_t kd_int;


	// Loop frequency
	float loop_freq;

	// Derivative filter
	float w_cutoff;
	uint32_t filter_c1;
	uint32_t filter_c2;

	// Saturation
	float Umax;
	float Umin;
	int32_t Umax_int;
	int32_t Umin_int;

	// Other signals
	int32_t xi_k;
	int32_t xp_k;
	int32_t xd_k;
	int32_t h_km1;

	// Multiplier
	uint32_t multiplier;
} PID;

/*
 * FUNCTIONS
 */

void PID_Initialize(PID *pid);
void PID_Update(PID *pid);

#endif /* INC_PID_H_ */
