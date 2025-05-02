/*
 * pid.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Guim
 */

#include "pid.h"

/**
  * @brief pidLoop class initialization
  * @param PID data type
  * @retval none
  */
void PID_Initialize(PID *pid){
	pid->update_flag = 0;

	pid->x_k = 0;
	pid->h_k = 0;

	// PID gains
	pid->kp_int = pid->kp * MULTIPLIER;
	pid->ki_int = (pid->ki * MULTIPLIER);
	pid->kd_int = (pid->kd * MULTIPLIER);


	// Derivative filter
	float tau = 1.f / (float)pid->w_cutoff;
	float T = 1.f / (float)pid->loop_freq;
	pid->filter_c1 = (2.f * MULTIPLIER)  / (T + 2 * tau);
	pid->filter_c2 = ((T - 2.f * tau) * MULTIPLIER) / (T + 2 * tau);

	// Saturation
	pid->Umax_int = (pid->Umax * MULTIPLIER);
	pid->Umin_int = (pid->Umin * MULTIPLIER);

	// Other signals
	pid->xi_k = 0;
	pid->xp_k = 0;
	pid->xd_k = 0;
	pid->h_km1 = 0;

	pid->multiplier = MULTIPLIER;
}

/**
  * @brief pidLoop Update (+1 loop cycle)
  * @param PID data type
  * @retval none
  */
void PID_Update(PID *pid){
	// Proportional
	pid->xp_k = pid->x_k - pid->h_k;
	// Integral
	if(pid->u_k == pid->y_k){
		pid->xi_k += ((((int64_t)pid->xp_k * (int64_t)pid->kp_int) / MULTIPLIER) * (int64_t)pid->ki_int) / MULTIPLIER / pid->loop_freq;
	}
	// Derivative
	pid->xd_k = -((int64_t)pid->xd_k * pid->filter_c2) / MULTIPLIER;
	pid->xd_k += (((((int64_t)pid->kd_int * (int64_t)pid->filter_c1) / MULTIPLIER) * ((int64_t)pid->h_k - (int64_t)pid->h_km1)) * pid->loop_freq) / MULTIPLIER;

	pid->h_km1 = pid->h_k;

	// Control action
	pid->u_k = ((int64_t)pid->kp_int * (int64_t)(pid->xp_k - pid->xd_k)) / MULTIPLIER + pid->xi_k;

	// Output
	if(pid->u_k > pid->Umax_int) pid->y_k = pid->Umax_int;
	else if(pid->u_k < pid->Umin_int) pid->y_k = pid->Umin_int;
	else pid->y_k = pid->u_k;

}

