/*
 * pid.h
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "encoder.h"
void PID_Init(PID *p);
void PID_Cal(int targetSpeed,int currentSpeed,PID *p);

#endif /* INC_PID_H_ */
