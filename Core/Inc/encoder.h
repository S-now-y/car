/*
 * encoder.h
 *
 *  Created on: Feb 17, 2025
 *      Author: 22684
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "motor.h"

//宏



//声明
void GetEncoderpulse(void);
float CalculatePulse(int pulse);

#endif /* INC_ENCODER_H_ */
