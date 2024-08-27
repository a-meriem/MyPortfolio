/*
 * NavConfig.h
 *
 *  Created on: 28 sept. 2023
 *      Author: ASUS
 */

#ifndef INC_NAVCONFIG_H_
#define INC_NAVCONFIG_H_


#define ROBOT_2

#ifdef ROBOT_1

#define ENCODER_RIGHT	TIM2
#define ENCODER_LEFT	TIM5
#define MOTOR_PWM		TIM1

#define RIGHT_FORWARD	CCR1
#define RIGHT_REVERSE	CCR2
#define LEFT_FORWARD	CCR4
#define LEFT_REVERSE	CCR3


#elif defined ROBOT_2

#define ENCODER_RIGHT	TIM2
#define ENCODER_LEFT	TIM5
#define MOTOR_PWM		TIM1

#define RIGHT_FORWARD	CCR2
#define RIGHT_REVERSE	CCR1
#define LEFT_FORWARD	CCR3
#define LEFT_REVERSE	CCR4


#endif

#endif /* INC_NAVCONFIG_H_ */
