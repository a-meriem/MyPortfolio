/*
 * NavConstants.h
 *
 *  Created on: 28 sept. 2023
 *      Author: ASUS
 */



#ifndef INC_NAVCONSTANTS_H_
#define INC_NAVCONSTANTS_H_


#define M_2PI			6.28318530717958647692
#define SAMPLING_PERIOD   		0.005
#define MAXIMUM_MOTOR_COMMAND  					5000
#define MINIMUM_SATURATION_COMMAND				50

#define Nav_No_Rotate_After 	1000
#define ROBOT_Going_Backward 	4
#define ROBOT_Going_Forward		3




#ifdef ROBOT_1
#define MINIMUM_RIGHT_MOTOR_FORWARD_COMMAND	 	505

#define MINIMUM_RIGHT_MOTOR_REVERSE_COMMAND		460

#define MINIMUM_LEFT_MOTOR_FORWARD_COMMAND		440

#define MINIMUM_LEFT_MOTOR_REVERSE_COMMAND		410


#elif defined ROBOT_2

#define MINIMUM_RIGHT_MOTOR_FORWARD_COMMAND	 	580

#define MINIMUM_RIGHT_MOTOR_REVERSE_COMMAND		535

#define MINIMUM_LEFT_MOTOR_FORWARD_COMMAND		660

#define MINIMUM_LEFT_MOTOR_REVERSE_COMMAND		490

#endif




#ifdef ROBOT_1
#define TRACK_MM     					280.25
#define WHEEL_LEFT_DIAMETER_MM  		64.0
#define WHEEL_RIGHT_DIAMETER_MM   	    64.0
#define WHEEL_DIAMETER_MM  				60.95
#define LEFT_WHEEL_GAIN					1.0035
#define ENCODER_RESOLUTION		2048.0

#define FORWARD_LINEAR_TRAJECTORY_PID	      \
		Nav_KpDistance_d    = 119.5147519 ;   \
		Nav_KiDistance_d    = 0.002172995 ;   \
		Nav_KdDistance_d    = 1032.172858 ;   \
		Nav_KpAngle_d       =  100 ;  \
		Nav_KiAngle_d       =  0.007964444 ;  \
		Nav_KdAngle_d       =  1500.503704 ;

#define REVERSE_LINEAR_TRAJECTORY_PID  \
		Nav_KpDistance_d    = 119.5147519 ;   \
		Nav_KiDistance_d    = 0.002172995 ;   \
		Nav_KdDistance_d    = 1032.172858 ;   \
		Nav_KpAngle_d       =  260.1718519 ;  \
		Nav_KiAngle_d       =  0.007964444 ;  \
		Nav_KdAngle_d       =  1500.503704 ;


#define FORWARD_ANGULAR_TRAJECTORY_PID \
		Nav_KpDistance_d    = 119.5147519 ;   \
		Nav_KiDistance_d    = 0.002172995 ;   \
		Nav_KdDistance_d    = 1032.172858 ;   \
		Nav_KpAngle_d       =  260.1718519 ;  \
		Nav_KiAngle_d       =  0.007964444 ;  \
		Nav_KdAngle_d       =  3000.503704 ;

#elif defined ROBOT_2

#define TRACK_MM     					280.25
#define WHEEL_DIAMETER_MM  				60.95
#define LEFT_WHEEL_GAIN					1.0035
#define ENCODER_RESOLUTION		2048.0

#define FORWARD_LINEAR_TRAJECTORY_PID	      \
		Nav_KpDistance_d    = 120 ;   \
		Nav_KiDistance_d    = 0.002172995 ;   \
		Nav_KdDistance_d    = 1032.172858 ;   \
		Nav_KpAngle_d       = 100		  ;  \
		Nav_KiAngle_d       =  0.007964444 ;  \
		Nav_KdAngle_d       =  1480.503704 ;

#define REVERSE_LINEAR_TRAJECTORY_PID  \
		Nav_KpDistance_d    = 120 ;   \
		Nav_KiDistance_d    = 0.002172995 ;   \
		Nav_KdDistance_d    = 1032.172858 ;   \
		Nav_KpAngle_d       = 100 		  ;  \
		Nav_KiAngle_d       = 0.007964444 ;  \
		Nav_KdAngle_d       = 1480.503704 ;


#define FORWARD_ANGULAR_TRAJECTORY_PID \
		Nav_KpDistance_d    = 122.5147519 ;   \
		Nav_KiDistance_d    = 0.002172995 ;   \
		Nav_KdDistance_d    = 1032.172858 ;   \
		Nav_KpAngle_d       = 290.7107583  ;  \
		Nav_KiAngle_d       =  0.007964444 ;  \
		Nav_KdAngle_d       =  2100.503704 ;


#endif


#endif /* INC_NAVCONSTANTS_H_ */
