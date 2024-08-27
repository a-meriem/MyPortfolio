/*
 * Navigation.h
 *
 *  Created on: 11 déc. 2022
 *      Author: ASUS
 */

#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

/* Private typedef BEGIN------------------------------------------------------*/
typedef enum Nav_Navigator_Indicators
{
	MASTER_Navigator_MessageRecieved,  		// NAvigator indicates its status .
	MASTER_Navigator_Stop_Flag,           	// NAvigator indicates its status .
	MASTER_Navigator_WaitForOrders, 		// Forced by MASTER
	MASTER_Navigator_Can_GO,				// Forced by MASTER
	MASTER_Navigator_Still_IDLE				// Forced by MASTER

}Nav_Navigator_Indicators;

typedef struct Recieved_Orders_FromROS
{
	Nav_Navigator_Indicators Indicator ;
	float X_target_i32 ;
	float Y_Target_i32 ;
	float TETA_Target_i32 ;
}Recieved_Orders_FromROS;

typedef struct ROBOT_CurrentState_typedef
{
	Nav_Navigator_Indicators Indicator  ;
	int32_t Current_X_i32 	 			;
	int32_t Current_Y_i32 	 			;
	int32_t Current_TETA_i32		    ;

}ROBOT_CurrentState_typedef;

typedef enum Nav_TrajectoryStates_ten
{
	Nav_Trajectory_Idle_en,
	Nav_Trajectory_ResetVariables_en,
	Nav_Trajectory_SetPath_en,
	Nav_Trajectory_WaitToExecute_en,
	Nav_Trajectory_Execute_en,
	Nav_Trajectory_OrderSuddenStop_en,
	Nav_Trajectory_ExecuteSuddenStop_en,
	Nav_Trajectory_WaitAfterArriving_en,
	Nav_Trajectory_Finish_SuddenStop_en,
	Nav_Trajectory_Finish_en
}Nav_TrajectoryStates_ten;

typedef enum Nav_TestPathStates_ten
{
	Nav_Test_Idle_en,
	Nav_Test_WallAlign_en,
	Nav_Test_Forward_1000mm_en,
	Nav_Test_Rotate_360deg_en,
	Nav_Test_Reverse_1000mm_en,
	Nav_Test_Finished_en
}Nav_TestPathStates_ten;

typedef enum Nav_GoToXY_States_ten
{
	Nav_GoToXY_Idle_en,
	Nav_GoToXY_Rotate_en,
	Nav_GoToXY_Forward_en,
	Nav_GoToXY_RotateAfter_en,
	Nav_GoToXY_OrderSuddenStop_en,
	Nav_GoToXY_ExecuteSuddenStop_en,
	Nav_GoToXY_Finished_en
}Nav_GoToXY_States_ten;

typedef enum Nav_Navigator_States_en
{
	Nav_Navigator_WaitForOrders_en,
	Nav_Navigator_StartExecutingPath_en,
	Nav_Navigator_SuddenStop_en,
	Nav_Navigator_ConfirmedStop_en,
	Nav_Navigator_Finished_en
}Nav_Navigator_States_en;


typedef enum Nav_ExecutePath_States_ten
{
	Nav_ExecutePath_Idle_en,
	Nav_ExecutePath_SetNextTarget_en,
	Nav_ExecutePath_GoToXY_en,
	Nav_ExecutePath_Finished_en

}Nav_ExecutePath_States_ten;



typedef enum Nav_TrajectoryModes_ten
{
	Nav_ForwardLinearTrajectory_en,
	Nav_ReverseLinearTrajectory_en,
	Nav_ForwardAngularTrajectory_en,
	Nav_ReverseAngularTrajectory_en
}Nav_TrajectoryModes_ten;

typedef enum Nav_TrapezePhases_ten
{
	Nav_Trapeze_Idle_en,
	Nav_Trapeze_Accelerating_en,
	Nav_Trapeze_ConstantVelocity_en,
	Nav_Trapeze_Decelerating_en,
	Nav_Trapeze_OrderUrgentDecelerating_en,
	Nav_Trapeze_UrgentDecelerating_en,
	Nav_Trapeze_Finished_en
}Nav_TrapezePhases_ten;


/* Private typedef END--------------------------------------------------------*/


/* Private define BEGIN-------------------------------------------------------*/
#define ENCODER_RIGHT	TIM2
#define ENCODER_LEFT	TIM5
#define MOTOR_PWM		TIM1


#define M_2PI			6.28318530717958647692

#define MOTOR_DIRECTION_FORWARD					1
#define MOTOR_DIRECTION_REVERSE					-1
#define MAXIMUM_MOTOR_COMMAND  					5000

#define MINIMUM_RIGHT_MOTOR_FORWARD_COMMAND	 	535

#define MINIMUM_RIGHT_MOTOR_REVERSE_COMMAND		480

#define MINIMUM_LEFT_MOTOR_FORWARD_COMMAND		390

#define MINIMUM_LEFT_MOTOR_REVERSE_COMMAND		450

#define MINIMUM_SATURATION_COMMAND				50

#define SAMPLING_PERIOD   		0.005

/* Private define END---------------------------------------------------------*/


/* Private macro BEGIN--------------------------------------------------------*/
/* Private macro END----------------------------------------------------------*/


void Nav_vNavigatorInit();
void Nav_vNavigationRoutine();
void Nav_vFeedbackRoutine();
void Nav_vLocalisation();
void Nav_vPidDistance();
void Nav_vPidAngle();
void Nav_vCalculateMotorsCommand();
void Nav_vMotorsRoutine();
void Nav_vTargetDistanceCalculationRoutine();
void Nav_vTargetAngleCalculationRoutine();
void Nav_vCalculateLinearTrajectory();
void Nav_vCalculateAngularTrajectory();
void Nav_TrapezeLinearVelocity();
void Nav_TrapezeAngularVelocity();
void Nav_TrapezeLinearVelocity2();
void Nav_TrapezeAngularVelocity2();
void Nav_TrapezeReverseAngularVelocity2();
void Nav_vSaturateRemainingAngle();
void Nav_vIndependantTrajectoryStateMachine();
void Nav_Navigator_StateMachine ();
void Nav_vCurrentTargetAngleFollower();
void Nav_vGoToXYStateMachine();
#endif /* INC_NAVIGATION_H_ */
