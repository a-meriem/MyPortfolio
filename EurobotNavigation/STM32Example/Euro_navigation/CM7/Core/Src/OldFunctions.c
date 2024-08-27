/*
 * OldFunctions.c
 *
 *  Created on: Feb 16, 2023
 *      Author: flooki
 */
#include "OldFunctions.h"
Nav_ExecutePath_States_ten Nav_ExecutePath_State_en = Nav_ExecutePath_Idle_en;
Nav_TestPathStates_ten Nav_TestPathState_en = Nav_Test_Idle_en ;
Nav_WallAlignStates_ten Nav_WallAlignState_en = Nav_WallAlign_Idle_en ;
Nav_CalibrationPath_States_ten Nav_CalibrationPath_State_en = Nav_Calibration_Idle_en;

void Nav_TrapezeAngularVelocity()
{
	switch( Nav_TrapezePhase_en )
	{

	case Nav_Trapeze_Idle_en:
	{

	}break;
	case Nav_Trapeze_Accelerating_en:
	{
		Nav_ReferenceAngle_deg_d = ( Nav_TargetAngularVelocity_d * Nav_TargetAngularVelocity_d) / ( 2 * Nav_AngularAcc_d ) ;
		if( Nav_RemainingAngle_deg_d > Nav_ReferenceAngle_deg_d )
		{
			if ( Nav_TargetAngularVelocity_d < Nav_MaxAngularVelocity_d )
			{
				Nav_TargetAngularVelocity_d = Nav_AngularAcc_d * Nav_CurrentSample_u32 ;
			}
			else
			{
				Nav_TrapezePhase_en = Nav_Trapeze_ConstantVelocity_en;
			}
		}
		else
		{
			Nav_TrapezePhase_en = Nav_Trapeze_Decelerating_en;
		}

	}break;

	case Nav_Trapeze_ConstantVelocity_en:
	{
		Nav_ReferenceAngle_deg_d = (Nav_TargetAngularVelocity_d * Nav_TargetAngularVelocity_d) / ( 2 * Nav_AngularAcc_d ) ;
		if( Nav_RemainingAngle_deg_d > Nav_ReferenceAngle_deg_d )
		{
			Nav_TargetAngularVelocity_d = Nav_MaxAngularVelocity_d ;
		}
		else
		{
			Nav_AccelerationSamples_u32 = Nav_CurrentSample_u32 - 1;
			Nav_ReferenceAngularVelocity_d = Nav_TargetAngularVelocity_d;
			Nav_TrapezePhase_en = Nav_Trapeze_Decelerating_en;
		}
	}break;

	case Nav_Trapeze_Decelerating_en:
	{
		Nav_ReferenceAngle_deg_d = (Nav_TargetAngularVelocity_d * Nav_TargetAngularVelocity_d) / ( 2 * Nav_AngularAcc_d ) ;

		if( Nav_RemainingAngle_deg_d > Nav_ReferenceAngle_deg_d )
		{
			Nav_AngularAcc_d = ( Nav_TargetAngularVelocity_d * Nav_TargetAngularVelocity_d ) / ( 2 * Nav_RemainingAngle_deg_d );
			Nav_ReferenceAngularVelocity_d = Nav_TargetAngularVelocity_d;
			Nav_AccelerationSamples_u32 = Nav_CurrentSample_u32 - 1;
		}

		Nav_TargetAngularVelocity_d = Nav_ReferenceAngularVelocity_d - ( Nav_AngularAcc_d * ( Nav_CurrentSample_u32 -Nav_AccelerationSamples_u32 ) )  ;
		if( Nav_RemainingAngle_deg_d <= 0 )
		{
			Nav_TargetAngularVelocity_d = 0 ;
			Nav_TrapezePhase_en = Nav_Trapeze_Finished_en ;
		}
	}break;
	case Nav_Trapeze_OrderUrgentDecelerating_en:
	{
		Nav_AccelerationSamples_u32 = Nav_CurrentSample_u32 - 1;
		Nav_ReferenceLinearVelocity_d = Nav_TargetLinearVelocity_d;
		Nav_TrapezePhase_en = Nav_Trapeze_UrgentDecelerating_en;
	}break;
	case Nav_Trapeze_UrgentDecelerating_en :
	{
		Nav_TargetAngularVelocity_d = Nav_ReferenceAngularVelocity_d - ( Nav_AngularAcc_d * ( Nav_CurrentSample_u32 -Nav_AccelerationSamples_u32 ) )  ;
		if( Nav_RemainingAngle_deg_d <= 0 )
		{
			Nav_TargetAngularVelocity_d = 0 ;
			Nav_TrapezePhase_en = Nav_Trapeze_Finished_en ;

		}
	}break;

	case Nav_Trapeze_Finished_en:
	{
		Nav_TargetAngularVelocity_d = 0.0 ;
		Nav_TrapezePhase_en = Nav_Trapeze_Idle_en ;
	}break;

	}


}

void Nav_TrapezeLinearVelocity()
{
	switch( Nav_TrapezePhase_en )
	{
	case Nav_Trapeze_Idle_en:
	{

	}break;
	case Nav_Trapeze_Accelerating_en:
	{
		Nav_ReferenceDistance_d = (Nav_TargetLinearVelocity_d * Nav_TargetLinearVelocity_d) / ( 2 * Nav_Acc_d ) ;
		if( Nav_RemainingDistance_mm_d > Nav_ReferenceDistance_d )
		{
			if ( Nav_TargetLinearVelocity_d < Nav_MaxVelocity_d )
			{
				Nav_TargetLinearVelocity_d = Nav_Acc_d * Nav_CurrentSample_u32 ;
			}
			else
			{
				Nav_TrapezePhase_en = Nav_Trapeze_ConstantVelocity_en;
			}
		}
		else
		{
			Nav_TrapezePhase_en = Nav_Trapeze_Decelerating_en;
		}

	}break;

	case Nav_Trapeze_ConstantVelocity_en:
	{
		Nav_ReferenceDistance_d = (Nav_TargetLinearVelocity_d * Nav_TargetLinearVelocity_d) / ( 2 * Nav_Acc_d ) ;
		if( Nav_RemainingDistance_mm_d > Nav_ReferenceDistance_d )
		{
			Nav_TargetLinearVelocity_d = Nav_MaxVelocity_d ;
		}
		else
		{
			Nav_AccelerationSamples_u32 = Nav_CurrentSample_u32 - 1;
			Nav_ReferenceLinearVelocity_d = Nav_TargetLinearVelocity_d;
			Nav_TrapezePhase_en = Nav_Trapeze_Decelerating_en;
		}
	}break;

	case Nav_Trapeze_Decelerating_en:
	{
		Nav_ReferenceDistance_d = (Nav_TargetLinearVelocity_d * Nav_TargetLinearVelocity_d) / ( 2 * Nav_Acc_d ) ;

		if( Nav_RemainingDistance_mm_d > Nav_ReferenceDistance_d )
		{
			Nav_Acc_d=(Nav_TargetLinearVelocity_d * Nav_TargetLinearVelocity_d)/( 2 * Nav_RemainingDistance_mm_d );
			Nav_ReferenceLinearVelocity_d = Nav_TargetLinearVelocity_d;
			Nav_AccelerationSamples_u32 = Nav_CurrentSample_u32 - 1;
		}

		Nav_TargetLinearVelocity_d = Nav_ReferenceLinearVelocity_d - ( Nav_Acc_d * ( Nav_CurrentSample_u32 -Nav_AccelerationSamples_u32 ) )  ;

		if( Nav_RemainingDistance_mm_d <= 0 )
		{
			Nav_TargetLinearVelocity_d = 0 ;
			Nav_TrapezePhase_en = Nav_Trapeze_Finished_en ;
		}

	}break;

	case Nav_Trapeze_Finished_en:
	{
		Nav_TargetLinearVelocity_d = 0.0 ;
		Nav_TrapezePhase_en = Nav_Trapeze_Idle_en ;
	}break;

	}
}

void Nav_vWallAlign()
{
	switch(Nav_WallAlignState_en)
	{

	case Nav_WallAlign_Idle_en :
	{

	}break;
	case Nav_WallAlign_ResetVariables_en :
	{
		//reset the current sample
		Nav_CurrentSample_u32 = 0;

		//reset measuring variables
		ENCODER_RIGHT->CNT = ENCODER_RIGHT->ARR / 2 ;
		ENCODER_LEFT->CNT = ENCODER_LEFT->ARR / 2 ;
		Nav_PrevMeasuredDistance_mm_d = 0.0 ;
		Nav_PrevMeasuredAngle_rad_d   = 0.0 ;
		Nav_PrevMeasuredAngle_deg_d   = 0.0 ;
		Nav_vFeedbackRoutine();

		//reset the Target Distance variables
		Nav_CurrentTargetDistance_mm_d = 0.0 ;
		Nav_PrevTargetDistance_mm_d = 0.0 ;


		//reset the pid variables
		Nav_SumDistanceError_d = 0.0 ;
		Nav_SumAngleError_d = 0.0 ;


		Nav_WallAlignState_en = Nav_WallAlign_SetTarget_en;
	}break;
	case Nav_WallAlign_SetTarget_en :
	{
		Nav_MaxVelocity_d = Nav_MaxVelocity_d * SAMPLING_PERIOD ;
		Nav_Acc_d = Nav_Acc_d * SAMPLING_PERIOD * SAMPLING_PERIOD ;
		REVERSE_LINEAR_TRAJECTORY_PID
		Nav_WallAlignState_en = Nav_WallAlign_ConstantVelocityMove_en;
	}break;
	case Nav_WallAlign_ConstantVelocityMove_en :
	{
		Nav_CurrentSample_u32++;
		Nav_TrapezeLinearVelocity();
		Nav_vTargetDistanceCalculationRoutine();
		if(  ( Nav_CurrentSample_u32 >  Nav_AccelerationSamples_u32  )
				&& ( Nav_MeasuredVelocity_mm_d == 0 )
				&& ( Nav_MeasuredAngularVelocity_deg_d == 0 ))
		{
			Nav_Counter_u32 = 0 ;
			Nav_WallAlignState_en = Nav_WallAlign_ArrivedAtWall_en;
		}
	}break;

	case Nav_WallAlign_ArrivedAtWall_en :
	{
		//reset the current sample
		Nav_CurrentSample_u32 = 0;

		//reset measuring variables
		ENCODER_RIGHT->CNT = ENCODER_RIGHT->ARR / 2 ;
		ENCODER_LEFT->CNT = ENCODER_LEFT->ARR / 2 ;
		Nav_PrevMeasuredDistance_mm_d = 0.0 ;
		Nav_PrevMeasuredAngle_rad_d   = 0.0 ;
		Nav_PrevMeasuredAngle_deg_d   = 0.0 ;
		Nav_vFeedbackRoutine();

		//reset the Target Distance variables
		Nav_CurrentTargetDistance_mm_d = 0.0 ;
		Nav_PrevTargetDistance_mm_d = 0.0 ;

		//reset the pid variables
		Nav_SumDistanceError_d = 0.0 ;
		Nav_SumAngleError_d = 0.0 ;


		Nav_WallAlignState_en = Nav_WallAlign_Finish_en;
	}break;

	case Nav_WallAlign_Finish_en :
	{
		//Nav_WallAlignState_en = Nav_WallAlign_Idle_en;
	}break;
	}

}

void Nav_vExecutePath()
{
	switch (Nav_ExecutePath_State_en)
	{

	case Nav_ExecutePath_Idle_en :
	{
		Nav_ExecutePath_State_en = Nav_ExecutePath_SetNextTarget_en; // This transition will be placed in ROS_communication.cpp after receiving the X Y THETA array .
	}break;

	case Nav_ExecutePath_SetNextTarget_en :
	{
		if ( Nav_CurrentTargetNumber_u8 < ( Nav_NumberOfTargets_u8 ) )
		{
			Nav_ExecutePath_State_en = Nav_ExecutePath_GoToXY_en;
		}
		else
		{
			Nav_ExecutePath_State_en = Nav_ExecutePath_Finished_en ;
		}
	}break;

	case Nav_ExecutePath_GoToXY_en :
	{
		if( Nav_GoToXY_State_en == Nav_GoToXY_Idle_en )
		{
			Nav_TargetX_mm_d = Nav_XYTargets_ad[Nav_CurrentTargetNumber_u8][0];
			Nav_TargetY_mm_d = Nav_XYTargets_ad[Nav_CurrentTargetNumber_u8][1];
			Nav_TargetArrivingAngle_deg_d = Nav_XYTargets_ad[Nav_CurrentTargetNumber_u8][2];

			Nav_GoToXY_State_en = Nav_GoToXY_Rotate_en;
		}
		else if ( Nav_GoToXY_State_en == Nav_GoToXY_Finished_en )
		{

			Nav_CurrentTargetNumber_u8++;
			Nav_ExecutePath_State_en = Nav_ExecutePath_SetNextTarget_en ;
			Nav_GoToXY_State_en = Nav_GoToXY_Idle_en ;
		}
		else
		{
			Nav_vGoToXYStateMachine();
		}
	}break;

	case Nav_ExecutePath_Finished_en :
	{

	}break;

	}

	Nav_vNavigationRoutine();

}

void Nav_vXYPathStateMachine()
{
	switch( Nav_TestPathState_en )
	{

	case Nav_Test_Idle_en :
	{
		Nav_TestPathState_en = Nav_Test_Rotate_360deg_en;
	}break;
	case Nav_Test_WallAlign_en :
	{
		if ( Nav_WallAlignState_en == Nav_WallAlign_Idle_en )
		{
			Nav_MaxVelocity_d = -120 ; //400 ; //300 ;
			Nav_Acc_d = -350 ;
			Nav_TargetDistance_mm_d = -1000 ;
			Nav_WallAlignState_en = Nav_WallAlign_ResetVariables_en;
		}
		else if ( Nav_WallAlignState_en == Nav_WallAlign_Finish_en )
		{
			Nav_TestPathState_en = Nav_Test_Forward_1000mm_en;
			Nav_WallAlignState_en = Nav_WallAlign_Idle_en ;
		}
		else
		{
			Nav_vWallAlign();
		}
	}break;

	case Nav_Test_Forward_1000mm_en :
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			Nav_TrajectoryMode_en = Nav_ForwardLinearTrajectory_en;
			Nav_MaxVelocity_d = 500 ; //400 ; //300 ;//700
			Nav_Acc_d = 750 ;//750
			Nav_TargetDistance_mm_d = sqrt ( pow( Nav_TargetX_mm_d - Nav_StartX_mm_d ,2 ) + pow( Nav_TargetY_mm_d - Nav_StartY_mm_d , 2 ) ) ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_TestPathState_en = Nav_Test_Finished_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Test_Rotate_360deg_en :
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			//Nav_TrajectoryMode_en = Nav_AngularTrajectory_en;
			Nav_MaxAngularVelocity_d = 70 ; //70//150 ;
			Nav_AngularAcc_d = 150 ;//  //230
			Nav_TargetAngle_deg_d = ( atan2(Nav_TargetX_mm_d - Nav_StartX_mm_d , Nav_TargetY_mm_d - Nav_StartY_mm_d ) * 180.0 ) / M_PI;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{

			Nav_TestPathState_en = Nav_Test_Forward_1000mm_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Test_Reverse_1000mm_en :
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			Nav_TrajectoryMode_en = Nav_ReverseLinearTrajectory_en;
			Nav_MaxVelocity_d = -300 ; //-400 ; //-300 ;
			Nav_Acc_d = -750 ;
			Nav_TargetDistance_mm_d = -800 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_TestPathState_en = Nav_Test_Finished_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Test_Finished_en :
	{
		//Nav_TestPathState_en = Nav_Test_Idle_en;
	}break;
	}
	Nav_vNavigationRoutine();
}

void Nav_vTestPathStateMachine()
{
	switch( Nav_TestPathState_en )
	{

	case Nav_Test_Idle_en :
	{
		Nav_TestPathState_en = Nav_Test_Rotate_360deg_en;
	}break;
	case Nav_Test_WallAlign_en :
	{
		if ( Nav_WallAlignState_en == Nav_WallAlign_Idle_en )
		{
			Nav_MaxVelocity_d = -120 ; //400 ; //300 ;
			Nav_Acc_d = -350 ;
			Nav_TargetDistance_mm_d = -1000 ;
			Nav_WallAlignState_en = Nav_WallAlign_ResetVariables_en;
		}
		else if ( Nav_WallAlignState_en == Nav_WallAlign_Finish_en )
		{
			Nav_TestPathState_en = Nav_Test_Forward_1000mm_en;
			Nav_WallAlignState_en = Nav_WallAlign_Idle_en ;
		}
		else
		{
			Nav_vWallAlign();
		}
	}break;

	case Nav_Test_Forward_1000mm_en :
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			Nav_TrajectoryMode_en = Nav_ForwardLinearTrajectory_en;
			Nav_MaxVelocity_d = 700 ; //400 ; //300 ;
			Nav_Acc_d = 750 ;
			Nav_TargetDistance_mm_d = 800 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_TestPathState_en = Nav_Test_Reverse_1000mm_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Test_Rotate_360deg_en :
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			//Nav_TrajectoryMode_en = Nav_AngularTrajectory_en;
			Nav_MaxAngularVelocity_d = 150 ; //70 ;
			Nav_AngularAcc_d = 230 ;
			Nav_TargetAngle_deg_d = 20 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_TestPathState_en = Nav_Test_Reverse_1000mm_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Test_Reverse_1000mm_en :
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			Nav_TrajectoryMode_en = Nav_ReverseLinearTrajectory_en;
			Nav_MaxVelocity_d = -300 ; //-400 ; //-300 ;
			Nav_Acc_d = -750 ;
			Nav_TargetDistance_mm_d = -800 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_TestPathState_en = Nav_Test_Finished_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Test_Finished_en :
	{
		//Nav_TestPathState_en = Nav_Test_Idle_en;
	}break;
	}
	Nav_vNavigationRoutine();
}

void Nav_vWheelDiameterCalibrationStateMachine()
{
	switch(Nav_CalibrationPath_State_en)
	{
	case Nav_Calibration_Idle_en:
	{
		Nav_CalibrationPath_State_en = Nav_Calibration_WallAlign_en;
	}break;

	case Nav_Calibration_WallAlign_en:
	{

		if ( Nav_WallAlignState_en == Nav_WallAlign_Idle_en )
		{
			Nav_MaxVelocity_d = -120 ; //400 ; //300 ;
			Nav_Acc_d = -350 ;
			Nav_TargetDistance_mm_d = -1000 ;
			Nav_WallAlignState_en = Nav_WallAlign_ResetVariables_en;

		}
		else if ( Nav_WallAlignState_en == Nav_WallAlign_Finish_en )
		{
			Nav_CalibrationPath_State_en = Nav_Calibration_Forward_1300mm_en;
			Nav_WallAlignState_en = Nav_WallAlign_Idle_en ;
		}
		else
		{
			Nav_vWallAlign();
		}

	}break;

	case Nav_Calibration_Forward_1300mm_en:
	{
		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			Nav_TrajectoryMode_en = Nav_ForwardLinearTrajectory_en;
			Nav_MaxVelocity_d = 300 ; //400 ; //700 ;
			Nav_Acc_d = 750 ;
			Nav_TargetDistance_mm_d = 1000 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_CalibrationPath_State_en = Nav_Calibration_Rotate_180_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}
	}break;

	case Nav_Calibration_Rotate_180_en:
	{

		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			//Nav_TrajectoryMode_en = Nav_AngularTrajectory_en;
			Nav_MaxAngularVelocity_d = 150 ; //70 ;
			Nav_AngularAcc_d = 230 ;
			Nav_TargetAngle_deg_d = 180 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_CalibrationPath_State_en = Nav_Calibration_Forward_1000mm_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}

	}break;

	case Nav_Calibration_Forward_1000mm_en:
	{

		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			Nav_TrajectoryMode_en = Nav_ForwardLinearTrajectory_en;
			Nav_MaxVelocity_d = 300 ; //400 ; //700 ;
			Nav_Acc_d = 750 ;
			Nav_TargetDistance_mm_d = 1000 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_CalibrationPath_State_en = Nav_Calibration_RotateReverse_180_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}

	}break;

	case Nav_Calibration_RotateReverse_180_en:
	{

		if ( Nav_TrajectoryState_en == Nav_Trajectory_Idle_en )
		{
			//Nav_TrajectoryMode_en = Nav_AngularTrajectory_en;
			Nav_MaxAngularVelocity_d = -150 ; //70 ;
			Nav_AngularAcc_d = -230 ;
			Nav_TargetAngle_deg_d = -180 ;
			Nav_TrajectoryState_en = Nav_Trajectory_ResetVariables_en;
		}
		else if ( Nav_TrajectoryState_en == Nav_Trajectory_Finish_en)
		{
			Nav_CalibrationPath_State_en = Nav_Calibration_CalculateWallAlign_en;
			Nav_TrajectoryState_en = Nav_Trajectory_Idle_en;
		}
		else
		{
			Nav_vIndependantTrajectoryStateMachine();
		}

	}break;

	case Nav_Calibration_CalculateWallAlign_en:
	{
		if ( Nav_WallAlignState_en == Nav_WallAlign_Idle_en )
		{
			Nav_MaxVelocity_d = -120 ; //400 ; //300 ;
			Nav_Acc_d = -350 ;
			Nav_TargetDistance_mm_d = -1000 ;
			Nav_WallAlignState_en = Nav_WallAlign_ResetVariables_en;
		}
		else if ( Nav_WallAlignState_en == Nav_WallAlign_Finish_en )
		{
			Nav_CalibrationPath_State_en = Nav_Calibration_CalibrateWheelDiameters_en;
			Nav_WallAlignState_en = Nav_WallAlign_Idle_en ;
		}
		else
		{
			Nav_vWallAlign();
		}

	}break;

	case Nav_Calibration_CalibrateWheelDiameters_en:
	{

		//		Nav_WheelDiamaterFactor_d = ( Nav_MeasuredDistanceRight_mm_d -  Nav_MeasuredDistanceLeft_mm_d ) / ( Nav_MeasuredDistance_mm_d  ) ;
		//
		//		Nav_NewLeftWheelGain_d =  ( 1 + Nav_WheelDiamaterFactor_d ) ; //* Nav_OldLeftWheelGain_d  ;
		//		Nav_NewRightWheelGain_d = ( 1 - Nav_WheelDiamaterFactor_d ) ; //* Nav_OldRightWheelGain_d ;
		//
		//		Nav_OldLeftWheelGain_d = Nav_NewLeftWheelGain_d;
		//		Nav_OldRightWheelGain_d = Nav_NewRightWheelGain_d;

		//Nav_NewLeftWheelGain_d = (   ( Nav_MeasuredDistanceRight_mm_d )    /   (     (double)  (   ( 2 * Nav_MeasuredDistanceRight_mm_d ) - Nav_MeasuredDistanceLeft_mm_d   )  )  )  ;

		Nav_TargetTicksLeft_d = ( (double)   Nav_LeftTicks_i32  - (    ( ( double) (  Nav_MeasuredDistanceRight_mm_d - Nav_MeasuredDistanceLeft_mm_d  ) )  / ( (double) ( Nav_ConvertRight_Ticks2mm_d * Nav_LeftWheelGain_d ) ) )  );

		Nav_NewLeftWheelGain_d = (double) ( Nav_MeasuredDistanceRight_mm_d / ( ( double ) (Nav_TargetTicksLeft_d * Nav_ConvertRight_Ticks2mm_d )  ) );

		Nav_vNavigatorInit();

		Nav_OldDifferenceBetweenWheels_d = ( Nav_MeasuredDistanceRight_mm_d -  Nav_MeasuredDistanceLeft_mm_d );

		//		Nav_WheelDiamaterDifference_d = (   (    (double) (Nav_RightTicks_i32)    )    /    (     (double) (    (2 * Nav_RightTicks_i32) - Nav_LeftTicks_i32   ) ) ) ;

		Nav_CalibrationPath_State_en = Nav_Calibration_Finished_en ;
	}break;

	case Nav_Calibration_Finished_en:
	{
		//Nav_CalibrationPath_State_en = Nav_Calibration_Idle_en ;
	}break;

	}
	Nav_vNavigationRoutine();
}

void Nav_XY_Requirements()
{
	double Diff_X=Nav_TargetX_mm_d-Nav_StartX_mm_d;
	double Diff_Y=Nav_TargetY_mm_d-Nav_StartY_mm_d;

	Nav_TargetDistance_mm_d=sqrt(pow(Diff_X,2 )+pow(Diff_Y,2));
	Nav_TargetAngle_deg_d=atan2(Diff_X,Diff_Y);


}

//void Nav_CalculateNextTarget()
//{
//
//	//Alpha = atan ( ( Y_Target - Y )  /  ( X_Target - X) ) ;
//
//	//AlphaDeg = Alpha * 180.0  / M_PI ;
//
//	//Theta_New = 0.0 ;
//
//	//Theta_New  = Alpha * Entrax * Conversion_Metre_Ticks - Theta_ticks   ;
//
//	//Theta_New_Rad = Theta_New / ( Entrax * Conversion_Metre_Ticks ) ;
//
//	//Distance_rest = sqrt ( ( Y_Target - Y ) * ( Y_Target - Y ) + ( X_Target - X) * ( X_Target - X) ) ;
//
//
//
//	//if ( X_Target < 0 )
//	//		{
//	//			if ( Y_Target < 0 )
//	//			{
//	//				Theta_New = -3.14159 + atan( Fprime ) ;  //     -180< theta < -90
//	//			}
//	//			else if ( final_y > 0 )
//	//			{
//	//				theta_new = 3.14159 + atan( Fprime )  - AngleOffset ;  //       90< theta <180
//	//			}
//	//		}
//
//	//angle a atteindre par le centre du robot
//
//	// Theta_New  =  Theta_New = M_PI * Entrax * Conversion_Metre_Ticks ; /*- Theta_ticks*/;
//
//}
