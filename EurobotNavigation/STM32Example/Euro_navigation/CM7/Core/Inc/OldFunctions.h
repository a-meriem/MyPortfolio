/*
 * OldFunctions.h
 *
 *  Created on: Feb 16, 2023
 *      Author: flooki
 */

#ifndef INC_OLDFUNCTIONS_H_
#define INC_OLDFUNCTIONS_H_


typedef enum Nav_WallAlignStates_ten
{
	Nav_WallAlign_Idle_en,
	Nav_WallAlign_ResetVariables_en,
	Nav_WallAlign_SetTarget_en,
	Nav_WallAlign_ConstantVelocityMove_en,
	Nav_WallAlign_ArrivedAtWall_en,
	Nav_WallAlign_Finish_en
}Nav_WallAlignStates_ten;


typedef enum Nav_Calibration_PathStates_ten
{
	Nav_Calibration_Idle_en,
	Nav_Calibration_WallAlign_en,
	Nav_Calibration_Forward_1300mm_en,
	Nav_Calibration_Rotate_180_en,
	Nav_Calibration_Forward_1000mm_en,
	Nav_Calibration_RotateReverse_180_en,
	Nav_Calibration_CalculateWallAlign_en,
	Nav_Calibration_CalibrateWheelDiameters_en,
	Nav_Calibration_Finished_en
}Nav_CalibrationPath_States_ten;


void Nav_vTestPathStateMachine();
void Nav_vWheelDiameterCalibrationStateMachine();
void Nav_vWallAlign();
void Nav_vXYPathStateMachine();
void Nav_vExecutePath();

#endif /* INC_OLDFUNCTIONS_H_ */
