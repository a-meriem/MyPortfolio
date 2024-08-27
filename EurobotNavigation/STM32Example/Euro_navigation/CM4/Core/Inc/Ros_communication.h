/*
 * my_main.h
 *
 *  Created on: 06/01/2023
 *      Author: flooki
 */

#ifndef MY_MAIN_H_
#define MY_MAIN_H_




typedef enum
{

ROS_Node_idle_en ,
ROS_Node_Communicating

}ROS_Node_States_ten;

void ROS_SpinOnce(void);
void vNav_CurrentStatus_Publisher();
void vROS_Setup();
void vNav_CurrentXYTheta_Publisher();
void USART_CharReception_Callback();

#ifdef __cplusplus
extern "C"{
#endif


#ifdef __cplusplus
}
#endif

#endif /* MY_MAIN_H_ */
