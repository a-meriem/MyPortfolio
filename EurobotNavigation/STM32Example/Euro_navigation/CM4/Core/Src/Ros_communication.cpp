/*
 * my_main.cpp
 *
 *  Created on: 2018年6月22日
 *      Author: Johnson
 *
 *
   	1st terminal. roscore
	2nd terminal. rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
	3rd terminal. rostopic list 、 rostopic echo /chatter
 *
 */
#include "common.h"
#include "ringbuff.h"
#include "math.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "ringbuffer.h"
#include "Ros_communication.h"
#include <ros.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include <string>


uint8_t ROS_DelayCounter = 0 ;
extern UART_HandleTypeDef huart3;


extern float ROBOT_CurrentXYTheta_fromCM7_arr [4] ;
float ROBOT_CurrentStatuts_i8 [] = {0.0} ;
extern volatile ringbuff_t* rb_cm4_to_cm7 ;
extern volatile ringbuff_t* rb_cm7_to_cm4 ;
ROS_Node_States_ten ROS_Node_CurrentState_en = ROS_Node_idle_en ;

const static uint16_t rbuflen = 1024;
uint8_t RxBuffer[rbuflen];
struct ringbuffer rb;

ros::NodeHandle nh;

int8_t MASTER_Navigator_MessageRecieved = 0         ;
int8_t MASTER_Navigator_Stop_Flag = 1               ;
int8_t MASTER_Navigator_WaitForOrders = 2 			;
int8_t MASTER_Navigator_Can_GO = 3                  ;
int8_t MASTER_Navigator_Still_IDLE = 4 				;

float Nav_Orders_fromROS_arr [20] = {0,0};

nav_msgs::Odometry ROBOT_CurrentXYTheta_toROS_arr ;
std_msgs::Float32MultiArray Nav_XYTargets_RecievedFromMaster;
std_msgs::Int8 				Nav_Robot_Current_Status_i8 ;


void Nav_Orders_MessageRecievedCallback( const std_msgs::Float32MultiArray &msg ) ;

ros::Publisher Nav_CurrentXYThetaPublisher("Location_state", &ROBOT_CurrentXYTheta_toROS_arr);
ros::Publisher Nav_CurrentStatutsPublisher("Robot_Status"  , &Nav_Robot_Current_Status_i8 ) ;
ros::Subscriber<std_msgs::Float32MultiArray> Nav_Orders_Subscriber("MASTER_Orders", & Nav_Orders_MessageRecievedCallback);

void ROS_SpinOnce(void)
{
	nh.spinOnce();
}

/*
 * Brief : This function will be called every time MASTER publish new orders.
 *  first state is to verify the two  nodes are connected.
 * Second state is the real communication and will be sending ORDERS to CM7
 */
void Nav_Orders_MessageRecievedCallback(const std_msgs::Float32MultiArray & msg)
{
	nh.loginfo("STM GOT THE ORDERS");
	switch( ROS_Node_CurrentState_en)
	{
	case ROS_Node_idle_en :
	{
		vNav_CurrentStatus_Publisher();
		ROS_Node_CurrentState_en = ROS_Node_Communicating ;
		nh.spinOnce();
	}break;

	case ROS_Node_Communicating :
	{

		Nav_XYTargets_RecievedFromMaster.data = msg.data ;
		for ( uint8_t index = 0 ; index < msg.data_length ; index ++ )
		{
			Nav_Orders_fromROS_arr[index] = (float) Nav_XYTargets_RecievedFromMaster.data[index] ;
		}
		if (Nav_Orders_fromROS_arr[0] == MASTER_Navigator_Still_IDLE)
		{
			Nav_Robot_Current_Status_i8.data =(int8_t) MASTER_Navigator_MessageRecieved 	;
			Nav_CurrentStatutsPublisher.publish(&Nav_Robot_Current_Status_i8)		;
		}
		else if(  Nav_Orders_fromROS_arr[0] == MASTER_Navigator_Stop_Flag )
		{
			nh.loginfo("SuddenStop");
			Nav_Robot_Current_Status_i8.data =(int8_t) MASTER_Navigator_Stop_Flag 	;
			//Nav_CurrentStatutsPublisher.publish(&Nav_Robot_Current_Status_i8)		;


		}
		else
		{

			// Navigator acknowledging he got the message !
			Nav_Robot_Current_Status_i8.data =(int8_t) MASTER_Navigator_MessageRecieved 	;
			Nav_CurrentStatutsPublisher.publish(&Nav_Robot_Current_Status_i8)	;

		}
		ringbuff_write(rb_cm4_to_cm7, Nav_Orders_fromROS_arr, 4 * sizeof(float));
	}break;
	ROS_SpinOnce();

	}
}


extern "C" void USART_RX_Callback()
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	ringbuffer_putchar(&rb, huart3.Instance->RDR);
}

void vROS_Setup()
{

	ringbuffer_init(&rb, RxBuffer, rbuflen);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	//	__HAL_UART_ENABLE_IT(&huart3,UART_IT_TXE);

	// Initialize ROS
	nh.initNode();
	nh.subscribe(Nav_Orders_Subscriber);
	nh.advertise(Nav_CurrentStatutsPublisher);
	nh.advertise(Nav_CurrentXYThetaPublisher) ;

	nh.spinOnce();

}

/*
 * Brief : This function will be called from Timer callback  ONLY WHEN there is an update in the status coming from CM7
 * */

void vNav_CurrentStatus_Publisher()
{
	nh.loginfo("Status PUBLISHER");

	Nav_Robot_Current_Status_i8.data =(int8_t) ROBOT_CurrentStatuts_i8 [0] 	;
	Nav_CurrentStatutsPublisher.publish(&Nav_Robot_Current_Status_i8)	;
	nh.spinOnce();
}

/*
 * Brief : This function will be called on every TIMER call back
 */

void vNav_CurrentXYTheta_Publisher()
{
	nh.loginfo("X Y THETA PUBLISHER");
	ROBOT_CurrentXYTheta_toROS_arr.pose.pose.position.x 	= ROBOT_CurrentXYTheta_fromCM7_arr [0]  ;
	ROBOT_CurrentXYTheta_toROS_arr.pose.pose.position.y 	= ROBOT_CurrentXYTheta_fromCM7_arr [1]  ;
	ROBOT_CurrentXYTheta_toROS_arr.pose.pose.orientation.z 	= ROBOT_CurrentXYTheta_fromCM7_arr [2]  ;
	ROBOT_CurrentXYTheta_toROS_arr.twist.twist.linear.x		= ROBOT_CurrentXYTheta_fromCM7_arr [3]  ;
	Nav_CurrentXYThetaPublisher.publish(&ROBOT_CurrentXYTheta_toROS_arr) ;
	nh.spinOnce();
}



