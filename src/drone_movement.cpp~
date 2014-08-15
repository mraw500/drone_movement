///////////////////////////////////////////////////////////////////////////////
//Source for drone_movment node to send control commands to AR-Drone				 //
//v1.0									     																								 //
//First creation							    																					 //
//Huseyin Emre Erdem																										     //
//15.08.2014																														     //
///////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <sstream>
#include "std_msgs/Duration.h"
#include "pos.h"
#include "UInt8.h"
#include "ardrone_autonomy/Navdata.h"//For tag detection

/*Prototypes*/
void getTargetPos(const drone_movement::pos posReceived);//Processes data read from drone_target_pos topic
void getPattern(const ardrone_autonomy::Navdata& navdataReceived);//Processes pattern recognition data read from /ardrone/navdata topic

/*Main function*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_movement"); //Create node called talker
  ros::NodeHandle n;//Create nodehandle to alter node parameters
  ros::Duration d = ros::Duration(2,0);//Duration object to pause node

	/*Publishers & Subscribers*/
  ros::Publisher landPub = n.advertise<std_msgs::Empty>("/ardrone/land",1);//Publisher to send landing commands
  ros::Publisher takeoffPub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);//Publisher to send takeoff commands
  //ros::Publisher drone_pub = n.advertise<std_msgs::String>("tum_ardrone/com",50);
	ros::Subscriber targetSub = n.subscribe("drone_target_pos", 1, getTargetPos);//To collect the target position published by i90_sensor_board
	ros::Subscriber patternSub = n.subscribe("/ardrone/navdata", 1, getPattern);//To collect the target position published by i90_sensor_board

  d.sleep(); //Wait for 2 seconds to allow publishers & subscribers to get ready
	
	/*Send takeoff command*/
  takeoffPub.publish(std_msgs::Empty());//Publish take0ff message
  ROS_INFO("TAKEOFF!");//Inform user on the terminal

  /*Initialize PTAM*/
	/*std_msgs::String s;
	std::string c;
	c = "c autoInit";
	s.data = c.c_str();
	drone_pub.publish(s);
	d.sleep();*/
  //land_pub.publish(std_msgs::Empty());
  //c = "c land";
	//s.data = c.c_str();
	//drone_pub.publish(s);
	//ROS_INFO("LAND!");
	/*Land*/
	
	landPub.publish(std_msgs::Empty());//Publish landing command
	ROS_INFO("LAND!");//Inform user

  while(ros::ok()){
  
  ros::spinOnce();

  //loop_rate.sleep();
  }
  return 0;
}

/*Processes data read from drone_target_pos topic*/
void getTargetPos(const drone_movement::pos posReceived){
	ROS_INFO("Target position: [%f]/t[%f]", posReceived.fXPos, posReceived.fYPos, posReceived.fYawAngle);
}

/*Processes pattern recognition data read from /ardrone/navdata topic*/
void getPattern(const ardrone_autonomy::Navdata& navdataReceived){
	ROS_INFO("Tags detected: [%u]", navdataReceived.tags_count);
}

