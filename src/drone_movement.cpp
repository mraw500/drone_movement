///////////////////////////////////////////////////////////////////////////////
//Source for drone_movment node to send control commands to AR-Drone				 //
//v1.0									     																								 //
//First creation							    																					 //
//Huseyin Emre Erdem																										     //
//Edgar Buchanan																														 //
//15.08.2014																														     //
///////////////////////////////////////////////////////////////////////////////

//Pattern recognition node added
//Error compensation implemented
//Timing between commands modified

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <sstream>
#include "std_msgs/Duration.h"
#include <unistd.h>
#include "pos.h"
#include "UInt8.h"
#include "Navdata.h"//For tag detection
#include <cmath>
#include <cstdio>
#include "filter_state.h"

#define fSearchDistance 0.5 //Define distance to explore at moment of searching for tag
#define fDroneHeight 1.25

/*Variables*/
float fTargetPosX;
float fTargetPosY;
float fActualPosX;
float fActualPosY;
float fYawAngle;
float fPassingX;
float fPassingY;
int iInitilizeFlag;
int iReturnFlag;
int iPatternFlag;

/*Prototypes*/
void getTargetPos(const drone_movement::pos posReceived);//Processes data read from drone_target_pos topic
void getPattern(const ardrone_autonomy::Navdata& navdataReceived);//Processes pattern recognition data read from /ardrone/navdata topic
void getActualPos(const tum_ardrone::filter_state& actualPos);//Processes drone position from /ardrone/predictedPose topic

/*Main function*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_movement"); //Create node called drone_movement
  ros::NodeHandle n;//Create nodehandle to alter node parameters
  ros::Duration d = ros::Duration(2,0);//Duration object to pause node

	/*Publishers*/
  ros::Publisher landPub = n.advertise<std_msgs::Empty>("/ardrone/land",1);//Publisher to send landing commands
  ros::Publisher takeoffPub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);//Publisher to send takeoff commands
  ros::Publisher drone_pub = n.advertise<std_msgs::String>("tum_ardrone/com",50);//to send (Publish) command to Ar-Drone
	ros::Publisher drone_target_reached_pub = n.advertise<drone_movement::UInt8>("drone_target_reached", 1); //to publish message saying drone_target_reached to topic
	/*Subscribers*/
	ros::Subscriber targetSub = n.subscribe("i90_current_pos", 1, getTargetPos);//To collect the target position published by i90_sensor_board
	ros::Subscriber patternSub = n.subscribe("/ardrone/navdata", 1, getPattern);//To collect patter recognition information
	ros::Subscriber actualPosSub = n.subscribe("/ardrone/predictedPose", 1, getActualPos);//To collect actual robot position

	d.sleep(); //Wait for 2 seconds to allow publishers & subscribers to get ready

  /*Initialize PTAM*/
	std_msgs::String s;
	std::string c;
	int count = 0;
	static pthread_mutex_t send_CS;
	drone_movement::UInt8 message;

	iInitilizeFlag = 0;
	iReturnFlag = 0;
	iPatternFlag = 0;

  while(ros::ok()){
		message.data = count++;
		drone_target_reached_pub.publish(message);
	
		usleep(500000);
		ROS_INFO("Pattern flag: [%i]", iPatternFlag);
		ROS_INFO("Current pos: X = [%f] Y = [%f]", fActualPosX, fActualPosY);
		ROS_INFO("Target pos: X = [%f] Y = [%f]", fTargetPosX, fTargetPosY);

		/*Once target coordinates are received launch and move towards target*/
		if(iInitilizeFlag == 1 && iReturnFlag == 0){		
			
			/*Set origin in actual position*/
			c = "c setReference $POSE$"; //Set origin in robot's actual position
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);			
			ROS_INFO("setReference!");//Inform user on the terminal

			d.sleep();			
			
			/*Takeoff drone*/
			takeoffPub.publish(std_msgs::Empty());
			ROS_INFO("TAKEOFF!");
			
			d.sleep();
			d.sleep();
			d.sleep();


			/*Set initial parameters*/
			c = "c autoTakeOver 500 800";
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("autoTakeOver!");

			c = "c setMaxControl 0.1";	//Limit AR-drone high speed
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setMaxControl!");

			c = "c setInitialReachDist 0.25";	//Reach target within 0.1m 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setInitialReachDist!");

			c = "c setStayWithinDist 0.25";		//Stay in target location within 0.1m
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setStayWithinDist!");	

			c = "c setStayTime 2";						//Stay in target location for 3s
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setStayTime!");	

			c = "c lockScaleFP";							//Fix PTAM
			s.data = c.c_str();
			drone_pub.publish(s);
			d.sleep();
			ROS_INFO("lockScaleFP!");		


			/*Select passing point as starting point*/	
			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 0.75 %.2f", fPassingX * 1.5, fPassingY * 1.5, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fPassingX * 1.5, fPassingY * 1.5);

			/*Move towards target*/
			c = " ";		
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX, fTargetPosY, fYawAngle);	
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fTargetPosX, fTargetPosY);
			
			/*Search tag*/
			c = " ";	
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX + fPassingY - fSearchDistance, fTargetPosY + fPassingX - fSearchDistance, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("goto 1 0!");
	
			c = " ";		
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX + fPassingY - fSearchDistance, fTargetPosY + fPassingY - fSearchDistance, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("goto 0 -1!");

			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX  + fPassingX - fSearchDistance, fTargetPosY + fPassingY - fSearchDistance, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("goto -1 0!");

			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX  + fPassingX - fSearchDistance, fTargetPosY  + fPassingX - fSearchDistance, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("goto 0 1!");

			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX + fPassingY - fSearchDistance, fTargetPosY + fPassingX - fSearchDistance, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("goto 1 0!");
		
			/*Return home in case that target is not found*/
			/*Return to appropiate passing point*/
			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 0.75 %.2f", fPassingX * 1.5, fPassingY * 1.5, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fPassingX * 1.5, fPassingY * 1.5);

			/*Return origin*/
			c = " ";
			c = "c goto -0.5 -0.5 0.25 0";						
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("goto origin!");

			iInitilizeFlag = 0;
		}
		
		/*Return home are once pattern is found*/
		if(iPatternFlag == 1 && iReturnFlag == 0){

			/*Clear commands*/
			c = " ";
			c = "c clearCommands";						
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("clearCommands!");
			iReturnFlag = 1;

			/*Move towards target*/
			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", fTargetPosX, fTargetPosY, fYawAngle);	
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fTargetPosX, fTargetPosY);

			/*Return to appropiate passing point*/
			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 0.75 %.2f", fPassingX * 1.5, fPassingY * 1.5, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fPassingX * 1.5, fPassingY * 1.5);

			/*Return origin*/
			c = " ";
			c = "c goto -0.5 -0.5 0.25 0";						
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("goto origin!");
		}
		
		/*Land once drone is in home area*/
		if(iReturnFlag == 1 && (fActualPosX < 0.2 &&  fActualPosY < 0.2)){ 
			c = " ";
			landPub.publish(std_msgs::Empty());
			ROS_INFO("LAND!");
		}

		ros::spinOnce();																										     //
  }
  return 0;
}
/*Processes data read from drone_target_pos topic*/
void getTargetPos(const drone_movement::pos posReceived){
	ROS_INFO("Target position: [%f]/t[%f]/t[%f]", posReceived.fXPos, posReceived.fYPos, posReceived.fYawAngle);
	fTargetPosX = posReceived.fXPos;
	fTargetPosY = posReceived.fYPos;
 	fYawAngle = posReceived.fYawAngle;	
	float fPassingPoint1[2] = {2,0},fPassingPoint2[2] = {0,2};
  double dDistance1, dDistance2;
  dDistance1 = sqrt(((fPassingPoint1[0] - fTargetPosX) * (fPassingPoint1[0] - fTargetPosX)) + ((fPassingPoint1[1] - fTargetPosY) * (fPassingPoint1[1] - fTargetPosY)));
  dDistance2 = sqrt(((fPassingPoint2[0] - fTargetPosX) * (fPassingPoint2[0]-fTargetPosX)) + ((fPassingPoint2[1] - fTargetPosY) * (fPassingPoint2[1] - fTargetPosY)));
  if(dDistance1 <= dDistance2){
		fPassingX = 1;
		fPassingY = 0;	
	}
	else{
		fPassingX = 0;
		fPassingY = 1;
		
	}
	fTargetPosX -= 1.5;
	fTargetPosY -= 1.0;
	iInitilizeFlag = 1;
}

/*Processes pattern recognition data read from /ardrone/navdata topic*/
void getPattern(const ardrone_autonomy::Navdata& navdataReceived){
	if(navdataReceived.tags_count > 0) iPatternFlag = 1;
	else iPatternFlag = 0;
	
}
/*Processes drone position data read from /tum_ardrone/filter_state topic*/
void getActualPos(const tum_ardrone::filter_state& actualPos){
	fActualPosX = actualPos.x;
	fActualPosY = actualPos.y;
}
