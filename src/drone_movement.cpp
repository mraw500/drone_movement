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

/*Variables*/
float fTargetPosX;
float fTargetPosY;
float fActualPosX;
float fActualPosY;
float fYawAngle;
int iInitilizeFlag;
int iReturnFlag;
int iPassingPoint;
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
		if(iInitilizeFlag == 1 && iReturnFlag == 0){		//Wait until receive coordinates from i90 actual position		
			
			c = "c setReference $POSE$"; //Set origin in robot's actual position
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);			
			ROS_INFO("setReference!");//Inform user on the terminal

			d.sleep();			
			
			//Take off AR-drone
			takeoffPub.publish(std_msgs::Empty());
			ROS_INFO("TAKEOFF!");//Inform user 
			
			d.sleep();
			d.sleep();
			d.sleep();


			//Set initial parameters
			c = "c autoTakeOver 500 800";
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("autoTakeOver!");//Inform user on the terminal

			c = "c setMaxControl 0.1";	//Limit AR-dron high speed
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setMaxControl!");//Inform user on the terminal

			c = "c setInitialReachDist 0.25";	//Reach target within 0.1m 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setInitialReachDist!");//Inform user on the terminal

			c = "c setStayWithinDist 0.25";		//Stay in target location within 0.1m
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setStayWithinDist!");//Inform user on the terminal	

			c = "c setStayTime 1";						//Stay in target location for 3s
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("setStayTime!");//Inform user on the terminal	

			c = "c lockScaleFP";							//Fix PTAM
			s.data = c.c_str();
			drone_pub.publish(s);
			d.sleep();
			ROS_INFO("lockScaleFP!");//Inform user on the terminal			


			//Select passing point as starting point
			if(iPassingPoint == 2){ 
				c = "c goto 0 1.5 0.5 0"; //Original value First test. Error of 0.5m
				s.data = c.c_str();
				pthread_mutex_lock(&send_CS);
				drone_pub.publish(s);
				pthread_mutex_unlock(&send_CS);
				ROS_INFO("goto 0 1.5!");//Inform user on the terminal
			}		
			else {
				c = "c goto 1.5 0 0.5 0";	//Error of 1.5m
				s.data = c.c_str();
				pthread_mutex_lock(&send_CS);
				drone_pub.publish(s);
				pthread_mutex_unlock(&send_CS);
				ROS_INFO("goto 1.5 0!");//Inform user on the terminal
			}

			//Go to target
			sprintf(&c[0],"c goto %.2f %.2f 0.75 %.2f", fTargetPosX, fTargetPosY, fYawAngle);	
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fTargetPosX, fTargetPosY);

			iInitilizeFlag = 0;
		}
		
		if(iPatternFlag == 1 && iReturnFlag == 0){//Search pattern while travelling
			//ClearCommands and stay in that position
			c = "c clearCommands";						//Clear targets and current commands in order to execute next command inmiedetely
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("clearCommands!");//Inform user on the terminal
			iReturnFlag = 1;
			//Return to passing point
			if(iPassingPoint == 2){ 
				c = "c goto 0 1.5 0.5 0";
				s.data = c.c_str();
				drone_pub.publish(s);
				ROS_INFO("goto 0 1.5!");//Inform user on the terminal
			}		
			else {
				c = "c goto 1.5 0 0.5 0";
				s.data = c.c_str();
				drone_pub.publish(s);
				ROS_INFO("goto 1.5 0!");//Inform user on the terminal
			}

			//Return origin
			c = "c goto 0 0 0.25 0";						//Increase AR-drone altitude in order avoid ground obstacles (1.45m)
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("goto origin!");//Inform user on the terminal
		}
		
		
		if(iReturnFlag == 1 && (fActualPosX < 0.2 &&  fActualPosY < 0.2)){ //Land once robot is back in home area
			landPub.publish(std_msgs::Empty());
			ROS_INFO("LAND!");//Inform user
		}
		
		ros::spinOnce();
  }
  return 0;
}
//iPatternFlag = 1;
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
  if(dDistance1 <= dDistance2)
     iPassingPoint=1;
	else
		 iPassingPoint=2;
	iInitilizeFlag = 1;
	
	fTargetPosX -= 1.25;
	fTargetPosY -= 0.75;
}

/*Processes pattern recognition data read from /ardrone/navdata topic*/
void getPattern(const ardrone_autonomy::Navdata& navdataReceived){
	if(navdataReceived.tags_count > 0) iPatternFlag = 1;
	else iPatternFlag = 0;
	
}

void getActualPos(const tum_ardrone::filter_state& actualPos){
	fActualPosX = actualPos.x;
	fActualPosY = actualPos.y;
}
