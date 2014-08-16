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

/*Variables*/
float fTargetPosX;
float fTargetPosY;
float fYawAngle;
int iInitilizeFlag;
int iPassingPoint;

/*Prototypes*/
void getTargetPos(const drone_movement::pos posReceived);//Processes data read from drone_target_pos topic
//void getPattern(const ardrone_autonomy::Navdata& navdataReceived);//Processes pattern recognition data read from /ardrone/navdata topic

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
	//ros::Subscriber targetSub = n.subscribe("drone_target_pos", 1, getTargetPos);//To collect the target position published by i90_sensor_board
	//ros::Subscriber patternSub = n.subscribe("/ardrone/navdata", 1, getPattern);//To collect the target position published by i90_sensor_board

	d.sleep(); //Wait for 2 seconds to allow publishers & subscribers to get ready

  /*Initialize PTAM*/
	std_msgs::String s;
	std::string c;
	int count = 0;
	static pthread_mutex_t send_CS;
	drone_movement::UInt8 message;

	iInitilizeFlag = 0;

  while(ros::ok()){
		message.data = count++;
		drone_target_reached_pub.publish(message);
		d.sleep();
		while (iInitilizeFlag == 1){		//Wait until receive coordinates from i90 actual position		
			
			//Take off AR-drone
			takeoffPub.publish(std_msgs::Empty());//Publish take0ff message
			ROS_INFO("TAKEOFF!");//Inform user on the terminal
			d.sleep();

			//Set initial parameters
			c = "c autoTakeOver 500 800";
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);

			d.sleep();
			ROS_INFO("autoTakeOver!");//Inform user on the terminal

			c = "c setReference $POSE$"; //Set origin in robot's actual position
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);

			d.sleep();		
			ROS_INFO("setReference!");//Inform user on the terminal

			c = "c setMaxControl 0.1";	//Limit AR-dron high speed
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);

			ROS_INFO("setMaxControl!");//Inform user on the terminal

			c = "c setInitialReachDist 0.1";	//Reach target within 0.1m 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);

			ROS_INFO("setInitialReachDist!");//Inform user on the terminal

			c = "c setStayWithinDist 0.1";		//Stay in target location within 0.1m
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);

			ROS_INFO("setStayWithinDist!");//Inform user on the terminal	

			c = "c setStayTime 3";						//Stay in target location for 3s
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

			c = "c goto 0 0 0.5 0";						//Increase AR-drone altitude in order avoid ground obstacles (1.45m)
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);

			d.sleep();
			ROS_INFO("goto allocated pos!");//Inform user on the terminal

			char buffer [50]; //NOTE for EDGAR; <---- check this !!!!! 
			//Select passing point as starting point
			if(iPassingPoint==1){ 
				c = "c goto 0 2 0.5 0";
				s.data = c.c_str();
				pthread_mutex_lock(&send_CS);
				drone_pub.publish(s);
				pthread_mutex_unlock(&send_CS);
				d.sleep();
				fTargetPosY = fTargetPosY - 2;		//Change coordinates to new origin
				c=sprintf(buffer,"c goto 0 2 0.5 %.2f",fYawAngle);	//Turn towards target.
				s.data = c.c_str();
				drone_pub.publish(s);
			}		
			else {
				c = "c goto -2 0 0.5 0";
				s.data = c.c_str();
				pthread_mutex_lock(&send_CS);
				drone_pub.publish(s);
				pthread_mutex_unlock(&send_CS);
				d.sleep();
				fTargetPosX = fTargetPosX + 2;		//Change coordinates to new origin
				c=sprintf(buffer,"c goto -2 0 0.5 %.2f",fYawAngle);	//Turn towards target.
				s.data = c.c_str();
				drone_pub.publish(s);
			}
			
			d.sleep();

			c=sprintf(buffer,"c goto %.2f %.2f 0.5 %.2f",fTargetPosX,fTargetPosY, fYawAngle);	//Move towards target
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			d.sleep();
			
			/*Find pattern
			while(NO_PATTERN_FOUND){
				LOOk_FOR_PATTERN
			}
			*/

			// Sending message to i90
			//message.data = 1;
			//drone_target_reached_pub.publish(message);
			

			//Return no passing point
			if(iPassingPoint==1){ 
				c = "c goto 0 2 0.5 0";
				s.data = c.c_str();
				drone_pub.publish(s);
				d.sleep();
				fTargetPosY = fTargetPosY - 2;		//Change coordinates to new origin
				c=sprintf(buffer,"c goto 0 2 0.5 %.2f",fYawAngle);	//Turn towards target.
				s.data = c.c_str();
				drone_pub.publish(s);
			}		
			else {
				c = "c goto -2 0 0.5 0";
				s.data = c.c_str();
				drone_pub.publish(s);
				d.sleep();
				fTargetPosX = fTargetPosX + 2;		//Change coordinates to new origin
				c=sprintf(buffer,"c goto -2 0 0.5 %.2f",fYawAngle);	//Turn towards target.
				s.data = c.c_str();
				drone_pub.publish(s);
			}

			//Return origin
			c = "c goto 0 0 0.5 0";						//Increase AR-drone altitude in order avoid ground obstacles (1.45m)
			s.data = c.c_str();
			drone_pub.publish(s);
			d.sleep();
			
			//LAnd!!
			landPub.publish(std_msgs::Empty());//Publish landing command
			ROS_INFO("LAND!");//Inform user
			iInitilizeFlag = 0;
		}

  	ros::spinOnce();
		//loop_rate.sleep();
  }
  return 0;
}

/*Processes data read from drone_target_pos topic*/
void getTargetPos(const drone_movement::pos posReceived){
	ROS_INFO("Target position: [%f]/t[%f]/t[%f]", posReceived.fXPos, posReceived.fYPos, posReceived.fYawAngle);
	fTargetPosX = posReceived.fXPos;
	fTargetPosY = posReceived.fYPos;
 	fYawAngle = posReceived.fYawAngle;	
	iInitilizeFlag = 1;
	float fPassingPoint1[2]={2,0},fPassingPoint2[2]={0,2};
  double dDistance1, dDistance2;
  dDistance1=sqrt(((fPassingPoint1[0]-fTargetPosX)*(fPassingPoint1[0]-fTargetPosX))+((fPassingPoint1[1]-fTargetPosY)*(fPassingPoint1[1]-fTargetPosY)));
  dDistance2=sqrt(((fPassingPoint2[0]-fTargetPosX)*(fPassingPoint2[0]-fTargetPosX))+((fPassingPoint2[1]-fTargetPosY)*(fPassingPoint2[1]-fTargetPosY)));
  if(dDistance1<=dDistance2)
     iPassingPoint=1;
	else
		iPassingPoint=2;
}

/*Processes pattern recognition data read from /ardrone/navdata topic*/
void getPattern(const ardrone_autonomy::Navdata& navdataReceived){
	ROS_INFO("Tags detected: [%u]", navdataReceived.tags_count);
}

