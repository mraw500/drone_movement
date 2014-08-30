///////////////////////////////////////////////////////////////////////////////
//Source for drone_movment node to send control commands to AR-Drone				 //
//v1.0									     																								 //
//First creation							    																					 //
//Huseyin Emre Erdem																										     //
//Edgar Buchanan																														 //
//29.08.2014																														     //
///////////////////////////////////////////////////////////////////////////////

//Communication with i90 implemented.

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
#include <fcntl.h>
#include <termios.h>

#define fSearchDistance 0.5 //Define distance to explore at moment of searching for tag
#define fDroneHeight 1.25
#define fErrorCompX 1.75
#define fErrorCompY 1.25
#define fErrorCompCloseX 0.87
#define fErrorCompCloseY 0.62



/*Variables*/
char *cpI90PortName = "/dev/ttyUSB0"; //Retrieve information sent by i90
char cData[4];//Characters to be read from the serial port
char *cpData = &cData[0];	//Pointer to data
//float fTargetPosArr[2]; 	//Element 0 is for X and element 1 is for Y
float fTargetPosX;				//Beacon position in X
float fTargetPosY;				//Beacon position in Y
float fActualPosX;				//Drone actual position in X
float fActualPosY;				//Drone actual position in Y
float fYawAngle;					//Drone angle
float fPassingX;					//Passing point in X axis
float fPassingY;					//Passing point in Y axis
int iInitilizeFlag;				//Flag to launch drone
int iReturnFlag;					//Flag to indicate drone to return home area	
int iPatternFlag;					//Flag that indicates that tag was found
int iLaunchFlag;					//Flag to keep drone waiting for i90's signal
int iCounter;							//Timer to send launch command

/*Prototypes*/
void getTargetPos(const drone_movement::pos posReceived);//Processes data read from drone_target_pos topic
void getPattern(const ardrone_autonomy::Navdata& navdataReceived);//Processes pattern recognition data read from /ardrone/navdata topic
void getActualPos(const tum_ardrone::filter_state& actualPos);		//Processes drone position from /ardrone/predictedPose topic
int setInterfaceAttribs (int iFd, int iSpeed, int iParity);				//Sets serial port parameters

int iI90Port = open(cpI90PortName, O_RDWR | O_NOCTTY | O_NDELAY);

/*Main function*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_movement");	 //Create node called drone_movement
  ros::NodeHandle n;												//Create nodehandle to alter node parameters
  ros::Duration d = ros::Duration(2,0);			//Duration object to pause node

	/*Publishers*/
  ros::Publisher landPub = n.advertise<std_msgs::Empty>("/ardrone/land",1);				//Publisher to send landing commands
  ros::Publisher takeoffPub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);	//Publisher to send takeoff commands
  ros::Publisher drone_pub = n.advertise<std_msgs::String>("tum_ardrone/com",50);	//Publisher to send commands to Ar-Drone

	/*Subscribers*/
	ros::Subscriber targetSub = n.subscribe("i90_current_pos", 1, getTargetPos);				//To collect the target position published by i90_sensor_board
	ros::Subscriber patternSub = n.subscribe("/ardrone/navdata", 1, getPattern);					//To collect patter recognition information
	ros::Subscriber actualPosSub = n.subscribe("/ardrone/predictedPose", 1, getActualPos);//To collect actual robot position

	d.sleep(); //Wait for 2 seconds to allow publishers & subscribers to get ready

	std_msgs::String s;
	std::string c;

	static pthread_mutex_t send_CS; 	

	/*Initilize flags*/
	iInitilizeFlag = 0;
	iReturnFlag = 0;
	iPatternFlag = 0;
	iCounter = 0;
	iLaunchFlag = 1; //CHANGE!!! 0

	/*Check serial port is open*/
  if(iI90Port == -1){
	  ROS_INFO("Error opening the port");//Inform user on the terminal
  }
  else{
	  ROS_INFO("I90 PC serial port is open");
  }

	setInterfaceAttribs (iI90Port, B9600, 0);

  while(ros::ok()){

		while(iLaunchFlag == 0){

			/*Read serial until special character is received from i90*/			 
			read(iI90Port, cpData, 1);//Read 1 bytes from buffer

			if(cData[0] == 'a'){
				tcflush(iI90Port, TCIFLUSH);	//Empty buffer				
				write(iI90Port,"b",1);				//Send acknowledgement
				ROS_INFO("Data request sent");
				usleep(5000000);							//Wait until all information is received				
				read(iI90Port, cpData, 4);		//Read 4 bytes from buffer

				/*Convert information characters received into float numbers*/
				/*fTargetPosArr[0] = 0;
				fTargetPosArr[1] = 0;				
				for(int i=0; i<4; i++){//Ir fraction					
					printf("%u\n", cData[i]);
				}
				for(int i=0; i<2; i++){//Ir fraction					
					fTargetPosArr[i] += cData[1 + 2 * i] / 100.00;
				}
				for(int i=0; i<2; i++){//Ir integer
					fTargetPosArr[i] += (float) cData[2 * i];
				}*/
 				fTargetPosX = cData[0] + cData[1] / 100.00;
				fTargetPosY = cData[2] + cData[3] / 100.00;

				ROS_INFO("Target received: [%f]/t[%f]", fTargetPosX, fTargetPosY);
				
				float fPassingPoint1[2] = {2,0},fPassingPoint2[2] = {0,2};
				double dDistance1, dDistance2;
				dDistance1 = sqrt(((fPassingPoint1[0] - fTargetPosX) * (fPassingPoint1[0] - fTargetPosX)) + ((fPassingPoint1[1] - fTargetPosY) * (fPassingPoint1[1] - fTargetPosY)));
				dDistance2 = sqrt(((fPassingPoint2[0] - fTargetPosX) * (fPassingPoint2[0] - fTargetPosX)) + ((fPassingPoint2[1] - fTargetPosY) * (fPassingPoint2[1] - fTargetPosY)));
				if(dDistance1 <= dDistance2){
					fPassingX = 1;
					fPassingY = 0;	
				}
				else{
					fPassingX = 0;
					fPassingY = 1;
				}	

				if(fTargetPosX > 3)	fTargetPosX -= fErrorCompX;
				else fTargetPosX -= fErrorCompCloseX;
				if(fTargetPosY > 3) fTargetPosY -= fErrorCompY;
				else fTargetPosY -= fErrorCompCloseY;

				//fTargetPosX = fTargetPosArr[0] - 1.5;
				//fTargetPosY = fTargetPosArr[1] -1.0;
				/*Change flags and display target*/

				

				iLaunchFlag = 1;
				iInitilizeFlag = 1;
				
			}
		}
	
		/*Sample for tag and display current position at 2Hz*/
		usleep(500000);	
		iCounter++;
		if(iCounter > 360) iReturnFlag = 1;
		ROS_INFO("Time left: [%i]s", iCounter/2);
		ROS_INFO("Pattern flag: [%i]", iPatternFlag);
		ROS_INFO("Current pos: X = [%f] Y = [%f]", fActualPosX, fActualPosY);
		ROS_INFO("Target pos: X = [%f] Y = [%f]", fTargetPosX, fTargetPosY);

		/*Once target coordinates are received launch and move towards target*/
		if(iInitilizeFlag == 1 && iReturnFlag == 0){		
			
			/*RESET*/
			/*c = " ";
			c = "f reset";						
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("RESET!");
			d.sleep();*/

			/*Set origin in actual position*/
			/*c = "c setReference $POSE$"; //Set origin in drone's actual position
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);			
			ROS_INFO("setReference!");*////MODIFIED!!!

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

			c = "c setStay (Parallel Tracking and Mapping) is a camera tracking system for auTime 1";						//Stay in target location for 3s
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
			
			/*Search tag in square shape*/
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

			/*RESET*/
			/*c = " ";
			c = "f reset";						
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("RESET!");
			d.sleep();*/

			/*Return to appropiate passing point*/
			c = " ";
			sprintf(&c[0],"c goto %.2f %.2f 0.75 %.2f", fPassingX * 1.5, fPassingY * 1.5, fYawAngle); 
			//sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", -fTargetPosX + fPassingX, -fTargetPosY + fPassingY, fYawAngle);
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fPassingX * 1.5, fPassingY * 1.5);

			/*Return origin*/
			c = " ";
			c = "c goto -0.5 -0.5 0.25 0";
			//sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", -fTargetPosX, -fTargetPosY, fYawAngle);			
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("goto origin!");

			iInitilizeFlag = 0;
		}
		
		/*Return home once pattern is found*/
		if(iPatternFlag == 1 && iReturnFlag == 0){
			
			/*RESET*/
			/*c = " ";
			c = "f reset";						
			s.data = c.c_str();
			drone_pub.publish(s);
			ROS_INFO("RESET!");
			iReturnFlag = 1;
			d.sleep();*/

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
			//sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", -fTargetPosX + fPassingX, -fTargetPosY + fPassingY, fYawAngle); 
			s.data = c.c_str();
			pthread_mutex_lock(&send_CS);
			drone_pub.publish(s);
			pthread_mutex_unlock(&send_CS);
			ROS_INFO("Goto: [%f][%f]", fPassingX * 1.5, fPassingY * 1.5);

			/*Return origin*/
			c = " ";
			c = "c goto -0.5 -0.5 0.25 0";
			//sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f", -fTargetPosX, -fTargetPosY, fYawAngle);						
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
	if(fTargetPosX > 3)	fTargetPosX -= fErrorCompX;
	else fTargetPosX -= fErrorCompCloseX;
	if(fTargetPosY > 3) fTargetPosY -= fErrorCompY;
	else fTargetPosY -= fErrorCompCloseY;
	iInitilizeFlag = 1;
}

/*Processes pattern recognition data read from /ardrone/navdata topic*/
void getPattern(const ardrone_autonomy::Navdata& navdataReceived){
	if(navdataReceived.tags_count > 0){
		write(iI90Port,"c",1);									//Send target found confirmation to i90
		ROS_INFO("Tag detection confirmation");
		iPatternFlag = 1;												
	}
	else iPatternFlag = 0;
}

/*Processes drone position data read from /tum_ardrone/filter_state topic*/
void getActualPos(const tum_ardrone::filter_state& actualPos){
	fActualPosX = actualPos.x;
	fActualPosY = actualPos.y;
}

int setInterfaceAttribs (int iFd, int iSpeed, int iParity){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (iFd, &tty) != 0)
	{
    printf("error from tcgetattr");
    return -1;
	}

	cfsetospeed (&tty, iSpeed);
	cfsetispeed (&tty, iSpeed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
		                              // no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
		                              // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= iParity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (iFd, TCSANOW, &tty) != 0)
	{
    printf("error from tcsetattr");
    return -1;
	}
	return 0;
}
