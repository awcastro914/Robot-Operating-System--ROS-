#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <algorithm>
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

using namespace std;

//global variables
ros::Publisher roam_pub;
ros::Publisher ball_pushed_pub;
int stalledCounter = 0;
int stalledDistance = 0;
double currentDistance = 0;
nav_msgs::Odometry prevPos;
nav_msgs::Odometry currPos;
bool takingBreak = false;
int detectedBalls = 0;
int previousDetectedBalls = 0;
int detectedBallCounter = 0;

void roamScan(const sensor_msgs::LaserScan laserScan) {

    std_msgs::String msg;
    std_msgs::Bool pushedMsg;

    if(takingBreak == 0) { 
	int min = 100;

	if(stalledCounter == 0){
	 stalledDistance = currentDistance;
	}
	if(currentDistance < stalledDistance + 1){
	 stalledCounter++;
	 //std::cout << "stalledCounter= " << stalledCounter << "\n";
	} else {
	 stalledCounter = 0;
	}

	if(stalledCounter >= 10000){
	 //stalled, do a u-turn to recover
	 msg.data = "u-turn";
	 roam_pub.publish(msg);
	 stalledCounter = 0;
	 std::cout << "u-turn" << "\n";
	 return;
	}

	bool wallInFront = false;
	for(int i = 160; i < 360; i++){
		if(laserScan.ranges[i] < 0.40 && laserScan.ranges[719] > 0.3 &&  !(laserScan.ranges[i] != laserScan.ranges[i])) {
			if(detectedBalls > 0 && laserScan.ranges[719] < 0.7) {
				msg.data = "go forward";
				roam_pub.publish(msg);
				wallInFront = true;
				pushedMsg.data = true;
				ball_pushed_pub.publish(pushedMsg);

				break;
				
			} 
			else if(!(previousDetectedBalls == 1 && detectedBalls == 0)){
				msg.data = "turn left";
				roam_pub.publish(msg);
				wallInFront = true;
				break;
			}
		}
	}
	
	bool checkingWallDistance = false;
	double minRight = 99;
	double maxRight = 0;
	if(!wallInFront){
		//check 0-20 degrees on right, make sure to stay next to wall
		for(int i = 0; i < 40; i++){
			if(laserScan.ranges[i] < minRight){
				minRight = laserScan.ranges[i];
			}
			if(laserScan.ranges[i] > maxRight){
				maxRight = laserScan.ranges[i];
			}
		}

		//veer toward wall
		if(maxRight > 0.55){
			msg.data = "veer right";
			roam_pub.publish(msg);
			checkingWallDistance = true;
		}
		//veer away from wall
		else if(minRight < 0.45){
			if(minRight < 0.2){
				msg.data = "turn left";
			} else {
				msg.data = "veer left";
			}
			
			roam_pub.publish(msg);
			checkingWallDistance = true;
		}
	}  
        
	if(!wallInFront && !checkingWallDistance){
		msg.data = "go forward";
		roam_pub.publish(msg);
	}
      

	/*int i = 360;
	int offset = 1;
	bool plus = true;
	while(i > 179 && i < 541) {
		if(laserScan.ranges[i] < 0.50 && !(laserScan.ranges[i] != laserScan.ranges[i])) {
			//more open space on right, turn right
			if(laserScan.ranges[1] > laserScan.ranges[718]) {
				msg.data = "turn right";
				roam_pub.publish(msg);
				return;
			} 
			//more open space on left, turn left
			else {
				msg.data = "turn left";
				roam_pub.publish(msg);
				return;
			}
		}

		 if(laserScan.ranges[i] < min){
		    min = laserScan.ranges[i];
		 }

		 //adjust offset  for middle-out looping
		 if(plus) {
		  i += offset;
		 } else {
		  i -= offset;
		 }
		 offset++;
		 plus = !plus;
	}


	//control forward speed
	if(min > 3){
	msg.data = "go forward 0.3";
	stalledCounter = 0;
	}
	else if(1 <= min <= 3){
	msg.data = "go forward 0.2";
	stalledCounter = 0;
	}
	else {
	msg.data = "go forward 0.05";
	}

	roam_pub.publish(msg);*/
  }
  else {
	msg.data = "stop";
	roam_pub.publish(msg);
  }
}

void distanceTraveled(const nav_msgs::Odometry odom) {
   currPos = odom;
   currentDistance += sqrt(pow(currPos.pose.pose.position.x - prevPos.pose.pose.position.x, 2.0) + pow(currPos.pose.pose.position.y - prevPos.pose.pose.position.y, 2.0));
   prevPos = currPos;    
}

void takeBreak(const std_msgs::Bool takeBreakMessage) {
    takingBreak = takeBreakMessage.data;
}

void updateCurrentBalls(const std_msgs::Int16 numBalls) {
    if(detectedBallCounter > 0){
        detectedBallCounter--;
	detectedBalls = previousDetectedBalls;
    } else {
        previousDetectedBalls = detectedBalls;
        detectedBalls = numBalls.data;
    }

    if(previousDetectedBalls == 1 && detectedBalls == 0){
        detectedBallCounter = 30;
    }
}

int main(int argc, char **argv)
{
     //initialize
     ros::init(argc, argv, "roam");
     ros::NodeHandle n;

     //subscribers
     ros::Subscriber master_sub = n.subscribe("/mybot/laser/scan", 100, roamScan);  
     ros::Subscriber take_break_sub = n.subscribe("/take_break", 100, takeBreak);
     ros::Subscriber odom_sub = n.subscribe("/odom", 1, distanceTraveled);   
     ros::Subscriber num_balls_sub = n.subscribe("/red_ball_detector/num_balls", 100, updateCurrentBalls);    

     //publishers
     roam_pub = n.advertise<std_msgs::String>("/roam/path", 100);     
     ball_pushed_pub = n.advertise<std_msgs::Bool>("/ball_pushed", 1000);   
    
     sleep(2);
 
     ros::spin();

     return 0;
}


