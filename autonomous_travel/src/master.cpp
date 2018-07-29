#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <cmath>
#include <math.h>
#include "angles/angles.h"
#include "tf/tf.h"

#define PI 3.14159265

using namespace std;

//track balls global variables
typedef struct {
   double x ;
   double y;
}Ball;
Ball foundBalls[20];
int foundBallsCount = 0;
int detectedBalls = 0;
int previousDetectedBalls = 0;
int detectedBallImageXCenter = 0;

//robot state global variables
double startingX = 0.0;
double startingY = 0.0;
nav_msgs::Odometry prevPos;
nav_msgs::Odometry currPos;
double currentX = 0.0;
double currentY = 0.0;
double currentOrientation = 0.0;

//take break global variables
bool takingBreak = false;

//must detect correctly 5 times in a row to add ball
double baseX = 0.0;
double baseY = 0.0;
int timesInARow = 0;

//for not double detecting pushed ball
bool ballPreviouslyPushed = false;



void updateRobotLocation(const nav_msgs::Odometry odom) {
   currPos = odom;
   currentX = currPos.pose.pose.position.x + startingX;
   currentY = currPos.pose.pose.position.y + startingY;
   currentOrientation = angles::to_degrees(tf::getYaw(odom.pose.pose.orientation));
   if(currentOrientation < 0) {
       currentOrientation += 360;
   }
}

void updateCurrentBalls(const std_msgs::Int16 numBalls) {
    previousDetectedBalls = detectedBalls;
    detectedBalls = numBalls.data;
}

void getBallXImage(const std_msgs::Int16 ballXImage) {
    detectedBallImageXCenter = ballXImage.data;
}

void estimateBallLocation(const sensor_msgs::LaserScan laserScan) {	
	double estimatedBallCenterIndex = 0;
	double estimatedBallRange = 0;

	double estimatedBallX = 0;
	double estimatedBallY = 0;

	if(takingBreak == 0){
		takingBreak = true;
		sleep(3);
	}

	if(detectedBalls > 0) {
		//get laserScan index from detectedBallImageXCenter
		estimatedBallCenterIndex = ((1.4 - (detectedBallImageXCenter/800.0)*1.4 + 0.87)/(1.57*2.0))*720.0;
		
		//get range at this index
		estimatedBallRange = laserScan.ranges[estimatedBallCenterIndex];
		
		//adjust to center of ball
		estimatedBallRange = estimatedBallRange + 0.2;

		//use trig to estimate the X,Y of the ball
		estimatedBallX = currentX + estimatedBallRange*cos((currentOrientation+((estimatedBallCenterIndex-360)*0.25)) * PI/180.0);
		estimatedBallY = currentY + estimatedBallRange*sin((currentOrientation+((estimatedBallCenterIndex-360)*0.25)) * PI/180.0);

		cout << "ballPreviuoslyPushed: " << ballPreviouslyPushed << endl;
		if(estimatedBallRange < 5){
			for(int i = 0; i < 20; i++){
				//found same ball
				if(ballPreviouslyPushed && previousDetectedBalls == 0) {
					cout << "got in this" << endl;
					if(((abs(estimatedBallX - foundBalls[i].x) < 1.50) && (abs(estimatedBallY - foundBalls[i].y) < 1.50))){
						//update same ball
						foundBalls[i].x = estimatedBallX;
						foundBalls[i].y = estimatedBallY;
						timesInARow = 0;
						if(((abs(estimatedBallX - foundBalls[i].x) < 0.20) && (abs(estimatedBallY - foundBalls[i].y) < 0.20))){
							ballPreviouslyPushed = false;
						}
						
						break;
					}
				}
				else if(((abs(estimatedBallX - foundBalls[i].x) < 0.20) && (abs(estimatedBallY - foundBalls[i].y) < 0.20))){
					//update same ball
					foundBalls[i].x = estimatedBallX;
					foundBalls[i].y = estimatedBallY;
					timesInARow = 0;
					break;
				}
				else if(foundBalls[i].x == 99.0 && foundBalls[i].y == 99.0){
					if(timesInARow < 10){
						if(timesInARow == 0){
							baseX = estimatedBallX;
							baseY = estimatedBallY;
							timesInARow++;
						} else {
							if(((abs(estimatedBallX - baseX) > 0.2) || (abs(estimatedBallY - baseY) > 0.2))){
								timesInARow = 0;
							} else {
								timesInARow++;
							}
						}
						break;
					} else {
						//insert new ball
						foundBalls[i].x = estimatedBallX;
						foundBalls[i].y = estimatedBallY;
						cout << "Found a ball with robot at location (" << currentX << "," << currentY << ") and ball at (" << estimatedBallX << "," << estimatedBallY << ")" << endl;
						foundBallsCount++;
						timesInARow = 0;
						break;
					}
				}
			}
		}

		for(int i = 0; i < foundBallsCount; i++) {
			cout << "Ball " << i+1 << ": (" << foundBalls[i].x << "," << foundBalls[i].y << ")" << endl;	
		}
		cout << "--------------" << endl;	
		cout << "--------------" << endl;	
		cout << "--------------" << endl;

	}
}

void takeBreak(const std_msgs::Bool takeBreakMessage) {
    takingBreak = takeBreakMessage.data;
}

void ballPushed(const std_msgs::Bool ballPushed) {
	ballPreviouslyPushed = ballPushed.data;
}

int main(int argc, char **argv)
{
     //initialize
     ros::init(argc, argv, "master");
     ros::NodeHandle n;

     //get param values
     ros::param::get("~x", startingX);
     ros::param::get("~y", startingY);

     //make sure this is initialized to zero to start
     prevPos.pose.pose.position.x = 0;
     prevPos.pose.pose.position.y = 0;

     //print values of received params
     std::cout << "starting x=" << startingX << " starting y=" << startingY << "\n";

     for(int i = 0; i < 20; i++){
          foundBalls[i].x = 99.0;
	  foundBalls[i].y = 99.0;
     } 

     //subscribers
     ros::Subscriber odom_sub = n.subscribe("/odom", 100, updateRobotLocation);
     ros::Subscriber take_break_sub = n.subscribe("/take_break", 100, takeBreak);
     ros::Subscriber num_balls_sub = n.subscribe("/red_ball_detector/num_balls", 100, updateCurrentBalls);
     ros::Subscriber ball_detected_sub = n.subscribe("/red_ball_detector/ball_one_image_x", 100, getBallXImage);
     ros::Subscriber estimate_ball_loc_sub = n.subscribe("/mybot/laser/scan", 100, estimateBallLocation);
     ros::Subscriber ball_pushed_sub = n.subscribe("/ball_pushed", 1, ballPushed);

     sleep(2);
 
     ros::spin();

     return 0;
}
