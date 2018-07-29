#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

using namespace std;

ros::Publisher move_bot_pub;

void move(const std_msgs::String msg) {
     /*if(msg.data == "go forward 0.05"){
          geometry_msgs::Twist straight;
          straight.linear.x = 0.1;
          straight.linear.y = 0.0;
          straight.linear.z = 0.0;
          straight.angular.x = 0.0;
          straight.angular.y = 0.0;
          straight.angular.z = 0.0;
          
          move_bot_pub.publish(straight);
     }
     else if(msg.data == "go forward 0.2"){
          geometry_msgs::Twist straight;
          straight.linear.x = 0.3;
          straight.linear.y = 0.0;
          straight.linear.z = 0.0;
          straight.angular.x = 0.0;
          straight.angular.y = 0.0;
          straight.angular.z = 0.0;

          move_bot_pub.publish(straight);
     }
     else if(msg.data == "go forward 0.3"){
          geometry_msgs::Twist straight;
          straight.linear.x = 1.0;
          straight.linear.y = 0.0;
          straight.linear.z = 0.0;
          straight.angular.x = 0.0;
          straight.angular.y = 0.0;
          straight.angular.z = 0.0;

          move_bot_pub.publish(straight);
     }
     else if(msg.data == "turn left"){
          geometry_msgs::Twist turnLeft;
          turnLeft.linear.x = 0.0;
          turnLeft.linear.y = 0.0;
          turnLeft.linear.z = 0.0;
          turnLeft.angular.x = 0.0;
          turnLeft.angular.y = 0.0;
          turnLeft.angular.z = 0.1;

          move_bot_pub.publish(turnLeft);
     }
     else if(msg.data == "turn right"){
          geometry_msgs::Twist turnRight;
          turnRight.linear.x = 0.0;
          turnRight.linear.y = 0.0;
          turnRight.linear.z = 0.0;
          turnRight.angular.x = 0.0;
          turnRight.angular.y = 0.0;
          turnRight.angular.z = -0.1;

          move_bot_pub.publish(turnRight);
     }*/

	if(msg.data == "turn left"){
		geometry_msgs::Twist turnLeft;
		turnLeft.linear.x = 0.00;
		turnLeft.linear.y = 0.0;
		turnLeft.linear.z = 0.0;
		turnLeft.angular.x = 0.0;
		turnLeft.angular.y = 0.0;
		turnLeft.angular.z = 0.3;

		move_bot_pub.publish(turnLeft);
		cout << "turn left" << endl;
	}
	else if(msg.data == "veer left"){
		geometry_msgs::Twist veerLeft;
		veerLeft.linear.x = 0.22;
		veerLeft.linear.y = 0.0;
		veerLeft.linear.z = 0.0;
		veerLeft.angular.x = 0.0;
		veerLeft.angular.y = 0.0;
		veerLeft.angular.z = 1.0;
		move_bot_pub.publish(veerLeft);
		cout << "veer left" << endl;
	}
	else if(msg.data == "veer right"){
		geometry_msgs::Twist veerRight;
		veerRight.linear.x = 0.22;
		veerRight.linear.y = 0.0;
		veerRight.linear.z = 0.0;
		veerRight.angular.x = 0.0;
		veerRight.angular.y = 0.0;
		veerRight.angular.z = -1.0;

		move_bot_pub.publish(veerRight);
		cout << "veer right" << endl;
	}
	else if(msg.data == "go forward"){
		geometry_msgs::Twist straight;
		straight.linear.x = 0.30;
		straight.linear.y = 0.0;
		straight.linear.z = 0.0;
		straight.angular.x = 0.0;
		straight.angular.y = 0.0;
		straight.angular.z = 0.0;

		move_bot_pub.publish(straight);
		cout << "go forward" << endl;
	}


	else if(msg.data == "u-turn"){
		geometry_msgs::Twist uTurn;
		uTurn.linear.x = 0.0;
		uTurn.linear.y = 0.0;
		uTurn.linear.z = 0.0;
		uTurn.angular.x = 0.0;
		uTurn.angular.y = 0.0;
		uTurn.angular.z = 4.0;

		move_bot_pub.publish(uTurn);
		sleep(2);
		move_bot_pub.publish(uTurn);
	}
	else if(msg.data == "stop"){
		geometry_msgs::Twist stop;

		stop.linear.x = 0.0;
		stop.linear.y = 0.0;
		stop.linear.z = 0.0;
		stop.angular.x = 0.0;
		stop.angular.y = 0.0;
		stop.angular.z = 0.0;

		move_bot_pub.publish(stop);
	}
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "move_bot");

     ros::NodeHandle n;
     ros::Subscriber go_forward_sub = n.subscribe("/roam/path", 1, move);

     move_bot_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

     ros::spin();

     return 0;
}

