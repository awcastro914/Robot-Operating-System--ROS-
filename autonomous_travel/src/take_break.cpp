#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <cmath>

using namespace std;

//global variables
ros::Publisher take_break_pub;

int breakCounter = 0;
bool takingBreak = false;
int breakFrequency = 0;
int breakLength = 0;

void breakFunc(int length, int frequency) {
     std_msgs::Bool takingBreakMessage;   

     if(!takingBreak) {
          if(breakCounter == frequency) {
              breakCounter = 0;
              takingBreak = true;
              takingBreakMessage.data = true;
              take_break_pub.publish(takingBreakMessage);
          }
     }
     else if(takingBreak) {
          if(breakCounter == length) {
              breakCounter = 0;
              takingBreak = false;
              takingBreakMessage.data = false;
              take_break_pub.publish(takingBreakMessage);
          }
     }
}

int main(int argc, char **argv)
{
     //initialize
     ros::init(argc, argv, "take_break");
     ros::NodeHandle n;

     //get param values
     ros::param::get("~breakFrequency", breakFrequency);
     ros::param::get("~breakLength", breakLength);

     cout << breakFrequency << endl;
     cout << breakLength << endl;
     //publishers
     take_break_pub = n.advertise<std_msgs::Bool>("/take_break", 1000);     
    
     sleep(2);

     while(1){
         breakCounter++;
         cout << breakCounter << endl;
         sleep(1);
         breakFunc(breakLength, breakFrequency);
     }

     return 0;
}
