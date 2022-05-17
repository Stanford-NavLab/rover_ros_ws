/*
 * ROS mecanum
 * Subscribes to /cmd_vel Twist topic and issues motor commands
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "hardware.h"
#include "MecanumPWM.h"

ros::NodeHandle  nh;

float spdMax = 240;

// Initialize motor control
MecanumPWM mecPWM(pwmFRpin, pwmFLpin, pwmRRpin, pwmRLpin, 
                  dirFRpinA, dirFLpinA, dirRRpinA, dirRLpinA,
                  dirFRpinB, dirFLpinB, dirRRpinB, dirRLpinB, spdMax);

// ROS cmd callback
void cmd_callback(const geometry_msgs::Twist& data){
  // determine vals from twist message
  float driveVal = -data.linear.x;
  float strafeVal = data.linear.y;
  float turnVal = data.angular.z;
  
  // calculate and apply pulse width to PWM outputs to motor controllers
  mecPWM.commandMotors(driveVal, turnVal, strafeVal);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_callback);


void setup()
{
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  // Command all motors to stop
  mecPWM.allStop();
}


void loop()
{
  nh.spinOnce();
  delay(500);
}
