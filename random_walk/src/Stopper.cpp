/*
 * Stopper.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#include "Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper()
{

  this->FORWARD_SPEED = 0.3;
  this->ANGULAR_SPEED = 0.5;
  keepMoving = true;

    // Advertise a new publisher for the robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
  this->contador_rotaciones = 0;
  node.getParam("min_dist", this->MIN_DIST_FROM_OBSTACLE);
}

Stopper::Stopper(double dist){
  Stopper();
  this->MIN_DIST_FROM_OBSTACLE = dist;
}

Stopper::Stopper(double speed, double ang_spd){
  Stopper();
  this->FORWARD_SPEED = speed;
  this->ANGULAR_SPEED = ang_spd;
}

Stopper::Stopper(double speed, double ang_spd, double dist){
  Stopper();
  this->FORWARD_SPEED = speed;
  this->ANGULAR_SPEED = ang_spd;
  this->MIN_DIST_FROM_OBSTACLE = dist;
}

// Send a velocity command
void Stopper::moveForward() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    ROS_INFO("avanzando");
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}

void Stopper::rotate(int direccion) {
  double ang_spd;
  this->contador_rotaciones++;
  
  if(direccion == 0) //derecha
    ang_spd = -1 * ANGULAR_SPEED;
  else //izquierda
    ang_spd = ANGULAR_SPEED;

  //Envio del mensage
  geometry_msgs::Twist msg;
  ROS_INFO("Rotando");
  msg.angular.z = ang_spd;
  commandPub.publish(msg);

}


void Stopper::rotate() {
  geometry_msgs::Twist msg;
  ROS_INFO("Rotando");
  msg.angular.z = ANGULAR_SPEED;
  commandPub.publish(msg);
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  bool isObstacleInFront = false;
  int max_dist = 0;
  int indice_mas_lejano;
  // Find the closest range between the defined minimum and maximum angles
  int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

  keepMoving = true;
  
  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
    if( scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE){
      keepMoving = false;
    }

    if( scan->ranges[currIndex] > max_dist){
      indice_mas_lejano = currIndex;
    }
  }

  if(!keepMoving){
    int izq = (minIndex+1+maxIndex)/3;
    int centro = izq+izq;

    if(indice_mas_lejano < izq){
      direccion = -1;
    } else if (indice_mas_lejano < centro){
      keepMoving = true;
    } else {
      direccion = 1;
    }
  }

}

void Stopper::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok()) {
      if(keepMoving){
	moveForward();
      }else{
	switch (direccion) {
	case -1: {
	  rotate();
	  break;
	}
	case 1: {
	  rotate(0);
	}
	}
      }
      
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep();
      
    }
}


