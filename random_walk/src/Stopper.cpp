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
  this->MIN_DIST_FROM_OBSTACLE = 0.8;
    keepMoving = true;

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
    this->contador_rotaciones = 0;
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
  
  if(direccion == 0) //Izquierda 
    ang_spd = -1 * ANGULAR_SPEED;
  else //Derecha
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
	int obstacleIndex = 0;
	int max_dist = 0;
	int indice_mas_lejano;
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {

      if(max_dist < scan->ranges[currIndex]){
	max_dist = scan->ranges[currIndex];
	indice_mas_lejano = currIndex;
      }
      
      if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE && !isObstacleInFront) {
	isObstacleInFront = true;
	obstacleIndex = currIndex;
	ROS_INFO("He visto un obstaculo");
      }

    }

    if (isObstacleInFront) {
      ROS_INFO("Stop!");
      keepMoving = false;
      int mitad = (minIndex+1 + maxIndex)/2;
      //Se supone que ocn esto evita los obsttaculos mas inteligentemente pero no funciona bien
      if (this->contador_rotaciones == 50){
	//this->contador_rotaciones = 0;
	rotate();
      } else if (indice_mas_lejano < mitad) {
	ROS_INFO("como el obstaculo esta a la izquierda giro a la derecha");
	  rotate(1);
      } else {
	  ROS_INFO("como el obstaculo esta a la derecha giro a la izquierda");
	  rotate(0);
      }
    } else {
      keepMoving = true;
      this->contador_rotaciones = 0;
    }
}

void Stopper::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok()) {
      if (!keepMoving) {
	/*if(this->contador_rotaciones == 100){
	  ROS_INFO("Como me he atascado giro a la izquierda a ver que pasa");
	  //this->contador_rotaciones = 0;
	  rotate();
	  }*/
      } else {
        moveForward();
      }
      
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep();
      
    }
}


