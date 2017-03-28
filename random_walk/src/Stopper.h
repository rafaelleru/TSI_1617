/*
 * Stopper.h
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#ifndef WANDER_BOT_SRC_STOPPER_H_
#define WANDER_BOT_SRC_STOPPER_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Stopper {
public:
    // Tunable parameters
  
  const static double MIN_SCAN_ANGLE = -30.0/180*M_PI;
  const static double MAX_SCAN_ANGLE = +30.0/180*M_PI;

  Stopper();
  Stopper(double dist);
  Stopper(double speed, double ang_spd);
  Stopper(double speed, double ang_spd, double dist);
    void startMoving();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    bool keepMoving; // Indicates whether the robot should continue moving

    void moveForward();
  void rotate(int direccion);
  void rotate();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  //Para controlar que no se atasque
  int contador_rotaciones;
  int direccion = 0;
  //Parametros que controlan el movimiento del robot
  double FORWARD_SPEED;
  double ANGULAR_SPEED;
  float MIN_DIST_FROM_OBSTACLE; // Should be smaller than sensor_msgs::LaserScan::range_max
};

#endif /* WANDER_BOT_SRC_STOPPER_H_ */
