// File:          GeckoRunner.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include<ctime>
#include<iostream>
#include<math.h>
//#include<Eigen/Core> 

//#include "Motion.hpp"
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace Eigen;
using namespace std;

Motor *limb_motors[3];
PositionSensor *limb_position_sensor[3];
Robot *robot;
int timeStep;
int counting;
//extern float limb_force_sensor[4];

//virtual enable (timeStep);
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
/*  // delay
static void sleep_ms(unsigned int secs)
{
struct timeval tval;

tval.tv_sec=secs/1000;

tval.tv_usec=(secs*1000)%1000000;

select(0,NULL,NULL,NULL,&tval);
}
*/

// This fution used to delay ms in linux environment 
int main(int argc, char **argv) 
{
  robot = new Robot();
  timeStep = (int)robot->getBasicTimeStep();
  
  for(int j=0;j<3;j++)
  {
    limb_motors[j] = robot->getMotor(MOTOR_NAMES[i][j]);
    limb_motors[j]->setPosition(0);
    limb_position_sensor[j] = robot->getPositionSensor(SENSOR_NAMES[i][j]);
    limb_position_sensor[j]->enable(timeStep);
  }
  for(int j=0;j<3;j++)
  {
    limb_motors[j]->setPosition(0);
  }
  counting = 0;
  //limb_motors[i][j]->setPosition(0);
  
  // get the time step of the current world.
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1)
  {
    cout<<counting<<endl;
    counting++;
  };

  // Enter here exit cleanup code.

}
