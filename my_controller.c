#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define TIME_STEP 10

WbDeviceTag motor_left;
WbDeviceTag motor_right;
WbDeviceTag s0, s1, s2, s3, s4,s5,s6,g0,g1; // Front & back & ground sensors


/*
prox.horizontal.0 : front left
prox.horizontal.1 : front middle-left
prox.horizontal.2 : front middle
prox.horizontal.3 : front middle-right
prox.horizontal.4 : front right
prox.horizontal.5 : back left
prox.horizontal.6 : back right
*/

void init_devices() {
  motor_left = wb_robot_get_device("motor.left");
  motor_right = wb_robot_get_device("motor.right");
  s0 = wb_robot_get_device("prox.horizontal.0");
  s1 = wb_robot_get_device("prox.horizontal.1");
  s2 = wb_robot_get_device("prox.horizontal.2");
  s3 = wb_robot_get_device("prox.horizontal.3");
  s4 = wb_robot_get_device("prox.horizontal.4");
  s5 = wb_robot_get_device("prox.horizontal.5");
  s6 = wb_robot_get_device("prox.horizontal.6");
  g0 = wb_robot_get_device("prox.ground.0");
  g1 = wb_robot_get_device("prox.ground.1");
  
 
  wb_distance_sensor_enable(s0,100);
  wb_distance_sensor_enable(s1,100);
  wb_distance_sensor_enable(s2,100);
  wb_distance_sensor_enable(s3,100);
  wb_distance_sensor_enable(s4,100);
  wb_distance_sensor_enable(s5,100);
  wb_distance_sensor_enable(s6,100);
  wb_distance_sensor_enable(g0,100);
  wb_distance_sensor_enable(g1,100);
}

void setVelocity(double leftVel, double rightVel)
  {
   wb_motor_set_position(motor_left, INFINITY);
   wb_motor_set_position(motor_right, INFINITY);
   wb_motor_set_velocity(motor_left, leftVel);
   wb_motor_set_velocity(motor_right, rightVel);
  }
  
void makeSyncMove(double leftTarget, double rightTarget, int maxT)
{  int t = 0;  
wb_motor_set_velocity(motor_left, leftTarget);  
wb_motor_set_velocity(motor_right, rightTarget);  
while ((wb_robot_step(TIME_STEP) != -1)&&(t < maxT))   
{    t += TIME_STEP;  }
}
 
int main(int argc, char **argv) {
  //int t = 0;
  double S0value,S1value,S2value,S3value,S4value,S5value,S6value,g0value,g1value;
  wb_robot_init();
  init_devices();
 
    
  while (wb_robot_step(TIME_STEP) != -1) 
  {

      S0value= wb_distance_sensor_get_value(s0);
      S1value= wb_distance_sensor_get_value(s1);
      S2value= wb_distance_sensor_get_value(s2);
      S3value= wb_distance_sensor_get_value(s3);
      S4value= wb_distance_sensor_get_value(s4);
      S5value= wb_distance_sensor_get_value(s5);
      S6value= wb_distance_sensor_get_value(s6);
      g0value= wb_distance_sensor_get_value(g0);
      g1value= wb_distance_sensor_get_value(g1);
 
        setVelocity(5-((S3value+S4value)/2)-S2value,5-((S0value+S1value)/2)-0.2); 
         
        if(S5value> 2270 || S6value> 2270 ){
          
         makeSyncMove(0,0,1000);  
         makeSyncMove(5,-5,1000);  
        
        }
        if ((g0value>723) || (g1value>723)){
        setVelocity(0,0);
        wb_robot_cleanup();
        return 0;
        }
  
}
wb_robot_cleanup();
  return 0;
}

