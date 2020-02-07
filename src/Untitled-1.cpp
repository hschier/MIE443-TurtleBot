#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <cstdlib>

#include <chrono>

// bumper
#define LEFT 0
#define CENTER 1
#define RIGHT 2
// state
#define RELEASED 0
#define PRESSED 1

kobuki_msgs::BumperEvent bum;
sensor_msgs::LaserScan laser;

void bumperCallback(const kobuki_msgs::BumperEvent msg)
{
    bum = msg;
}

void laserCallback(const sensor_msgs::LaserScan msg) {
	laser = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed =

    float angular = 0.0;
    float linear = 0.1;
    int state = 0;
    float turntime;
    float turndur = 3.5;
    int randsign;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        if(bum.state == PRESSED) {
            turntime = secondsElapsed;
            linear = 0;
            if(bum.bumper == RIGHT){
                state = 1;
            }
            else if(bum.bumper == LEFT){
                state = 2;

            }
            else{
                state = 3;
                randsign = copysign(1, rand()%3-1);
            }
        }

        if(state == 0){
            linear = 0.1;
            angular = 0.0;
        }

        if(state == 1){
            if(secondsElapsed - turntime < 0.5){
                linear = -0.1;
            }
            else if(secondsElapsed - turntime > 0.5){
                linear = 0;
                angular = 0.5;
            }
            if(secondsElapsed - turntime > turndur){
                state = 0;
            }
        }

        if(state == 2){
            if(secondsElapsed - turntime < 0.5){
                linear = -0.1;
            }

        if(state == 3){
            if(secondsElapsed - turntime < 0.5){
                linear = -0.1;
            }
            else if(secondsElapsed - turntime > 0.5){
                linear = 0;
                angular = randsign*0.5;
            }
            if(secondsElapsed - turntime > turndur){
                state = 0;
            }
        }
        
        




        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
