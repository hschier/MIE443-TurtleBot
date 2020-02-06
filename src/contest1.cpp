#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>

#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <unistd.h>
// comment
#define MAPPING_RATE 10000 // milliseconds

// bumper
#define LEFT 0
#define CENTER 1
#define RIGHT 2
// state
#define RELEASED 0
#define PRESSED 1

#define USING_SIM 1 // Comment me out if using a real robot

kobuki_msgs::BumperEvent bum;
kobuki_msgs::BumperEvent laser_bum;
sensor_msgs::LaserScan laser;

void bumperCallback(const kobuki_msgs::BumperEvent msg) {
    bum = msg;
}

void laserCallback(const sensor_msgs::LaserScan msg) {
	laser = msg;
    int points_count = laser.ranges.size();
    int mid = points_count/2;
    bool left = false;
    bool right = false;
    bool center = false;
    laser_bum.state = RELEASED;
    // laser bumper for middle
    for (int i = 0; i < points_count; i++) {
        if (laser.ranges[i] < 0.55) {
            laser_bum.state = PRESSED;
            if (i < mid-5) {
                left = true;
            }
            else if (i > mid+5) {
                right = true;
            }
            else{
                center = true;
            }
        }
    }
    if ((left && right) || (center && !left && !right)) {
        laser_bum.bumper = CENTER;
    }
    else if (left) {
        laser_bum.bumper = LEFT;
    }
    else if (right) {
        laser_bum.bumper = RIGHT;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("gmapping/dynamic_map");
    nav_msgs::GetMap map_srv;

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock>
        start = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock>
    
    
    most_recent_bump = std::chrono::system_clock::now() - std::chrono::milliseconds(10000);
    uint64_t secondsElapsed = 0;

    //hyperparamaters
    int spinfreq = 60;
    float linspeed = 0.1;
    float angspeed = 0.5;
    float angular = 0.0;
    float linear = linspeed;
    int state = 0;
    float turnstarttime = 0;
    float turndur = 0.8;
    float backdur = 0;
    int randsign;


    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        ROS_DEBUG("spun");
        if (secondsElapsed > 120 && secondsElapsed % spinfreq < 1){
            turnstarttime = secondsElapsed;
            state = 4;
        }
        else if(laser_bum.state == PRESSED && state == 0) {
            linear = 0;
            if(laser_bum.bumper == RIGHT) {
                state = 1;
            }
            else if(laser_bum.bumper == LEFT) {
                state = 2;

            }
            else{
                state = 3;
                randsign = copysign(1, rand()%3-1);
            }
        }

        if(state == 0) {
            linear = linspeed;
            angular = 0.0;
        }

        if(state == 1) {
            if(secondsElapsed - turnstarttime < backdur) {
                linear = -linspeed;
            }
            else{
                linear = 0;
                angular = -angspeed;
            }
            if(laser_bum.state == RELEASED || secondsElapsed - turnstarttime > turndur) {
                state = 0;
            }
        }

        if(state == 2) {
            if(secondsElapsed - turnstarttime < backdur) {
                linear = -linspeed;
            }
            else{
                linear = 0;
                angular = angspeed;
            }
            if(laser_bum.state == RELEASED || secondsElapsed - turnstarttime > turndur){
                state = 0;
            }
        }
        
        if(state == 3) {
            if(secondsElapsed - turnstarttime < backdur) {
                linear = -linspeed;
            }
            else{
                linear = 0;
                angular = randsign*angspeed;
            }
            if(laser_bum.state == RELEASED || secondsElapsed - turnstarttime > turndur) {
                state = 0;
            }
        }

        if(state == 4){
            angular = angspeed;
            linear = 0;
            if(secondsElapsed - turnstarttime > 5){
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
