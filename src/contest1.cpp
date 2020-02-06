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
    int points_count = size(laser.ranges);
    int mid = points_count/2;
    bool left = false;
    bool right = false;
    bool center = false;
    laser_bum.state = UNPRESSED;
    // laser bumper for middle
    for (int i = 0; i < points_count; i++) {
        if (laser.ranges(i) < 0.55) {
            laser_bum.state = PRESSED;
            if (i < mid-5){
                left = true;
            }
            else if (i > mid+5){
                right = true;
            }
            else{
                center = true;
            }
        }
    }
    if ((left && right) || (center && !left && !right)){
        laser_bum.bumper = CENTER;
    }
    else if (left){
        laser_bum.bumper = LEFT;
    }
    else if (right){
        laser_bum.bumper = RIGHT;
    }
}

int main(int argc, char **argv){
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

    bool evading_center = 0;
    bool evading_left = 0;
    bool evading_right = 0;

    while (ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        std::chrono::time_point<std::chrono::system_clock>
            now = std::chrono::system_clock::now();
            
        if (laser_bumper.center) {
            evading_center = 1;
            ROS_DEBUG("Bumped Center");
        } else if (laser_bumper.left) {
            evading_left = 1;
            ROS_DEBUG("Bumped Left");
        } else if (laser_bumper.right) {
            evading_right = 1;
            ROS_DEBUG("Bumped Right");
        } else {
            ROS_DEBUG("Nothing Bumped");
        }

        ms_since_last_bump = std::chrono::duration_cast<std::chrono::milliseconds>(now - most_recent_bump).count();

        if (ms_since_last_bump < 500) {
            if
        } else {
            evading_center = 0;
            evading_left = 0;
            evading_right = 0;
        }

        vel.angular.z; // angular
        vel.linear.x; // linear
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
