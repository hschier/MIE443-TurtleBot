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

struct laser_bumper_struct {
    bool left;
    bool center;
    bool right;
} laser_bumper;

kobuki_msgs::BumperEvent bum;
sensor_msgs::LaserScan laser;

void bumperCallback(const kobuki_msgs::BumperEvent msg) {
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

    ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("gmapping/dynamic_map");
    nav_msgs::GetMap map_srv;

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // update timer for mapping
    static std::chrono::time_point<std::chrono::system_clock> last_map_update;
    last_map_update = std::chrono::system_clock::now();

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

        // check if we should do mapping
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_map_update).count() > MAPPING_RATE) {
            
            if(!map_client.call(map_srv)) {
                ROS_ERROR("MAP SERVICE DID NOT RESPOND!");
            }

            now = std::chrono::system_clock::now();
            last_map_update = now;
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
