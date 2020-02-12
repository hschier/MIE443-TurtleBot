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
#include <string>

// TODO: Tune speeds
// TODO: modify gmapping for the tiny laptop (unless they allow bucket lol)
// TODO: modify traversal a bit, maybe random turns? It's pretty good rn.

// bumper
#define LEFT 0
#define CENTER 1
#define RIGHT 2
// state
#define RELEASED 0
#define PRESSED 1

kobuki_msgs::BumperEvent bumper;
kobuki_msgs::BumperEvent laser_bumper;
sensor_msgs::LaserScan laser;

float min_dist = 999.9;
int min_idx = 0;

void bumperCallback(const kobuki_msgs::BumperEvent msg) {
    bumper = msg;
}

void laserCallback(const sensor_msgs::LaserScan msg) {
	laser = msg;
    int points_count = laser.ranges.size();
    int mid = points_count/2;
    bool left = false;
    bool right = false;
    bool center = false;
    laser_bumper.state = RELEASED;
    // laser bumper for middle
    for (int i = 0; i < points_count; i++) {
        if (laser.ranges[i] < 0.56) {
            laser_bumper.state = PRESSED;
            if (i < mid) {
                right = true;
            } else {
                left = true;
            }
        }
    }
    min_dist = 9999;
    for (int i = 0; i < points_count; i++) {
        if (laser.ranges[i] < min_dist) {
            min_dist = laser.ranges[i];
            min_idx = i;
        }
    }

    if (left) {
        ROS_WARN("LEFT");
        laser_bumper.bumper = LEFT;
    } else if (right) {
        ROS_WARN("RIGHT");
        laser_bumper.bumper = RIGHT;
    }
}

bool w_coin_flip(float weight) {
    // weighted coin flip. val is the ratio of 1 to 0
    float val = rand() / float(RAND_MAX);
    return (val < weight);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("gmapping/dynamic_map");
    nav_msgs::GetMap map_srv;

    ros::Rate loop_rate(60);
    ros::Rate overshoot_time(500);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock>
        start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    std::chrono::time_point<std::chrono::system_clock>
        now = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock>
        state_timestamp = std::chrono::system_clock::now();
    
    std::chrono::time_point<std::chrono::system_clock>
        last_spin_timestamp = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock>
        last_backup_timestamp = std::chrono::system_clock::now();

    uint64_t ms_in_state = 0;
    uint64_t ms_since_spin = 0;
    uint64_t ms_since_backup = 0;

    // hyperultragigauberparamaters
    float BACKUP_SPEED = -0.19;
    float FORWARD_SPEED = 0.15; // best .15
    float TURN_SPEED = 1;
    float AVOID_SPEED = 0.5;
    float PEEK_TURN_SPEED = 1.2; // best : 1.2
    float FULL_TURN_SPEED = 0.5;

    int peekTime = 500; // best : 500
    int fullTurnTime = 12566*1.2;

    float angular = 0.0;
    float linear = 0.0;

    std::string state = "slow 360 spin"; 

    while(ros::ok() && secondsElapsed <= 8*60) {
        ros::spinOnce();
        now = std::chrono::system_clock::now();
        // figure out how long we've been in current state
        ms_in_state = std::chrono::duration_cast<std::chrono::milliseconds>(now - state_timestamp).count();
        ms_since_spin = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_spin_timestamp).count();
        ms_since_backup = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_backup_timestamp).count();
        ROS_INFO("in state %s for %d ms", state.c_str(), ms_in_state);
        ROS_INFO("min dist is %f m at index %d", min_dist, min_idx);

        ///////////////////////////////////////////////////
        // Override current state if these events happen //
        ///////////////////////////////////////////////////

        // nothing here right now...

        ///////////////////////////////////////////////////
        // Define our states                             //
        ///////////////////////////////////////////////////
        if (state.compare("go forward") == 0) {
            linear = FORWARD_SPEED;
            angular = 0;
            if (bumper.state == PRESSED) {
                state = secondsElapsed < 240 ? "back up and turn left" : "back up and turn right";
                state_timestamp = now;
            } else if(ms_since_spin > 45000){
                state = "360 spin";
                last_spin_timestamp = now;
                state_timestamp = now;
            } else if (laser_bumper.state == PRESSED && laser_bumper.bumper == LEFT) {
                state = "avoiding left wall";
                state_timestamp = now;
            } else if (laser_bumper.state == PRESSED && laser_bumper.bumper == RIGHT) {
                state = "avoiding right wall";
                state_timestamp = now;
            }
        } else if (state.compare("back up and turn right") == 0) {
            if (ms_in_state < 1000){
                linear = BACKUP_SPEED;
                angular = 0;
            }
            else if (ms_in_state < 2000) {
                linear = 0;
                angular = -TURN_SPEED;
            }
            else {
                state = "go forward";
                state_timestamp = now;
            }
        } else if (state.compare("back up and turn left") == 0) {
            if (ms_in_state < 1000){
                linear = BACKUP_SPEED;
                angular = 0;
            }
            else if (ms_in_state < 2000) {
                linear = 0;
                angular = TURN_SPEED;
            }
            else {
                state = "go forward";
                state_timestamp = now;
            }
        } else if (state.compare("avoiding left wall") == 0) {
            linear = 0;
            angular = -AVOID_SPEED;
            if (laser_bumper.state == RELEASED) {
                state = "go forwards";
                state_timestamp = now;
            }
        } else if (state.compare("avoiding right wall") == 0) {
            linear = 0;
            angular = AVOID_SPEED;
            if (laser_bumper.state == RELEASED) {
                state = "go forwards";
                state_timestamp = now;
            }
        } else if(state.compare("360 spin") == 0){
            linear = 0;
            angular = FULL_TURN_SPEED;
            if (ms_in_state > fullTurnTime){
                state = "go forwards";
                state_timestamp = now;
            }
        } else if(state.compare("slow 360 spin") == 0){
            linear = 0;
            angular = FULL_TURN_SPEED/2;
            if (ms_in_state > fullTurnTime*2 || (laser_bumper.state == PRESSED && laser_bumper.bumper == LEFT)){
                state = "go forwards";
                state_timestamp = now;
            }
        }

        vel.linear.x = linear;
        vel.angular.z = angular;
        // vel.linear.x = 0;
        // vel.angular.z = 0;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    return 0;
}
