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

kobuki_msgs::BumperEvent bum;
kobuki_msgs::BumperEvent laser_bum;
sensor_msgs::LaserScan laser;

enum State_enum {go_forwads, backup_then_turn_right, backup_then_turn_left, rng_turn, backup_then_rng_turn, bumper_right_turn, bumper_left_turn } state;

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
            } else if (i > mid+5) {
                right = true;
            }
            else {
                center = true;
            }
        }
    }
    if ((left && right) || (center && !left && !right)) {
        laser_bum.bumper = CENTER;
    } else if (left) {
        laser_bum.bumper = LEFT;
    } else if (right) {
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

    // hyperparamaters
    int spin_freq = 60;
    float lin_speed = 0.1;
    float ang_speed = 0.3;
    float angular = 0.0;
    float linear = lin_speed;
    int state = go_forwads;
    float start_time = 0;
    float turn_dur = 2;
    float back_dur = 2;
    int randsign;


    while(ros::ok() && secondsElapsed <= 8*60) {
        ros::spinOnce();
        ROS_INFO("State is %i", state);

        // Determine which case we should be in
        if (bum.state == PRESSED && state != backup_then_turn_right) {
            start_time = secondsElapsed;
            randsign = copysign(1, rand()%3-1);
            state = backup_then_turn_right;
        }
        else if (secondsElapsed > 120 && secondsElapsed % spin_freq == 0) {
            // if it's pressed and we are going forwards
            start_time = secondsElapsed;
            randsign = copysign(1, rand()%3-1);
            state = rng_turn;
        }
        else if (laser_bum.state == PRESSED && state == go_forwads) {
            linear = 0;
            if (laser_bum.bumper == RIGHT) {
                start_time = secondsElapsed;
                state = backup_then_turn_left;
            } else if (laser_bum.bumper == LEFT) {
                start_time = secondsElapsed;
                state = backup_then_turn_right;
            } else {
                start_time = secondsElapsed;
                randsign = copysign(1, rand()%3-1);
                state = backup_then_rng_turn;
            }
        }

        switch (state) {
            case go_forwads: // default case, go forwards
                linear = lin_speed;
                angular = 0.0;
            break;
            case backup_then_turn_left:
                if (secondsElapsed - start_time < back_dur) {
                    linear = -lin_speed;
                } else {
                    linear = 0;
                    angular = -ang_speed;
                }
                if (secondsElapsed - start_time > turn_dur) {
                    state = go_forwads;
                }
            break;
            case backup_then_turn_right:
                if (secondsElapsed - start_time < back_dur) {
                    linear = -lin_speed;
                } else {
                    linear = 0;
                    angular = ang_speed;
                } if (secondsElapsed - start_time > turn_dur) {
                    state = go_forwads;
                }
            break;
            case backup_then_rng_turn:
                if (secondsElapsed - start_time < back_dur) {
                    angular = 0;
                    linear = -lin_speed;
                } else {
                    linear = 0;
                    angular = randsign*ang_speed;
                }
                if (secondsElapsed - start_time > turn_dur) {
                    state = go_forwads;
                }
            break;
            case rng_turn:
                angular = randsign*ang_speed;
                linear = 0;
                if (secondsElapsed - start_time > turn_dur) {
                    state = go_forwads;
                }
            break;
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
