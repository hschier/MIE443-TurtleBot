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

// comment
#define MAPPING_RATE 10000 // milliseconds

// bumper
#define LEFT 0
#define CENTER 1
#define RIGHT 2
// state
#define RELEASED 0
#define PRESSED 1

kobuki_msgs::BumperEvent bumper;
kobuki_msgs::BumperEvent laser_bum;
sensor_msgs::LaserScan laser;

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

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock>
        start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    std::chrono::time_point<std::chrono::system_clock>
        now = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock>
        state_timestamp = std::chrono::system_clock::now();

    uint64_t ms_in_state = 0;

    // hyperultragigauberparamaters
    float BACKUP_SPEED = -0.1;
    float FORWARD_SPEED = 0.1;
    float TURN_SPEED = 0.3;

    float angular = 0.0;
    float linear = 0.0;

    std::string state = "go forwards"; 

    while(ros::ok() && secondsElapsed <= 8*60) {
        ros::spinOnce();
        ROS_INFO("in state %s for %d ms", state.c_str(), ms_in_state);

        now = std::chrono::system_clock::now();
        // figure out how long we've been in current state
        ms_in_state = std::chrono::duration_cast<std::chrono::milliseconds>(now - state_timestamp).count();

        ///////////////////////////////////////////////////
        // Override current state if these events happen //
        ///////////////////////////////////////////////////

        // nothing here right now...

        ///////////////////////////////////////////////////
        // Define our states                             //
        ///////////////////////////////////////////////////
        if (state.compare("go fowards")) {
            ROS_INFO("going fowards");
            linear = FORWARD_SPEED;
            angular = 0;
            if (bumper.state == PRESSED) {
                state = "backing up before rng turn";
                state_timestamp = now;
            } else if (laser_bum.state == PRESSED && laser_bum.bumper == CENTER) {
                state = "backing up before rng turn";
                state_timestamp = now;
            } else if (laser_bum.state == PRESSED && laser_bum.bumper == LEFT) {
                state = "avoiding left wall";
                state_timestamp = now;
            } else if (laser_bum.state == PRESSED && laser_bum.bumper == RIGHT) {
                state = "avoiding right wall";
                state_timestamp = now;
            }
        } else if (state.compare("backing up before right turn")) {
            linear = BACKUP_SPEED;
            angular = 0;
            if (ms_in_state > 1000) {
                state = "right turn";
                state_timestamp = now;
            }
        } else if (state.compare("backing up before left turn")) {
            linear = BACKUP_SPEED;
            angular = 0;
            if (ms_in_state > 1000) {
                state = "left turn";
                state_timestamp = now;
            }
        } else if (state.compare("backing up before rng turn")) {
            linear = BACKUP_SPEED;
            angular = 0;
            if (ms_in_state > 1000) {
                state = "rng turn";
                state_timestamp = now;
            }
        } else if (state.compare("right turn")) {
            linear = 0;
            angular = TURN_SPEED;
            if (ms_in_state > 1000) {
                state = "go fowards";
                state_timestamp = now;
            }
        } else if (state.compare("left turn")) {
            linear = 0;
            angular = -TURN_SPEED;
            if (ms_in_state > 1000) {
                state = "go fowards";
                state_timestamp = now;
            }
        } else if (state.compare("rng turn")) {
            state = w_coin_flip(0.50) ? "left turn" : "left right";
            state_timestamp = now;
        } else if (state.compare("avoiding left wall")) {
            linear = 0;
            angular = -TURN_SPEED;
            if (laser_bum.state == RELEASED) {
                state = "go fowards";
                state_timestamp = now;
            }
        } else if (state.compare("avoiding right wall")) {
            linear = 0;
            angular = TURN_SPEED;
            if (laser_bum.state == RELEASED) {
                state = "go fowards";
                state_timestamp = now;
            }
        }

        vel.linear.x = linear;
        vel.angular.z = angular;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    return 0;
}
