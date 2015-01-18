#ifndef ADAPTIVE_VELOCITY_CONTROLLER_H
#define ADAPTIVE_VELOCITY_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "hokuyo_node/hokuyo.h"
#include "hokuyo_node/HokuyoConfig.h"
#include "nav_msgs/Path.h"
#include <vector>
#include <fstream>
#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

class adaptiveVelocityController
{
public:
    adaptiveVelocityController();
    //~adaptiveVelocityController();
    ros::NodeHandle nh_;
    float ranges[720];  //Read all ranges Mustafa
    int nearestPointIndex;          //Index of the nearest global point to the current position Mustafa
    std_msgs::Int16 control_Mode;
    //Publishers Mustafa
    ros::Publisher avc_pub;
    std_msgs::Int16 avc_vel_msg;
    //Subscriptions Mustafa
    ros::Subscriber global_path_sub;
    ros::Subscriber local_path_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber laser_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber wii_communication_sub;
    double MIN_CORNER_ANGLE, MAX_CORNER_ANGLE;
    int MIN_CORNER_INDEX, MAX_CORNER_INDEX, MIN_AREA_INDEX, MAX_AREA_INDEX;
    double MIN_AREA_ANGLE, MAX_AREA_ANGLE;
    std::vector<double> min_dist_lookup_table;

    void computeLookupTable();
    double clear_path_distance();
    double calc_min_allowed_distance(double angle);
    double calc_min_allowed_distance(int index);
    double index_to_angle(int index);
    int angle_to_index(double angle);
    int cmd_vel_converter();

    bool move_base_communication_error;

private:

    float pos_x;
    float pos_y;

    float cmd_vel;

    float global_plan_granularity, local_plan_granularity;

    double local_plan_curvature;
    double global_plan_curvature;

    double min_obstacle_distance;

    void scanCallback(const sensor_msgs::LaserScan msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped p);

    //Subscribe to global plan Mustafa
    void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg);

    void localPlanCallback(const nav_msgs::Path::ConstPtr& msg);

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

    void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

};

#endif // adaptiveVelocityController_H

