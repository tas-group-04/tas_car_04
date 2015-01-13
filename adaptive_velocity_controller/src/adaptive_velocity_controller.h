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
    std::vector<float> global_x;    //Global path x positions Mustafa
    std::vector<float> global_y;    //Global path y positions Mustafa
    std::vector<float> local_x;     //Local path x positions Mustafa
    std::vector<float> local_y;     //Local path y positions Mustafa
    double local_plan_curvature;
    double global_plan_curvature;
    int nearestPointIndex;          //Index of the nearest global point to the current position Mustafa
    float curvatureMeasure();    //Calculate curvature of the path in front Mustafa
    std_msgs::Int16 control_Mode;
    //Subscriptions Mustafa
    ros::Subscriber global_path_sub;
    ros::Subscriber local_path_sub;
    ros::Subscriber laser_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber wii_communication_sub;
    double MIN_CORNER_ANGLE, MAX_CORNER_ANGLE;
    int MIN_CORNER_INDEX, MAX_CORNER_INDEX, MIN_AREA_INDEX, MAX_AREA_INDEX, CURV_SECT_SIZE, CURV_SECT_OVERLAP_SIZE;
    double MIN_AREA_ANGLE, MAX_AREA_ANGLE;
    std::vector<double> min_dist_lookup_table;

    void computeLookupTable();
    double clear_path_distance();
    double calc_curvature();
    double calc_min_allowed_distance(double angle);
    double calc_min_allowed_distance(int index);
    double index_to_angle(int index);
    int angle_to_index(double angle);
    int cmd_vel_converter();

    bool move_base_communication_error;

private:

    float pos_x;
    float pos_y;

    void scanCallback(const sensor_msgs::LaserScan msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped p);

    //Subscribe to global plan Mustafa
    void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg);

    void localPlanCallback(const nav_msgs::Path::ConstPtr& msg);

    void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

};

#endif // adaptiveVelocityController_H

