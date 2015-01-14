#ifndef CONTROL_H
#define CONTROL_H

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
#include <QFile>
#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

//typedef struct _myscanData myscanData;



class control
{
public:
    control();
    struct myscanData{
        float range;
        float ownPos_x;
        float ownPos_y;
    }sD;
    ros::NodeHandle nh_;
    ros::Publisher control_servo_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber wii_communication_sub;

    ros::Subscriber avc_sub;

    //tmp
    ros::Subscriber laser_sub_;
    ros::Subscriber pose_sub_;

    std_msgs::Int16 control_Brake; /* flag for brake */
    std_msgs::Int16 control_Mode; /* flag for car mode: manual or autonomous */

    int avc_vel;

    double cmd_linearVelocity;
    double cmd_angularVelocity;
    double cmd_steeringAngle;

    double odom_linearVelocity;
    double odom_angularVelocity;
    double odom_steeringAngle;

    geometry_msgs::Vector3 control_servo;
    std::vector<myscanData> scanValues;
    float ranges[720];  //Read all ranges Mustafa
    /*td::vector<float> global_x;    //Global path x positions Mustafa
    std::vector<float> global_y;    //Global path y positions Mustafa
    int nearestPointIndex;          //Index of the nearest global point to the current position Mustafa
    //Subscriptions Mustafa
    ros::Subscriber global_path_sub;
    ros::Subscriber local_path_sub;*/

private:

    float pos_x;
    float pos_y;

    void AVCCallback(const std_msgs::Int16::ConstPtr& msg);

    /* subscribe the cmd message from move_base */

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* subscribe the virtual odom message as a feedback for controller */
    void odomCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* check the wii states and switch the flag for manual mode and autonomous mode */
    void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

    //tmp
    void scanCallback(const sensor_msgs::LaserScan laser);
};

#endif // CONTROL_H
