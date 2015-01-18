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
#include "sensor_msgs/Imu.h" // include imu messages
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hokuyo_node/hokuyo.h"
#include "hokuyo_node/HokuyoConfig.h"
#include "kmeans/kmeans.h"
#include <vector>

#include "wifi_information/wifi_msgs.h" // INCLUDE WIFI MSG - FOR TESTING


#include <QVector>
#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

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

    ros::Subscriber imu_data_sub_;
    //tmp
    ros::Subscriber laser_sub_;
    ros::Subscriber pose_sub_;

	ros::Subscriber wifi_sub_;



    std_msgs::Int16 control_Brake; /* flag for brake */
    std_msgs::Int16 control_Mode; /* flag for car mode: manual or autonomous */

    double cmd_linearVelocity;
    double cmd_angularVelocity;
    double cmd_steeringAngle;

    double odom_linearVelocity;
    double odom_angularVelocity;
    double odom_steeringAngle;

    int parkDetected;

    geometry_msgs::Vector3 control_servo;
    std::vector<myscanData> scanValues;
    int detectJumps();

    double eucl_distance;
    float pos_x;
    float pos_y;

    void printStuff();
   int findReferencePosition();
   int calculateWaypoints();
   //********************PARKING PARAMETERS ****************************/
// initializing___________________________________
float dist_b_1j;
float dist_a_1j;
float dist_b_2j   ;
float dist_a_2j   ;
float x_ll  ;
float y_ll  ;
float x_ul  ;
float y_ul  ;
float x_ur  ;
float y_ur  ;
float x_lr  ;
float y_lr  ;
float x_goal  ;
float y_goal  ;
float x_start  ;
float y_start  ;
float x_g_c  ;
float y_g_c  ;
float x_s_c  ;
float y_s_c  ;
float x_turn  ;
float y_turn  ;

// car position when 1st and 2nd jump were detected
float x1j   ;
float y1j   ;
float x2j   ;
float y2j   ;

float wifi_x, wifi_y;

private:
    float ranges[720];
    int first_jump_index;
    int jump;
    int second_jump_index;
    int cntr;
    double cluster_centroid[2];
    QVector<int> cluster_assignment_final,derivative_of_scans;
    QVector<double> clustered_dataSet,smoothedRanges;
    void calculateDerivatives();
    void smoothData();
    void runKmeans();
	
    double qx,qy,qz,qw; //quaternion from imu

    int scanCtr;
    int counter;
    int imuCtr;

    /* subscribe the cmd message from move_base */

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* subscribe the virtual odom message as a feedback for controller */
    void odomCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* check the wii states and switch the flag for manual mode and autonomous mode */
    void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

    //tmp
    void scanCallback(const sensor_msgs::LaserScan laser);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped p);

    void imuCallback(const sensor_msgs::Imu imu_data);



    void wifiCallback(const wifi_information::wifi_msgs wifi_data);



};


void moveForward(control* control_object);
void velocityStop(control* control_object);
void moveBackward(control* control_object);


#define car_w 0.44 // car width
#define car_l 0.66 // car length
#define safety_left 0.09 // 9cm safety distance to the wall
#define safety_back 0.04 // 1cm safety distance to box
#define turn_circ 0.775 // turning circle of the car
#endif // CONTROL_H
