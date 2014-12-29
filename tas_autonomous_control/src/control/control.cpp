#include "control.h"
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QFileInfo>
#include <cmath>    //Include for sqrt Mustafa

int las_count = 0;

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    //Global Plan subscriber Mustafa
    global_path_sub = nh_.subscribe<nav_msgs::Path>("global_plan",1,&control::globalPlanCallback,this);

    //Local Plan subscriber Mustafa
    local_path_sub = nh_.subscribe<nav_msgs::Path>("global_plan",1,&control::globalPlanCallback,this);

    //Initialize nearest point index to -1 for checking
    nearestPointIndex = -1;



    // tmp
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &control::scanCallback,this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &control::poseCallback,this);
    //
    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

    //    Fp = 10;// need to test! defult:125

    //    current_ServoMsg.x = 1500;
    //    current_ServoMsg.y = 1500;

    //    previous_ServoMsg.x = 1500;
    //    previous_ServoMsg.y = 1500;

}
// We can subscribe to the odom here and get some feedback signals so later we can build our controllers
void control::odomCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    odom_linearVelocity = msg->linear.x;
    odom_angularVelocity = msg->angular.z;

    odom_steeringAngle = 180/PI*atan(odom_angularVelocity/odom_linearVelocity*CAR_LENGTH);

    odom_steeringAngle = 1500 + 500/30*odom_steeringAngle;

    if(odom_steeringAngle > 2000)
    {
        odom_steeringAngle = 2000;
    }
    else if(odom_steeringAngle < 1000)
    {
        odom_steeringAngle = 1000;
    }
}

//Subscribe to the local planner and map the steering angle (and the velocity-but we dont do that here-) to pulse width modulation values.
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_linearVelocity = msg->linear.x;

    cmd_angularVelocity = msg->angular.z;

    cmd_steeringAngle = 180/PI*atan(cmd_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle;

    if(cmd_steeringAngle > 2000)
    {
        cmd_steeringAngle = 2000;
    }
    else if(cmd_steeringAngle < 1000)
    {
        cmd_steeringAngle = 1000;
    }
}
// a flag method that tells us if we are controlling the car manually or automatically
void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}

//Global Plan Callback Mustafa
void control::globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    global_x.clear();
    global_y.clear();

    for (unsigned int i=0; i<msg->poses.size(); i++)
    {
        global_x.push_back(msg->poses[i].pose.position.x);
        global_y.push_back(msg->poses[i].pose.position.y);
    }
}



//geometry_msgs::Vector3 control::P_Controller()
//{
//    current_ServoMsg.x = previous_ServoMsg.x + Fp*(cmd_linearVelocity - odom_linearVelocity);

//    current_ServoMsg.y = cmd_steeringAngle;


//    if(current_ServoMsg.x > 1580)
//    {
//        current_ServoMsg.x = 1580;
//    }
//    else if(current_ServoMsg.x < 1300)
//    {
//        current_ServoMsg.x = 1300;
//    }

//    if(current_ServoMsg.y > 2000)
//    {
//        current_ServoMsg.y = 2000;
//    }
//    else if(current_ServoMsg.y < 1000)
//    {
//        current_ServoMsg.y = 1000;
//    }

//    previous_ServoMsg = current_ServoMsg;

//    return current_ServoMsg;
//}


//Pose callback with calculation of the nearest global path point Mustafa
void control::poseCallback(const geometry_msgs::PoseWithCovarianceStamped p){
    pos_x = p.pose.pose.position.x;
    pos_y = p.pose.pose.position.y;
    double min_dist = 10^6;
    unsigned int min_dist_index;
    double dist;
    if(global_x.size() == global_y.size() && global_x.size()>0){
        for(unsigned int i=0; i<global_x.size(); i++){
            dist = sqrt( exp2(pos_x-global_x.at(i)) + exp2(pos_y-global_y.at(i)) );
            if(dist < min_dist){
                min_dist = dist;
                min_dist_index = i;
            }
            //If we moved to a point which is at least 1 meter farther away from the minimum distance, break.
            //If there is a u turn to the same point, this could cause bugs
            if(dist >= min_dist+1)
                break;
        }
        nearestPointIndex = min_dist_index;
    }
}

//Curvature measure function Mustafa
float control::curvatureMeasure(){
    int CURV_SECT_SIZE_ = 100;
    float CURV_SECT_LENGTH_ = 2.5;
    if(nearestPointIndex == -1)
        return -1;
    if(nearestPointIndex+CURV_SECT_SIZE_ >= global_x.size()){
//        nearestPointIndex = global_x.size()-CURV_SECT_SIZE_-1;
        ROS_ERROR("Curvature calculation section goes beyond the goal, no curvature calculated");
        return -1;
    }
    float x_diff = global_x.at(nearestPointIndex) != global_x.at(nearestPointIndex+CURV_SECT_SIZE_);
    float y_diff = global_y.at(nearestPointIndex) != global_y.at(nearestPointIndex+CURV_SECT_SIZE_);
    if(x_diff==0 && y_diff==0){
        ROS_ERROR("Global path makes a self loop in this section, no curvature calculated");
        return -1;
    }
    else{
        return sqrt(exp2(x_diff)+exp2(y_diff))/CURV_SECT_LENGTH_;
    }
}

void control::scanCallback(const sensor_msgs::LaserScan laser)
{
    //hokuyo::LaserConfig lConfig;
    //hokuyo::Laser::getConfig(&lConfig);

    //std::cout << "min_angle: " << lConfig.min_angle << std::endl;
    //std::cout << "max_angle: " << lConfig.max_angle << std::endl;
    //std::cout << "size: " << laser.ranges.size() << std::endl;
    //ROS_INFO("size[%d]: ", laser.intensities.size());
    for (unsigned int i=0; i<720;i++)
    {
        ranges[i] = laser.ranges[i];
        // std::cout << "self_time_stamp: " << endl;
    }
    // las_count ++;
    //  if(las_count == 20){
    las_count = 0;
    float temp_range = 0.0;
    for (unsigned int i=710; i<720;i++)
    {
        temp_range += laser.ranges[i];
    }
    sD.range = temp_range / 10.0;
    sD.ownPos_x = pos_x;
    sD.ownPos_y = pos_y;
    QFile myfile("/home/tas_group_04/example.txt");
    QFileInfo info1(myfile);
    //std::cout << info1.absolutePath().toStdString() << std::endl;

    if (!myfile.open(QIODevice::WriteOnly | QIODevice::Text)){
        std::cout << "UNABLE TO OPEN!" << std::endl;
        return;
    }

    QTextStream out(&myfile);
    for(int i=0; i<scanValues.size(); i++){
        out << scanValues.at(i).range << " " << scanValues.at(i).ownPos_x  << " " << scanValues.at(i).ownPos_y
               //   <<" " <<laser.header.stamp.sec << "."  << laser.header.stamp.nsec
            <<"\n";
    }
    out << sD.range << " " << sD.ownPos_x  << " " << sD.ownPos_y  <<" " <<laser.header.stamp.sec
           // << "." << laser.header.stamp.nsec
        << "\n";

    myfile.close();
    scanValues.push_back(sD);
    //std::cout << "time_stamp: " << laser.header.stamp.sec << "."  << laser.header.stamp.nsec<< std::endl;
    //std::cout << sD.range << " " << sD.ownPos_x  << " " << sD.ownPos_y  << std::endl;
    //    }
}
