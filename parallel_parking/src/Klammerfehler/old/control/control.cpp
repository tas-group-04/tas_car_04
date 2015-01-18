#include "control.h"
#include <iostream>
#include <tf/transform_datatypes.h>  // quaternion to roll pitch yaw



//// write to text file
#include <iostream>
#include <fstream>

///***********************Textfile for imu messages****************************/
const char *path="/home/tas_group_04/ImuPlots/imu_yaw.txt";	//set path here !!!
std::ofstream IMUfile(path); //open in constructor
std::string data;
std::stringstream ss1;


int las_count = 0;
using namespace std;
control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    // tmp
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &control::scanCallback,this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &control::poseCallback,this);
    //
    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);


    imu_data_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data",1000,&control::imuCallback,this);


    cluster_centroid[0] = 0.6;
    cluster_centroid[1] = 0.3;
    cntr = 0;
    scanCtr=0;
    counter = 0;
    parkDetected = 0;
    imuCtr=0;
    first_jump_index = -15;
    second_jump_index = -15;

    eucl_distance = 99.0;

    //********************PARKING PARAMETERS ****************************/
    // initializing___________________________________
    dist_b_1j = 0.0;
    dist_a_1j = 0.0;
    dist_b_2j = 0.0;
    dist_a_2j = 0.0;
    x_ll= 0.0;
    y_ll= 0.0;
    x_ul= 0.0;
    y_ul= 0.0;
    x_ur= 0.0;
    y_ur= 0.0;
    x_lr= 0.0;
    y_lr= 0.0;
    x_goal= 0.0;
    y_goal= 0.0;
    x_start= 0.0;
    y_start= 0.0;
    x_g_c= 0.0;
    y_g_c= 0.0;
    x_s_c= 0.0;
    y_s_c= 0.0;
    x_turn= 0.0;
    y_turn= 0.0;

    // car position when 1st and 2nd jump were detected
    x1j = 0.0;
    y1j = 0.0;
    x2j = 0.0;
    y2j = 0.0;
}


/********************************************************************************************************/
/********************************************************************************************************/
/*********************************SIMPLE MOVEMENT FUNCTIONS****************************************/
void moveForward(control* control_object){
    control_object->control_servo.x=1540;					// slow forward movement
    control_object->control_servo.y=1500;
    control_object->control_servo_pub_.publish(control_object->control_servo);
    ROS_INFO("Move forward!");
}

void velocityStop(control* control_object){
    control_object->control_servo.x=1500;
    control_object->control_servo.y=1500;
    control_object->control_servo_pub_.publish(control_object->control_servo);
    ROS_INFO("Velocity Stop!");
}

void moveBackward(control* control_object){
    control_object->control_servo.x=1460;					// slow backward movement
    control_object->control_servo.y=1500;
    control_object->control_servo_pub_.publish(control_object->control_servo);
    ROS_INFO("Move Backward!");
}
/*********************************SIMPLE MOVEMENT FUNCTIONS****************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/



/*********************************PARKING LOT DETECTION FUNCTIONS****************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/*********************************AVERAGING FILTER FUNCTION****************************************/
void control::smoothData()
{
    smoothedRanges.clear();
    double curRange;
    double smoothedRange;
    for(int i=0;i<scanValues.size();i++){
        curRange = scanValues.at(i).range;
        if(i<2 || i>(scanValues.size()-3))
            smoothedRanges.append(curRange);
        else{
            //smoothedRange = 0.2 * ( scanValues.at(i-2).range + scanValues.at(i-1).range + curRange +  scanValues.at(i+1).range +  scanValues.at(i+2).range);
            smoothedRanges.append(curRange);
            //smoothedRanges.append(smoothedRange);
        }
    }
}
/*********************************AVERAGING FILTER FUNCTION****************************************/


/*********************************KMeans FUNCTION****************************************/
void control::runKmeans()
{
    double clustered_scans[smoothedRanges.size()];
    cluster_centroid[0] = 0.6;
    cluster_centroid[1] = 0.3;
    int cluster_assignment_final_array[smoothedRanges.size()];
    for(int i =0; i < smoothedRanges.size();i++){
        clustered_scans[i] = smoothedRanges.at(i);
    }
    kmeans(1,clustered_scans,smoothedRanges.size(),2,cluster_centroid,cluster_assignment_final_array);
    cluster_assignment_final.clear();
    for(int i =0; i < smoothedRanges.size();i++){
        //std::cout << cluster_assignment_final_array[i] << std::endl;
        cluster_assignment_final.append(cluster_assignment_final_array[i]);
    }
    //std::cout << "cluster centroid at the end of kmeans call: " << cluster_centroid[0] << "and" << cluster_centroid[1] << std::endl;
}
/*********************************KMeans FUNCTION****************************************/


/********************************************************************************************************/
/*********************************DERIVATIVE FUNCTION****************************************/
void control::calculateDerivatives()
{
    derivative_of_scans.clear();
    int diff;
    for(int i=0;i < scanValues.size(); i++){
        if(i == 0 || i == (scanValues.size()-1))
            derivative_of_scans.append(0);
        else{
            diff = cluster_assignment_final.at(i) - cluster_assignment_final.at(i-1);
            derivative_of_scans.append(diff);
        }
    }
}
/*********************************DERIVATIVE FUNCTION****************************************/

/*********************************JUMP DETECTION FUNCTION****************************************/
int control::detectJumps()
{
    cntr = 0;
    //cout << "derivative size: "<<derivative_of_scans.size() << endl;
    for(int i=0; i<derivative_of_scans.size(); i++){
        //  std::cout << "derivative "<<derivative_of_scans[i] << std::endl;

        if(derivative_of_scans[i] != 0){
            cntr ++;
            if(cntr == 2){
                jump = derivative_of_scans[i];

                first_jump_index = i;
                break;
            }
        }
    }
    if(first_jump_index!= -15){
        for(int i= (first_jump_index+1); i < derivative_of_scans.size(); i++){
            //    std::cout << "derivative "<<derivative_of_scans[i] << std::endl;

            if(derivative_of_scans[i] == (-1 * jump)){
                second_jump_index = i;
                return 1;
            }
        }
    }
    return 0;
}
/*********************************JUMP DETECTION FUNCTION****************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/


int control::findReferencePosition(){
    //std::cout << scanValues.at(second_jump_index).ownPos_x << std::endl;
    double length_park;


    double TOL_referencePosition = 0.04;
    //eucl_distance = pow((scanValues.at(second_jump_index).ownPos_x-pos_x),2)+pow((scanValues.at(second_jump_index).ownPos_y-pos_y),2);
    //eucl_distance = sqrt(eucl_distance);
    eucl_distance = scanValues.at(second_jump_index).ownPos_y-pos_y;
    //std::cout <<"x,y at second: (" << scanValues.at(second_jump_index).ownPos_x <<", " << scanValues.at(second_jump_index).ownPos_y << ")" <<
    //            "x now: ( " << pos_x << " , " << pos_y << " )" << std::endl;
    length_park = pow((scanValues.at(first_jump_index).ownPos_x-scanValues.at(second_jump_index).ownPos_x),2) +  pow((scanValues.at(first_jump_index).ownPos_y-scanValues.at(second_jump_index).ownPos_y),2);;
    length_park = sqrt(length_park);
    // std::cout << "park length: " << length_park << std::endl;
    std::cout << "eucl_distance: " << eucl_distance << std::endl;
    if (eucl_distance - 0.6 < TOL_referencePosition){
        return 1;
    }

    return 0;
}

int control::calculateWaypoints(){
    if(parkDetected){
        x1j = scanValues.at(first_jump_index).ownPos_x;
        y1j = scanValues.at(first_jump_index).ownPos_y;
        x2j = scanValues.at(second_jump_index).ownPos_x;
        y2j = scanValues.at(second_jump_index).ownPos_y;


        // ranges before and after jumps
        dist_b_1j = scanValues.at(first_jump_index-2).range - 0.5*car_w; // range before 1st jump
        dist_a_1j = scanValues.at(first_jump_index+2).range - 0.5*car_w; // range after 1st jump
        dist_b_2j = scanValues.at(second_jump_index-2).range - 0.5*car_w; // range before 2nd jump
        dist_a_2j = scanValues.at(second_jump_index + 2).range - 0.5*car_w; // range after 2nd jump


        // corners of parking lot (LowerLeft, UpperLeft, UpperRight, LowerRight)
        x_ll = x1j + dist_b_1j;
        y_ll = y1j;
        x_ul = x1j + dist_a_1j;
        y_ul = y1j;
        x_ur = x2j + dist_b_2j;
        y_ur = y2j;
        x_lr = x2j + dist_a_2j;
        y_lr = y2j;

        // desired goal position
        x_goal = x_ul - safety_left - 0.5*car_w;
        y_goal = y_ul - safety_back - 0.5*car_l;
        x_g_c = x_goal - turn_circ; // x center of goal circle
        y_g_c = y_goal; // y center of goal cirlce


        // start position of car
        x_start = x2j;
        x_s_c = x_start + turn_circ; // x center of start circle
        y_s_c = y_g_c - sqrt(pow((2*turn_circ),2)-pow((x_s_c-x_g_c),2)); // y center of start cirlce
        y_start = y_s_c;

        // turn point
        x_turn = 0.5*(x_s_c + x_g_c);
        y_turn = 0.5*(y_s_c + y_g_c);
        return 1;
    }else{return 0;}
}


/***********************************CALLBACK FUNCTIONS******************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/

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





/******************************AMCL POSE CALLBACK*************************************************/
void control::poseCallback(const geometry_msgs::PoseWithCovarianceStamped p){
    pos_x = p.pose.pose.position.x;
    pos_y = p.pose.pose.position.y;

    if(second_jump_index != -15){
        eucl_distance = scanValues.at(second_jump_index).ownPos_y-pos_y;
        ROS_INFO_STREAM("Euc= " << eucl_distance);
    }

}
/******************************AMCL POSE CALLBACK*************************************************/





///*****************************IMU CALLBACK*************************************************/
void control::imuCallback(const sensor_msgs::Imu imu_data){

    qx=imu_data.orientation.x;
    qy=imu_data.orientation.y;
    qz=imu_data.orientation.z;
    qw=imu_data.orientation.w;




    //quaternion to roll pitch yaw
    //quaternion q(x,y,z,w);
    tf::Quaternion q(qx,qy,qz,qw);

    // get roll,pitch,yaw angles from quaternion
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // transform to degree
    roll = (roll * 180.0) / 3.141593;
    pitch = (pitch * 180.0) / 3.141593;
    yaw = (yaw * 180.0) / 3.141593;

    //    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

    imuCtr++;
    if (imuCtr > 25){ // WRITE EVERY 25th VALUE
        // WRITE TO IMU TEXT FILE
        ss1 << yaw;
        data =ss1.str();
        IMUfile << data;
        ss1.str(std::string());	// clear stringstream
        IMUfile << "\n";
        imuCtr = 0;
    }
}
///******************************IMU CALLBACK************************************************




/******************************LASERSCAN CALLBACK*************************************************/
void control::scanCallback(const sensor_msgs::LaserScan laser)
{
    scanCtr++;
    if(scanCtr >= 1){
        float temp_range = 0.0;

        scanCtr=0;
        for(unsigned int i=710; i<720;i++)
        {
            temp_range += laser.ranges[i];
        }
        sD.range = temp_range / 10.0;
        sD.ownPos_x = pos_x;
        sD.ownPos_y = pos_y;
        //      std::cout <<"x,y: (" <<  sD.ownPos_x << " , "<< sD.ownPos_y << " )"<< std::endl;
        scanValues.push_back(sD);
        if(!parkDetected){
            counter++;

            if (counter > 20){
                smoothData();
                runKmeans();
                calculateDerivatives();
                //std::cout << "return value is: "<< detectJumps() << std::endl;
                std::cout << "CALL OF smoothData(), runKmeans() and calculateDerivatives()" << std::endl;
                counter = 0;
            }
        }
    }
}
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/

void control::printStuff(){
    std::cout << "first jump index: " << first_jump_index << std::endl;
    std::cout << "second jump index: " << second_jump_index << std::endl;
    std::cout << "scan value at first jump index: " << scanValues.at(first_jump_index).range << std::endl;
    std::cout << "scan value at second jump index: " << scanValues.at(second_jump_index).range << std::endl;

    std::cout << "Distance to Wall: " << scanValues.at(first_jump_index+2).range << std::endl;
    std::cout << "Distance to Car: " << scanValues.at(first_jump_index-2).range << std::endl;     // better use cluster_centroids for this assignment here!
}
