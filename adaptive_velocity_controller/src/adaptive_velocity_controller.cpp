#include "adaptive_velocity_controller.h"
#include <iostream>
#include <cmath>    //Include for sqrt Mustafa

#include <dynamic_reconfigure/server.h>
#include <adaptive_velocity_controller/AVCConfig.h>

#include "std_msgs/Int16.h"

using namespace std;

//Parameters
double MAX_ANGLE, MIN_ANGLE, ANGLE_RESOLUTION, CAR_WIDTH, MAX_LOOK_AHEAD_DIST;
double MIN_LOOK_AHEAD_DIST, DISTANCE_TOLERANCE, CURV_SECT_LENGTH, CURV_SECT_OVERLAP;
bool ignore_local_planner_vel_cmd;

//Calculations
/*double MIN_CORNER_ANGLE, MAX_CORNER_ANGLE;
int MIN_CORNER_INDEX, MAX_CORNER_INDEX, MIN_AREA_INDEX, MAX_AREA_INDEX, CURV_SECT_SIZE, CURV_SECT_OVERLAP_SIZE;
double MIN_AREA_ANGLE, MAX_AREA_ANGLE;
vector<double> min_dist_lookup_table;*/
sensor_msgs::LaserScan last_scan_message;

int MAX_SPEED = 1605;
int MIN_SPEED = 1565;
double EXP_L = 2;
double EXP_G = 2;
double EXP_P = 2;
double EXP_C = 2;
float SLOPE, Y_INTERSECT;

int adaptiveVelocityController::cmd_vel_converter(){
    float max_vel, min_vel;
    int counter = 0;
    double speed;
    double speed_diff = MAX_SPEED-MIN_SPEED;
    if(ignore_local_planner_vel_cmd == false){
        while(ros::param::get("/move_base_node/TrajectoryPlannerROS/max_vel_x", max_vel) == false ||
              ros::param::get("/move_base_node/TrajectoryPlannerROS/min_vel_x", min_vel)==false){
            ros::Duration d = ros::Duration(1.0);
            ROS_WARN("Waiting for the move_base_node/TrajectoryPlannerROS to become active");
            d.sleep();
            counter++;
            //After waiting ten seconds, shut down node
            if(counter==20){
                ROS_FATAL("Move base node not active. Adaptive velocity control node will shut down");
                return 1;
            }
            /*if(auto_control->control_Mode.data = 0){
            return 2;
        }*/
        }
        if(max_vel != min_vel){
            SLOPE = (MAX_SPEED - MIN_SPEED)/(max_vel-min_vel);
            Y_INTERSECT = MAX_SPEED - max_vel*SLOPE;
            speed = SLOPE*cmd_vel + Y_INTERSECT;
            speed_diff = speed-MIN_SPEED;
        }
        else{
            ROS_FATAL("Minimum and maximum velocities of local planner are equal. Node will shot down");
            return 1;
        }
    }

    speed = MIN_SPEED + speed_diff*pow(local_plan_curvature, EXP_L)*pow(global_plan_curvature, EXP_G)*pow(min_obstacle_distance/MAX_LOOK_AHEAD_DIST, EXP_C);
    speed = speed+0.5;
    int ret = (int)speed;
    ROS_INFO_STREAM("Clear Path= " << min_obstacle_distance << " Vel= " << ret << " G= " << global_plan_curvature << " L= " << local_plan_curvature);
    return ret;
}

adaptiveVelocityController::adaptiveVelocityController()
{
    move_base_communication_error = false;
    ignore_local_planner_vel_cmd = false;

    global_plan_granularity = 0.025;
    local_plan_granularity = 0.075;

    //Ignore the curvatures and min_obstacle_dist at the beginning
    local_plan_curvature = 1;
    global_plan_curvature = 1;
    min_obstacle_distance = MAX_LOOK_AHEAD_DIST;

    //TODO: At the end, the following lines should be active
    /*if(cmd_vel_converter() == 1){
        move_base_communication_error = true;
    }*/

    avc_pub = nh_.advertise<std_msgs::Int16>("avc_vel", 1);

    //Global Plan subscriber Mustafa
    global_path_sub = nh_.subscribe<nav_msgs::Path>("/move_base_node/TrajectoryPlannerROS/global_plan",1,&adaptiveVelocityController::globalPlanCallback,this);

    //Local Plan subscriber Mustafa
    local_path_sub = nh_.subscribe<nav_msgs::Path>("/move_base_node/TrajectoryPlannerROS/local_plan",1,&adaptiveVelocityController::localPlanCallback,this);

    cmd_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &adaptiveVelocityController::cmdCallback,this);

    //Initialize nearest point index to -1 for checking
    nearestPointIndex = -1;

    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &adaptiveVelocityController::scanCallback,this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &adaptiveVelocityController::poseCallback,this);
    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&adaptiveVelocityController::wiiCommunicationCallback,this);
}

// a flag method that tells us if we are controlling the car manually or automatically
void adaptiveVelocityController::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    //control_Brake.data = msg->data[1];
}

//Local Plan Callback Mustafa
void adaptiveVelocityController::localPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    double LOCAL_PLAN_LENGTH = local_plan_granularity * msg->poses.size();
    if(LOCAL_PLAN_LENGTH == 0){
        ROS_ERROR("Local plan is empty, local plan curvature is ignored!");
        local_plan_curvature = 1;
        return;
    }
    /*local_x.clear();
    local_y.clear();

    for(unsigned int i=0; i<msg->poses.size(); i++)
    {
        local_x.push_back(msg->poses[i].pose.position.x);
        local_y.push_back(msg->poses[i].pose.position.y);
    }

    float x_diff = local_x[local_x.size()-1]-local_x[0];
    float y_diff = local_y[local_y.size()-1]-local_y[0];*/

    double x_diff = msg->poses[msg->poses.size()-1].pose.position.x - msg->poses[0].pose.position.x;
    double y_diff = msg->poses[msg->poses.size()-1].pose.position.y - msg->poses[0].pose.position.y;

    local_plan_curvature = sqrt(x_diff*x_diff + y_diff*y_diff)/LOCAL_PLAN_LENGTH;
    /*if(local_plan_curvature == 0 || local_plan_curvature>1){
        for(int i=0; i<msg->poses.size(); i++){
            std::cout << "X1: " << msg->poses[0].pose.position.x << " X2: " << msg->poses[msg->poses.size()-1].pose.position.x << " Y1: " << msg->poses[0].pose.position.y << " Y2: " << msg->poses[msg->poses.size()-1].pose.position.y << std::endl;
        }
    }*/
    if(local_plan_curvature == 0){
        ROS_WARN("Local plan curvature is 0 and ignored");
        local_plan_curvature = 1;
    }
}

//Global Plan Callback
void adaptiveVelocityController::globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    /*global_x.clear();
    global_y.clear();

    for (unsigned int i=0; i<msg->poses.size(); i++)
    {
        global_x.push_back(msg->poses[i].pose.position.x);
        global_y.push_back(msg->poses[i].pose.position.y);
    }*/
    double min_dist = 10^6;
    unsigned int min_dist_index = 0;
    double dist, x_diff, y_diff;
    for(unsigned int i=0; i<msg->poses.size(); i++){
        x_diff = pos_x - msg->poses[i].pose.position.x;
        y_diff = pos_y - msg->poses[i].pose.position.y;
        dist = sqrt(x_diff*x_diff + y_diff*y_diff);
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

    //Measuring curvature for the way in front
    int CURV_SECT_SIZE_ = MAX_LOOK_AHEAD_DIST / global_plan_granularity;
    float CURV_SECT_LENGTH_ = MAX_LOOK_AHEAD_DIST;
    if(nearestPointIndex+CURV_SECT_SIZE_ >= msg->poses.size()){
        CURV_SECT_SIZE_ = msg->poses.size() - nearestPointIndex - 1;
        CURV_SECT_LENGTH_ = CURV_SECT_SIZE_ * global_plan_granularity;
        ROS_WARN("Curvature calculation section goes beyond the goal, curvature is calculated until the end of global plan");
    }
    x_diff = msg->poses[nearestPointIndex+CURV_SECT_SIZE_].pose.position.x - msg->poses[nearestPointIndex].pose.position.x;
    y_diff = msg->poses[nearestPointIndex+CURV_SECT_SIZE_].pose.position.y - msg->poses[nearestPointIndex].pose.position.y;
    if(x_diff==0 && y_diff==0){
        ROS_ERROR("Global path makes a self loop in this section, global plan curvature is ignored");
        global_plan_curvature = 1;
    }
    else{
        global_plan_curvature = sqrt(x_diff*x_diff + y_diff*y_diff)/CURV_SECT_LENGTH_;
        //Near the end of global plan, CURV_SECT_SIZE decreases and due to precision, curvature can be slightly
        //greater than one. In order to avoid this, set it to 1 maximally.
        if(global_plan_curvature > 1){
            global_plan_curvature = 1;
        }
    }
}


void adaptiveVelocityController::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel = msg->linear.x;
}

//Pose callback with calculation of the nearest global path point Mustafa
void adaptiveVelocityController::poseCallback(const geometry_msgs::PoseWithCovarianceStamped p){
    pos_x = p.pose.pose.position.x;
    pos_y = p.pose.pose.position.y;
}

/*//Curvature measure function Mustafa
float adaptiveVelocityController::curvatureMeasure(){
    int CURV_SECT_SIZE_ = 100;
    float CURV_SECT_LENGTH_ = 2.5;
    if(nearestPointIndex == -1)
        return -1;
    if(nearestPointIndex+CURV_SECT_SIZE_ >= global_x.size()){
        //        nearestPointIndex = global_x.size()-CURV_SECT_SIZE_-1;
        ROS_ERROR("Curvature calculation section goes beyond the goal, no curvature calculated");
        return -1;
    }
    float x_diff = global_x.at(nearestPointIndex) - global_x.at(nearestPointIndex+CURV_SECT_SIZE_);
    float y_diff = global_y.at(nearestPointIndex) - global_y.at(nearestPointIndex+CURV_SECT_SIZE_);
    if(x_diff==0 && y_diff==0){
        ROS_ERROR("Global path makes a self loop in this section, no curvature calculated");
        return -1;
    }
    else{
        return sqrt(x_diff*x_diff + y_diff*y_diff)/CURV_SECT_LENGTH_;
    }
}*/

void adaptiveVelocityController::scanCallback(const sensor_msgs::LaserScan msg)
{
    //TODO: ranges variable may be deleted if not needed by clearpathdistance function
    for (unsigned int i=0; i<720;i++)
    {
        ranges[i] = msg.ranges[i];
        // std::cout << "self_time_stamp: " << endl;
    }
    min_obstacle_distance = MAX_LOOK_AHEAD_DIST;
    for(int i=0; i<min_dist_lookup_table.size(); i++){
        if(msg.ranges[MIN_AREA_INDEX+i] < min_dist_lookup_table.at(i)*(1.0-DISTANCE_TOLERANCE) &&
                msg.ranges[MIN_AREA_INDEX+i] < min_obstacle_distance){
            min_obstacle_distance = msg.ranges[MIN_AREA_INDEX+i]*sin(index_to_angle(MIN_AREA_INDEX+i));
        }
    }
}

int adaptiveVelocityController::angle_to_index(double angle){
    return (angle-MIN_ANGLE)/ANGLE_RESOLUTION;
}

double adaptiveVelocityController::index_to_angle(int index){
    return index*ANGLE_RESOLUTION-MIN_ANGLE;
}

double adaptiveVelocityController::calc_min_allowed_distance(int index){
    if(index<=MIN_CORNER_INDEX || index>=MAX_CORNER_INDEX){
        return (CAR_WIDTH/2)/abs(cos(index_to_angle(index)));
    }
    else{
        return MAX_LOOK_AHEAD_DIST/sin(index_to_angle(index));
    }
}

double adaptiveVelocityController::calc_min_allowed_distance(double angle){
    if(angle<=MIN_CORNER_ANGLE || angle>=MAX_CORNER_ANGLE){
        return (CAR_WIDTH/2)/abs(cos(angle));
    }
    else{
        return MAX_LOOK_AHEAD_DIST/sin(angle);
    }
}

double adaptiveVelocityController::clear_path_distance(){
    //Initialize min_obstacle distance
    min_obstacle_distance = MAX_LOOK_AHEAD_DIST;
    for(int i=0; i<min_dist_lookup_table.size(); i++){
        if(ranges[MIN_AREA_INDEX+i] < min_dist_lookup_table.at(i)*(1.0-DISTANCE_TOLERANCE) &&
                ranges[MIN_AREA_INDEX+i] < min_obstacle_distance){
            min_obstacle_distance = ranges[MIN_AREA_INDEX+i]*sin(index_to_angle(MIN_AREA_INDEX+i));
        }
    }
    cout << "Path clear up to " << min_obstacle_distance << " meters" << endl;
    return min_obstacle_distance;
}

void adaptiveVelocityController::computeLookupTable(){
    for(int i=MIN_AREA_INDEX; i<=MAX_AREA_INDEX; i++){
        min_dist_lookup_table.push_back(calc_min_allowed_distance(i));
    }

    for(int i=MIN_AREA_INDEX; i<=MAX_AREA_INDEX; i++){
        min_dist_lookup_table.push_back(calc_min_allowed_distance(i));
    }
}

void callback(adaptive_velocity_controller::AVCConfig &config, uint32_t level) {
    MAX_SPEED = config.max_speed;
    MIN_SPEED = config.min_speed;
    if(MAX_SPEED<=MIN_SPEED){
        MAX_SPEED = MIN_SPEED + 30;
        ROS_ERROR("Maximum speed is set less than or equal to minimum speed, it will be set to 'minimum speed + 30'");
        ROS_WARN("Maximum speed = %d", MAX_SPEED);
    }
    MAX_LOOK_AHEAD_DIST = config.max_look_ahead_dist;
    MIN_LOOK_AHEAD_DIST = config.min_look_ahead_dist;
    if(MAX_LOOK_AHEAD_DIST<=MIN_LOOK_AHEAD_DIST){
        MAX_LOOK_AHEAD_DIST = MIN_LOOK_AHEAD_DIST + 1;
        ROS_ERROR("Maximum lookahead distance is set less than or equal to minimum lookahead distance, it will be set to 'minimum lookahead distance + 1'");
        ROS_WARN("Maximum lookahead distance = %lf", MAX_LOOK_AHEAD_DIST);
    }
    DISTANCE_TOLERANCE = config.distance_tolerance;
    CURV_SECT_LENGTH = config.curvature_section_length;
    CURV_SECT_OVERLAP = config.curvature_section_overlap;
    if(CURV_SECT_LENGTH<=CURV_SECT_OVERLAP){
        CURV_SECT_LENGTH = CURV_SECT_OVERLAP + 0.5;
        ROS_ERROR("Curvature section length is set less than or equal to curvature section overlap, it will be set to 'curvature section overlap + 0.5'");
        ROS_WARN("Curvature section length = %lf", CURV_SECT_LENGTH);
    }
    ignore_local_planner_vel_cmd = config.ignore_local_planner_vel_cmd;
    EXP_L = config.exp_l;
    EXP_G = config.exp_g;
    EXP_P = config.exp_p;
    EXP_C = config.exp_c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_velocity_controller");

    dynamic_reconfigure::Server<adaptive_velocity_controller::AVCConfig> server;
    dynamic_reconfigure::Server<adaptive_velocity_controller::AVCConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh("~");
    nh.param("max_speed", MAX_SPEED, 1605);
    nh.param("min_speed", MIN_SPEED, 1565);
    nh.param("exp_l", EXP_L, 2.0);
    nh.param("exp_g", EXP_G, 2.0);
    nh.param("exp_p", EXP_P, 2.0);
    nh.param("exp_c", EXP_C, 2.0);
    nh.param("min_look_ahead_dist", MIN_LOOK_AHEAD_DIST, 0.75);
    nh.param("max_look_ahead_dist", MAX_LOOK_AHEAD_DIST, 3.5);
    nh.param("distance_tolerance", DISTANCE_TOLERANCE, 0.015);
    nh.param("curvature_section_length", CURV_SECT_LENGTH, 2.5);
    nh.param("curvature_section_overlap", CURV_SECT_OVERLAP, 0.25);

    //TODO: Min angle, max angle o anki laser konfigürasyonundan okunacak
    CAR_WIDTH = 0.45;
    ANGLE_RESOLUTION = 0.25*M_PI/180;
    MAX_ANGLE = M_PI-ANGLE_RESOLUTION;
    MIN_ANGLE = 0;

    std::string key;
    double sim_granularity;
    if (nh.searchParam("sim_granularity", key))
    {
        nh.getParam(key, sim_granularity);
    }
    else{
        sim_granularity = 0.25;
        ROS_WARN("Simulation granularity not found, set to 0.25 by default.");
    }

    adaptiveVelocityController vC;
    ROS_ASSERT(vC.move_base_communication_error==false);

    //Calculations
    vC.CURV_SECT_SIZE = ceil(CURV_SECT_LENGTH/sim_granularity);
    vC.CURV_SECT_OVERLAP_SIZE = ceil(CURV_SECT_OVERLAP/sim_granularity);

    vC.MIN_CORNER_INDEX = floor(vC.angle_to_index(atan(MAX_LOOK_AHEAD_DIST/(CAR_WIDTH/2))));
    vC.MAX_CORNER_INDEX = ceil(vC.angle_to_index(atan(MAX_LOOK_AHEAD_DIST/(-1*CAR_WIDTH/2))+M_PI));

    vC.MIN_CORNER_ANGLE = vC.index_to_angle(vC.MIN_CORNER_INDEX);
    vC.MAX_CORNER_ANGLE = vC.index_to_angle(vC.MAX_CORNER_INDEX);

    vC.MIN_AREA_ANGLE = atan(MIN_LOOK_AHEAD_DIST/(CAR_WIDTH/2));
    vC.MAX_AREA_ANGLE = M_PI-vC.MIN_AREA_ANGLE;

    vC.MIN_AREA_INDEX = floor(vC.angle_to_index(vC.MIN_AREA_ANGLE));
    vC.MAX_AREA_INDEX = ceil(vC.angle_to_index(vC.MAX_AREA_ANGLE));

    vC.computeLookupTable();

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        vC.avc_vel_msg.data = vC.cmd_vel_converter();
        vC.avc_pub.publish(vC.avc_vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
