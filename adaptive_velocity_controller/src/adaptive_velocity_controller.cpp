#include "adaptive_velocity_controller.h"
#include <iostream>
#include <cmath>    //Include for sqrt Mustafa

#include <dynamic_reconfigure/server.h>
#include <adaptive_velocity_controller/AVCConfig.h>

#include "std_msgs/Int16.h"

using namespace std;

//Parameters
double MAX_ANGLE, MIN_ANGLE, ANGLE_RESOLUTION, CAR_WIDTH, MAX_LOOK_AHEAD_DIST, MIN_LOOK_AHEAD_DIST, DISTANCE_TOLERANCE;
bool ignore_local_planner_vel_cmd;
int MAX_SPEED = 1605;
int MIN_SPEED = 1565;
double EXP_L = 2;
double EXP_G = 2;
double EXP_C = 2;
float SLOPE, Y_INTERSECT;

/*Parameter definitions:
 * MAX_ANGLE: Maximum scan angle in the virtual lane for checking clear path distance
 * MIN_ANGLE: Minimum scan angle in the virtual lane for checking clear path distance
 * ANGLE_RESOLUTION: Angular resolution of Hokuyo laser
 * MAX_LOOK_AHEAD_DIST: Clear path calculations are done up to this much of distance in the virtual lane
 * MIN_LOOK_AHEAD_DIST: Clear path calculations are done from this much of distance on in the virtual lane
 * DISTANCE_TOLERANCE: The tolerance of the scan ranges in the virtual lane. Measurements within this tolerance
 *                      are considered as no obstacle in the virtual lane.
 * ignore_local_planner_vel_cmd: Boolean variable for ignoring/considering the cmd_vel messages from move_base
 * MAX_SPEED: Maximum servo velocity
 * MIN_SPEED: Minimum servo velocity
 * EXP_L: Local plan curvature weighting exponential in velocity calculation. See cmd_vel_converter() for reference
 * EXP_G: Global plan curvature weighting exponential in velocity calculation. See cmd_vel_converter() for reference
 * EXP_C: Clear path ratio plan weighting exponential in velocity calculation. See cmd_vel_converter() for reference
 * */



//Calculations
/*double MIN_CORNER_ANGLE, MAX_CORNER_ANGLE;
int MIN_CORNER_INDEX, MAX_CORNER_INDEX, MIN_AREA_INDEX, MAX_AREA_INDEX, CURV_SECT_SIZE, CURV_SECT_OVERLAP_SIZE;
double MIN_AREA_ANGLE, MAX_AREA_ANGLE;
vector<double> min_dist_lookup_table;*/



int adaptiveVelocityController::cmd_vel_converter(){
    /*Function which maps the cmd_vel message from move_base to servo command range.
     * The range of servo command is between MIN_SPEED and MAX_SPEED parameters.
     * The function can ignore the velocity commands from move base. Depending on the
     * local plan curvature, global plan curvature and minimum obstacle distance
     * the velocity is adaptively changed. The more linear the global and local plans are and
     * the more is the distance to the nearest obstacle (minimum_obstacle_distance), the higher
     * becomes the servo velocity command. The weighting factor of each component (local plan curvature,
     * global plan curvature and clear path ratio to MAX_LOOK_AHEAD_DIST) is calculated by taking the EXP_L, EXP_G and EXP_C
     * power of each factor. Each factor is a number between zero and one.*/

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
        }
        if(max_vel != min_vel){
            /* Map the min_vel_x and max_vel_x range of move_base to the MIN_SPEED and MAX_SPEED range
            of servo commands*/
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
    local_plan_granularity = 0.025;

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

//Local Plan Callback
void adaptiveVelocityController::localPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    //Calculate the local plan curvature in this callback
    double LOCAL_PLAN_LENGTH = local_plan_granularity * msg->poses.size();
    if(LOCAL_PLAN_LENGTH == 0){
        ROS_ERROR("Local plan is empty, local plan curvature is ignored!");
        local_plan_curvature = 1;
        return;
    }

    double x_diff = msg->poses[msg->poses.size()-1].pose.position.x - msg->poses[0].pose.position.x;
    double y_diff = msg->poses[msg->poses.size()-1].pose.position.y - msg->poses[0].pose.position.y;

    local_plan_curvature = sqrt(x_diff*x_diff + y_diff*y_diff)/LOCAL_PLAN_LENGTH;

    if(local_plan_curvature == 0){
        ROS_WARN("Local plan curvature is 0 and ignored");
        local_plan_curvature = 1;
    }
}

//Global Plan Callback
void adaptiveVelocityController::globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    double min_dist = 10^6;
    unsigned int min_dist_index = 0;
    double dist, x_diff, y_diff;

    //Calculate the index of the point in global_plan to which the car is closest at that time instance
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

    //Measure the curvature for the way in front. It is calculated for the path in front with length equal to MAX_LOOK_AHEAD_DIST
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
    //Callback for getting the velocity commands from move_base
    cmd_vel = msg->linear.x;
}

void adaptiveVelocityController::poseCallback(const geometry_msgs::PoseWithCovarianceStamped p){
    //Pose callback
    pos_x = p.pose.pose.position.x;
    pos_y = p.pose.pose.position.y;
}

void adaptiveVelocityController::scanCallback(const sensor_msgs::LaserScan msg)
{
    /* Callback for laser scan. In this callback, the laser scans are stored in a vector
     * for computational purposes and the minimum obstacle distance is calculated, which
     * corresponds to the distance of the nearest obstacle.*/
    //TODO: ranges variable may be deleted if not needed by clearpathdistance function
    for (unsigned int i=0; i<720;i++)
    {
        ranges[i] = msg.ranges[i];
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
    //Convert laser measurement angle to the corresponding array index
    return (angle-MIN_ANGLE)/ANGLE_RESOLUTION;
}

double adaptiveVelocityController::index_to_angle(int index){
    //Convert laser measurement array index to the corresponding angle
    return index*ANGLE_RESOLUTION-MIN_ANGLE;
}

double adaptiveVelocityController::calc_min_allowed_distance(int index){
    /*Calculate the minimum allowed distance using the index of the laser measurement array
     * see calc_min_allowed_distance(double angle) function for reference*/
    if(index<=MIN_CORNER_INDEX || index>=MAX_CORNER_INDEX){
        return (CAR_WIDTH/2)/abs(cos(index_to_angle(index)));
    }
    else{
        return MAX_LOOK_AHEAD_DIST/sin(index_to_angle(index));
    }
}

double adaptiveVelocityController::calc_min_allowed_distance(double angle){
    /*Calculate the distance which signs that at that angle there is no obstacle.
     * If the laser measurement at the corresponding angle is less than the minimum allowed distance
     * this information is then used to sense there is an object in the virtual lane*/
    if(angle<=MIN_CORNER_ANGLE || angle>=MAX_CORNER_ANGLE){
        return (CAR_WIDTH/2)/abs(cos(angle));
    }
    else{
        return MAX_LOOK_AHEAD_DIST/sin(angle);
    }
}

double adaptiveVelocityController::clear_path_distance(){
    //Function for computing the clear path distance in front of the car.
    //This function is unused and is here for reference and debugging purposes

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
    //Compute the lookup table to compare with the real time laser data in order to compute
    //the clear path distance in virtual lane in real time
    for(int i=MIN_AREA_INDEX; i<=MAX_AREA_INDEX; i++){
        min_dist_lookup_table.push_back(calc_min_allowed_distance(i));
    }

    for(int i=MIN_AREA_INDEX; i<=MAX_AREA_INDEX; i++){
        min_dist_lookup_table.push_back(calc_min_allowed_distance(i));
    }
}

void callback(adaptive_velocity_controller::AVCConfig &config, uint32_t level) {
    //Dynamic reconfigure callback function to set the parameters dynamically during runtime
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
    ignore_local_planner_vel_cmd = config.ignore_local_planner_vel_cmd;
    EXP_L = config.exp_l;
    EXP_G = config.exp_g;
    EXP_C = config.exp_c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_velocity_controller");

    //Setup dynamic reconfigure server
    dynamic_reconfigure::Server<adaptive_velocity_controller::AVCConfig> server;
    dynamic_reconfigure::Server<adaptive_velocity_controller::AVCConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    //Get the command line parameters
    ros::NodeHandle nh("~");
    nh.param("max_speed", MAX_SPEED, 1605);
    nh.param("min_speed", MIN_SPEED, 1565);
    nh.param("exp_l", EXP_L, 2.0);
    nh.param("exp_g", EXP_G, 2.0);
    nh.param("exp_c", EXP_C, 2.0);
    nh.param("min_look_ahead_dist", MIN_LOOK_AHEAD_DIST, 0.75);
    nh.param("max_look_ahead_dist", MAX_LOOK_AHEAD_DIST, 3.5);
    nh.param("distance_tolerance", DISTANCE_TOLERANCE, 0.015);

    CAR_WIDTH = 0.45;
    ANGLE_RESOLUTION = 0.25*M_PI/180;           //Hokuyo angular resolution
    MAX_ANGLE = M_PI-ANGLE_RESOLUTION;          //Hokuyo maximum angle
    MIN_ANGLE = 0;                              //Hokuyo minimum angle

    //Get the sim_granularity parameter
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

    //Create an instance of adaptiveVelocityController class and make sure that it communicates with move_base
    adaptiveVelocityController vC;
    ROS_ASSERT(vC.move_base_communication_error==false);

    //Calculations of adaptive velocity controller parameters to create the virtual lane lookup table
    vC.MIN_CORNER_INDEX = floor(vC.angle_to_index(atan(MAX_LOOK_AHEAD_DIST/(CAR_WIDTH/2))));
    vC.MAX_CORNER_INDEX = ceil(vC.angle_to_index(atan(MAX_LOOK_AHEAD_DIST/(-1*CAR_WIDTH/2))+M_PI));

    vC.MIN_CORNER_ANGLE = vC.index_to_angle(vC.MIN_CORNER_INDEX);
    vC.MAX_CORNER_ANGLE = vC.index_to_angle(vC.MAX_CORNER_INDEX);

    vC.MIN_AREA_ANGLE = atan(MIN_LOOK_AHEAD_DIST/(CAR_WIDTH/2));
    vC.MAX_AREA_ANGLE = M_PI-vC.MIN_AREA_ANGLE;

    vC.MIN_AREA_INDEX = floor(vC.angle_to_index(vC.MIN_AREA_ANGLE));
    vC.MAX_AREA_INDEX = ceil(vC.angle_to_index(vC.MAX_AREA_ANGLE));

    //Compute the lookup table to compare it with laser scans in real time
    vC.computeLookupTable();

    //Setup the node loop rate
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        //Convert the velocity to servo velocity
        vC.avc_vel_msg.data = vC.cmd_vel_converter();
        //Publish the servo velocity to tas_autonomous_control_mode
        vC.avc_pub.publish(vC.avc_vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
