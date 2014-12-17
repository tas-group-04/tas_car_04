#include "control/control.h"
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include <string>

using namespace std;

int MAX_SPEED = 1590;
int MIN_SPEED = 1565;
int SPEED_DEGRADE = 2;
float SLOPE, Y_INTERSECT;

int cmd_vel_converter(){
    float max_vel;
    while(ros::param::get("/move_base_node/TrajectoryPlannerROS/max_vel_x", max_vel) == false){
        ros::Duration d = ros::Duration(0.5);
        ROS_INFO("Waiting for the move_base_node/TrajectoryPlannerROS to become active");
        d.sleep();
        /*if(auto_control->control_Mode.data = 0){
            return 2;
        }*/
    }
    float min_vel=ros::param::get("/move_base_node/TrajectoryPlannerROS/min_vel_x", min_vel);
    if(max_vel != min_vel){
        SLOPE = (MAX_SPEED - MIN_SPEED)/(max_vel-min_vel);
        Y_INTERSECT = MAX_SPEED - max_vel*SLOPE;
        return 0;
    }
    else{
        ROS_FATAL("Minimum and maximum velocities of local planner are equal.");
        return 1;
    }
}










//Parameters
double MAX_ANGLE, MIN_ANGLE, ANGLE_RESOLUTION, CAR_WIDTH, MAX_LOOK_AHEAD_DIST;
double MIN_LOOK_AHEAD_DIST, DISTANCE_TOLERANCE, CURV_SECT_LENGTH, CURV_SECT_OVERLAP;

//Calculations
double MIN_CORNER_ANGLE, MAX_CORNER_ANGLE, MIN_CORNER_INDEX, MAX_CORNER_INDEX;
double MIN_AREA_ANGLE, MAX_AREA_ANGLE, MIN_AREA_INDEX, MAX_AREA_INDEX, CURV_SECT_SIZE, CURV_SECT_OVERLAP_SIZE;
vector<double> min_dist_lookup_table;
sensor_msgs::LaserScan last_scan_message;


int angle_to_index(double angle){
    return (angle-MIN_ANGLE)/ANGLE_RESOLUTION;
}

double index_to_angle(int index){
    return index*ANGLE_RESOLUTION-MIN_ANGLE;
}

double calc_min_allowed_distance(int index){
    if(index<=MIN_CORNER_INDEX || index>=MAX_CORNER_INDEX){
        return (CAR_WIDTH/2)/abs(cos(index_to_angle(index)));
    }
    else{
        return MAX_LOOK_AHEAD_DIST/sin(index_to_angle(index));
    }
}

double calc_min_allowed_distance(double angle){
    if(angle<=MIN_CORNER_ANGLE || angle>=MAX_CORNER_ANGLE){
        return (CAR_WIDTH/2)/abs(cos(angle));
    }
    else{
        return MAX_LOOK_AHEAD_DIST/sin(angle);
    }
}

/*double calc_curvature(float path[]){

    return 0;
}*/

double clear_path_distance(){
    //Initialize min_obstacle distance
    double min_obstacle_distance = MAX_LOOK_AHEAD_DIST;
    for(int i=0; i<min_dist_lookup_table.size(); i++){
        if(last_scan_message.ranges[MIN_AREA_INDEX+i] < min_dist_lookup_table.at(i)*(1-DISTANCE_TOLERANCE) &&
                last_scan_message.ranges[MIN_AREA_INDEX+i] < min_obstacle_distance){
            min_obstacle_distance = last_scan_message.ranges[MIN_AREA_INDEX+i]*sin(index_to_angle(MIN_AREA_INDEX+i));
        }
    }
    cout << "Path clear up to " << min_obstacle_distance << " meters" << endl;
    return min_obstacle_distance;
}

void scanCallback(const sensor_msgs::LaserScan scan_msg){
    last_scan_message = scan_msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::NodeHandle nh("~");
    nh.param("max_speed", MAX_SPEED, 1590);
    nh.param("min_speed", MIN_SPEED, 1565);
    nh.param("speed_degradation_order", SPEED_DEGRADE, 2);

    //bool only_manual_mode;
    //nh.param("only_manual_mode", only_manual_mode, false);
    //if(only_manual_mode == false)

    cmd_vel_converter();


    //******************************************************************
    //Begin velocity_controller
    //******************************************************************

    nh.param("min_look_ahead_dist", MIN_LOOK_AHEAD_DIST, 5.0);
    nh.param("max_look_ahead_dist", MAX_LOOK_AHEAD_DIST, 1.0);
    nh.param("distance_tolerance", DISTANCE_TOLERANCE, 0.015);
    nh.param("curvature_section_length", CURV_SECT_LENGTH, 1.0);
    nh.param("curvature_section_overlap", CURV_SECT_OVERLAP, 0.25);

    std::string key;
    double sim_granularity;
    if (nh.searchParam("sim_granularity", key))
    {
        nh.getParam(key, sim_granularity);
    }
    else{
        sim_granularity = 0.25;
        std::cout << "Sim gran not found" << std::endl;
    }
    CURV_SECT_SIZE = ceil(CURV_SECT_LENGTH/sim_granularity);
    CURV_SECT_OVERLAP_SIZE = ceil(CURV_SECT_OVERLAP/sim_granularity);

    CAR_WIDTH = 0.45;
    ANGLE_RESOLUTION = 0.25*M_PI/180;
    MAX_ANGLE = M_PI-ANGLE_RESOLUTION;
    MIN_ANGLE = 0;
    //MAX_LOOK_AHEAD_DIST = 5;
    //MIN_LOOK_AHEAD_DIST = 1;
    //DISTANCE_TOLERANCE = 0.015;

    MIN_CORNER_INDEX = floor(angle_to_index(atan(MAX_LOOK_AHEAD_DIST/(CAR_WIDTH/2))));
    MAX_CORNER_INDEX = ceil(angle_to_index(atan(MAX_LOOK_AHEAD_DIST/(-1*CAR_WIDTH/2))+M_PI));

    MIN_CORNER_ANGLE = index_to_angle(MIN_CORNER_INDEX);
    MAX_CORNER_ANGLE = index_to_angle(MAX_CORNER_INDEX);

    MIN_AREA_ANGLE = atan(MIN_LOOK_AHEAD_DIST/(CAR_WIDTH/2));
    MAX_AREA_ANGLE = M_PI-MIN_AREA_ANGLE;

    MIN_AREA_INDEX = floor(angle_to_index(MIN_AREA_ANGLE));
    MAX_AREA_INDEX = ceil(angle_to_index(MAX_AREA_ANGLE));

    for(int i=MIN_AREA_INDEX; i<=MAX_AREA_INDEX; i++){
        min_dist_lookup_table.push_back(calc_min_allowed_distance(i));
    }

    //Subscribe to scan topic for velocity_controller
    string topic;
    if(nh.getParam("topic", topic) == false){
        topic = "scan";
    }
    ros::Subscriber sub = nh.subscribe(topic.c_str(), 1, scanCallback);

    for(int i=MIN_AREA_INDEX; i<=MAX_AREA_INDEX; i++){
        min_dist_lookup_table.push_back(calc_min_allowed_distance(i));
    }

    //******************************************************************
    //End velocity_controller
    //******************************************************************



    ros::Rate loop_rate(50);

    float diff;
    float speed;
    //bool cmd_vel_converted = false;
    int last_control_mode = 2;  //Dummy variable for mode outputting
    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            if(last_control_mode != 0)
                ROS_INFO("Manual Control Mode!");
            last_control_mode = 0;
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                if(last_control_mode != 0)
                    ROS_INFO("Automatic Control Mode!");
                last_control_mode = 1;
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    //BURADA DİREKSİYON AÇISINA GÖRE KONTROL EDİP EĞER GLOBAL PATH'TEN
                    //UZAKLAŞIYORSAK MAX SPEED'İ PUBLISH ETMEMEK LAZIM
                    if(clear_path_distance() == MAX_LOOK_AHEAD_DIST){
                        speed = MAX_SPEED;
                    }
                    else{
                        diff = abs(autonomous_control.cmd_steeringAngle - 1500);
                        speed = SLOPE*autonomous_control.cmd_linearVelocity + Y_INTERSECT;
                        if(diff != 0 && speed != MIN_SPEED){
                            speed -= (speed-MIN_SPEED)*pow((diff/500),SPEED_DEGRADE);
                            //autonomous_control.control_servo.x = MAX_SPEED-(MAX_SPEED-MIN_SPEED)*pow((diff/500),SPEED_DEGRADE);
                        }
                        else{
                            //autonomous_control.control_servo.x = speed;
                            //autonomous_control.control_servo.x = MAX_SPEED;
                        }
                    }
                    autonomous_control.control_servo.x = speed;
                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    autonomous_control.control_servo.x = 1300;
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
