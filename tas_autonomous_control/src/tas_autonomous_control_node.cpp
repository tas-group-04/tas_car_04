#include "control/control.h"
#include <cmath>

int MAX_SPEED = 1590;
int MIN_SPEED = 1565;
int SPEED_DEGRADE = 2;
float SLOPE, Y_INTERSECT;

int cmd_vel_converter(control *auto_control){
    float max_vel;
    while(ros::param::get("/move_base_node/TrajectoryPlannerROS/max_vel_x", max_vel) == false){
        ros::Duration d = ros::Duration(0.5);
        ROS_INFO("Waiting for the move_base_node/TrajectoryPlannerROS to become active");
        d.sleep();
        if(auto_control->control_Mode.data = 0){
            return 2;
        }
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::NodeHandle nh("~");
    nh.param("max_speed", MAX_SPEED, 1590);
    nh.param("min_speed", MIN_SPEED, 1565);
    nh.param("speed_degradation_order", SPEED_DEGRADE, 2);

    ros::Rate loop_rate(50);

    float diff;
    float speed;
    bool cmd_vel_converted = false;
    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }
        else
        {
            if(cmd_vel_converted == false){
                int conversion_result = cmd_vel_converter(&autonomous_control);
                if(conversion_result == 0){
                    cmd_vel_converted = true;
                }
                else if(conversion_result == 2){
                    /*ros::spinOnce();
                    loop_rate.sleep();*/
                    continue;
                }
            }

            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                ROS_INFO("Automatic Control!");
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    diff = abs(autonomous_control.cmd_steeringAngle - 1500);
                    speed = SLOPE*autonomous_control.cmd_linearVelocity + Y_INTERSECT;
                    if(diff != 0 && speed != MIN_SPEED){
                        autonomous_control.control_servo.x = speed-(speed-MIN_SPEED)*pow((diff/500),SPEED_DEGRADE);
                        //autonomous_control.control_servo.x = MAX_SPEED-(MAX_SPEED-MIN_SPEED)*pow((diff/500),SPEED_DEGRADE);
                    }
                    else{
                        autonomous_control.control_servo.x = speed;
                        //autonomous_control.control_servo.x = MAX_SPEED;
                    }
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
