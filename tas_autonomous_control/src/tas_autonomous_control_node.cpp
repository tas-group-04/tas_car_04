#include "control/control.h"
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include <string>
using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;
    ros::Rate loop_rate(50);
    int last_control_mode = 2; //Variable for mode outputting only at mode changes
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
                if(last_control_mode != 1)
                    ROS_INFO("Automatic Control Mode!");
                last_control_mode = 1;
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    if(autonomous_control.avc_vel != 1){
                        //Set the servo velocity equal to that calculated by adaptive_velocity_controller
                        autonomous_control.control_servo.x = autonomous_control.avc_vel;
                    }
                    else{
                        ROS_ERROR("Default velocity of 1550 is sent to the servo.");
                        autonomous_control.control_servo.x = 1550;
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
