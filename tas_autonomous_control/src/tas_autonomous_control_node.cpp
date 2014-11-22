#include "control/control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);

    //dummy number for old_control_mode at the beginning & control mode change flag
    int16_t old_control_mode = 15;
    bool control_mode_changed = false;

    while(ros::ok())
    {
        if(old_control_mode != autonomous_control.control_Mode.data)
        {
            control_mode_changed = true;
        }
        if(autonomous_control.control_Mode.data==0 && control_mode_changed)
        {
            ROS_INFO("\033[39mChanged to Manual Control Mode!\033[39m");
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
                if(control_mode_changed)
                {
                    ROS_INFO("\033[38;5;148mChanged to Automatic Control Mode!\033[39m");
                }
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1550;
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
        old_control_mode = autonomous_control.control_Mode.data;

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
