#include "control/control.h"
#include <cmath>

int MAX_SPEED = 1590;
int MIN_SPEED = 1565;
int SPEED_DEGRADE = 2;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
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
                ROS_INFO("Automatic Control!");
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    float diff = abs(autonomous_control.cmd_steeringAngle - 1500);
                    /*float diff;

                    if(autonomous_control.cmd_steeringAngle >=1500){
                        diff = autonomous_control.cmd_steeringAngle - 1500;
                    }
                    else{
                        diff = 1500 - autonomous_control.cmd_steeringAngle;
                    }*/

                    if(diff != 0){
                        autonomous_control.control_servo.x = MAX_SPEED-(MAX_SPEED-MIN_SPEED)*pow((diff/500),SPEED_DEGRADE);
                    }
                    else{
                        autonomous_control.control_servo.x = MAX_SPEED;
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
