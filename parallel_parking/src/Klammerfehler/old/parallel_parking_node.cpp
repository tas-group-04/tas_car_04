#include "control/control.h"

//********************PARKING PARAMETERS ****************************/
// initializing___________________________________
float dist_b_1j = 0.0;
float dist_a_1j = 0.0;
float dist_b_2j = 0.0;
float dist_a_2j = 0.0;
float x_ll= 0.0;
float y_ll= 0.0;
float x_ul= 0.0;
float y_ul= 0.0;
float x_ur= 0.0;
float y_ur= 0.0;
float x_lr= 0.0;
float y_lr= 0.0;
float x_goal= 0.0;
float y_goal= 0.0;
float x_start= 0.0;
float y_start= 0.0;
float x_g_c= 0.0;
float y_g_c= 0.0;
float x_s_c= 0.0;
float y_s_c= 0.0;
float x_turn= 0.0;
float y_turn= 0.0;

// car position when 1st and 2nd jump were detected
float x1j = 0.0;
float y1j = 0.0;
float x2j = 0.0;
float y2j = 0.0;




int main(int argc, char** argv)
{
    ros::init(argc, argv, "parallel_parking");
    control parking_control;
    int sleepCntr = 0;
    int referenceFound = 0;
    ros::Rate loop_rate(50);

    bool in_position = false;

    while(ros::ok())
    {

        if(parking_control.control_Mode.data==0)
        {
            //ROS_INFO("Manually Control!");
            parking_control.control_servo.x=1500;
            parking_control.control_servo.y=1500;

        }
        else
        {
            if(parking_control.control_Brake.data==1)
            {
                parking_control.control_servo.x=1500;
                parking_control.control_servo.y=1500;
            }
            else
            {
                if(!parking_control.parkDetected){
                    //    std::cout << "WII PRESSED" << std::endl;
                    parking_control.control_servo.x = 1545; //for slow movement

                    parking_control.control_servo.y = 1500; //no steering
                    //std::cout << "return value is: "<< parking_control.detectJumps() << std::endl;
                    if(parking_control.detectJumps()){
                        parking_control.parkDetected = 1;

                        //                        ros::Duration d = ros::Duration(1.15);
                        //                        ROS_WARN("Half a second continue");
                        //                        d.sleep();

                        std::cout << "HERE" << std::endl;
                    }
                }else if(!in_position){
                    if(parking_control.eucl_distance>0 && parking_control.eucl_distance < 0.6){
                        parking_control.control_servo.x = 1545; //for slow movement
                        parking_control.control_servo.y = 1500; //no steering
                    }else if(parking_control.eucl_distance <= 0.6 + 0.04 && parking_control.eucl_distance >= 0.6-0.04){
                        parking_control.control_servo.x=1500;
                        parking_control.control_servo.y=1500;
                        in_position = true;

                    }else if(parking_control.eucl_distance > 0.6 + 0.04){
                        parking_control.control_servo.x=1408 - (int)(parking_control.eucl_distance/0.1);
                        parking_control.control_servo.y=1500;
                    }
                }
                else if(in_position && parking_control.parkDetected){
                    if(parking_control.eucl_distance > 0.3 + 0.04){
                        parking_control.control_servo.x = 1397;
                        parking_control.control_servo.y = 2000;
                    }
                    else if(parking_control.eucl_distance > -0.2 - 0.04 && parking_control.eucl_distance <= 0.3 + 0.04){
                        parking_control.control_servo.x = 1397;
                        parking_control.control_servo.y = 1000;
                    }else if(parking_control.eucl_distance <= -0.2 - 0.04 && parking_control.eucl_distance >= -0.3-0.04){
                        parking_control.control_servo.x=1407;
                        parking_control.control_servo.y=1500;
                    }
                    else{
                        parking_control.control_servo.x=1500;
                        parking_control.control_servo.y=1500;
                        ROS_INFO("Stop");
                    }

                    //                    if(sleepCntr >= 50 || referenceFound){
                    //                        parking_control.control_servo.x=1408 - (int)(parking_control.eucl_distance/0.1);
                    //                        parking_control.control_servo.y=1500;
                    //                        if(parking_control.findReferencePosition()){
                    //                            std::cout << "Found Reference Position" << std::endl;
                    //                            // velocityStop(parking_control);
                    //                            referenceFound = 1;
                    //                            parking_control.control_servo.x=1500;
                    //                            parking_control.control_servo.y=1500; // stop
                    //                        }
                    //                    }
                    //                    else{
                    //                        sleepCntr++;
                    //                        parking_control.control_servo.x=1500;
                    //                        parking_control.control_servo.y=1500; // stop

                    //                    }
                    //                    // PARKING DETECTION NOT WORKING RIGHT NOW!!! ON FUTURAMA
                    //                    // parking_control.printStuff();
                    //                    //moveBackward(&parking_control);




                }

            }

            parking_control.control_servo_pub_.publish(parking_control.control_servo);

        }



        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}
