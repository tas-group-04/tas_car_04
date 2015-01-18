#include "control/control.h"

#define POS_TOLERANCE 0.04

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parallel_parking");
    control parking_control;
    int sleepCntr = 0;
    int referenceFound = 0;
    ros::Rate loop_rate(50);

    bool in_start_position = false;
    bool in_turn_position = false;
    bool in_goal_position = false;
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

                    parking_control.control_servo.y = parking_control.cmd_steeringAngle;
                    //std::cout << "return value is: "<< parking_control.detectJumps() << std::endl;
                    if(parking_control.detectJumps()){

                        if(parking_control.calculateWaypoints())
                            parking_control.parkDetected = 1;
                        else
                            std::cout << "WAYPOINTS RETURN 0" << std::endl;

                        //                        ros::Duration d = ros::Duration(1.15);
                        //                        ROS_WARN("Half a second continue");
                        //                        d.sleep();
                    }
                }else if(parking_control.parkDetected && !in_start_position && !in_turn_position && ! in_goal_position){
                    //if(parking_control.eucl_distance>0 && parking_control.eucl_distance < 0.6){
                    if(parking_control.pos_y > parking_control.y_start + POS_TOLERANCE)
                        parking_control.control_servo.x = 1545; //for slow movement
                    parking_control.control_servo.y = 1500; //no steering
                    std::cout << "driving to start position forward" << std::endl;
                }else if(parking_control.pos_y <= parking_control.y_start + POS_TOLERANCE && parking_control.pos_y >=  parking_control.y_start - POS_TOLERANCE){
                    parking_control.control_servo.x=1500;
                    parking_control.control_servo.y=1500;
                    in_start_position = true;

                }else if(parking_control.pos_y < parking_control.y_start - POS_TOLERANCE){
                    parking_control.control_servo.x=1408 - (int)(parking_control.eucl_distance/0.1);
                    parking_control.control_servo.y=1500;
                    std::cout << "driving to start position backward" << std::endl;
                }
            }
            else if(in_start_position && parking_control.parkDetected && !in_turn_position && !in_goal_position){
                if(parking_control.pos_y < parking_control.y_turn - POS_TOLERANCE){
                    parking_control.control_servo.x = 1397;
                    parking_control.control_servo.y = 2000;
                    std::cout << "driving to turn position backward" << std::endl;
                }
                else if(parking_control.pos_y >= parking_control.y_turn - POS_TOLERANCE && parking_control.pos_y <= parking_control.y_turn + POS_TOLERANCE){
                    parking_control.control_servo.x = 1500;
                    parking_control.control_servo.y = 1500;
                    in_turn_position = true;
                }else{
                    std::cout << "ERROR, MISSED TURN POSITION" << std::endl;
                }
            } else if(in_start_position && parking_control.parkDetected && in_turn_position && !in_goal_position){
                if(parking_control.pos_y < parking_control.y_goal - POS_TOLERANCE){
                    parking_control.control_servo.x = 1397;
                    parking_control.control_servo.y = 1000;
                    std::cout << "driving to goal position backward" << std::endl;
                }
                else if(parking_control.pos_y >= parking_control.y_goal - POS_TOLERANCE && parking_control.pos_y <= parking_control.y_goal + POS_TOLERANCE){
                    parking_control.control_servo.x = 1500;
                    parking_control.control_servo.y = 1500;
                    in_goal_position = true;
                }else{
                    std::cout << "ERROR, MISSED GOAL POSITION" << std::endl;
                }
            }else if(in_start_position && parking_control.parkDetected && in_turn_position && in_goal_position){
                std::cout << "reached goal position" << std::endl;
            }else{
                std::cout << "shouldn't be here! " << std::endl;
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
