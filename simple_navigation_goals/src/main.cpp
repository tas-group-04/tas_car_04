/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <fstream>
#include <string>
#include <qstring.h>
#include <qstringlist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb(){
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    geometry_msgs::Pose waypoint1;
    /*waypoint1.position.x = 22.0;
    waypoint1.position.y = 10.75;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = 0;
    waypoint1.orientation.w = 1;
    waypoints.push_back(waypoint1);*/


    ifstream file;
    file.open("../../goals.txt");

    string s_line;
    QString line;

    int num=0;

    bool* ok;

    while(!file.eof()){
        getline(file,s_line);
        line = s_line.c_str();
        if(line.contains("position")){
            num++;
            getline(file,s_line);
            line = s_line.c_str();
            QStringList value;
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.position.x = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.position.y = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.position.z = value.at(1).toDouble(ok);
            getline(file,s_line);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.x = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.y = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.z = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.w = value.at(1).toDouble(ok);
        }
        waypoints.push_back(waypoint1);
    }

    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal: x=%f, y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal: x=%f, y=%f", i, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        } else {
            ROS_INFO("The base failed to move to %d goal (x=%f, y=%f) for some reason", i, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        }
    }
    return 0;
}
