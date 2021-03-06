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
#include <QDir>

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

    ifstream file;
    file.open(QDir::homePath().append("/catkin_ws/src/tas_car_04/goals.txt").toAscii());

    string s_line;
    QString line;

    int num=0;

    //Read the goals from goals.txt file to construct the global plan
    while(!file.eof()){
        getline(file,s_line);
        line = s_line.c_str();
        if(line.contains("position")){
            num++;
            getline(file,s_line);
            line = s_line.c_str();
            QStringList value;
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.position.x = value.at(1).toFloat();
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.position.y = value.at(1).toFloat();
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.position.z = value.at(1).toFloat();
            getline(file,s_line);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.x = value.at(1).toFloat();
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.y = value.at(1).toFloat();
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.z = value.at(1).toFloat();
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            waypoint1.orientation.w = value.at(1).toFloat();
            waypoints.push_back(waypoint1);
        }
    }
    file.close();


    for(int i=0; i<waypoints.size(); i++){
        ROS_INFO("Goal %d: x=%f, y=%f, w=%f", i, waypoints.at(i).position.x, waypoints.at(i).position.y, waypoints.at(i).orientation.w);
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
