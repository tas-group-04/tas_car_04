#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <fstream>

#include <pwd.h>
#include <string>

using namespace std;

int message_count = 0;
int overall_count = 0;
int period = 10;
ofstream out_file;

void messageCallback(const sensor_msgs::LaserScan message)
{
    message_count ++;
    if(message_count == period){
        message_count = 0;
        /*std::vector<float> ranges;
        message_count = 0;
        sD;
        float temp_range = 0.0;
        for (unsigned int i=710; i<720;i++)
        {
            temp_range += message.ranges[i];
        }
        sD.range = temp_range / 10.0;
        sD.ownPos_x = pos_x;
        sD.ownPos_y = pos_y;
        QFile out_file("example.txt");
        QFileInfo info1(out_file);
        std::cout << info1.absolutePath().toStdString() << std::endl;

        if (!out_file.open(QIODevice::WriteOnly | QIODevice::Text)){
            std::cout << "UNABLE TO OPEN!" << std::endl;
            return;
        }

        QTextStream out(&out_file);
        for(int i=0; i<ranges.size(); i++){
            out << ranges.at(i).range << " " << ranges.at(i).ownPos_x  << " " << ranges.at(i).ownPos_y
                   //   <<" " <<message.header.stamp.sec << "."  << message.header.stamp.nsec
                <<"\n";
        }
        out << sD.range << " " << sD.ownPos_x  << " " << sD.ownPos_y  <<" " <<message.header.stamp.sec << "."  << message.header.stamp.nsec <<"\n";

        out_file.close();
        ranges.push_back(sD);
        std::cout << "time_stamp: " << message.header.stamp.sec << "."  << message.header.stamp.nsec<< std::endl;
        std::cout << sD.range << " " << sD.ownPos_x  << " " << sD.ownPos_y  << std::endl;*/
        for(int i=0; i<message.ranges.size(); i++){
            out_file << message.ranges[i] << " ";
        }
        out_file << std::endl;
        overall_count++;
        ROS_INFO("Message %d written", overall_count);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_filter");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    string topic;
    nh.param("period", period, 10);
    if(nh.getParam("topic", topic) == false){
       topic = "scan";
    }
    ROS_INFO("Subscribed to topic '%s' with period: %d" , topic.c_str(), period);
    ros::Subscriber sub = n.subscribe(topic.c_str(), 1, messageCallback);
    passwd* pw = getpwuid(getuid());
    string path(pw->pw_dir);
    path += "/";
    path += topic;
    path += ".txt";
    out_file.open(path.c_str());
    ROS_INFO("File opened: %s", path.c_str());
    ros::spin();
    out_file.close();
    cout << "File closed: " << path << endl;

    return 0;
}
