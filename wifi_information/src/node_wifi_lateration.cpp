//#include "readwifistrength.h"
#include "circleintersections.h"
#include "findlocation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "wifi_information/wifi_msgs.h"
#include <tf/transform_datatypes.h>
#include <string>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// write to text file
#include <iostream>
#include <fstream> 

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "buildWifiDatabase.h"

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "wifi_information");
    ros::NodeHandle n;
    WifiSignals wifi = readWifiStrength();

    std::string initial_pose;

    // ...(topic, queue_size, latch flag)
    ros::Publisher initial_pose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "/map";
    initialPose.header.stamp = ros::Time::now();

    float cov1[36] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float cov2[36] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};





/************************** easy initial pose setting ***************************/
    wifi = readWifiStrength();

    /*    	if (wifi.LL.signal_level > wifi.LR.signal_level){

        initial_pose = "Laboratory_left";
        // second position near LL

        initialPose.pose.pose.position.x = 23.6256599426;
        initialPose.pose.pose.position.y = 18.3180732727;
        initialPose.pose.pose.position.z = 0.0;

        tf::Quaternion q(0.0,0.0,0.68516809684,0.728384979988);
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(q, qMsg);
        initialPose.pose.pose.orientation = qMsg;
        for (int i = 0; i<36; i++){
        initialPose.pose.covariance[i] = cov2[i];
        }
        ROS_INFO("Setting pose: Starting Position 2");
        initial_pose_pub_.publish(initialPose);
}*/
    //     else if (wifi.LR.signal_level > wifi.LL.signal_level){
    initial_pose = "laboratory_right";

    // first position near LR


    initialPose.pose.pose.position.x = 11.520401001;
    initialPose.pose.pose.position.y = 18.5730876923;
    initialPose.pose.pose.position.z = 0.0;

    tf::Quaternion q(0.0,0.0,-0.728956516223,0.684560002817);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(q, qMsg);
    initialPose.pose.pose.orientation = qMsg;
    for (int i = 0; i<36; i++){
        initialPose.pose.covariance[i] = cov1[i];
    }

    ROS_INFO("Setting pose: Starting Position 1");

    initial_pose_pub_.publish(initialPose);             // publish on initialpose topic
    //	}
    //    	else {initial_pose = "unknown";
    //	ROS_INFO("Initial Pose UNKNOWN");
    //	};
    /************************** easy initial pose setting END***************************/



    /********************************************************** NEW: WIFI DATABASE - FINGERPRINTING/NN APPROACH*************************************************************/
    fingerprint myFP;
    PositionEstimate pe;

    /**************** BUILD DATABASE ************/
    /*
        const char *mypath1="/home/tas_group_04/WifiDatabase/fingerprint1.dat";
        const char *mypath2="/home/tas_group_04/WifiDatabase/fingerprint2.dat";
        const char *mypath3="/home/tas_group_04/WifiDatabase/fingerprint3.dat";
        const char *mypath4="/home/tas_group_04/WifiDatabase/fingerprint4.dat";
        const char *mypath5="/home/tas_group_04/WifiDatabase/fingerprint5.dat";
        const char *mypath6="/home/tas_group_04/WifiDatabase/fingerprint6.dat";
        const char *mypath7="/home/tas_group_04/WifiDatabase/fingerprint7.dat";
         const char *mypath8="/home/tas_group_04/WifiDatabase/fingerprint8.dat";

       createWifiFingerprint(xpos,ypos)
       myFP = createWifiFingerprint(17.4,19.3);


       BUILD UP THE DATABASE, BY SAVING FINGERPRINTS TO BINARY FILES
       saveWifiFingerprintToBinaryFile(myFP,mypath8);
*/
    /**************** BUILD DATABASE END************/



    // READ OT OF DATABASE
    std::vector <fingerprint> MyWifiDatabase;
    MyWifiDatabase = parseWifiDatabase();
    //printDatabase(MyWifiDatabase);

    /*--------------------------SEND WIFI_LATERATION_NODE MESSAGES ON TOPIC "wifi_lateration_node"-----------------*/
    ros::Publisher chatter_pub = n.advertise<wifi_information::wifi_msgs>("wifi_lateration_node", 1);
    ros::Rate loop_rate(2); // publish twice a second
    int count = 0;
    while (ros::ok())
    {
        wifi = readWifiStrength();

        myFP.pos.x=0;   // dummy
        myFP.pos.y=0;   // dummy
        myFP.meanSignalStrengthArray[0] = wifi.SL.signal_level;
        myFP.meanSignalStrengthArray[1] = wifi.SR.signal_level;
        myFP.meanSignalStrengthArray[2] = wifi.LL.signal_level;
        myFP.meanSignalStrengthArray[3] = wifi.LR.signal_level;

        pe = GetPositionEstimateByFingerprinting(MyWifiDatabase,myFP);  // GET POSITION ESTIMATE FROM NEAREST NEIGBOUR APPROACH

        std::cout << "Position estimate x: " << pe.x << std::endl;
        std::cout << "Position estimate y: " << pe.y << std::endl;

        wifi_information::wifi_msgs msg;


        msg.initial_pose_guess = initial_pose;

        msg.wifi_estimate_pos_x = pe.x;                 // PUBLISH POSITION ESTIMATE
        msg.wifi_estimate_pos_y = pe.y;
        // ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);


        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    /**************** OLD APPROACH MODELLING THE TRAVELLING SIGNAL: *******************/
    /*********** MAP THE SIGNAL STRENGTH TO A DISTANCE AND USE THIS DISTANCE AS A RADIUS OF A CIRCLE, ******************/
    /*********** THEN FIND THE INTERSECTIONS OF THIS CIRCLE WITH THE CIRCLES DERIVED FROM THE OTHER SIGNAL STRENGTHS ***/
    /*
    std::vector <float> intersections_SL_SR;
    std::vector <float> intersections_SL_LL;
    std::vector <float> intersections_LL_LR;
    std::vector <float> intersections_SR_LR;
    std::vector <float> intersections_LL_SR;
    std::vector <float> intersections_SL_LR;
    std::vector <std::vector <float> > all_intersections;
    std::vector <float> location_guess;
    float radius1, radius2;
    
    if(wifi.SL.signal_level!=0 && wifi.SR.signal_level!=0){
        radius1 = calculateDistance(wifi.SL.signalLevelInDb,wifi.SL.freqInMHz);
        radius2 = calculateDistance(wifi.SR.signalLevelInDb,wifi.SR.freqInMHz);
        std::cout << "SL & SR" << std::endl;
        std::cout << "radius1: " << radius1 << std::endl;
        std::cout << "radius2: " << radius2 << std::endl;
        intersections_SL_SR = compute_circle_intersections(radius1,radius2,wifi.SL.MAC_id,wifi.SR.MAC_id);
        all_intersections.push_back(intersections_SL_SR);
    }


    if(wifi.SL.signal_level!=0 && wifi.LL.signal_level!=0){
        radius1 = calculateDistance(wifi.SL.signalLevelInDb,wifi.SL.freqInMHz);
        radius2 = calculateDistance(wifi.LL.signalLevelInDb,wifi.LL.freqInMHz);
        std::cout << "SL & LL" << std::endl;
        std::cout << "radius1: " << radius1 << std::endl;
        std::cout << "radius2: " << radius2 << std::endl;
        intersections_SL_LL = compute_circle_intersections(radius1,radius2,wifi.SL.MAC_id,wifi.LL.MAC_id);
        all_intersections.push_back(intersections_SL_LL);
    }


    if(wifi.LL.signal_level!=0 && wifi.LR.signal_level!=0){
        radius1 = calculateDistance(wifi.LL.signalLevelInDb,wifi.LL.freqInMHz);
        radius2 = calculateDistance(wifi.LR.signalLevelInDb,wifi.LR.freqInMHz);
        std::cout << "LL & LR" << std::endl;
        std::cout << "radius1: " << radius1 << std::endl;
        std::cout << "radius2: " << radius2 << std::endl;
        intersections_LL_LR = compute_circle_intersections(radius1,radius2,wifi.LL.MAC_id,wifi.LR.MAC_id);
        all_intersections.push_back(intersections_LL_LR);
    }


    if(wifi.SR.signal_level!=0 && wifi.LR.signal_level!=0){
        radius1 = calculateDistance(wifi.SR.signalLevelInDb,wifi.SR.freqInMHz);
        radius2 = calculateDistance(wifi.LR.signalLevelInDb,wifi.LR.freqInMHz);
        std::cout << "SR & LR" << std::endl;
        std::cout << "radius1: " << radius1 << std::endl;
        std::cout << "radius2: " << radius2 << std::endl;
        intersections_SR_LR = compute_circle_intersections(radius1,radius2,wifi.SR.MAC_id,wifi.LR.MAC_id);
        all_intersections.push_back(intersections_SR_LR);
    }


    if(wifi.LL.signal_level!=0 && wifi.SR.signal_level!=0){
        radius1 = calculateDistance(wifi.LL.signalLevelInDb,wifi.LL.freqInMHz);
        radius2 = calculateDistance(wifi.SR.signalLevelInDb,wifi.SR.freqInMHz);
        std::cout << "LL & SR" << std::endl;
        std::cout << "radius1: " << radius1 << std::endl;
        std::cout << "radius2: " << radius2 << std::endl;
        intersections_LL_SR = compute_circle_intersections(radius1,radius2,wifi.LL.MAC_id,wifi.SR.MAC_id);
        all_intersections.push_back(intersections_LL_SR);
    }

    if(wifi.SL.signal_level!=0 && wifi.LR.signal_level!=0){
        radius1 = calculateDistance(wifi.SL.signalLevelInDb,wifi.SL.freqInMHz);
        radius2 = calculateDistance(wifi.LR.signalLevelInDb,wifi.LR.freqInMHz);
        std::cout << "SL & LR" << std::endl;
        std::cout << "radius1: " << radius1 << std::endl;
        std::cout << "radius2: " << radius2 << std::endl;
        intersections_SL_LR = compute_circle_intersections(radius1,radius2,wifi.SL.MAC_id,wifi.LR.MAC_id);
        all_intersections.push_back(intersections_SL_LR);
    }


    // find the x,y coordinate that is most likely in the intersections
    location_guess = readLocationGuess(all_intersections);

    for (int i=0; i<location_guess.size();i++){
        std::cout << location_guess.at(i) << std::endl;
    }
*/
    /**************** OLD APPROACH MODELLING THE TRAVELLING SIGNAL END*******************/
    return 0;
}

