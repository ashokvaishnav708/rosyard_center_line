#include "ros/ros.h"
#include "ros/rate.h"
#include "velocity_estimator.h"
#include "rosyard_common/CarStateRaw.h"
#include "rosyard_common/CarState.h"

#include <boost/bind.hpp>

void carstateRawCallback(rosyard_common::CarStateRaw* current_pose, const rosyard_common::CarStateRaw::ConstPtr& new_pose){
    if(current_pose != nullptr){
        *current_pose = *new_pose;
    }
}


int main(int argc, char **argv)
{
    Velocity_Estimator estimator;

    ros::init(argc, argv, "velocity");
    ros::NodeHandle n;

    // load configs
    int updateRate;
    n.getParam("/nodes/velocity_rate",updateRate);
    std::string subTopicName, pubTopicName;
    n.getParam("/nodes/velocity_topic_name",pubTopicName);
    n.getParam("/nodes/slam_car_topic_name",subTopicName);

    //init subscriber
    rosyard_common::CarStateRaw current_pose;
    rosyard_common::CarState velocity;
    ros::Subscriber pose_listener = n.subscribe<rosyard_common::CarStateRaw>(subTopicName, 100, boost::bind(carstateRawCallback, &current_pose ,_1));

    //init publisher
    ros::Publisher velocity_publisher = n.advertise<rosyard_common::CarState>(pubTopicName, 100);
    ros::Rate loop_rate(updateRate);

    while (ros::ok()){
        //publish new carstate with velocity
        if(estimator.estimateVelocity(current_pose, &velocity)){
            velocity_publisher.publish(velocity);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
