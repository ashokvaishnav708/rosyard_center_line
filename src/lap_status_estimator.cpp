#include "rosyard_common/LapData.h"
#include "rosyard_common/PassedStartFinish.h"
#include "rosyard_common/CarStateRaw.h"
#include "rosyard_common/RosyardStatus.h"
#include "std_msgs/String.h"

#include "ros/ros.h"
#include "ros/rate.h"

#include <boost/bind.hpp>

#include <cstdint>
#include <list>

struct Lap_time_data{
    uint32_t lap_count;
    std::list<double> list_of_all_times;
    double average_lap_time;
    double momentary_lap_time;
    double last_lap_time;
};

uint32_t sumUpList(std::list<double>* list){
    double sum = 0;
    for(std::list<double>::iterator it = list->begin(); it != list->end(); it++){
        sum = sum + *it;
    }
    return sum;
}

//void publishFinishStatus(ros::Publisher* finished_publisher){
//    rosyard_common::RosyardStatus finished_status;
//    finished_status.started = false;
//    finished_status.finished = true;
//    finished_status.kill = false;
//    finished_publisher->publish(finished_status);
//}

/* void passedStartFinishCallback(Lap_time_data* lap_data, const rosyard_common::PassedStartFinish::ConstPtr& msg){
    lap_data->lap_count++;
    lap_data->lap_times_seconds.push_back(msg->header.stamp.sec);
    lap_data->lap_times_nanoseconds.push_back(msg->header.stamp.nsec);
    lap_data->momentary_lap_time_seconds = msg->header.stamp.sec;
    lap_data->momentary_lap_time_nanoseconds = msg->header.stamp.nsec;
    ros::Duration t = msg->header.stamp - msg->header.stamp;
    ROS_INFO("sub sekunden: %i", t.sec);
    //ROS_INFO(msg->header.stamp);
} */

rosyard_common::LapData constructLapDataMessage(Lap_time_data* lap_data){
    rosyard_common::LapData lap_data_msg;
    lap_data_msg.lap_count = lap_data->lap_count;

    // add lap times
    std::list<double>::iterator it;

    // Make iterate point to begining and incerement it one by one till it reaches the end of list.
    for (it = lap_data->list_of_all_times.begin(); it != lap_data->list_of_all_times.end(); it++)
    {
       lap_data_msg.lap_times.push_back(*it);

    }

//    lap_data_msg.lap_times = lap_data->list_of_all_times;
//    lap_data_msg.average_lap_time = lap_data->average_lap_time;
    return lap_data_msg;
}

void setLapData(Lap_time_data* lap_data, const rosyard_common::CarStateRaw::ConstPtr msg){
    lap_data->lap_count += 1;
    double lap_time = msg->header.stamp.toSec() - lap_data->last_lap_time; 
    lap_data->list_of_all_times.push_back(lap_time);
    lap_data->average_lap_time = 
        sumUpList(&lap_data->list_of_all_times) / static_cast<double>(lap_data->list_of_all_times.size());
    lap_data->momentary_lap_time = lap_time;
    lap_data->last_lap_time =  msg->header.stamp.toSec();
}

void setConstructSendLapData(ros::Publisher* lap_data_publisher,
//                             ros::Publisher* finished_publisher,
                             Lap_time_data* lap_data,
                             const rosyard_common::CarStateRaw::ConstPtr msg
//                             int maximum_number_of_laps
                             ){
    setLapData(lap_data, msg);
//    if(lap_data->lap_count < maximum_number_of_laps){
        rosyard_common::LapData lap_data_msg = constructLapDataMessage(lap_data);
        lap_data_publisher->publish(lap_data_msg);
//    }
//    else if(lap_data->lap_count == maximum_number_of_laps){
//        rosyard_common::LapData lap_data_msg = constructLapDataMessage(lap_data);
//        lap_data_publisher->publish(lap_data_msg);
//        publishFinishStatus(finished_publisher);
//    }
}

bool isStartFinishedPassed(float x_old, float y, float x_new, bool x_was_bigger_than_10){
    return x_old < 0 && y< 2 && y>-2 && x_new >= 0 && x_was_bigger_than_10;
}

void carPositionCallback(Lap_time_data* lap_data,
                         ros::Publisher* lap_data_publisher,
//                         ros::Publisher* finished_publisher,
                         float* x_old,
                         bool* x_was_bigger_than_10,
//                         int maximum_number_of_laps,
                         const rosyard_common::CarStateRaw::ConstPtr& msg){
    float x_new = msg->car_state.x;
    float y = msg->car_state.y;
    if(isStartFinishedPassed(*x_old, y, x_new, *x_was_bigger_than_10)){
        setConstructSendLapData(lap_data_publisher, lap_data, msg);
        *x_old = x_new;
        *x_was_bigger_than_10 = false;
    }else{
        *x_old = x_new;
        if(!*x_was_bigger_than_10 || x_new > 10){
            *x_was_bigger_than_10 = x_new > 10;
        }
    }
}

void rosyardStatusCallback(Lap_time_data* lap_data,
                           rosyard_common::RosyardStatus* started,
                           const rosyard_common::RosyardStatus::ConstPtr& status_msg){
    if(started->started == false && status_msg->started){
        started->started = true;
        lap_data->last_lap_time = status_msg->header.stamp.toSec();
        ROS_INFO("Started!");
    }
}

//void testcallback(const rosyard_common::LapData::ConstPtr& msg){
//    ROS_INFO("lap count: %u", msg->lap_count);
//    ROS_INFO("momentary: %f", msg->momentary_lap_time);
//    ROS_INFO("average: %f", msg->average_lap_time);
//}

int main(int argc, char **argv){
    ros::init(argc, argv, "lap_status_estimator");
    ros::NodeHandle n("~");
    int estimation_lap_rate;
    n.getParam("/nodes/estimation_lap_rate", estimation_lap_rate);
    std::string estimation_lap_topic_name;
    n.getParam("/nodes/estimation_lap_topic_name", estimation_lap_topic_name);
    std::string car_position_topic_name;
    n.getParam("/nodes/slam_car_topic_name", car_position_topic_name);
    std::string rosyard_status_topic_name;
    n.getParam("/nodes/rosyard_status_topic_name", rosyard_status_topic_name);
    /* ros::Publisher passedStartFinishPublisher = 
        n.advertise<rosyard_common::PassedStartFinish>("/passedStartFinish", 30); */
    ros::Publisher lap_data_publisher = 
        n.advertise<rosyard_common::LapData>(estimation_lap_topic_name, estimation_lap_rate);
//    ros::Publisher finished_publisher =
//        n.advertise<rosyard_common::RosyardStatus>(rosyard_status_topic_name, 30);
    Lap_time_data lap_data;
    lap_data.lap_count = 0;
    lap_data.average_lap_time = 0;
    lap_data.momentary_lap_time = 0;
    lap_data.last_lap_time = 0;
    float x_old = 0;
    bool x_was_bigger_than_10 = false;
    ros::Subscriber car_position = 
        n.subscribe<rosyard_common::CarStateRaw>(car_position_topic_name,
                                                 30, 
                                                 boost::bind(carPositionCallback,
                                                             &lap_data, 
                                                             &lap_data_publisher,
//                                                             &finished_publisher,
                                                             &x_old,
                                                             &x_was_bigger_than_10,
//                                                             maximum_number_of_laps,
                                                             _1));
    /* ros::Subscriber passed_start_finish_listener = 
        n.subscribe<rosyard_common::PassedStartFinish>("/passedStartFinish",
                                                       30,
                                                       boost::bind(passedStartFinishCallback,
                                                                   &lap_data,
                                                                   _1)); */
    rosyard_common::RosyardStatus started;
    started.started = false;
    started.finished = false;
    started.running = false;
    ros::Subscriber rosyard_status_listener = 
        n.subscribe<rosyard_common::RosyardStatus>(rosyard_status_topic_name, 
                                                   30, 
                                                   boost::bind(rosyardStatusCallback, 
                                                               &lap_data,
                                                               &started,
                                                               _1));
//    ros::Subscriber test =
//        n.subscribe<rosyard_common::LapData>("/estimation/lap_data",
//                                             30,
//                                             testcallback);
    ros::spin();
}