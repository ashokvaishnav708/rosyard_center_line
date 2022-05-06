#include "velocity_estimator.h"
#include "ros/ros.h"

Velocity_Estimator::Velocity_Estimator(){

}


bool Velocity_Estimator::estimateVelocity(const rosyard_common::CarStateRaw& current_pose, rosyard_common::CarState* velocity){


    // return if not initialized
    if(current_pose.header.stamp.toNSec() == 0){
        return false;
    }

  uint64_t diff = 100000000; // 0.1 s
    bool calc_velocity = true;
    // decide to calculate a new velocity if something in the history and fresher than diff
    if (pose_history_.size() > 0 && current_pose.header.stamp.toNSec() < (pose_history_.back().header.stamp.toNSec() + diff)){

        calc_velocity = false;

    }

    if (calc_velocity) {
        pose_history_.push_back(current_pose);

        // calculate velocity if possible
        if(pose_history_.size() > 1){
            const rosyard_common::CarStateRaw& old_pose  = pose_history_[pose_history_.size()-2]; // get the pose before this one

            uint64_t time_difference = current_pose.header.stamp.toNSec()-old_pose.header.stamp.toNSec();
            double diff_x = current_pose.car_state.x - old_pose.car_state.x;
            double diff_y = current_pose.car_state.y - old_pose.car_state.y;

//            std::cerr << "diffx  " << diff_x << " diffy " << diff_y << "\n";

            //calculate velocity in m/s
            float velo_x = diff_x *1000000000.0 / float(time_difference);
            float velo_y = diff_y *1000000000.0 / float(time_difference);

            velocity_history_.push_back(std::pair<float,float>(velo_x,velo_y));
        }else{
            velocity_history_.push_back(std::pair<float,float>(0.0,0.0));
        }
    }

    //delete oldest pose and velocity if max_history_size limit is reached
    if(pose_history_.size() > max_history_size_){
        pose_history_.erase(pose_history_.begin());
    }
    if(velocity_history_.size() > max_velocity_history_size){
        velocity_history_.erase(velocity_history_.begin());
    }

    //calculate velocity - take mean over history
    float mean_velo_x = 0.0;
    float mean_velo_y = 0.0;
    for(auto const& velocity: velocity_history_) {
        mean_velo_x += velocity.first;
        mean_velo_y += velocity.second;
     }
//     std::cerr << "size  " << velocity_history_.size() << "\n";
//     std::cerr << "velocx  " << mean_velo_x << " veloy " << mean_velo_y << "\n";
     mean_velo_x /= float(velocity_history_.size());
     mean_velo_y /= float(velocity_history_.size());

//     std::cerr << "velocx  " << mean_velo_x << " veloy " << mean_velo_y << "\n";

     //set timestamp from current pose
    velocity->header.stamp = current_pose.header.stamp;
    //calculate velocity in m/s
    velocity->velo_x = mean_velo_x;
    velocity->velo_y = mean_velo_y;


    //copy car pose
    velocity->car_state = current_pose.car_state;

    return true;
}

