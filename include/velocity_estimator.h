#pragma once

#include "geometry_msgs/PoseStamped.h"
#include "rosyard_common/CarStateRaw.h"
#include "rosyard_common/CarState.h"

class Velocity_Estimator {

public:

    ///
    /// \brief Velocity_Estimator is the standard constructor
    ///
    Velocity_Estimator();

    ///
    /// \brief estimateVelocity calculates the current velocity of the car based on the given pose as well as the pose history
    /// \param current_pose     pose of the car as calculated by the slam node
    /// \return                 2D velocity relative to the cars frame, i.e. the x direction describes the forward/backward velocity and y the left/right velocity
    ///

    ///
    /// \brief estimateVelocity calculates the current velocity of the car based on the given pose as well as the pose history
    /// \param current_pose     pose of the car as calculated by the slam node
    /// \param velocity         2D velocity relative to the cars frame, i.e. the x direction describes the forward/backward velocity and y the left/right velocity
    /// \return                 true if successful
    ///
    bool estimateVelocity(const rosyard_common::CarStateRaw& current_pose, rosyard_common::CarState* velocity);

private:
    size_t max_history_size_ = 10;
    size_t max_velocity_history_size = 3;
    std::vector<rosyard_common::CarStateRaw> pose_history_;
    std::vector<std::pair<float,float>> velocity_history_;
};
