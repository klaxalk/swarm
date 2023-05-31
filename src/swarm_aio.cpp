#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <random>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/HwApiVelocityHdgRateCmd.h>

#include <swarm/UserParams.h>
#include <swarm/Swarming.h>

#include <geometry_msgs/Pose.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>

//}

/* defines //{ */

#define TAU 2 * M_PI

//}

namespace swarm_aio
{

/* class Swarm //{ */

class SwarmAio : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  double _main_timer_rate_;
  double _swarming_timer_rate_;

  int _n_uavs_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<swarm::UserParams> sh_params_;

  std::vector<mrs_lib::SubscribeHandler<nav_msgs::Odometry>> sh_odom_;

  // | ----------------------- publishers ----------------------- |

  std::vector<mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>> ph_velocity_reference_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  ros::Timer timer_swarming_;
  void       timerStatus(const ros::TimerEvent &event);

  // | --------------- methods for action handlers -------------- |

  bool setCommand(const int idx, const Eigen::Vector3d &velocity, const double heading_rate);

  std::tuple<bool, double> separationWeighting(const double distance, const double visibility, const double safety_distance, const double desired_distance);
  Eigen::Vector3d          calcSeparation(const std::vector<Eigen::Vector3d> &neighbors);
  Eigen::Vector3d          calcCohesion(const std::vector<Eigen::Vector3d> &neighbors);
  Eigen::Vector3d          calcAlignment(const std::vector<Eigen::Vector3d> &neighbors_vels, const std::vector<Eigen::Vector3d> &neighbors);
  Eigen::Vector3d          saturateVector(const Eigen::Vector3d &vec, const double max_len);

  // | -------------------------- other ------------------------- |

  bool running_ = false;
};

//}

/* onInit() //{ */

void SwarmAio::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "Swarm");

  param_loader.loadParam("main_rate", _main_timer_rate_);
  param_loader.loadParam("swarming_rate", _swarming_timer_rate_);

  std::vector<std::string> uav_names;
  param_loader.loadParam("uav_names", uav_names);

  _n_uavs_ = uav_names.size();

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Swarm]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Swarm";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_params_ = mrs_lib::SubscribeHandler<swarm::UserParams>(shopts, "params_in");

  // | ----------------------- publishers ----------------------- |


  // subscribe
  for (int i = 0; i < _n_uavs_ - 1; i++) {

    unsigned int uav_id = i + 1;

    {
      std::stringstream ss;
      ss << "/multirotor_simulator/uav" << uav_id << "/velocity_hdg_rate_cmd";
      ph_velocity_reference_.push_back(mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>(nh_, ss.str(), 10, true));
    }

    {
      std::stringstream ss;
      ss << "/multirotor_simulator/uav" << uav_id << "/odom";
      sh_odom_.push_back(mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, ss.str()));
    }
  }

  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(_main_timer_rate_), &SwarmAio::timerMain, this);
  timer_swarming_ = nh_.createTimer(ros::Rate(_swarming_timer_rate_), &SwarmAio::timerStatus, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Swarm]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void SwarmAio::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Swarm]: timerMain() spinning");

  if (!running_) {

    if (!sh_params_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[Swarm]: waiting for params");
      return;
    }

    for (int i = 0; i < _n_uavs_ - 1; i++) {
      if (!sh_odom_[i].hasMsg()) {
        ROS_WARN_THROTTLE(1.0, "[Swarm]: waiting for all swarm member odoms (missing uav%d)", i + 1);
        return;
      }
    }

    ROS_INFO_ONCE("[Swarm]: got all data");

    running_ = true;
  }

  auto params = sh_params_.getMsg();

  for (int i = 0; i < _n_uavs_ - 1; i++) {

    auto odom = sh_odom_[i].getMsg();

    // | -------------------- this uav position ------------------- |

    Eigen::Vector3d my_pose(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);

    // | ------------------------ atraction ----------------------- |

    std::vector<Eigen::Vector3d> neighbours;
    std::vector<Eigen::Vector3d> neighbours_vels;
    std::vector<double>          neighbours_hdg;

    for (int j = 0; j < _n_uavs_ - 1; j++) {

      if (i == j) {
        continue;
      }

      auto n_odom = sh_odom_[j].getMsg();

      const Eigen::Vector3d neigh_position    = Eigen::Vector3d(n_odom->pose.pose.position.x, n_odom->pose.pose.position.y, n_odom->pose.pose.position.z);
      const Eigen::Vector3d relative_position = neigh_position - my_pose;

      Eigen::Matrix3d R = mrs_lib::AttitudeConverter(n_odom->pose.pose.orientation);

      const Eigen::Vector3d neigh_vel_world = R * Eigen::Vector3d(n_odom->twist.twist.linear.x, n_odom->twist.twist.linear.y, n_odom->twist.twist.linear.z);

      neighbours.push_back(relative_position);

      neighbours_vels.push_back(neigh_vel_world);

      neighbours_hdg.push_back(mrs_lib::AttitudeConverter(n_odom->pose.pose.orientation).getHeading());
    }

    Eigen::Vector3d atraction(0, 0, 0);
    Eigen::Vector3d cohesion   = calcCohesion(neighbours);
    Eigen::Vector3d separation = calcSeparation(neighbours);
    Eigen::Vector3d alignment  = calcAlignment(neighbours_vels, neighbours);
    Eigen::Vector3d target;

    target = -my_pose;

    // | ------------------------- control ------------------------ |

    Eigen::Vector3d des_velocity = params->param1 * cohesion + params->param2 * separation + params->param3 * alignment + params->param4 * target;

    Eigen::Matrix3d R = mrs_lib::AttitudeConverter(odom->pose.pose.orientation);

    Eigen::Vector3d des_velocity_body = R.transpose() * des_velocity;

    double des_hdg_rate = des_velocity_body(1);

    if (!std::isfinite(des_hdg_rate)) {
      des_hdg_rate = 0;
      ROS_ERROR("NaN detected in variable \"des_hdg_rate\", setting it to 0 and returning!!!");
      return;
    } else if (des_hdg_rate > 1) {
      des_hdg_rate = 1;
    } else if (des_hdg_rate < -1) {
      des_hdg_rate = -1;
    }

    des_velocity_body(1) = 0;

    if (des_velocity_body(0) < 0) {
      des_velocity_body(0) = 0;
    }

    Eigen::Vector3d des_velocity_aligned = R * des_velocity_body;

    des_velocity = saturateVector(des_velocity_aligned, params->param7);

    setCommand(i, des_velocity, des_hdg_rate);
  }
}

//}

/* timerStatus() //{ */

void SwarmAio::timerStatus([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Swarm]: timerStatus() spinning");
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setCommand() //{ */

bool SwarmAio::setCommand(const int idx, const Eigen::Vector3d &velocity, const double heading_rate) {

  ROS_INFO_ONCE("[Swarm]: setCommand() called");

  mrs_msgs::HwApiVelocityHdgRateCmd velocity_reference;

  velocity_reference.velocity.x = velocity[0];
  velocity_reference.velocity.y = velocity[1];
  velocity_reference.velocity.z = velocity[2];

  velocity_reference.heading_rate = heading_rate;

  ph_velocity_reference_[idx].publish(velocity_reference);

  return true;
}

//}

/* separationWeighting() //{ */

std::tuple<bool, double> SwarmAio::separationWeighting(const double distance, const double visibility, const double safety_distance,
                                                       [[maybe_unused]] const double desired_distance) {

  if (distance > safety_distance && distance <= visibility) {
    return {true, ((1.0 / (distance - safety_distance)) - (1.0 / (visibility - safety_distance)))};
  } else if (distance > visibility) {
    return {true, 0};
  } else {
    return {false, 0};
  }
}

//}

/* calcSeparation() //{ */

Eigen::Vector3d SwarmAio::calcSeparation(const std::vector<Eigen::Vector3d> &neighbors) {

  auto params = sh_params_.getMsg();

  Eigen::Vector3d result(0, 0, 0);

  for (auto &neighbour : neighbors) {

    const double visibility      = params->param8;
    const double safety_distance = params->param9;

    auto [defined, weight] = separationWeighting(neighbour.norm(), visibility, safety_distance, 0.0);

    if (defined) {
      result += -weight * neighbour;
    } else {
      result += -1e6 * neighbour;
    }
  }

  result /= neighbors.size();

  return result;
}

//}

/* calcCohesion() //{ */

Eigen::Vector3d SwarmAio::calcCohesion(const std::vector<Eigen::Vector3d> &neighbors) {

  Eigen::Vector3d result(0, 0, 0);

  for (auto &neighbour : neighbors) {

    result += neighbour;
  }

  result /= neighbors.size();

  return result;
}

//}

/* calcAlignment() //{ */

Eigen::Vector3d SwarmAio::calcAlignment(const std::vector<Eigen::Vector3d> &neighbors_vels, const std::vector<Eigen::Vector3d> &neighbors) {

  auto params = sh_params_.getMsg();

  Eigen::Vector3d result(0, 0, 0);

  const double visibility = params->param8;

  int n_neighbours = 0;

  for (unsigned int i = 0; i < neighbors_vels.size(); i++) {

    if (neighbors[i].norm() > visibility) {
      continue;
    }

    result += neighbors_vels[i];

    n_neighbours++;
  }

  if (n_neighbours > 0) {
    result /= n_neighbours;
  } else {
    return Eigen::Vector3d(0, 0, 0);
  }

  return result;
}

//}

/* saturateVector() //{ */

Eigen::Vector3d SwarmAio::saturateVector(const Eigen::Vector3d &vec, const double max_len) {

  if (vec.norm() > max_len) {
    return max_len * vec.normalized();
  }

  return vec;
}

//}

}  // namespace swarm_aio

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(swarm_aio::SwarmAio, nodelet::Nodelet)
