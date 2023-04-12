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
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <swarm/UserParams.h>
#include <swarm/Swarming.h>

#include <geometry_msgs/Pose.h>

#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>

//}

/* defines //{ */

#define TAU 2 * M_PI

//}

namespace swarm
{

/* class Swarm //{ */

class Swarm : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  double _main_timer_rate_;
  double _swarming_timer_rate_;

  int _n_uavs_;

  int _uav_id_;

  std::string _uav_name_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<swarm::UserParams>                   sh_params_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>            sh_tracker_cmd_;

  std::vector<mrs_lib::SubscribeHandler<swarm::Swarming>> sh_swarming_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::VelocityReferenceStamped> ph_velocity_reference_;
  mrs_lib::PublisherHandler<swarm::Swarming>                    ph_swarming_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  ros::Timer timer_swarming_;
  void       timerStatus(const ros::TimerEvent &event);

  // | --------------- methods for action handlers -------------- |

  bool setCommand(const Eigen::Vector3d &velocity, const double heading_rate);

  std::tuple<bool, double> separationWeighting(const double distance, const double visibility, const double safety_distance, const double desired_distance);
  Eigen::Vector3d          calcSeparation(const std::vector<Eigen::Vector3d> &neighbors);
  Eigen::Vector3d          calcCohesion(const std::vector<Eigen::Vector3d> &neighbors);
  Eigen::Vector3d          saturateVector(const Eigen::Vector3d &vec, const double max_len);

  // | -------------------------- other ------------------------- |

  bool running_ = false;
};

//}

/* onInit() //{ */

void Swarm::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "Swarm");

  param_loader.loadParam("main_rate", _main_timer_rate_);
  param_loader.loadParam("swarming_rate", _swarming_timer_rate_);
  param_loader.loadParam("uav_name", _uav_name_);

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

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in");
  sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");
  sh_params_               = mrs_lib::SubscribeHandler<swarm::UserParams>(shopts, "params_in");

  // | ----------------------- publishers ----------------------- |

  ph_velocity_reference_ = mrs_lib::PublisherHandler<mrs_msgs::VelocityReferenceStamped>(nh_, "velocity_reference_out", 10, true);
  ph_swarming_           = mrs_lib::PublisherHandler<swarm::Swarming>(nh_, "swarming_out", 10, true);

  // subscribe to position cmd
  for (int i = 0; i < _n_uavs_; i++) {

    unsigned int uav_id = i + 1;

    std::stringstream uav_name;
    uav_name << "uav" << uav_id;

    if (_uav_name_ == uav_name.str()) {

      _uav_id_ = i;

      srand(uav_id);
      continue;
    }

    std::stringstream ss;
    ss << "/uav" << uav_id << "/swarm/swarming";
    sh_swarming_.push_back(mrs_lib::SubscribeHandler<swarm::Swarming>(shopts, ss.str()));
  }

  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(_main_timer_rate_), &Swarm::timerMain, this);
  timer_swarming_ = nh_.createTimer(ros::Rate(_swarming_timer_rate_), &Swarm::timerStatus, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Swarm]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void Swarm::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Swarm]: timerMain() spinning");

  if (!running_) {

    if (!sh_params_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[Swarm]: waiting for params");
      return;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[Swarm]: waiting for tracker cmd");
      return;
    }

    for (int i = 0; i < _n_uavs_ - 1; i++) {
      if (!sh_swarming_[i].hasMsg()) {
        ROS_WARN_THROTTLE(1.0, "[Swarm]: waiting for all swarm member states");
        return;
      }
    }

    ROS_INFO_ONCE("[Swarm]: got all swarm member states");

    if (!sh_control_manager_diag_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[Swarm]: timerMain(): waiting for control manager diagnostics");
      return;
    }

    auto control_diag = sh_control_manager_diag_.getMsg();

    if (!control_diag->flying_normally) {
      ROS_WARN_THROTTLE(1.0, "[Swarm]: timerMain(): waiting to be flying normally");
      return;
    }

    ROS_INFO_ONCE("[Swarm]: got all data");

    running_ = true;
  }

  auto params      = sh_params_.getMsg();
  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  std::vector<swarm::Swarming::ConstPtr> swarm_members;
  for (int i = 0; i < _n_uavs_ - 1; i++) {
    swarm_members.push_back(sh_swarming_[i].getMsg());
  }

  for (int i = 0; i < _n_uavs_ - 1; i++) {
    if (!swarm_members[i]->ready) {
      ROS_WARN_THROTTLE(1.0, "[Swarm]: timerMain(): waiting for all uavs to be flying normally");
      return;
    }
  }

  // | -------------------- this uav position ------------------- |

  Eigen::Vector3d my_pose(tracker_cmd->position.x, tracker_cmd->position.y, tracker_cmd->position.z);

  // | ------------------------ atraction ----------------------- |

  std::vector<Eigen::Vector3d> neighbours;

  for (unsigned int i = 0; i < swarm_members.size(); i++) {

    const Eigen::Vector3d neigh_position    = Eigen::Vector3d(swarm_members[i]->pose.x, swarm_members[i]->pose.y, swarm_members[i]->pose.z);
    const Eigen::Vector3d relative_position = neigh_position - my_pose;

    neighbours.push_back(relative_position);
  }

  Eigen::Vector3d atraction(0, 0, 0);
  Eigen::Vector3d cohesion   = calcCohesion(neighbours);
  Eigen::Vector3d separation = calcSeparation(neighbours);

  // | ------------------------- control ------------------------ |

  Eigen::Vector3d des_velocity = params->param1 * cohesion + params->param2 * separation;

  des_velocity = saturateVector(des_velocity, params->param7);

  setCommand(des_velocity, 0);
}

//}

/* timerStatus() //{ */

void Swarm::timerStatus([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Swarm]: timerStatus() spinning");

  if (!sh_tracker_cmd_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[Swarm]: waiting for tracker cmd");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[Swarm]: timerMain(): waiting for control manager diagnostics");
    return;
  }

  auto tracker_cmd  = sh_tracker_cmd_.getMsg();
  auto control_diag = sh_control_manager_diag_.getMsg();

  swarm::Swarming msg;

  msg.ready = control_diag->flying_normally;

  msg.pose = tracker_cmd->position;

  ph_swarming_.publish(msg);
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setCommand() //{ */

bool Swarm::setCommand(const Eigen::Vector3d &velocity, const double heading_rate) {

  ROS_INFO_ONCE("[Swarm]: setCommand() called");

  mrs_msgs::VelocityReferenceStamped velocity_reference;

  velocity_reference.reference.velocity.x = velocity[0];
  velocity_reference.reference.velocity.y = velocity[1];
  velocity_reference.reference.velocity.z = velocity[2];

  velocity_reference.reference.use_heading_rate = true;
  velocity_reference.reference.heading_rate     = heading_rate;

  ph_velocity_reference_.publish(velocity_reference);

  return true;
}

//}

/* separationWeighting() //{ */

std::tuple<bool, double> Swarm::separationWeighting(const double distance, const double visibility, const double safety_distance,
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

Eigen::Vector3d Swarm::calcSeparation(const std::vector<Eigen::Vector3d> &neighbors) {

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

Eigen::Vector3d Swarm::calcCohesion(const std::vector<Eigen::Vector3d> &neighbors) {

  Eigen::Vector3d result(0, 0, 0);

  for (auto &neighbour : neighbors) {

    result += neighbour;
  }

  result /= neighbors.size();

  return result;
}

//}

/* saturateVector() //{ */

Eigen::Vector3d Swarm::saturateVector(const Eigen::Vector3d &vec, const double max_len) {

  if (vec.norm() > max_len) {
    return max_len * vec.normalized();
  }
  return vec;
}

//}


}  // namespace swarm

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(swarm::Swarm, nodelet::Nodelet)
