#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>

#include <swarm/UserParams.h>

#include <dynamic_reconfigure/server.h>
#include <swarm/param_serverConfig.h>

//}

namespace swarm
{

/* class ParamServer //{ */

class ParamServer : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<swarm::UserParams> ph_params_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef swarm::param_serverConfig      DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(swarm::param_serverConfig &params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;

  // | ------------------------- methods ------------------------ |

  void publishParams(swarm::param_serverConfig &params);
};

//}

/* onInit() //{ */

void ParamServer::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ParamServer");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[ParamServer]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("param1", params_.param1);
  param_loader.loadParam("param2", params_.param2);
  param_loader.loadParam("param3", params_.param3);
  param_loader.loadParam("param4", params_.param4);
  param_loader.loadParam("param5", params_.param5);
  param_loader.loadParam("param6", params_.param6);
  param_loader.loadParam("param7", params_.param7);
  param_loader.loadParam("param8", params_.param8);
  param_loader.loadParam("param9", params_.param9);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ParamServer]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  ph_params_ = mrs_lib::PublisherHandler<swarm::UserParams>(nh_, "params_out", 1, true);

  // | --------------------------- drs -------------------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&ParamServer::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------- publish the default params --------------- |

  publishParams(params_);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[ParamServer]: initialized, version %s", VERSION);
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackDrs() */

void ParamServer::callbackDrs(swarm::param_serverConfig &params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  ROS_INFO("[ParamServer]: DRS updated");

  publishParams(params);
}

//}

// | ------------------------- methods ------------------------ |

/* publishParams() //{ */

void ParamServer::publishParams(swarm::param_serverConfig &params) {

  swarm::UserParams params_msg;

  params_msg.param1 = params.param1;
  params_msg.param2 = params.param2;
  params_msg.param3 = params.param3;
  params_msg.param4 = params.param4;
  params_msg.param5 = params.param5;
  params_msg.param6 = params.param6;
  params_msg.param7 = params.param7;
  params_msg.param8 = params.param8;
  params_msg.param9 = params.param9;

  ph_params_.publish(params_msg);
}

//}

}  // namespace swarm

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(swarm::ParamServer, nodelet::Nodelet)
