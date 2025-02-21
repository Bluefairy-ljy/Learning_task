//
// Created by ljyi on 25-2-17.
//

#ifndef WINDMILL_CONTROLLER_H
#define WINDMILL_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <std_msgs/String.h>
#include <random>
#include <pluginlib/class_list_macros.h>

namespace windmill_controller {
class WindmillController:public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;

  private:
  ros::Subscriber mode_sub;
  void mode_callback(const std_msgs::StringConstPtr& msg);
  void reset_params();
  hardware_interface::JointHandle joint;
  control_toolbox::Pid pid;
  std::string joint_name;
  enum Mode { STOP, SMALL, BIG };
  Mode current_mode = STOP;
  ros::Time mode_time;
  double kf;
  double prev_target;
  double small_speed;
  double a_min, a_max, w_min, w_max;
  double a, w, b;
  std::mt19937 rng;
  std::uniform_real_distribution<double> a_rng;
  std::uniform_real_distribution<double> w_rng;
};

}
#endif //WINDMILL_CONTROLLER_H
