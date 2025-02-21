//
// Created by ljyi on 25-2-17.
//

#include <learning/windmill_controller.h>
namespace windmill_controller {
bool WindmillController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh) {
  joint = hw->getHandle("rotation_joint");

  ros::NodeHandle pid_nh(ctrl_nh, "pid");
  if (!pid.init(pid_nh)) {
    ROS_ERROR("Could not init pid.");
    return false;
  }

    ctrl_nh.param("kf", kf, 0.3);

    ctrl_nh.param("small_speed", small_speed, 1.047);

    ros::NodeHandle big_speed_nh(ctrl_nh, "big_speed");
    big_speed_nh.param("a_min", a_min, 0.780);
    big_speed_nh.param("a_max", a_max, 1.045);
    big_speed_nh.param("w_min", w_min, 1.884);
    big_speed_nh.param("w_max", w_max, 2.000);

  std::random_device rd;
  rng = std::mt19937(rd());
  a_rng = std::uniform_real_distribution<double>(a_min, a_max);
  w_rng = std::uniform_real_distribution<double>(w_min, w_max);
  reset_params();

  mode_sub = root_nh.subscribe("/mode", 10, &WindmillController::mode_callback, this);

  prev_target = 0;

  return true;
}

void WindmillController::mode_callback(const std_msgs::StringConstPtr& msg) {
  if (msg->data == "small") {
    current_mode = SMALL;
    mode_time = ros::Time::now();
    prev_target = 0;
    ROS_INFO("SMALL mode");
  }
  else if (msg->data == "big" && current_mode != BIG) {
    current_mode = BIG;
    mode_time = ros::Time::now();
    reset_params();
    prev_target = 0;
    ROS_INFO("BIG mode");
  }
  else if (msg->data == "stop") {
    current_mode = STOP;
    prev_target = 0;
    ROS_INFO("STOP mode");
  }
}

void WindmillController::starting(const ros::Time& time) {
  pid.reset();
  current_mode = STOP;
  prev_target = 0;
}

void WindmillController::reset_params() {
  a = a_rng(rng);
  w = w_rng(rng);
  b = 2.09 - a;
  ROS_INFO("New params: a=%.3f w=%.3f b=%.3f", a, w, b);
}

void WindmillController::update(const ros::Time& time, const ros::Duration& dt) {
  double target = 0;
  switch (current_mode) {
    case SMALL:
      target = small_speed;
      break;
    case BIG: {
      double t = (time - mode_time).toSec();
      target = a * sin(w * t) + b;
      break;
    }
    case STOP:
      break;
  }
  if (current_mode != STOP) {
    double current = joint.getVelocity();
    double error = target - current;
    double pid_out = pid.computeCommand(error, dt);
    // std::cout <<"pid_out : " << pid_out << std::endl;
    // std::cout <<"target : " << target << std::endl;
    // std::cout <<"current : " << current << std::endl;
    std::cout <<"error : "<< error << std::endl;

    double target_change = target - prev_target;
    double ff = kf * target_change;
    prev_target = target;

    joint.setCommand(pid_out + ff);
    //ROS_INFO("torque: %.3f", pid_out + ff);
  }
  else {
    joint.setCommand(0.0);
    pid.reset();
    prev_target = 0;
  }
}
PLUGINLIB_EXPORT_CLASS(windmill_controller::WindmillController, controller_interface::ControllerBase);
}  // namespace windmill_controller