#include <Node.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <FlightTaskManualPosition.hpp>

using namespace Eigen;

namespace mc_pos_control
{

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh), init_(false) {
  ROS_INFO("Subscribing to odometry");
  subOdom_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &Node::odomCallback, this);

  ROS_INFO("Subscribing to extended state");
  subExtendedState_ = nh_.subscribe("/mavros/extended_state", 1, &Node::extendedStateCallback, this);

  ROS_INFO("Subscribing to state");
  subState_ = nh_.subscribe("/mavros/state", 1, &Node::stateCallback, this);

  ROS_INFO("Subscribing to manual control");
  subManualControl_ = nh_.subscribe<mavros_msgs::ManualControl>("/mavros/manual_control/control", 1, &Node::manualControlCallback, this);

  pubAttitudeTarget_ = nh_.advertise<mavros_msgs::AttitudeTarget>("attitude_target", 1);

  int publish_rate = default_publish_rate_;
  ros::param::get("~publish_rate", publish_rate);
  pubTimer_ = nh_.createTimer(ros::Duration(1.0f/publish_rate), &Node::publishAttitudeTarget, this);

  int control_mode = default_control_mode_;
  ros::param::get("~control_mode", control_mode);

  switch(control_mode) {
    case 0  : flightTask_ = new FlightTaskManualPosition();
    case 1  : flightTask_ = new FlightTaskManualAltitude();
    case 2  : flightTask_ = new FlightTaskManual();
    default : flightTask_ = new FlightTaskManualPosition();
  }
}

void Node::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  tf::pointMsgToEigen(odom->pose.pose.position, states_.position);
  tf::vectorMsgToEigen(odom->twist.twist.linear, states_.velocity);
  states_.yaw = dcm2vec(quat2dcm(quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z)))(2);
}

void Node::extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr& extendedStateMsg) {
}

void Node::stateCallback(const mavros_msgs::StateConstPtr& stateMsg) {
  armed_ = stateMsg->armed;
}

void Node::manualControlCallback(const mavros_msgs::ManualControl::ConstPtr& manualControl) {
  if(armed_)
    flightTask_->updateSticks(manualControl);
}

void Node::publishAttitudeTarget(const ros::TimerEvent& timerEvent) {
  if(!init_) {
    init_  = flightTask_->activate() && flightTask_->updateInitialize();
  }

  static size_t trace_id = 0;
  std_msgs::Header header;
  header.frame_id = "/attitude_target";
  header.seq = trace_id++;
  header.stamp = ros::Time::now();

  ros::Time now = ros::Time::now();
  if (prevStamp_.sec != 0) {
    const double dt = (now - prevStamp_).toSec();
    if(armed_) {
      flightTask_->updateState(states_);
      flightTask_->update();
      mavros_msgs::PositionTarget setpoint = flightTask_->getPositionSetpoint();
      Constraints constraints = flightTask_->getConstraints();
      posControl_.setDt(dt);
      posControl_.updateConstraints(constraints);
      posControl_.updateState(states_);
      posControl_.updateSetpoint(setpoint);
      // Generate desired thrust and yaw.
      posControl_.generateThrustYawSetpoint(dt);
      vec3 thr_sp = posControl_.getThrustSetpoint();
      scalar_t yaw_sp = posControl_.getYawSetpoint();
      mavros_msgs::AttitudeTarget cmd = thrustToAttitude(thr_sp, yaw_sp);
      cmd.header = header;
      // Convert yawspeed from wrt NED to wrt ENU
      cmd.body_rate.z = -posControl_.getYawspeedSetpoint();
      pubAttitudeTarget_.publish(cmd);
    }
  }
  prevStamp_ = now;
}

}