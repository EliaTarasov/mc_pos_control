#ifndef MC_POS_CONTROL_NODE_HPP_
#define MC_POS_CONTROL_NODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <message_filters/subscriber.h>
#include <PositionControl.hpp>
#include <FlightTasks.hpp>

namespace mc_pos_control {

  class Node {
  public:
    static constexpr int default_publish_rate_ = 100;
    static constexpr int default_control_mode_ = 0;
    Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  private:
    ros::NodeHandle nh_;

    // subscribers
    ros::Subscriber subOdom_;
    ros::Subscriber subExtendedState_;
    ros::Subscriber subState_;
    ros::Subscriber subManualControl_;

    // publishers
    ros::Publisher pubAttitudeTarget_;

    // timestamps
    ros::Time prevStamp_;

    // implementation
    FlightTasks* flightTask_; /**< class generating position controller setpoints depending on vehicle task */
    PositionControl posControl_; /**< class for core PID position control */
    PositionControlStates states_; /**< structure containing vehicle state information for position control */
    ros::Timer pubTimer_;
    bool init_;
    bool armed_;

    //  callbacks
    void odomCallback(const nav_msgs::OdometryConstPtr&);
    void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr&);
    void stateCallback(const mavros_msgs::StateConstPtr&);
    void publishAttitudeTarget(const ros::TimerEvent&);
    void manualControlCallback(const mavros_msgs::ManualControl::ConstPtr&);
  };
} //  namespace mc_pos_control

#endif // MC_POS_CONTROL_NODE_HPP_