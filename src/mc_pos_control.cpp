#include <Node.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mc_pos_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  mc_pos_control::Node node(nh, pnh);
  ros::spin();
  return 0;
}