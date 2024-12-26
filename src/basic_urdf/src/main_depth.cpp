#include "basic_urdf/final_depth_test.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "final_depth_test");

  final_depth_test fdt;
  MakePoint mp_;

  ros::spin();

  return 0;
};
