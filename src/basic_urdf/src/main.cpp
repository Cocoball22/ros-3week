#include "/home/cona/catkin_ws/src/basic_urdf/include/basic_urdf/test.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_point");

  MakePoint mp_;

  ros::spin();

  return 0;
}
