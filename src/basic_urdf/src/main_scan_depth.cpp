#include <basic_urdf/scan_depth.hpp>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "scan_depth");

    scan_depth sd_;

    ros::spin();

    return 0;

}

