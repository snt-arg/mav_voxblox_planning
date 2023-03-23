#include "voxblox_rrt_planner/voxblox_sparse_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxblox_sparse_planner");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // planner
    mav_planning::VoxbloxSparsePlanner node(nh, nh_private);
    ROS_INFO("Initialized Sprase Planner Voxblox node.");

    ros::spin();
    return 0;
}
