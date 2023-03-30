#include "voxblox_rrt_planner/voxblox_hl_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxblox_hl_planner");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // planner
    mav_planning::VoxbloxHlPlanner node(nh, nh_private);
    ROS_INFO("Initialized High Level Planner Voxblox node.");

    ros::spin();
    return 0;
}
