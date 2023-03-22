#include "voxblox_skeleton/skeleton_server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxblox_skeletonizer_server");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  voxblox::SkeletonServer node(nh, nh_private);
  ros::spin();

  return 0;
}