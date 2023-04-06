#pragma once

#include <functional>

#include <interactive_markers/interactive_marker_server.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

namespace mav_planning_rviz {

class InspectionMarker {
public:
  typedef std::function<void(const mav_msgs::EigenTrajectoryPoint &pose)>
      PoseUpdatedFunctionType;

  InspectionMarker(const ros::NodeHandle &nh);
  ~InspectionMarker() {}

  void setFrameId(const std::string &frame_id);
  // Bind callback for whenever pose updates.

  void setPoseUpdatedCallback(const PoseUpdatedFunctionType &function) {
    pose_updated_function_ = function;
  }

  void enableSetPoseMarker(const mav_msgs::EigenTrajectoryPoint &pose);

  void initialize();

  void setPose(const mav_msgs::EigenTrajectoryPoint &pose);

  void processSetPoseFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
  // Creates markers without adding them to the marker server.
  void createMarkers();

  // ROS stuff.
  ros::NodeHandle nh_;
  interactive_markers::InteractiveMarkerServer marker_server_;

  // Settings.
  std::string frame_id_;
  std::string id_ = "inspection_marker";

  // State.
  bool initialized_;
  visualization_msgs::InteractiveMarker set_pose_marker_;

  // State:
  PoseUpdatedFunctionType pose_updated_function_;
};

} // end namespace mav_planning_rviz
