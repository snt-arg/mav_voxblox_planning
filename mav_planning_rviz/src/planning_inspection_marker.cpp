#include <mav_visualization/helpers.h>

#include "mav_planning_rviz/planning_inspection_marker.h"

namespace mav_planning_rviz {

InspectionMarker::InspectionMarker(const ros::NodeHandle &nh)
    : nh_(nh), marker_server_("inspection_marker"), frame_id_("map"),
      initialized_(false) {}

void InspectionMarker::setFrameId(const std::string &frame_id) {
  frame_id_ = frame_id;
  set_pose_marker_.header.frame_id = frame_id_;
}

void InspectionMarker::initialize() {
  createMarkers();
  enableSetPoseMarker({});
  initialized_ = true;
}

void InspectionMarker::enableSetPoseMarker(
    const mav_msgs::EigenTrajectoryPoint &pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  set_pose_marker_.pose = pose_stamped.pose;

  marker_server_.insert(set_pose_marker_);
  marker_server_.setCallback(
      set_pose_marker_.name,
      boost::bind(&InspectionMarker::processSetPoseFeedback, this, _1));
  marker_server_.applyChanges();
}

void InspectionMarker::createMarkers() {
  // First we set up the set point marker.
  set_pose_marker_.header.frame_id = frame_id_;
  set_pose_marker_.name = id_;
  set_pose_marker_.scale = 1.0;
  set_pose_marker_.controls.clear();

  const double kSqrt2Over2 = std::sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_pose_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.name = "rotate_yaw";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  set_pose_marker_.controls.push_back(control);
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move z";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = kSqrt2Over2;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = kSqrt2Over2;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = 0.9239;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.3827;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x_y";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = 0.3827;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.9239;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y_x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.x = control.orientation.y = control.orientation.z = 0.0;
  control.orientation.w = 1.0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  control.markers.push_back(marker);
  control.always_visible = true;
  control.name = "center";
  set_pose_marker_.controls.push_back(control);

  // marker_server_.insert(set_pose_marker_);
  // marker_server_.setCallback(
  //     set_pose_marker_.name,
  //     boost::bind(&InspectionMarker::processSetPoseFeedback, this, _1));
  // marker_server_.applyChanges();
}

void InspectionMarker::setPose(const mav_msgs::EigenTrajectoryPoint &pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  set_pose_marker_.pose = pose_stamped.pose;
  marker_server_.setPose(set_pose_marker_.name, set_pose_marker_.pose);
  marker_server_.applyChanges();
}

void InspectionMarker::processSetPoseFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    if (pose_updated_function_) {
      mav_msgs::EigenTrajectoryPoint pose;
      mav_msgs::eigenTrajectoryPointFromPoseMsg(feedback->pose, &pose);
      pose_updated_function_(pose);
    }
  }

  marker_server_.applyChanges();
}

} // end namespace mav_planning_rviz
