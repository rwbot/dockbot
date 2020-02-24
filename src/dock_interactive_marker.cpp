#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <math.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%

// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg) {
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent &) {
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter) / 140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter) / 140.0, 0.0));
  br.sendTransform(
      tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", "
                   << feedback->mouse_point.y << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id
                            << " clicked" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    ROS_INFO_STREAM(s.str()
                    << ": pose changed"
                    << "\nposition = " << feedback->pose.position.x << ", "
                    << feedback->pose.position.y << ", "
                    << feedback->pose.position.z
                    << "\norientation = " << feedback->pose.orientation.w
                    << ", " << feedback->pose.orientation.x << ", "
                    << feedback->pose.orientation.y << ", "
                    << feedback->pose.orientation.z
                    << "\nframe: " << feedback->header.frame_id
                    << " time: " << feedback->header.stamp.sec << "sec, "
                    << feedback->header.stamp.nsec << " nsec");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
    break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(Dock)%
void makeDockMarker(const tf::Vector3 &position) {
  InteractiveMarker im;
  im.header.frame_id = "base_link";
  tf::pointTFToMsg(position, im.pose.position);
  im.scale = 1;

  im.name = "dock";
  im.description = "DOCK";

  InteractiveMarkerControl control;

  // Set Control To Rotate About Specified Axis
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  // Set Quarternion Aligned With Z Axis (Yes, the quarternion is confusing)
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation); // Specify Rotation Axis
  // Send Rotation
  im.controls.push_back(control);

  // Send Translation
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  im.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back(makeBox(im));
  control.always_visible = true;
  im.controls.push_back(control);

  // we want to use our special callback function
  server->insert(im);
  server->setCallback(im.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  //   server->setCallback(im.name, &alignMarker,
  //   visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}
// %EndTag(Dock)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(main)%
int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_interactive_marker");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer("dock_interactive_marker", "", false));

  ros::Duration(0.1).sleep();

//   menu_handler.insert("First Entry", &processFeedback);
//   menu_handler.insert("Second Entry", &processFeedback);
//   interactive_markers::MenuHandler::EntryHandle sub_menu_handle =
//       menu_handler.insert("Submenu");
//   menu_handler.insert(sub_menu_handle, "First Entry", &processFeedback);
//   menu_handler.insert(sub_menu_handle, "Second Entry", &processFeedback);

  tf::Vector3 position;
  position = tf::Vector3(1, 1, 0);
  makeDockMarker(position);

  server->applyChanges();

  ros::spin();

  server.reset();
}
// %EndTag(main)%
