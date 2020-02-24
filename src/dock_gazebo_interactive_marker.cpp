#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <std_msgs/ColorRGBA.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%

// %Tag(poseString)%
std::string poseString(geometry_msgs::Pose pose) {
  // Convert Pose->Position to String
  std::ostringstream positionSS;
  positionSS << std::fixed << std::setprecision(2) << "[ "<< pose.position.x << ",  " << pose.position.y /*<< ",  " << pose.position.z*/ << " ] ";

  // Convert Pose->Orientation to String
  std::ostringstream quarternionSS;
  quarternionSS << std::fixed << std::setprecision(2) << "[ "<< pose.orientation.x << ",  " << pose.orientation.y << ",  " << pose.orientation.z << ",  " << pose.orientation.w << " ]";

  // Extract Yaw from Quarternion
  tf::Pose tfPose;
  tf::poseMsgToTF(pose, tfPose);
  double yaw = tf::getYaw(tfPose.getRotation());
  std::ostringstream yawSS;
  yawSS << std::fixed << std::setprecision(2) << "YAW: " << yaw << "\n";

  // Concatenate strings
  std::string poseString = positionSS.str() + yawSS.str() + quarternionSS.str();
  return poseString;
}
// %EndTag(poseString)%

// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg, std_msgs::ColorRGBA rgba) {
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = rgba.r;//0 .5;
  marker.color.g = rgba.g;//0.5;
  marker.color.b = rgba.b;//0.5;
  marker.color.a = rgba.a;//1.0;
  return marker;
}
InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg, std_msgs::ColorRGBA rgba)
{
  InteractiveMarkerControl control;
  control.name = "Box Control";
  control.always_visible = true;
  control.markers.push_back(makeBox(msg, rgba));
  msg.controls.push_back(control);

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(processFeedback)%
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

  InteractiveMarker im;
  server->get(feedback->marker_name,im);
  std::string poseStr = poseString(feedback->pose);
  im.description = poseStr;
  server->insert(im);

  std::ostringstream s;
  // s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";

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
    // make
    break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    ROS_INFO_STREAM(poseStr);
    break;

  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(setGazeboPose)%
void setGazeboPose( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;
  std::string serviceName = "/gazebo/set_model_state";

  gazebo_msgs::ModelState dockModelState;
  dockModelState.model_name = (std::string) "dock";
  dockModelState.pose = pose;

  gazebo_msgs::SetModelState setServiceMsg;
  setServiceMsg.request.model_state = dockModelState;

  if (ros::service::call(serviceName, setServiceMsg)) {
    // ROS_INFO_STREAM("SUCCESSFULLY SET POSE BY SERVICE CALL to gazebo/SetModelState");
    // ROS_INFO_STREAM("Dock pose: " << setServiceMsg.response);
  } else {
    ROS_WARN("FAILED TO SET POSE BY SERVICE CALL to gazebo/SetModelState");
  }
}
// %EndTag(setGazeboPose)%


// %Tag(getGazeboPose)%
bool getGazeboPose(geometry_msgs::Pose &pose) {

  ROS_INFO_STREAM("CALLING SERVICE gazebo/GetModelState");
  std::string serviceName = "/gazebo/get_model_state";

  gazebo_msgs::GetModelState getServiceMsg;
  getServiceMsg.request.model_name = "dock";
  if (ros::service::call(serviceName, getServiceMsg))
  {
    ROS_INFO_STREAM("SUCCESSFUL CALL TO GetModelState SERVICE");
    ROS_INFO_STREAM("Dock pose: " << getServiceMsg.response.pose);
    pose = getServiceMsg.response.pose;
    return true;
  } else {
    ROS_WARN("FAILED TO GET POSE BY SERVICE CALL to gazebo/GetModelState ");
    return false;
  }
}
// %EndTag(getGazeboPose)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(Dock)%
void makeDockMarker(geometry_msgs::Pose pose) {
  std_msgs::ColorRGBA green, red;
  green.a = red.a = green.g = red.r = 1.0;
  InteractiveMarker im;
  im.header.frame_id = "odom";
  im.pose = pose;
  im.scale = 0.6;

  im.name = "dock";
  im.description = poseString(pose);

  InteractiveMarkerControl control;



  // Set Quarternion Aligned With Z Axis (Yes, the quarternion is confusing)
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation); // Specify Rotation Axis
  control.name = "ROTATE YAW";
  // Set Control To Rotate About Specified Axis
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  // Send Rotation
  im.controls.push_back(control);



  // Send Translation
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  // control.independent_marker_orientation = true;
  control.name = "TRANSLATE IN X-Y PLANE";


  double positionValue = pose.position.x + pose.position.y;
  if (positionValue == 0) {
    // make a box which also moves in the plane
    ROS_WARN_STREAM("makeDockMarker: Zero Pose - Creating Zero Marker");
    control.markers.push_back(makeBox(im, red));
  } else {
    // make a box which also moves in the plane
    ROS_INFO_STREAM("makeDockMarker: Pose Given - Creating Green Marker");
    control.markers.push_back(makeBox(im, green));
  }

  control.always_visible = true;
  im.controls.push_back(control);

  // we want to use our special callback function
  server->insert(im);
  server->setCallback(im.name, &processFeedback);

  // Callback to update Gazebo pose
  // set different callback for POSE_UPDATE feedback
  server->setCallback(im.name, &setGazeboPose, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}
// %EndTag(Dock)%

// %Tag(Dock)%
void makeDockMarker(geometry_msgs::Pose pose, bool fixed6DoF = false)
{
  std_msgs::ColorRGBA green, red, color;
  green.a = red.a = green.g = red.r = 1.0;
  InteractiveMarker im;
  im.header.frame_id = "odom";
  im.pose = pose;
  im.scale = 0.6;

  im.name = "dock";
  im.description = poseString(pose);

  double positionValue = pose.position.x + pose.position.y;
  if (positionValue == 0)
  {
    // make a box which also moves in the plane
    ROS_WARN_STREAM("makeDockMarker: Zero Pose - Creating Red Zero Marker");
    color = red;
  }
  else
  {
    // make a box which also moves in the plane
    ROS_INFO_STREAM("makeDockMarker: Pose Given - Creating Green Pose Marker");
    color = green;
  }

  makeBoxControl(im,color);
  // Make Box Control XY Translation
  im.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

  InteractiveMarkerControl control;

  // Set Quarternion Aligned With Z Axis (Yes, the quarternion is confusing)
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation); // Specify Rotation Axis
  control.name = "ROTATE YAW";
  // Set Control To Rotate About Specified Axis
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  // control.always_visible = true;
  // Send Rotation
  im.controls.push_back(control);

  if (fixed6DoF)
  {
    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "TRANSLATE X";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::FIXED;
    im.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "TRANSLATE Y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::FIXED;
    im.controls.push_back(control);
  }

  control.always_visible = true;
  im.controls.push_back(control);

  // we want to use our special callback function
  server->insert(im);
  server->setCallback(im.name, &processFeedback);

  // Callback to update Gazebo pose
  // set different callback for POSE_UPDATE feedback
  server->setCallback(im.name, &setGazeboPose, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}
// %EndTag(Dock)%

// %Tag(Menu)%
void makeMenuMarker(geometry_msgs::Pose pose)
{
  InteractiveMarker im;
  im.header.frame_id = "odom";
  im.pose.position = pose.position;
  im.scale = 0.6;

  im.name = "Dock Menu";
  im.description = "Dock Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "Dock Menu Control";

  std_msgs::ColorRGBA color;
  color.a = color.b = 0.5;
  Marker marker = makeBox(im, color);
  control.markers.push_back(marker);
  control.always_visible = true;
  im.controls.push_back(control);

  server->insert(im);
  server->setCallback(im.name, &processFeedback);

  // Callback to update Gazebo pose
  // set different callback for POSE_UPDATE feedbackS
  server->setCallback(im.name, &setGazeboPose, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);

  menu_handler.apply(*server, im.name);
}
// %EndTag(Menu)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(main)%
int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_interactive_marker");
  ros::NodeHandle n;
  ros::Duration halfSec(0.5);

  server.reset(new interactive_markers::InteractiveMarkerServer("dock_interactive_marker", "", false));

  menu_handler.insert("(1,  0,0)", &processFeedback);
  menu_handler.insert("(1,0.5,0)", &processFeedback);
  menu_handler.insert("(1,  1,0)", &processFeedback);

  // make sure service is available before attempting to proceed, else node will crash
  bool getServiceReady, setServiceReady;
  getServiceReady = setServiceReady = false;
  ros::WallTime start = ros::WallTime::now();

  while (!getServiceReady && !getServiceReady) {
    getServiceReady = ros::service::exists("/gazebo/get_model_state", true);
    setServiceReady = ros::service::exists("/gazebo/set_model_state", true);
    ROS_INFO_STREAM("dock_gazebo_interactive_marker: waiting for set_model_state & get_model_state service");
    halfSec.sleep();
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration elapsed = end - start;
    ROS_WARN_STREAM( "dock_gazebo_interactive_marker " << elapsed.toSec() << " seconds elapsed waiting for set_model_state & get_model_state services");
    if ((ros::WallTime::now()-start).toSec() > 5.0)
    {
      ROS_WARN_STREAM("dock_gazebo_interactive_marker: Timeout waiting for set_model_state & get_model_state services");
      ROS_WARN_STREAM("dock_gazebo_interactive_marker: Unable to find model");
      break;
    }
  }

  if (getServiceReady && getServiceReady)
  {
    ROS_INFO("dock_gazebo_interactive_marker: set_model_state & get_model_state service exists");
  }

  geometry_msgs::Pose poseDefault, poseZero, poseX1Y1, poseX1Yp5;
  tf::Quaternion q(0.0, 0.0, 1.0, 1.0);
  q.normalize();
  tf::quaternionTFToMsg(q, poseDefault.orientation); // Specify Rotation Axis
  poseZero = poseX1Y1 = poseX1Yp5 = poseDefault;
  poseDefault.position.x = 1.0;

  geometry_msgs::Pose getPose;
  if (getGazeboPose(getPose))
  {
    makeDockMarker(getPose,true);
    // makeMenuMarker(getPose);
    ROS_INFO_STREAM("dock_gazebo_interactive_marker: Pose Given - Setting Gazebo Pose");
  }
  else
  {
    makeDockMarker(poseZero,true);
    // makeMenuMarker(poseZero);
    ROS_INFO_STREAM("dock_gazebo_interactive_marker: No Pose Specified - Setting Zero Gazebo Pose");
  }

  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM("main: applying changes to interactive marker server");
  server->applyChanges();

  ros::spin();

  server.reset();
}
// %EndTag(main)%
