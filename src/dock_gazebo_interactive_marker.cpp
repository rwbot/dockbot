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
std::string poseString(geometry_msgs::Pose pose, std::string label)
{
  // Convert Pose->Position to String
  std::ostringstream positionSS;
  positionSS << std::fixed << std::setprecision(2) << label << std::endl << "[ "<< pose.position.x << ",  " << pose.position.y << ",  " << pose.position.z << " ] ";
  // positionSS << std::fixed << std::setprecision(2) << "[ "<< pose.position.x << ",  " << pose.position.y << " ] ";

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
  marker.color.a = 0.169;//rgba.a;//1.0;
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

bool resetGazebo() {
  ROS_INFO_STREAM("resetGazebo() - CALLING SERVICE /gazebo/reset_world");
  std::string serviceName = "/gazebo/reset_world";

  gazebo_msgs::GetModelState getServiceMsg;
  if (ros::service::call(serviceName, getServiceMsg))
  {
    ROS_INFO_STREAM("resetGazebo() - SUCCESSFUL CALL TO reset_world SERVICE");
    return true;
  } else {
    ROS_WARN("resetGazebo() - FAILED TO RESET WORLD BY SERVICE CALL to /gazebo/reset_world ");
    return false;
  }
}

// %Tag(processFeedback)%
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // ROS_INFO_STREAM("PROCESS FEEDBACK -- PROCESS FEEDBACK -- PROCESS FEEDBACK");
  InteractiveMarker im;
  server->get(feedback->marker_name,im);
  std::string poseStr = poseString(feedback->pose, "GROUND TRUTH");
  im.description = poseStr;
  server->insert(im);

  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";

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
    switch (feedback->menu_entry_id){
      case(0):
        resetGazebo();
        break;
    }
    break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    // ROS_INFO_STREAM(s.str() << std::endl << poseStr);
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    // ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    // ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
    break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(setGazeboPose)%
void setGazeboPose( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  // ROS_INFO_STREAM("setGazeboPose: ATTEMPTING TO SET POSE BY SERVICE CALL to gazebo/SetModelState");
  geometry_msgs::Pose pose = feedback->pose;
  std::string serviceName = "/gazebo/set_model_state";

  gazebo_msgs::ModelState dockModelState;
  dockModelState.model_name = (std::string) "dock";
  dockModelState.pose = pose;

  gazebo_msgs::SetModelState setServiceMsg;
  setServiceMsg.request.model_state = dockModelState;

  if (ros::service::call(serviceName, setServiceMsg)) {
    // ROS_INFO_STREAM("setGazeboPose: SUCCESSFULLY SET POSE BY SERVICE CALL to gazebo/SetModelState");
    // ROS_INFO_STREAM("setGazeboPose: Dock pose: " << pose);
    // ROS_INFO("setGazeboPose: APPLYING CHANGES");
    InteractiveMarker im;
    server->get(feedback->marker_name, im);
    std::string poseStr = poseString(feedback->pose, "GROUND TRUTH");
    im.description = poseStr;
    server->insert(im);
    server->applyChanges();
  } else {
    ROS_WARN("setGazeboPose() FAILED TO SET POSE BY SERVICE CALL to gazebo/SetModelState");
  }

}
// %EndTag(setGazeboPose)%


bool getGazeboPose(geometry_msgs::Pose &pose) {

  ROS_INFO_STREAM("getGazeboPose() - CALLING SERVICE gazebo/GetModelState");
  std::string serviceName = "/gazebo/get_model_state";

  gazebo_msgs::GetModelState getServiceMsg;
  getServiceMsg.request.model_name = "dock";
  if (ros::service::call(serviceName, getServiceMsg))
  {
    ROS_INFO_STREAM("getGazeboPose() - SUCCESSFUL CALL TO GetModelState SERVICE");
    ROS_INFO_STREAM("getGazeboPose() - Dock pose: " << getServiceMsg.response.pose);
    pose = getServiceMsg.response.pose;
    return true;
  } else {
    ROS_WARN("getGazeboPose() - FAILED TO GET POSE BY SERVICE CALL to gazebo/GetModelState ");
    return false;
  }
}

// %Tag(alignMarker)%
void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO(" alignMarker- ");
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x - 0.5) + 0.5;
  pose.position.y = round(pose.position.y - 0.5) + 0.5;

  ROS_INFO_STREAM(feedback->marker_name << ":"
                                        << " aligning position = "
                                        << feedback->pose.position.x
                                        << ", " << feedback->pose.position.y
                                        << ", " << feedback->pose.position.z
                                        << " to "
                                        << pose.position.x
                                        << ", " << pose.position.y
                                        << ", " << pose.position.z);

  server->setPose(feedback->marker_name, pose);
  server->applyChanges();
}
// %EndTag(alignMarker)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(Menu)%
void makeMenuMarker(geometry_msgs::Pose pose)
{
  std_msgs::ColorRGBA green, red, color;
  green.a = red.a = green.g = red.r = 1.0;
  InteractiveMarker im;
  im.header.frame_id = "base_link";
  im.pose = pose;
  im.scale = 0.6;

  im.name = "context_menu";
  im.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = makeBox(im,color);
  control.markers.push_back(marker);
  control.always_visible = true;
  im.controls.push_back(control);

  server->insert(im);
  server->setCallback(im.name, &processFeedback);
  menu_handler.apply(*server, im.name);
}
// %EndTag(Menu)%

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
  im.description = poseString(pose, "GROUND TRUTH");

  double positionValue = pose.position.x + pose.position.y;
  if (positionValue == 0)
  {
    // make a box which also moves in the plane
    ROS_WARN_STREAM(" makeDockMarker- Zero Pose - Creating Red Zero Marker");
    color = red;
  }
  else
  {
    // make a box which also moves in the plane
    ROS_INFO_STREAM(" makeDockMarker- Pose Given - Creating Green Pose Marker");
    color = green;
  }

  makeBoxControl(im,color);
  // Make Box Control in Plane Translation
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

  // Set Orientation of Box Control Plane for XY Translation
  im.controls[0].orientation = control.orientation;


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

////////////////////////////////////////////////////////////////////////////////////

// %Tag(main)%
int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_interactive_marker");
  ros::NodeHandle n;
  ros::Duration halfSec(0.5);

  server.reset(new interactive_markers::InteractiveMarkerServer("dock_interactive_marker", "", false));

  // menu_handler.insert("Reset World", &processFeedback);
  // menu_handler.insert("(1,0.5,0)", &processFeedback);
  // menu_handler.insert("(1,  1,0)", &processFeedback);

  // make sure service is available before attempting to proceed, else node will crash
  bool getServiceReady, setServiceReady;
  getServiceReady = setServiceReady = false;
  ros::WallTime start = ros::WallTime::now();

  while (!getServiceReady && !getServiceReady) {
    getServiceReady = ros::service::exists("/gazebo/get_model_state", true);
    setServiceReady = ros::service::exists("/gazebo/set_model_state", true);
    ROS_INFO_STREAM(" waiting for set_model_state & get_model_state service");
    halfSec.sleep();
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration elapsed = end - start;
    ROS_WARN_STREAM( "dock_gazebo_interactive_marker " << elapsed.toSec() << " seconds elapsed waiting for set_model_state & get_model_state services");
    if ((ros::WallTime::now()-start).toSec() > 5.0)
    {
      ROS_WARN_STREAM(" Timeout waiting for set_model_state & get_model_state services");
      ROS_WARN_STREAM(" Unable to find model");
      break;
    }
  }

  if (getServiceReady && getServiceReady)
  {
    ROS_INFO(" set_model_state & get_model_state service exists");
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
    ROS_INFO_STREAM(" Pose Given - Setting Gazebo Pose");
  }
  else
  {
    makeDockMarker(poseZero,true);
    // makeMenuMarker(poseZero);
    ROS_INFO_STREAM(" No Pose Specified - Setting Zero Gazebo Pose");
  }

  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM(" main- applying changes to interactive marker server");
  server->applyChanges();

  ROS_INFO_STREAM(" main- ros::spin()");
  ros::spin();

  ROS_INFO_STREAM(" main- reseting interactive marker server");
  server.reset();
}
// %EndTag(main)%
