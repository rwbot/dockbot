#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
ros::Publisher dockPosePub;
ros::ServiceClient getClient;
// static tf::TransformBroadcaster br;
// void poseCallback(const ros::TimerEvent&)
// {
//   static tf::TransformBroadcaster br;
//   tf::Transform t;
//   ros::Time time = ros::Time::now();
//   t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//   t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
//   br.sendTransform(tf::StampedTransform(t, time, "world", "dock_truth"));
// }

void broadcastTF(geometry_msgs::PoseStamped& pose)
{
  static tf::TransformBroadcaster br;
  tf::Transform tf;
  ros::Time time = ros::Time::now();
  tf::poseMsgToTF(pose.pose, tf);
  geometry_msgs::TransformStamped tfStampedMsg;
  
  tf::StampedTransform tfs(tf, time, "odom", "dock_truth");
  tf::transformStampedTFToMsg(tfs, tfStampedMsg);

  br.sendTransform(tfs);
  // ROS_INFO_STREAM("broadcastTF -- Transform \n" << tfStampedMsg);
  dockPosePub.publish(pose);
}

void getGazeboPose(ros::ServiceClient& getClient, geometry_msgs::PoseStamped& pose) {
  // ROS_INFO_STREAM("CALLING SERVICE gazebo/GetModelState");
  ros::Duration secs3(3.0);
  gazebo_msgs::GetModelState getServiceMsg;
  getServiceMsg.request.model_name = "dock";
  getClient.call(getServiceMsg);
  if (getServiceMsg.response.success) 
  {
    // ROS_INFO_STREAM("SUCCESSFUL CALL TO GetModelState SERVICE");
    // ROS_INFO_STREAM("Dock pose: " << getServiceMsg.response.pose);
    pose.header.stamp = ros::Time(0);
    pose.pose = getServiceMsg.response.pose;
    // broadcastTF(pose);
  } 
  else 
  {
    ROS_WARN("FAILED TO GET POSE BY SERVICE CALL to gazebo/GetModelState ");
    secs3.sleep();
  }
    
}

void setGazeboPose(const geometry_msgs::Pose::ConstPtr &pose) {
  std::string serviceName = "/gazebo/set_model_state";

  gazebo_msgs::ModelState dockModelState;
  dockModelState.model_name = (std::string) "dock";
  dockModelState.pose = *pose;

  gazebo_msgs::SetModelState setServiceMsg;
  setServiceMsg.request.model_state = dockModelState;

  if (ros::service::call(serviceName, setServiceMsg)) {
    ROS_INFO_STREAM("SUCCESSFULLY SET POSE BY SERVICE CALL to gazebo/SetModelState");
    // ROS_INFO_STREAM("Dock pose: " << setServiceMsg.response);
  } 
  else 
  {
    ROS_WARN("FAILED TO SET POSE BY SERVICE CALL to gazebo/SetModelState");
  }
  
}

void outputTF()
{
  geometry_msgs::PoseStamped getPose;
  getGazeboPose(getClient, getPose);
  getPose.header.frame_id = "odom";
  dockPosePub.publish(getPose);
  broadcastTF(getPose);
}

void timerCallback(const ros::TimerEvent&)
{
  // ROS_INFO_STREAM("timerCallback: ");
  outputTF();
}



// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dock_pose_gazebo");
  ROS_INFO_STREAM("dock_pose_gazebo: INITIALIZED DOCK_POSE_GAZEBO NODE");
  ros::NodeHandle n;
  ros::Duration wait(2.0);
  wait.sleep();
  std::string getClientName = "/gazebo/get_model_state";
  // ros::Publisher dockPosePub = n.advertise<geometry_msgs::PoseStamped>("dock_pose_gazebo", 100);
  // ros::ServiceClient getClient = n.serviceClient<gazebo_msgs::GetModelState>(getClientName);
  dockPosePub = n.advertise<geometry_msgs::PoseStamped>("dock_pose_gazebo", 100);
  getClient = n.serviceClient<gazebo_msgs::GetModelState>(getClientName);

  // ros::Subscriber sub = n.subscribe("/chatter", 100, setGazeboPose);

  
  ROS_INFO_STREAM("dock_pose_gazebo: WAITING FOR SERVICE " << getClientName);
  ros::service::waitForService(getClientName);
  ROS_INFO_STREAM("dock_pose_gazebo: SERVICE DISCOVERED " << getClientName);

  // ros::ServiceClient setClient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // while(!ros::service::exists(getClientName)){
  // }
  // create a timer to update the published transforms
  ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
  ros::spin();
  // ros::Rate rate(20);
  // while(ros::ok())
  // {
  //   ROS_INFO_STREAM("main.while(): ");
  //   outputTF();
  //   // ros::spin();
  //   rate.sleep();
  // }

  

}
// %EndTag(main)%
