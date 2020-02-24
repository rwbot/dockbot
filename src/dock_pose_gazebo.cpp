#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

void poseCallback(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;
  tf::Transform t;
  ros::Time time = ros::Time::now();
  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "world", "dock_truth"));
}

void getGazeboPose(ros::ServiceClient& getClient, geometry_msgs::Pose& pose) {
  ROS_INFO_STREAM("CALLING SERVICE gazebo/GetModelState");
  gazebo_msgs::GetModelState getServiceMsg;
  getServiceMsg.request.model_name = "dock";
  getClient.call(getServiceMsg);
  if (getServiceMsg.response.success) 
  {
    ROS_INFO_STREAM("SUCCESSFUL CALL TO GetModelState SERVICE");
    ROS_INFO_STREAM("Dock pose: " << getServiceMsg.response.pose);
    pose = getServiceMsg.response.pose;
  } 
  else 
  {
    ROS_WARN("FAILED TO GET POSE BY SERVICE CALL to gazebo/GetModelState ");
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

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dock_gazebo_pose");
  ROS_INFO_STREAM("INITIALIZED DOCK_GAZEBO_POSE NODE");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  // ros::Timer timer = n.createTimer(ros::Duration(0.01), poseCallback);

  ros::Publisher dockPosePub = n.advertise<geometry_msgs::Pose>("dock_gazebo_pose", 100);
  ros::Subscriber sub = n.subscribe("/chatter", 100, setGazeboPose);

  ros::ServiceClient getClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  // ros::ServiceClient setClient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  while(ros::ok())
  {
    geometry_msgs::Pose getPose; 
    getGazeboPose(getClient,getPose);
    dockPosePub.publish(getPose);
  }

  ros::spin();

}
// %EndTag(main)%