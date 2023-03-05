#include "probot_grasping/grasping_demo.hpp"
#include "math.h"
#include <stdio.h>

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth) :
    it_(n_), 
    armgroup("arm"), 
    grippergroup("gripper"), 
    vMng_(length, breadth)
{
  this->nh_ = n_;

  try
  {
    this->tf_camera_to_robot.waitForTransform("/star", "/camera_link", ros::Time(0), ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  try
  {
    this->tf_camera_to_robot.lookupTransform("/star", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;
  
  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(5.0).sleep();
  ROS_INFO_STREAM("Getting into the Grasping Position....");

    armgroup.setGoalPositionTolerance(0.001);
    armgroup.setGoalOrientationTolerance(0.001);

    //设置允许的最大速度和加速度
    armgroup.setMaxAccelerationScalingFactor(0.4);
    armgroup.setMaxVelocityScalingFactor(0.4);
 this->client= this->nh_.serviceClient<probot_grasping::picture_tf>("picture_tf");  //.创建 客户端 对象
   attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
}
void GraspingDemo::imageCb(float x,float y,float z )//处理图像对于摄像头的位置信息，转化为物体与机械臂位置信息
{
    obj_camera_frame.setZ(-y);
    obj_camera_frame.setY(-x);
    obj_camera_frame.setX(z);
    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }


void GraspingDemo::attainPosition(float x, float y, float z)
{
  // ROS_INFO("The attain position function called");

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
 
 go2goal(x,y,z);
  // armgroup.setPoseTarget(target_pose1);

  // armgroup.move();
}

void GraspingDemo::attainObject()
{
  // ROS_INFO("The attain Object function called");
  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ() + 0.04);

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  // Slide down the Object
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;

  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  target_pose1.position.z = obj_robot_frame.getZ() - 0.02;
  
go2goal(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
}

void GraspingDemo::grasp()
{
  // ROS_INFO("The Grasping function called");

  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("close");
  grippergroup.move();
}

void GraspingDemo::lift()
{
  // ROS_INFO("The lift function called");

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  // Starting Postion after picking
  //target_pose1.position.z = target_pose1.position.z + 0.06;

  if(rand() % 2)
  {
    target_pose1.position.y = target_pose1.position.y + 0.02;
  }
  else
  {
    target_pose1.position.y = target_pose1.position.y - 0.02;
  }
  
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  target_pose1.position.z = target_pose1.position.z + 0.06;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::goHome()
{
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  // Go to Home Position
  attainPosition(homePose.pose.position.x, homePose.pose.position.y, homePose.pose.position.z);
}

void GraspingDemo::initiateGrasping()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(3.0).sleep();

  homePose = armgroup.getCurrentPose();
  
  ROS_INFO_STREAM("Approaching the Object....");
  attainObject();

  ROS_INFO_STREAM("Attempting to Grasp the Object now..");
  grasp();

  // ROS_INFO_STREAM("Lifting the Object....");
  // lift();

  ROS_INFO_STREAM("Going back to home position....");
  goHome();

  grasp_running = false;
}

bool GraspingDemo::send_date2camerer(float andy)
{
picture_size_kehu.request.num1=(_Float64)andy;
picture_size_kehu.request.num2=(_Float64)0;
ROS_INFO_STREAM("printf date ");
 bool flag;
do
{
    flag = client.call(picture_size_kehu);
} while (!flag);
return flag;
}

 bool GraspingDemo::go_or_nogo(void)
 {
    if(grasp_running ==true)
    {
        grasp_running=false;
        return true;
    }
    else
    {
        return false;
    }
 }

  float GraspingDemo::help_rad(float a,float b ,float c)
  {
	float mom=sqrt((pow(a,2)+pow(b,2)));
	float res=asin(c/mom);
	float delt=atan2(b,a);
	float rad=res-delt;
	return rad;
  }

  void GraspingDemo::go2goal(double x,double y,double z)
  {

   double (&v)[4]=inverseKinematics(y*100, x*100, z*100);  
    static std::vector<double> joint_group_positions(6);
    joint_group_positions[0]=RAD2ANG*(270-v[0]);
    joint_group_positions[1]=RAD2ANG*(90-v[1]);
    joint_group_positions[2]=RAD2ANG*(-v[2]);
    joint_group_positions[3]=RAD2ANG*(-v[3]);
    joint_group_positions[4]=-3.17;
    // joint_group_positions[5]=0;
    armgroup.setJointValueTarget(joint_group_positions);
    armgroup.move();
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_grasping");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;

  if (!n.getParam("probot_grasping/table_length", length))
    length = 0.3;
  if (!n.getParam("probot_grasping/table_breadth", breadth))
    breadth = 0.4;
  if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x))
    pregrasp_x = 0.20;
  if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y))
    pregrasp_y = -0.17;
  if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z))
    pregrasp_z = 0.28;

  GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);

  ros::service::waitForService("picture_tf");
bool flag=simGrasp.send_date2camerer(1);

  while (ros::ok())
  {


     if (flag)
     {
         simGrasp.imageCb(simGrasp.picture_size_kehu.response.place_x,simGrasp.picture_size_kehu.response.place_y,simGrasp.picture_size_kehu.response.place_z);  
     }
     if(simGrasp.go_or_nogo())
     {
         simGrasp.initiateGrasping();
     }
  ros::spinOnce();

  }
  return 0;
}

