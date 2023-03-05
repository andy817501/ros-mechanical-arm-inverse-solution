/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef PROBOT_GRASPING_DEMO
#define PROBOT_GRASPING_DEMO
#define RAD2ANG (3.1415926535898/180.0)
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include "probot_grasping/picture_tf.h"
#include "probot_grasping/vision_manager.h"
#include "math.h"

class GraspingDemo
{
  private:
	/**
 	 * @brief NodeHandle of the current node
 	 */
	ros::NodeHandle nh_;
	/**
     * @brief target_pose_1 is the pose to moveit!
     */
	geometry_msgs::Pose target_pose1;
	/**
     * @brief armgroup moveit interface for arm
     */
	moveit::planning_interface::MoveGroupInterface armgroup;
	/**
     * @brief grippegroup moveit interface for gripper
     */
	moveit::planning_interface::MoveGroupInterface grippergroup;
	/**
     * @brief it_ takes care of message to image conversion
     */
	image_transport::ImageTransport it_;
	/**
     * @brief image_sub_ subscribes to image/raw topic
     */
	image_transport::Subscriber image_sub_;
	/**
     * @brief boolean to control the grasping movements
     */

	bool grasp_running;
	/**
     * @brief cv_ptr is the pointer to image as received by it_
     */

	cv_bridge::CvImagePtr cv_ptr;
	/**
     * @brief vMng_ is the instance of the library for object detection
     */

	VisionManager vMng_;
	/**
	 * @brief camera_to_robot takes care of the transformation from camera to robot frame
	 */

	tf::StampedTransform camera_to_robot_;
	/**
	 * @brief tf_camera_to_robot is an instance of tf_listener
	 */

	tf::TransformListener tf_camera_to_robot;
	/**
	 * @brief obj_camera_frame, obj_robot_frame are instance of tf::Vector
	 */

	tf::Vector3 obj_camera_frame, obj_robot_frame;
	/**
	 * @brief homePose is StampedPose keeping Home position of arm
	 */

	geometry_msgs::PoseStamped homePose;
	/**
	 * @brief my_plan is an instance of moveit! planning interface
	 */

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	/**
	 * @brief pregrasp_x, pregrasp_y and pregrasp_z is the pregrasp position of arm
	 */



	float pregrasp_x, pregrasp_y, pregrasp_z;
	/**
	 * @brief      attainPosition achieved the given position of the arm
	 *
	 * @param[in]  x     x-position of gripping frame	
	 * @param[in]  y     y-position of gripping frame	
	 * @param[in]  z     z-position of gripping frame
	 */

	void attainPosition(float x, float y, float z);
	/**
	 * @brief      attainObject tries to get the gripper next to object
	 */
	void attainObject();
	/**
	 * @brief      grasp executes the grasping action
	 */
	void grasp();
	/**
	 * @brief      lift attempts to lift the object
	 */
	void lift();
	 double v1[4]={0,0,0,0};
    double v2[4]={0,0,0,0};
	 ros::ServiceClient client;
decltype(v1) &inverseKinematics(double x, double y, double z)
{
		double a, b, c;
	double L1 = 12.98, L2 = 14.688, L3 = 18.031;//3节手臂的长度
	double m, n, t, q, p;
	double j1, j2, j3, j0;//4个舵机的旋转角度
	double x1, y1, z1;        //逆解后正解算出来的值，看是否与逆解值相等
	char i = 0;
	j0 = atan2(y, x);
	a = x / cos(j0);
	if (x == 0) a = y; //如果x为0，需要交换x，y
	b = z;

	for (j3 = 90; j3 >= -90; j3--)
	{
		j3 *= RAD2ANG;

         float rx=-cos(j3)*(2*L3*L2)+pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2);
        j1=help_rad(2*a*L1,2*b*L1,rx);

		m = L2 * sin(j1) + L3 * sin(j1)*cos(j3) + L3 * cos(j1)*sin(j3);
		n = L2 * cos(j1) + L3 * cos(j1)*cos(j3) - L3 * sin(j1)*sin(j3);
		t = a - L1 * sin(j1);
		p = pow(pow(n, 2) + pow(m, 2), 0.5);
		q = asin(m / p);
		j2 = asin(t / p) - q;


		x1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*cos(j0);
		y1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*sin(j0);
		z1 = L1 * cos(j1) + L2 * cos(j1 + j2) + L3 * cos(j1 + j2 + j3);
		j1 = ANG2RAD(j1);
		j2 = ANG2RAD(j2);
		j3 = ANG2RAD(j3);
		if (x1<(x + 1) && x1 >(x - 1) && y1<(y + 1) && y1 >(y - 1) && z1<(z + 1) && z1 >(z - 1))
		{
			 printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
                v1[0]=ANG2RAD(j0);
                v1[1]=j1;
                v1[2]=j2;
                v1[3]=j3;


            break;
			i = 1;
		}
	}

	for (j3 = 90; j3 >= -90; j3--)
	{
		j3 *= RAD2ANG;

         float rx=-cos(j3)*(2*L3*L2)+pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2);
        j1=help_rad(2*a*L1,2*b*L1,rx);
		m = L2 * sin(j1) + L3 * sin(j1)*cos(j3) + L3 * cos(j1)*sin(j3);
		n = L2 * cos(j1) + L3 * cos(j1)*cos(j3) - L3 * sin(j1)*sin(j3);
		t = a - L1 * sin(j1);
		p = pow(pow(n, 2) + pow(m, 2), 0.5);
		q = asin(m / p);
		j2 = -(asin(t / p) - q);
		//if (abs(ANG2RAD(j2)) >= 135) { j1 = ANG2RAD(j1); continue; }
		x1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*cos(j0);
		y1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*sin(j0);
		z1 = L1 * cos(j1) + L2 * cos(j1 + j2) + L3 * cos(j1 + j2 + j3);
		j1 = ANG2RAD(j1);
		j2 = ANG2RAD(j2);
		j3 = ANG2RAD(j3);
		if (x1<(x + 1) && x1 >(x - 1) && y1<(y + 1) && y1 >(y - 1) && z1<(z + 1) && z1 >(z - 1))
		{
			 printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
			i = 1;
                v2[0]=ANG2RAD(j0);
                v2[1]=j1;
                v2[2]=j2;
                v2[3]=j3;
            break;
		}
	}

	if (i == 0)
    {printf("无解");
    }
    double dleat1=90-(v1[3]);
    double dleat2=90-(v2[3]);
    if(dleat1<=dleat2)
    {
        return v1;
    }
    else{
        return v2;
    }
}

float help_rad(float a,float b ,float c);
	
  public:
	/**
 	 * @brief      GraspingDemo behaviour Constructor
 	 *
 	 * @param[in]  n_          ros_NodeHandle
 	 * @param[in]  pregrasp_x  Desired PregraspingX
 	 * @param[in]  pregrasp_y  Desired PregraspingY
 	 * @param[in]  pregrasp_z  Desired PregraspingZ
 	 * @param[in]  length      The length of table
 	 * @param[in]  breadth     The breadth of table
 	 */
		 probot_grasping::picture_tf picture_size_kehu;
	GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length = 1, float breadth = 0.6);
	/**
	 * @brief      imageCb is called when a new image is received from the camera
	 *
	 * @param[in]  msg   Image received as a message
	 */
	
	void imageCb(float x,float y,float z );
	/**
	 * @brief      initiateGrasping initiates the grasping behaviour
	 */
	void initiateGrasping();
	/**
	 * @brief      Function brings the arm back to home configuration
	 */
	void goHome();
	bool send_date2camerer(float andy);
	 bool  go_or_nogo(void);

};
	 double v1[4]={0,0,0,0};
    double v2[4]={0,0,0,0};
#endif