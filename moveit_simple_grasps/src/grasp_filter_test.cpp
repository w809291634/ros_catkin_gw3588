/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Tests the grasp generator filter
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Baxter specific properties
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/custom_environment2.h>
#include <iostream>
#include "moveit_simple_grasps/GenerateSolutions.h"


namespace moveit_simple_grasps
{
  // Table dimensions
  static const double TABLE_HEIGHT = .92;
  static const double TABLE_WIDTH = .85;
  static const double TABLE_DEPTH = .47;
  static const double TABLE_X = 0.66;
  static const double TABLE_Y = 0;
  static const double TABLE_Z = -0.9/2+0.01;
  static const double BLOCK_SIZE = 0.025;

  class GraspGeneratorTest
  {
    private:
      // 抓取产生器
      static moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;
      // 在Rviz中可视化事物的工具
      static moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
      // 定义抓取过滤
      static  moveit_simple_grasps::GraspFilterPtr grasp_filter_;
      // 用于生成抓取的数据
      static moveit_simple_grasps::GraspData grasp_data_;
      static planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

      static std::string arm_;
      static std::string ee_group_name_;
      static std::string planning_group_name_;

    public:
      // 共享节点句柄
      ros::NodeHandle nh_;
    
      // 构造函数
      GraspGeneratorTest(): nh_("~")
      {
        // 从param服务器获取arm信息
        nh_.param("arm", arm_, std::string("manipulator"));
        nh_.param("ee_group_name", ee_group_name_, std::string("gripper"));
        planning_group_name_ = arm_ ;

        ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
        ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
        ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

        // ---------------------------------------------------------------------------------------------
        // 从此节点的参数服务器中获取数据
        if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
          ros::shutdown();
      }

      static bool GraspGenerator(moveit_simple_grasps::GenerateSolutions::Request &req  ,moveit_simple_grasps::GenerateSolutions::Response &res)
      { 
        int num_tests = 1;                              //测试次数
        // ---------------------------------------------------------------------------------------------
        // Load planning scene to share
        planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

        // ---------------------------------------------------------------------------------------------
        // 加载Robot Viz工具以发布到Rviz
        const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_, "/Block", planning_scene_monitor_));
        visual_tools_->setLifetime(3);                  //设置物体存在时间
        // visual_tools_->loadEEMarker(ee_jmg);

        // 清除旧的碰撞对象
        visual_tools_->removeAllCollisionObjects();

        // ---------------------------------------------------------------------------------------------
        // 加载抓取发生器
        simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );

        // ---------------------------------------------------------------------------------------------
        // 加载grasp_filter_类
        robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
        grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_) );   //引用类的构造函数

        // ---------------------------------------------------------------------------------------------
        // 为目标生成可能的抓取姿态
        std::vector<moveit_msgs::Grasp> possible_grasps;
        std::vector<trajectory_msgs::JointTrajectoryPoint> grasp_ik_solutions; // save each grasps ik solution for visualization
        std::vector<trajectory_msgs::JointTrajectoryPoint> pregrasp_ik_solutions; // save each pregrasps ik solution for visualization
        
        // 测试循环
        for (int i = 0; i < num_tests; ++i)
        {
          ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

          // 显示生成的目标
          visual_tools_->publishCuboid(req.pose, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE, rviz_visual_tools::RED);
          visual_tools_->trigger();                        
          possible_grasps.clear();     
          grasp_ik_solutions.clear();    
          pregrasp_ik_solutions.clear();    

          // 生成抓取
          simple_grasps_->generateBlockGrasps( req.pose, grasp_data_, possible_grasps);

          // 根据ik解算器过滤无法到达的姿态
          bool filter_pregrasps = true;
          grasp_filter_->filterGrasps(possible_grasps, grasp_ik_solutions, pregrasp_ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_);

          // 可视化可到达的姿态
          // visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
          const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
          // visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.25);  
          visual_tools_->trigger();
         
          //筛选
          moveit_msgs::Grasp Required_grasp;
          trajectory_msgs::JointTrajectoryPoint grasp_Required_trajectory;
          trajectory_msgs::JointTrajectoryPoint pregrasp_Required_trajectory;
          std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
          double quality_max=0;
          double quality_st;
          for(int t=0;t<possible_grasps.size();t++)           
          { 
            quality_st=possible_grasps[t].grasp_quality;
            if(quality_st>quality_max)
            {
              quality_max=quality_st;
              Required_grasp=possible_grasps[t];
              grasp_Required_trajectory=grasp_ik_solutions[t];
              pregrasp_Required_trajectory=pregrasp_ik_solutions[t];
            }
          }
          // std::cout << grasp_Required_trajectory << std::endl; 
          // std::cout << pregrasp_Required_trajectory << std::endl; 
          ik_solutions.push_back(grasp_Required_trajectory);
          ik_solutions.push_back(pregrasp_Required_trajectory);
          res.ik_solutions=ik_solutions;

          //可视化筛选的姿态
          geometry_msgs::PoseStamped ik_pose;
          ik_pose = Required_grasp.grasp_pose;    
          // Eigen::Affine3d eigen_pose;
          // tf::poseMsgToEigen(ik_pose.pose, eigen_pose);                 
          int ret=visual_tools_->publishArrow(ik_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXSMALL,0.1); 
          visual_tools_->trigger();

          // visual_tools_->deleteAllMarkers();
          return true;
        }
    }
  }; // end of class
} // namespace

moveit_simple_grasps::SimpleGraspsPtr moveit_simple_grasps::GraspGeneratorTest::simple_grasps_;
moveit_visual_tools::MoveItVisualToolsPtr moveit_simple_grasps::GraspGeneratorTest::visual_tools_;
moveit_simple_grasps::GraspFilterPtr moveit_simple_grasps::GraspGeneratorTest::grasp_filter_;
moveit_simple_grasps::GraspData moveit_simple_grasps::GraspGeneratorTest::grasp_data_;
planning_scene_monitor::PlanningSceneMonitorPtr moveit_simple_grasps::GraspGeneratorTest::planning_scene_monitor_;
std::string moveit_simple_grasps::GraspGeneratorTest::arm_;
std::string moveit_simple_grasps::GraspGeneratorTest::ee_group_name_;
std::string moveit_simple_grasps::GraspGeneratorTest::planning_group_name_;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_test");   

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (much slower)");
        verbose = true;
      }
    }
  }

  // 运行测试
  moveit_simple_grasps::GraspGeneratorTest tester;
  ros::ServiceServer service = tester.nh_.advertiseService("GenerateSolutions",tester.GraspGenerator);
  ROS_INFO("GenerateSolutions Service OK");

  ros::spin();
  return 0;
}
