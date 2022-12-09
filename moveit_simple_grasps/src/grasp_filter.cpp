/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

#include <moveit_simple_grasps/grasp_filter.h>
#include <moveit/transforms/transforms.h>
#include <ros/ros.h>
#include <iostream>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
// Conversions
#include <eigen_conversions/eigen_msg.h> // add to pkg TODO
#include <moveit/kinematics_base/kinematics_base.h>

namespace moveit_simple_grasps
{

// Constructor
GraspFilter::GraspFilter( robot_state::RobotState robot_state,moveit_visual_tools::MoveItVisualToolsPtr& visual_tools ):
  robot_state_(robot_state),
  visual_tools_(visual_tools),
  verbose_(false)
{
  ROS_DEBUG_STREAM_NAMED("filter","Loaded simple grasp filter");
  Rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link","/grasp_filter/grasp_pose"));
  Rviz_visual_tools_->setLifetime(2);
}

GraspFilter::~GraspFilter()
{
}

bool GraspFilter::chooseBestGrasp( const std::vector<moveit_msgs::Grasp>& possible_grasps, moveit_msgs::Grasp& chosen )
{
  // TODO: better logic here
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","There are no grasps to choose from");
    return false;
  }
  chosen = possible_grasps[0]; // just choose first one
  return true;
}

// 过滤抓取
bool GraspFilter::filterGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps,
  std::vector<trajectory_msgs::JointTrajectoryPoint>& grasp_ik_solutions, std::vector<trajectory_msgs::JointTrajectoryPoint>& pregrasp_ik_solutions,bool filter_pregrasp,
  const std::string &ee_parent_link, const std::string& planning_group)
{
  // -----------------------------------------------------------------------------------------------
  // Error check
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","Unable to filter grasps because vector is empty");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  int num_threads = boost::thread::hardware_concurrency();
  if( num_threads > possible_grasps.size() )
    num_threads = possible_grasps.size();

  if(false)
  {
    num_threads = 1;
    ROS_WARN_STREAM_NAMED("grasp_filter","Using only " << num_threads << " threads");
  }
  // num_threads=1;  //使用rviz的话使用单核，避免崩溃
  // -----------------------------------------------------------------------------------------------
  // 从kinetics.yaml获取解算器超时
  double timeout = robot_state_.getRobotModel()->getJointModelGroup( planning_group )->getDefaultIKTimeout();
  timeout = 0.05;
  ROS_DEBUG_STREAM_NAMED("grasp_filter","Grasp filter IK timeout " << timeout);

  // -----------------------------------------------------------------------------------------------
  // 加载运动学解算器
  if( kin_solvers_[planning_group].size() != num_threads )
  {
    kin_solvers_[planning_group].clear();

    const robot_model::JointModelGroup* jmg = robot_state_.getRobotModel()->getJointModelGroup(planning_group);

    // 为每个线程创建ik解算器
    for (int i = 0; i < num_threads; ++i)
    {
      //ROS_INFO_STREAM_NAMED("filter","Creating ik solver " << i);

      kin_solvers_[planning_group].push_back(jmg->getSolverInstance());

      // 测试以确保我们有一个有效的运动学解算器
      if( !kin_solvers_[planning_group][i] )
      {
        ROS_ERROR_STREAM_NAMED("grasp_filter","No kinematic solver found");
        return false;
      }
    }
  }

  // Transform poses -------------------------------------------------------------------------------
  const std::string &ik_frame = kin_solvers_[planning_group][0]->getBaseFrame();
  Eigen::Affine3d link_transform;
  if (!moveit::core::Transforms::sameFrame(ik_frame, robot_state_.getRobotModel()->getModelFrame()))  //判断ik计算器的坐标系和机械臂的坐标系是否相等
  {
    const robot_model::LinkModel *lm = robot_state_.getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
    if (!lm)
      return false;
    //pose = getGlobalLinkTransform(lm).inverse() * pose;
    link_transform = robot_state_.getGlobalLinkTransform(lm).inverse();
  }

  ros::Time start_time;
  start_time = ros::Time::now();

  // -----------------------------------------------------------------------------------------------
  // 通过姿势循环，找到那些在运动上可行的姿势
  std::vector<moveit_msgs::Grasp> filtered_grasps;    

  boost::thread_group bgroup;                         
  boost::mutex lock; 

  ROS_INFO_STREAM_NAMED("filter", "Filtering possible grasps with " << num_threads << " threads");

  // 在线程之间分割工作，根据收到的可能的possible_grasps的大小进行分割，根据内核、线程的数量进行分割，得到每个线程应该分的处理数量
  double num_grasps_per_thread = double(possible_grasps.size()) / num_threads;
  //ROS_INFO_STREAM("total grasps " << possible_grasps.size() << " per thead: " << num_grasps_per_thread);

  int grasps_id_start;
  int grasps_id_end = 0;

  //根据线程数量对数据进行分配
  for(int i = 0; i < num_threads; ++i)
  {
    grasps_id_start = grasps_id_end;                    
    grasps_id_end = ceil(num_grasps_per_thread*(i+1));  
    if( grasps_id_end >= possible_grasps.size() )
      grasps_id_end = possible_grasps.size();
    //ROS_INFO_STREAM_NAMED("filter","low " << grasps_id_start << " high " << grasps_id_end);

    IkThreadStruct tc(possible_grasps, filtered_grasps, grasp_ik_solutions, pregrasp_ik_solutions ,link_transform, grasps_id_start,
      grasps_id_end, kin_solvers_[planning_group][i], filter_pregrasp, ee_parent_link, timeout, &lock, i);  //创建线程运行的结构体
    bgroup.create_thread( boost::bind( &GraspFilter::filterGraspThread, this, tc ) );    //线程组的创建
  }
  std::cout << "Graspsfilter:" << ee_parent_link << std::endl; 
  ROS_DEBUG_STREAM_NAMED("filter","Waiting to join " << num_threads << " ik threads...");
  bgroup.join_all(); 

  ROS_INFO_STREAM_NAMED("filter", "Grasp filter complete, found " << filtered_grasps.size() << " IK solutions out of " <<
    possible_grasps.size() );

  possible_grasps = filtered_grasps;

  if (verbose_)
  {
    // End Benchmark time
    double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
    ROS_INFO_STREAM_NAMED("filter","Grasp generator IK grasp filtering benchmark time:");
    std::cout << duration << "\t" << possible_grasps.size() << "\n";

    ROS_INFO_STREAM_NAMED("filter","Possible grasps filtered to " << possible_grasps.size() << " options.");
  }

  return true;
}

void GraspFilter::filterGraspThread(IkThreadStruct ik_thread_struct)
{
  // Seed state - start at zero
  std::vector<double> ik_seed_state(5); // fill with zeros 
  // TODO do not assume 5 dof

  std::vector<double> grasp_solution;
  std::vector<double> pregrasp_solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::PoseStamped ik_pose;

  // Rviz_visual_tools_->deleteAllMarkers();

  // kinematics::KinematicsQueryOptions options;
  // options.lock_redundant_joints=false;
  // options.return_approximate_solution=false;
  // options.discretization_method=kinematics::DiscretizationMethods::NO_DISCRETIZATION;
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    //ROS_DEBUG_STREAM_NAMED("filter", "Checking grasp #" << i);
    // Clear out previous solution just in case - not sure if this is needed
    grasp_solution.clear();
    pregrasp_solution.clear();
    ik_pose = ik_thread_struct.possible_grasps_[i].grasp_pose;    

    // Test it with IK
    ik_thread_struct.kin_solver_->searchPositionIK(ik_pose.pose, ik_seed_state, ik_thread_struct.timeout_, grasp_solution, error_code);
    
    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      // //显示抓取姿态可到达的位姿
      // Eigen::Affine3d eigen_pose;
      // tf::poseMsgToEigen(ik_pose.pose, eigen_pose);                 
      // int ret=Rviz_visual_tools_->publishArrow(eigen_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXSMALL,0.2); //可视化筛选的抓取姿态
      // Rviz_visual_tools_->trigger();
      // std::cout << ik_pose.pose << std::endl;  
      ROS_INFO_STREAM_NAMED("filter","Found IK Solution");
      // boost::this_thread::sleep(boost::posix_time::seconds(2));      

      // Copy solution to seed state so that next solution is faster
      ik_seed_state = grasp_solution;
      // optionally check the pregrasp----------------------------------------------------------
      if (ik_thread_struct.filter_pregrasp_)       // optionally check the pregrasp
      {
        // Convert to a pre-grasp  
        ik_pose = SimpleGrasps::getPreGraspPose(ik_thread_struct.possible_grasps_[i], ik_thread_struct.ee_parent_link_);
        
        // Test it with IK
        ik_thread_struct.kin_solver_->searchPositionIK(ik_pose.pose, ik_seed_state, ik_thread_struct.timeout_, pregrasp_solution, error_code);

        // Results
        if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
        {
          ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pre-grasp pose.");
          continue;
        }
        else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
        {
          //ROS_DEBUG_STREAM_NAMED("filter","Unable to find IK solution for pre-grasp pose: Timed Out.");
          continue;
        }
        else if( error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS )
        {
          ROS_INFO_STREAM_NAMED("filter","IK solution error for pre-grasp: MoveItErrorCodes.msg = " << error_code);
          continue;
        }
        // // 显示预抓取位姿
        // Eigen::Affine3d eigen_pose;
        // tf::poseMsgToEigen(ik_pose.pose, eigen_pose);
        // int ret=Rviz_visual_tools_->publishArrow(eigen_pose, rviz_visual_tools::YELLOW, rviz_visual_tools::XXSMALL,0.2); //可视化筛选的预抓取姿态
        // Rviz_visual_tools_->trigger();
      }

      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps_.push_back( ik_thread_struct.possible_grasps_[i] );

        trajectory_msgs::JointTrajectoryPoint point;
        //point.positions = ik_seed_state; // show the grasp solution
        point.positions = grasp_solution; // show the pre-grasp solution

        // Copy solution so that we can optionally use it later
        ik_thread_struct.grasp_ik_solutions_.push_back(point);

        point.positions = pregrasp_solution; // show the pre-grasp solution

        // // Copy solution so that we can optionally use it later
        ik_thread_struct.pregrasp_ik_solutions_.push_back(point);
      }

      // End pre-grasp section -------------------------------------------------------
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pose.");
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
    {
      //ROS_DEBUG_STREAM_NAMED("filter","Unable to find IK solution for pose: Timed Out.");
    }
    else
      ROS_INFO_STREAM_NAMED("filter","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }

  //ROS_DEBUG_STREAM_NAMED("filter","Thread " << ik_thread_struct.thread_id_ << " finished");

}

} // namespace
