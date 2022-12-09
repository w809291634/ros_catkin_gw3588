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

#include <moveit_simple_grasps/simple_grasps.h>
#include <iostream>

namespace moveit_simple_grasps
{

// Constructor
SimpleGrasps::SimpleGrasps(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose) :
  visual_tools_(visual_tools),
  verbose_(verbose)
{
  ROS_DEBUG_STREAM_NAMED("grasps","Loaded simple grasp generator");
}

// Deconstructor
SimpleGrasps::~SimpleGrasps()
{
}

// 为对象创建所有可能的抓取位置
bool SimpleGrasps::generateBlockGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Calculate grasps in two axis in both directions
  // generateAxisGrasps( object_pose, X_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // got no grasps with this alone
  // generateAxisGrasps( object_pose, X_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gives some grasps... looks ugly
  // generateAxisGrasps( object_pose, Y_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // GOOD ONES!
  generateAxisGrasps( object_pose, Y_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gave a grasp from top... bad 筛选出自己需要的一部分姿态

  return true;
}

// 在一个轴上创建抓取位置
bool SimpleGrasps::generateAxisGrasps(
  const geometry_msgs::Pose& object_pose,
  grasp_axis_t axis,
  grasp_direction_t direction,
  grasp_rotation_t rotation,
  double hand_roll,
  const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Create a transform from the object's frame (center of object) to /base_link
  tf::poseMsgToEigen(object_pose, object_global_transform_);

  // ---------------------------------------------------------------------------------------------
  // Grasp parameters

  // Create re-usable approach motion
  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
  pre_grasp_approach.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable retreat motion
  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
  post_grasp_retreat.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // ---------------------------------------------------------------------------------------------
  // Angle calculations
  double radius = grasp_data.grasp_depth_; //0.12
  double xb;
  double yb = 0.0; // stay in the y plane of the object
  double zb;
  double theta1 = 0.0; //生成角度的起始点，初始值为0
  double theta2 = 0.0; // UP 'direction'

  // Gripper direction (UP/DOWN) rotation. UP set by default
  if( direction == DOWN )
  {
    theta1 = M_PI;    //theta2 = M_PI;
  }

  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------
  // Begin Grasp Generator Loop
  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------

  /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the object, then later convert it to the base link
   */
  for(int i = 0; i <= grasp_data.angle_resolution_; ++i)
  {
    // Create a Grasp message
    moveit_msgs::Grasp new_grasp;

    // Calculate grasp pose
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    Eigen::Affine3d grasp_pose;

    switch(axis)
    {
      case X_AXIS:
        grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

        break;
      case Y_AXIS:
        grasp_pose =
          Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

        break;
      case Z_AXIS:
        grasp_pose =
                Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitZ())
                *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

            grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

        break;
    }

    /* The estimated probability of success for this grasp, or some other measure of how "good" it is.
     * Here we base bias the score based on how far the wrist is from the surface, preferring a greater
     * distance to prevent wrist/end effector collision with the table
     */
    double score = sin(theta1);
    new_grasp.grasp_quality = std::max(score,0.1); // don't allow score to drop below 0.1 b/c all grasps are ok

    // Calculate the theta1 for next time
    if (rotation == HALF)
      theta1 += M_PI / grasp_data.angle_resolution_;
    else if(rotation == FULL)
    {
      theta1 += 2*M_PI / grasp_data.angle_resolution_;
    }
    else if(rotation == ONE_FOURTH)
    {
      theta1 += 0.5*M_PI / grasp_data.angle_resolution_;
    }

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;
    // std::cout << grasp_id << std::endl;   //打印生成的grasps

    // PreGrasp and Grasp Postures --------------------------------------------------------------------------

    // The internal posture of the hand for the pre-grasp only positions are used
    new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;

    // The internal posture of the hand for the grasp positions and efforts are used
    new_grasp.grasp_posture = grasp_data.grasp_posture_;

    // Grasp ------------------------------------------------------------------------------------------------


    // DEBUG - show original grasp pose before tranform to gripper frame
    if( verbose_ )
    {
      tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
      visual_tools_->publishArrow(grasp_pose_msg.pose, rviz_visual_tools::GREEN);
    }

    // ------------------------------------------------------------------------
    // Optionally roll wrist with respect to object pose
    Eigen::Affine3d roll_gripper;
    roll_gripper = Eigen::AngleAxisd(hand_roll, Eigen::Vector3d::UnitX());
    grasp_pose = grasp_pose * roll_gripper;

    // ------------------------------------------------------------------------
    // Change grasp to frame of reference of this custom end effector

    // Convert to Eigen
    Eigen::Affine3d eef_conversion_pose;
    tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose);

    // Transform the grasp pose
    grasp_pose = grasp_pose * eef_conversion_pose;

    // ------------------------------------------------------------------------
    // Convert pose to global frame (base_link)
    tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);

    // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
    new_grasp.grasp_pose = grasp_pose_msg;

    // Other ------------------------------------------------------------------------------------------------

    // the maximum contact force to use while grasping (<=0 to disable)
    new_grasp.max_contact_force = 0;

    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------
    // Approach and retreat
    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------

    // Straight down ---------------------------------------------------------------------------------------
    // With respect to the base link/world frame

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.base_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = 1; // 撤退方向(pos z axis)
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

//下面的坐标系是tool0
    // // Angled with pose -------------------------------------------------------------------------------------
    // // Approach with respect to end effector orientation

    // // Approach
    // pre_grasp_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
    // pre_grasp_approach.direction.vector.x = 0;
    // pre_grasp_approach.direction.vector.y = 0;
    // pre_grasp_approach.direction.vector.z = 1;
    // new_grasp.pre_grasp_approach = pre_grasp_approach;

    // // Retreat
    // post_grasp_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
    // post_grasp_retreat.direction.vector.x = 0;
    // post_grasp_retreat.direction.vector.y = 0;
    // post_grasp_retreat.direction.vector.z = -1;
    // new_grasp.post_grasp_retreat = post_grasp_retreat;

    // // Add to vector
    // possible_grasps.push_back(new_grasp);

  }

  ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  return true;
}

geometry_msgs::PoseStamped SimpleGrasps::getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link)
{
  // 掌握姿势变量
  geometry_msgs::PoseStamped grasp_pose = grasp.grasp_pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose.pose, grasp_pose_eigen);

  // Get pre-grasp pose first
  geometry_msgs::PoseStamped pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen = grasp_pose_eigen; // Copy original grasp pose to pre-grasp pose

  // 接近方向变量
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // 预抓取的方向
  // 基于百分比计算当前动画位置
  Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
    -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance,
    -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance,
    -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance
  );

  // Decide if we need to change the approach_direction to the local frame of the end effector orientation
  if( grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link )
  {
    // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
    pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }
  else
  {
    pre_grasp_approach_direction_local = pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }

  // Update the grasp matrix usign the new locally-framed approach_direction
  pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

  // Convert eigen pre-grasp position back to regular message
  tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose.pose);

  // Copy original header to new grasp
  pre_grasp_pose.header = grasp_pose.header;

  return pre_grasp_pose;
}



} // namespace
