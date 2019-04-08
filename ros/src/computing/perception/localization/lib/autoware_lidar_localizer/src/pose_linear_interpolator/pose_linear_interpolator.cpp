/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lidar_localizer/pose_linear_interpolator/pose_linear_interpolator.h"

PoseStamped interpolatePose(const PoseStamped &pose_a,
                            const PoseStamped &pose_b,
                            const double time_stamp) {
  if (pose_a.stamp == 0 || pose_b.stamp == 0 || time_stamp == 0) {
    return PoseStamped();
  }

  Velocity v(pose_a, pose_b);

  const double dt = time_stamp - pose_b.stamp;

  PoseStamped p;
  p.pose.x = pose_b.pose.x + v.linear.x * dt;
  p.pose.y = pose_b.pose.y + v.linear.y * dt;
  p.pose.z = pose_b.pose.z + v.linear.z * dt;
  p.pose.roll = pose_b.pose.roll;
  p.pose.pitch = pose_b.pose.pitch;
  p.pose.yaw = pose_b.pose.yaw + v.angular.z * dt;
  p.stamp = time_stamp;
  return p;
}

PoseStamped interpolatePose(const PoseStamped &pose_a,
                            const PoseStamped &pose_b,
                            const nav_msgs::Odometry &odom,
                            const double time_stamp) {
  if (pose_a.stamp == 0 || pose_b.stamp == 0 || time_stamp == 0) {
    return PoseStamped();
  }

  Velocity v(pose_a, pose_b);
  v.angular.x = odom.twist.twist.angular.x;
  v.angular.y = odom.twist.twist.angular.y;
  v.angular.z = odom.twist.twist.angular.z;

  const double dt = time_stamp - pose_b.stamp;

  PoseStamped p;
  p.pose.x = pose_b.pose.x + v.linear.x * dt;
  p.pose.y = pose_b.pose.y + v.linear.y * dt;
  p.pose.z = pose_b.pose.z + v.linear.z * dt;
  p.pose.roll = pose_b.pose.roll;
  p.pose.pitch = pose_b.pose.pitch;
  p.pose.yaw = pose_b.pose.yaw + v.angular.z * dt;
  p.stamp = time_stamp;
  return p;
}

PoseLinearInterpolator::PoseLinearInterpolator() {}

void PoseLinearInterpolator::clearOdom() {
  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = 0;
}

void PoseLinearInterpolator::clearPoseStamped() {
  current_pose_.clear();
  prev_pose_.clear();
}

bool PoseLinearInterpolator::isNotSetPoseStamped() const {
  return (current_pose_ == PoseStamped() && prev_pose_ == PoseStamped());
}
void PoseLinearInterpolator::pushbackPoseStamped(const PoseStamped &pose) {
  prev_pose_ = current_pose_;
  current_pose_ = pose;
}

PoseStamped PoseLinearInterpolator::getInterpolatePoseStamped(
    const double time_stamp) const {
  if (use_odom_){
    Velocity v(prev_pose_, current_pose_);
    std::cerr << v << std::endl;
    return interpolatePose(prev_pose_, current_pose_, odom_, time_stamp);
  }else{
    Velocity v(prev_pose_, current_pose_);
    std::cerr << v << std::endl;
    return interpolatePose(prev_pose_, current_pose_, time_stamp);
  }
}

PoseStamped PoseLinearInterpolator::getCurrentPoseStamped() const {
  return current_pose_;
}

PoseStamped PoseLinearInterpolator::getPrevPoseStamped() const {
  return prev_pose_;
}

Velocity PoseLinearInterpolator::getVelocity() const {
  return Velocity(prev_pose_, current_pose_);
}

void PoseLinearInterpolator::setOdom(const nav_msgs::Odometry odom){
  odom_ = odom;
}

void PoseLinearInterpolator::setUseOdom(const bool use_odom){
  use_odom_ = use_odom;
}
