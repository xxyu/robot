//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef POSE_INFO_CONTAINER_H__
#define POSE_INFO_CONTAINER_H__

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core>

class PoseInfoContainer{
public:

  void update(const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov, const ros::Time& stamp, const std::string& frame_id);

  const geometry_msgs::PoseStamped& getPoseStamped() { return stampedPose_; };
  const geometry_msgs::PoseStamped& getPoseStampedLast()  { return stampedPoseLast_; }; //2017-01-12, WT
  void setPoseStampedLast(const geometry_msgs::PoseStamped& stampedScanPose_)  { stampedPoseLast_ = stampedScanPose_; }; //2017-01-12, WT
  
  const geometry_msgs::PoseWithCovarianceStamped& getPoseWithCovarianceStamped() { return covPose_; };
  const tf::Transform& getTfTransform() { return poseTransform_; };

protected:
  geometry_msgs::PoseStamped stampedPose_, stampedPoseLast_; //2017-01-12, WT
  geometry_msgs::PoseWithCovarianceStamped covPose_;
  tf::Transform poseTransform_;

};

#endif

