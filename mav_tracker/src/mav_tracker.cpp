//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
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

#include <mav_tracker/mav_tracker.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PolygonStamped.h>

#include <cv_image_proc/cv_line_tools.h>


MavTracker::MavTracker(ros::NodeHandle& nh_,ros::NodeHandle& pnh_)
{
  dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
  dyn_rec_server_->setCallback(boost::bind(&MavTracker::dynRecParamCallback, this, _1, _2));

  image_transport::ImageTransport it_(pnh_);

  listener.reset(new tf::TransformListener());

  image_warper_.reset(new cv_image_warp::WarpProvider(listener, "world"));

  debug_img_provider_.reset(new CvDebugProvider(pnh_));

  poly_pub = pnh_.advertise<geometry_msgs::PolygonStamped>("plane_poly",1);
  pose_pub = pnh_.advertise<geometry_msgs::PoseStamped>("debug_pose",1);

  it_in_ .reset(new image_transport::ImageTransport(pnh_));
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  sub_ = it_in_->subscribeCamera("image", 20, &MavTracker::cameraCb, this, hints);

  p_sample_rectangle_size_ = 0.40;

  // For now we init the rectangle points here. Have to do this dynamically for different size objects.
  sample_rectangle_object_points_ = cv_image_warp::getSampleRectangleObjectPoints(p_sample_rectangle_size_);
}

void MavTracker::dynRecParamCallback(mav_tracker::MavTrackerConfig& config, uint32_t level)
{
  //p_gaussian_blur_kernel_size_ = config.;
  p_sample_rectangle_size_ = config.sample_rectangle_size;
  sample_rectangle_object_points_ = cv_image_warp::getSampleRectangleObjectPoints(p_sample_rectangle_size_);

  p_canny_lower_threshold_ = config.canny_lower_threshold;

  p_hough_lines_accum_threshold_ = config.hough_lines_accum_threshold;


  p_hough_lines_p_accum_threshold_ = config.hough_lines_p_accum_threshold;
  p_hough_lines_p_min_length_ = config.hough_lines_p_min_length;
  p_hough_lines_p_max_gap_ = config.hough_lines_p_max_gap;

  hough_lines_distance_dial_center_threshold = config.hough_lines_distance_dial_center_threshold;
}

void MavTracker::cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  latest_img_ = image_msg;
  latest_camera_info_ = info_msg;
  
  ROS_INFO("cb");
  
  
}

