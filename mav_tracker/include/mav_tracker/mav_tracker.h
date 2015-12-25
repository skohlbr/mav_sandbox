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


#ifndef MAV_TRACKER_H__
#define MAV_TRACKER_H__

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_subscriber.h>

#include <tf/transform_listener.h>

#include <cv_debug_provider/cv_debug_provider.h>
#include <cv_image_proc/cv_image_warp.h>

#include <dynamic_reconfigure/server.h>
#include <mav_tracker/MavTrackerConfig.h>

class MavTracker
{
public:
  MavTracker(ros::NodeHandle& nh_,ros::NodeHandle& pnh_);

  void dynRecParamCallback(mav_tracker::MavTrackerConfig& config, uint32_t level);

  void cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg);

  std::vector<cv::Point> findBiggestBlob(const cv::Mat &src);

protected:

  ros::Publisher poly_pub;
  ros::Publisher pose_pub;
  ros::Publisher percept_publisher_;
  boost::shared_ptr<tf::TransformListener> listener;

  ros::ServiceClient worldmodel_service;
  ros::ServiceServer detection_service_server;

  boost::shared_ptr<CvDebugProvider> debug_img_provider_;
  boost::shared_ptr<CvDebugProvider> final_debug_img_provider_;
  boost::shared_ptr<cv_image_warp::WarpProvider> image_warper_;

  boost::shared_ptr<image_transport::ImageTransport> it_in_;//, it_out_;
  image_transport::CameraSubscriber sub_;

  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;

  sensor_msgs::ImageConstPtr latest_img_;
  sensor_msgs::CameraInfoConstPtr latest_camera_info_;
  sensor_msgs::ImageConstPtr prior_img_;


  std::vector<Eigen::Vector3d> sample_rectangle_object_points_;

  typedef dynamic_reconfigure::Server<mav_tracker::MavTrackerConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> dyn_rec_server_;
  boost::recursive_mutex config_mutex_;

  int p_gaussian_blur_kernel_size_;
  double p_sample_rectangle_size_;
  int p_canny_lower_threshold_;
  int p_hough_lines_accum_threshold_;
  int p_hough_lines_p_accum_threshold_;
  int p_hough_lines_p_min_length_;
  int p_hough_lines_p_max_gap_;
  double hough_lines_distance_dial_center_threshold;
};

#endif
