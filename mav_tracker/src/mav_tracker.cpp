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
#include <cv_image_proc/cv_pca.h>


MavTracker::MavTracker(ros::NodeHandle& nh_,ros::NodeHandle& pnh_)
{
  dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
  dyn_rec_server_->setCallback(boost::bind(&MavTracker::dynRecParamCallback, this, _1, _2));

  image_transport::ImageTransport it_(pnh_);

  listener.reset(new tf::TransformListener());

  image_warper_.reset(new cv_image_warp::WarpProvider(listener, "world"));

  debug_img_provider_.reset(new CvDebugProvider(pnh_));
  final_debug_img_provider_.reset(new CvDebugProvider(ros::NodeHandle("~/final_img")));
  roi_debug_img_provider_.reset(new CvDebugProvider(ros::NodeHandle("~/roi_img")));


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

  if (prior_img_.get()){
    cv_bridge::CvImageConstPtr img_prev_ptr_(cv_bridge::toCvShare(prior_img_, sensor_msgs::image_encodings::MONO8));
    cv_bridge::CvImageConstPtr img_current_ptr_(cv_bridge::toCvShare(latest_img_, sensor_msgs::image_encodings::MONO8));

    cv::Mat diff1;
    //cv::Mat diff2;
    cv::Mat img_motion;
    cv::absdiff(img_prev_ptr_->image, img_current_ptr_->image, diff1);
    //cv::absdiff(img_current_ptr_->image, img_next_ptr->image, diff2);
    //cv::bitwise_and(diff1, diff2, img_motion);
    img_motion = diff1;
    debug_img_provider_->addDebugImage(img_motion);
    double motion_detect_threshold = 4.0;
    cv::threshold(img_motion, img_motion, motion_detect_threshold, 255, CV_THRESH_BINARY);
    //cv::Mat kernel_ero = getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    //cv::erode(img_motion, img_motion, kernel_ero);
    debug_img_provider_->addDebugImage(img_motion);

    if (final_debug_img_provider_->areDebugImagesRequested()){
      //cv::Mat result_img = this->findBiggestBlob(img_motion);

      //cv::Mat result_img;
      //img_current_ptr_->image.copyTo(result_img);

      final_debug_img_provider_->addDebugImage(img_current_ptr_->image);

      std::vector<cv::Point> biggest_blob = this->findBiggestBlob(img_motion);

      if (biggest_blob.size() > 0){

        cv::Rect boundbox;

        boundbox = cv::boundingRect(biggest_blob);

        cv::rectangle(final_debug_img_provider_->getLastAddedImage(), boundbox, cv::Scalar(255,0,0));

        cv_pca::getOrientation(biggest_blob, final_debug_img_provider_->getLastAddedImage());

        cv::Mat roi_img = img_current_ptr_->image(boundbox);

        roi_debug_img_provider_->addDebugImage(roi_img);


        cv::Mat thresholded_img;

        //cv::Point2d image_center (img.size().width/2,img.size().height/2);

        cv::threshold(roi_img, thresholded_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU );

        roi_debug_img_provider_->addDebugImage(thresholded_img);

        cv::Mat dilated;
        double dilation_size = 1;
        cv::Mat dilation_element = getStructuringElement( cv::MORPH_RECT,
                                 cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                 cv::Point( dilation_size, dilation_size ) );
        dilate( thresholded_img, dilated, dilation_element );

        roi_debug_img_provider_->addDebugImage(dilated);

        cv::Mat eroded;
        double erosion_size = 4;
        cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                 cv::Point( erosion_size, erosion_size ) );
        erode( thresholded_img, eroded, element );

        roi_debug_img_provider_->addDebugImage(eroded);
      }


      //final_debug_img_provider_->addDebugImage(result_img);
    }

  }
  
  prior_img_ = image_msg;

  debug_img_provider_->publishDebugImage();
  final_debug_img_provider_->publishDebugImage();
  roi_debug_img_provider_->publishDebugImage();
}

std::vector<cv::Point> MavTracker::findBiggestBlob(const cv::Mat &src){
  //code from http://stackoverflow.com/questions/16746473/opencv-find-bounding-box-of-largest-blob-in-binary-image
  int largest_area=0;
  int largest_contour_index=0;
  cv::Mat temp(src.rows,src.cols,CV_8UC1);
  cv::Mat dst(src.rows,src.cols,CV_8UC1,cv::Scalar::all(0));
  src.copyTo(temp);

  std::vector<std::vector<cv::Point> > contours; // storing contour
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours( temp, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

  for( int i = 0; i< contours.size(); i++ ) // iterate
  {
    double a=cv::contourArea( contours[i],false);  //Find the largest area of contour
    if(a>largest_area)
    {
      largest_area=a;
      largest_contour_index=i;
    }

  }

  //cv::drawContours( dst, contours,largest_contour_index, cv::Scalar(255), CV_FILLED, 8, hierarchy );
  // Draw the largest contour
  if (contours.size() > 0){
    return contours[largest_contour_index];
  }else{
    return std::vector<cv::Point>();
  }
}

