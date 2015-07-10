/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

#include <opencv_creative/reader.h>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <DepthSense.hxx>
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>

int
main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "creative_bringup_node");

  ros::NodeHandle nh("~");

  ros::Publisher pub_rgb_info;
  ros::Publisher pub_depth_info;
  image_transport::Publisher pub_rgb;
  image_transport::Publisher pub_depth;
  sensor_msgs::CameraInfo rgb_info;
  sensor_msgs::CameraInfo depth_info;

  //cv::VideoWriter video_writer("video.mpg", CV_FOURCC('P', 'I', 'M', '1'), 30, cv::Size(320, 240), false);
  creative::Reader reader;
  reader.setImageTypes(creative::Reader::COLOR + creative::Reader::DEPTH + creative::Reader::POINTS3D);

  bool visualize = false;

  reader.initialize();
  //creative::Reader::initialize();
  //cv::namedWindow("color", 0);
  //cv::namedWindow("depth", 0);

  // Get frame id from parameter server
  std::string softkinetic_link;
  if (!nh.hasParam("camera_link"))
  {
    ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'camera_link' is missing.");
  }

  nh.param<std::string>("camera_link", softkinetic_link, "softkinetic_link");

  // Initialize image transport object
  image_transport::ImageTransport it(nh);

  // Initialize publishers
  pub_rgb = it.advertise("rgb/image_color", 1);
  pub_depth = it.advertise("depth/image_raw", 1);
  pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
  pub_rgb_info = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);

  // Fill in the color and depth images message header frame id
  std::string colorFrame, depthFrame;
  if (!nh.getParam("rgb_optical_frame", colorFrame))
    colorFrame = "/softkinetic_rgb_optical_frame";

  if (!nh.getParam("depth_optical_frame", depthFrame))
    depthFrame = "/softkinetic_depth_optical_frame";

  std::string calibration_file;
  if (nh.getParam("rgb_calibration_file", calibration_file))
  {

    camera_info_manager::CameraInfoManager camera_info_manager(nh, "senz3d", "file://" + calibration_file);
    rgb_info = camera_info_manager.getCameraInfo();
  }

  if (nh.getParam("depth_calibration_file", calibration_file))
  {
    camera_info_manager::CameraInfoManager camera_info_manager(nh, "senz3d", "file://" + calibration_file);
    depth_info = camera_info_manager.getCameraInfo();
  }

  rgb_info.header.frame_id = colorFrame.c_str();
  reader.setCamInfoColor(rgb_info);

  depth_info.header.frame_id = depthFrame.c_str();
  reader.setCamInfoDepth(depth_info);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    /*std::vector<cv::Mat> images(3);
    reader.getImages(images);
    cv::Mat color, depth, points3d;
    color = images[0];
    depth = images[1];
    points3d = images[2];

    if (!color.empty())
      cv::imshow("color", color);

    if (!depth.empty())
    {
      cv::Mat visible_depth;
      depth.convertTo(visible_depth, CV_8U, 255./400., -100);
      cv::imshow("depth", visible_depth);
    }

    if ((!color.empty()) || (!depth.empty()))
      cv::waitKey(10);*/

    //color image
    pub_rgb.publish(reader.getDataColor());
    pub_rgb_info.publish(reader.getDataCamInfoColor());
    //depth image
    pub_depth.publish(reader.getDataDepth());
    pub_depth_info.publish(reader.getDataCamInfoDepth());
  }
  ros::shutdown();
}
