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

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <opencv_creative/reader.h>

#include <DepthSense.hxx>

#include <iostream>

namespace creative
{

void setupCameraInfo(const DepthSense::IntrinsicParameters& params, sensor_msgs::CameraInfo& cam_info)
{
  cam_info.distortion_model = "plumb_bob";
  cam_info.height = params.height;
  cam_info.width  = params.width;

  // Distortion parameters D = [k1, k2, t1, t2, k3]
  cam_info.D.resize(5);
  cam_info.D[0] = params.k1;
  cam_info.D[1] = params.k2;
  cam_info.D[2] = params.p1;
  cam_info.D[3] = params.p2;
  cam_info.D[4] = params.k3;

  // Intrinsic camera matrix for the raw (distorted) images:
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cam_info.K[0] = params.fx;
  cam_info.K[2] = params.cx;
  cam_info.K[4] = params.fy;
  cam_info.K[5] = params.cy;
  cam_info.K[8] = 1.0;

  // Rectification matrix (stereo cameras only)
  //     [1 0 0]
  // R = [0 1 0]
  //     [0 0 1]
  cam_info.R[0] = 1.0;
  cam_info.R[4] = 1.0;
  cam_info.R[8] = 1.0;

  // Projection/camera matrix; we use the same values as in the raw image, as we are not
  // applying any correction (WARN: is this ok?). For monocular cameras, Tx = Ty = 0.
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  cam_info.P[0] = params.fx;
  cam_info.P[2] = params.cx;
  cam_info.P[5] = params.fy;
  cam_info.P[6] = params.cy;
  cam_info.P[10] = 1.0;
}

  void
  getAvailableNodes(DepthSense::Context context, DepthSense::ColorNode &color_node, DepthSense::DepthNode &depth_node)
  {
    // obtain the list of devices attached to the host
    std::vector<DepthSense::Device> devices = context.getDevices();
    for (std::vector<DepthSense::Device>::const_iterator iter = devices.begin(); iter != devices.end(); iter++)
    {
      DepthSense::Device device = *iter;
      // obtain the list of nodes of the current device
      std::vector<DepthSense::Node> nodes = device.getNodes();
      for (std::vector<DepthSense::Node>::const_iterator nodeIter = nodes.begin(); nodeIter != nodes.end(); nodeIter++)
      {
        DepthSense::Node node = *nodeIter;
        if (!color_node.isSet())
          color_node = node.as<DepthSense::ColorNode>();
        if (!depth_node.isSet())
          depth_node = node.as<DepthSense::DepthNode>();
      }
      break;
    }
    // return an unset color node
    if (!color_node.isSet())
      color_node = DepthSense::ColorNode();
    if (!depth_node.isSet())
      depth_node = DepthSense::DepthNode();
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** Class where all the magic happens to interface with the Creative camera
   * @return
   */
  class ReaderImpl
  {
  public:
    ReaderImpl()
    {
    }

    ~ReaderImpl()
    {
      context_.stopNodes();
      while (!context_.getRegisteredNodes().empty()) {
        context_.unregisterNode(context_.getRegisteredNodes().back());
      }
      context_.quit();
    }

    static void
    onNewColorSample(DepthSense::ColorNode obj, DepthSense::ColorNode::NewSampleReceivedData data)
    {
      if (image_types_ > 1)
      {
        static size_t index = 0;
        // VERY HACKISH: but wait for another N frames (totally arbitrary) before getting a depth node in there
        if (index < 10)
          ++index;
        else if (index == 10)
        {
          all_nodes_are_up_ = true;
          context_.registerNode(depth_node_);
          ++index;
        }
      }

      // Read the color buffer and display
      /*int32_t w, h;
      // If this is the first sample, we fill the constant values
      DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);*/

      // Set the size, at first time
      if (img_rgb_.data.empty())
      {
        img_rgb_.encoding = sensor_msgs::image_encodings::BGR8;
        if (!hasImageType(Reader::DEPTH))
        {
          int32_t w, h;
          FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

          img_rgb_.width  = w;
          img_rgb_.height = h;
          img_rgb_.data.resize(w * h * 3);
          img_rgb_.step = w * 3;
        }
        else
          return;

        //color_.create(img_rgb_.height, img_rgb_.width, CV_8UC3);
        //color_yuy2.create(img_rgb_.height, img_rgb_.width, CV_8UC2);
      }
      cv::Mat color_yuy2(img_rgb_.height, img_rgb_.width, CV_8UC2, const_cast<void*>((const void*) (data.colorMap)));

      {
        boost::unique_lock<boost::mutex> lock(color_mutex_);
        /*color_.data = reinterpret_cast<uchar *>(
              const_cast<uint8_t *>(static_cast<const uint8_t *>(data.colorMap)));*/
        cv::cvtColor(color_yuy2, color_, CV_YUV2BGR_YUY2);

        std::memcpy(img_rgb_.data.data(),  color_.ptr(),  img_rgb_.data.size());
        // set the timestamps and frame ids for images and camera info
        img_rgb_.header.stamp = ros::Time::now();
        rgb_info_.header = img_rgb_.header;
      }
      color_cond_.notify_all();
    }

    static void
    onNewDepthSample(DepthSense::DepthNode obj, DepthSense::DepthNode::NewSampleReceivedData data)
    {
      // Read the color buffer and display
      /*int32_t w, h;
      DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
      cv::Mat depth_single(h, w, CV_32FC1, const_cast<void*>((const void*) (data.depthMapFloatingPoint)));*/

      // Set the size, at first time
      if (img_depth_.data.empty())
      {
        int32_t w, h;
        FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

        img_depth_.width = w;
        img_depth_.height = h;
        img_depth_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        img_depth_.is_bigendian = 0;
        img_depth_.step = sizeof(float) * w;
        img_depth_.data.resize(w * h * sizeof(float));

        img_rgb_.width  = w;
        img_rgb_.height = h;
        img_rgb_.data.resize(w * h * 3);
        img_rgb_.step = w * 3;

        // fill camera info with the parameters provided by the camera
        if (rgb_info_.D.size() == 0)
          setupCameraInfo(data.stereoCameraParameters.colorIntrinsics, rgb_info_);

        if (depth_info_.D.size() == 0)
          setupCameraInfo(data.stereoCameraParameters.depthIntrinsics, depth_info_);
      }

      {
        boost::unique_lock<boost::mutex> lock(depth_mutex_);
        //depth_single.copyTo(depth_);

        std::memcpy(img_depth_.data.data(), data.depthMapFloatingPoint, img_depth_.data.size());
        for (int count = 0; count < img_depth_.width * img_depth_.height; count++)
        {
          // Saturated pixels on depthMapFloatingPoint have -1 value, but on openni are NaN
          if (data.depthMapFloatingPoint[count] < 0.0)
          {
            *reinterpret_cast<float*>(&img_depth_.data[count*sizeof(float)]) =
                std::numeric_limits<float>::quiet_NaN();
                  continue;
          }
        }

        img_depth_.header.stamp = ros::Time::now();
        depth_info_.header = img_depth_.header;
      }
      depth_cond_.notify_all();
    }

    static void
    initialize()
    {
      if (is_initialized_)
        return;

      // create a connection to the DepthSense server at localhost
      context_ = DepthSense::Context::create();
      // get the first available color sensor
      getAvailableNodes(context_, color_node_, depth_node_);

      // enable the capture of the color map
      // Get RGB data
      context_.requestControl(color_node_);
      color_node_.setEnableColorMap(true);
      DepthSense::ColorNode::Configuration color_configuration(DepthSense::FRAME_FORMAT_QVGA, 30,
                                                               DepthSense::POWER_LINE_FREQUENCY_50HZ,
                                                               DepthSense::COMPRESSION_TYPE_YUY2);
      color_node_.setConfiguration(color_configuration);
      context_.releaseControl(color_node_);

      // Get depth data
      context_.requestControl(depth_node_);
      if (hasImageType(Reader::DEPTH))
      {
        depth_node_.setEnableDepthMap(true);
        depth_node_.setEnableDepthMapFloatingPoint(true);
      }
      if (hasImageType(Reader::POINTS3D))
        depth_node_.setEnableVerticesFloatingPoint(true);
      DepthSense::DepthNode::DepthNode::Configuration depth_configuration(DepthSense::FRAME_FORMAT_QVGA, 30,
                                                                          DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE,
                                                                          true);
      depth_node_.setConfiguration(depth_configuration);
      depth_node_.setConfidenceThreshold(50);
      context_.releaseControl(depth_node_);

      // connect a callback to the newSampleReceived event of the color node
      color_node_.newSampleReceivedEvent().connect(ReaderImpl::onNewColorSample);
      depth_node_.newSampleReceivedEvent().connect(ReaderImpl::onNewDepthSample);

      // If only one node to register, great go ahead
      if (hasImageType(Reader::COLOR))
      {
        context_.registerNode(color_node_);
        all_nodes_are_up_ = (image_types_ == 1);
      }
      else
      {
        all_nodes_are_up_ = true;
        context_.registerNode(depth_node_);
      }
      context_.startNodes();

      // Spawn the thread that will just run
      thread_ = boost::thread(run);

      is_initialized_ = true;
    }

    static void
    setImageTypes(int image_types)
    {
      image_types_ = image_types;
    }

    static bool
    hasImageType(Reader::IMAGE_TYPE image_type)
    {
      return (image_types_ & image_type);
    }

    void
    getImages(std::vector<cv::Mat> &images) const
    {
      if (hasImageType(Reader::COLOR))
      {
        {
          boost::unique_lock<boost::mutex> lock(color_mutex_);
          color_cond_.wait(lock);
        }
        color_.copyTo(images[0]);
      }
      if ((hasImageType(Reader::DEPTH)) || (hasImageType(Reader::POINTS3D)))
      {
        boost::unique_lock<boost::mutex> lock(depth_mutex_);
        depth_cond_.wait(lock);
        if (hasImageType(Reader::DEPTH))
        {
          depth_.copyTo(images[1]);
        }
      }
    }

    sensor_msgs::Image
    getDataDepth() const
    {
      {
        boost::unique_lock<boost::mutex> lock(depth_mutex_);
        depth_cond_.wait(lock);
      }
      return img_depth_;
    }

    sensor_msgs::Image
    getDataColor() const
    {
      {
        boost::unique_lock<boost::mutex> lock(color_mutex_);
        color_cond_.wait(lock);
      }
      return img_rgb_;
    }

    sensor_msgs::CameraInfo
    getDataCamInfoColor() const
    {
      return rgb_info_;
    }

    sensor_msgs::CameraInfo
    getDataCamInfoDepth() const
    {
      return depth_info_;
    }

    void
    setCamInfoColor(const sensor_msgs::CameraInfo &camInfoColor) const
    {
      rgb_info_ = camInfoColor;
      img_rgb_.header.frame_id = rgb_info_.header.frame_id;
    }

    void
    setCamInfoDepth(const sensor_msgs::CameraInfo &camInfoDepth) const
    {
      depth_info_ = camInfoDepth;
      img_depth_.header.frame_id = depth_info_.header.frame_id;
    }

    static bool is_initialized_;
  private:
    static void
    run()
    {
      context_.run();
    }

    static DepthSense::Context context_;
    static DepthSense::ColorNode color_node_;
    static DepthSense::DepthNode depth_node_;

    /** The thread in which the data will be captured */
    static boost::thread thread_;

    /** The iamges in which to store the different data types */
    static cv::Mat color_;
    static cv::Mat depth_;
    static cv::Mat_<cv::Vec3f> points3d_;
    static sensor_msgs::Image img_depth_;
    static sensor_msgs::Image img_rgb_;
    static sensor_msgs::CameraInfo rgb_info_;
    static sensor_msgs::CameraInfo depth_info_;

    /** Variable indicating whether all the nodes are up and register */
    static bool all_nodes_are_up_;

    /** a sum of Reader::IMAGE_TYPE declaring what info is retrieved from the nodes */
    static int image_types_;

    static boost::mutex color_mutex_;
    static boost::mutex depth_mutex_;
    static boost::condition_variable color_cond_;
    static boost::condition_variable depth_cond_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ReaderImpl *Reader::impl_ = new ReaderImpl();
  size_t Reader::count_ = 0;
  bool ReaderImpl::is_initialized_ = false;
  DepthSense::Context ReaderImpl::context_;
  boost::thread ReaderImpl::thread_;
  DepthSense::ColorNode ReaderImpl::color_node_;
  DepthSense::DepthNode ReaderImpl::depth_node_;
  cv::Mat ReaderImpl::color_;
  cv::Mat ReaderImpl::depth_;
  cv::Mat_<cv::Vec3f> ReaderImpl::points3d_;
  sensor_msgs::Image ReaderImpl::img_depth_;
  sensor_msgs::Image ReaderImpl::img_rgb_;
  sensor_msgs::CameraInfo ReaderImpl::rgb_info_;
  sensor_msgs::CameraInfo ReaderImpl::depth_info_;

  bool ReaderImpl::all_nodes_are_up_ = false;
  int ReaderImpl::image_types_ = 0;

  boost::mutex ReaderImpl::color_mutex_;
  boost::mutex ReaderImpl::depth_mutex_;
  boost::condition_variable ReaderImpl::color_cond_;
  boost::condition_variable ReaderImpl::depth_cond_;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Reader::Reader()
  {
    ++count_;
    if (!impl_)
      impl_ = new ReaderImpl();
  }

  /** Clean all the contexts and unregister the nodes
   */
  Reader::~Reader()
  {
    --count_;
    if (count_ == 0)
      delete impl_;
  }

  void
  Reader::initialize()
  {
    impl_->initialize();
  }

  void
  Reader::setImageTypes(int all_images)
  {
    impl_->setImageTypes(all_images);
  }

  bool
  Reader::hasImageType(IMAGE_TYPE image_type)
  {
    return impl_->hasImageType(image_type);
  }

  bool
  Reader::isInitialized()
  {
    return impl_->is_initialized_;
  }

  void
  Reader::getImages(std::vector<cv::Mat> &images)
  {
    impl_->getImages(images);
  }

  sensor_msgs::Image
  Reader::getDataDepth()
  {
    return impl_->getDataDepth();
  }

  sensor_msgs::Image
  Reader::getDataColor()
  {
    return impl_->getDataColor();
  }

  sensor_msgs::CameraInfo
  Reader::getDataCamInfoColor()
  {
    return impl_->getDataCamInfoColor();
  }

  sensor_msgs::CameraInfo
  Reader::getDataCamInfoDepth()
  {
    return impl_->getDataCamInfoDepth();
  }

  void
  Reader::setCamInfoColor(const sensor_msgs::CameraInfo &camInfoColor)
  {
    impl_->setCamInfoColor(camInfoColor);
  }

  void
  Reader::setCamInfoDepth(const sensor_msgs::CameraInfo &camInfoDepth)
  {
    impl_->setCamInfoDepth(camInfoDepth);
  }
}
