/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
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
 *   * Neither the name of Dataspeed Inc. nor the names of its
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

#include <LidarPubData.h>

#include <algorithm>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#include <gazebo/sensors/RaySensor.hh>
#endif
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>

#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#define STR_Gpu  "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu  ""
#define STR_GPU_ ""
#endif

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVelodyneLaser::GazeboRosVelodyneLaser() : nh_(NULL), gaussian_noise_(0), min_range_(0), max_range_(0)
{
    //simulate the light time of PandarQT
    laserQTOffset_[0] = 10.0f + 2.31f;//us
    laserQTOffset_[1] = 10.0f + 4.37f;
    laserQTOffset_[2] = 10.0f + 6.43f;
    laserQTOffset_[3] = 10.0f + 8.49f;
    laserQTOffset_[4] = 10.0f + 10.54f;
    laserQTOffset_[5] = 10.0f + 12.60f;
    laserQTOffset_[6] = 10.0f + 14.66f;
    laserQTOffset_[7] = 10.0f + 16.71f;
    laserQTOffset_[8] = 10.0f + 19.16f;
    laserQTOffset_[9] = 10.0f + 21.22f;
    laserQTOffset_[10] = 10.0f + 23.28f;
    laserQTOffset_[11] = 10.0f + 25.34f;
    laserQTOffset_[12] = 10.0f + 27.39f;
    laserQTOffset_[13] = 10.0f + 29.45f;
    laserQTOffset_[14] = 10.0f + 31.50f;
    laserQTOffset_[15] = 10.0f + 33.56f;

    laserQTOffset_[16] = 10.0f + 36.61f;
    laserQTOffset_[17] = 10.0f + 38.67f;
    laserQTOffset_[18] = 10.0f + 40.73f;
    laserQTOffset_[19] = 10.0f + 42.78f;
    laserQTOffset_[20] = 10.0f + 44.84f;
    laserQTOffset_[21] = 10.0f + 46.90f;
    laserQTOffset_[22] = 10.0f + 48.95f;
    laserQTOffset_[23] = 10.0f + 51.01f;
    laserQTOffset_[24] = 10.0f + 53.45f;
    laserQTOffset_[25] = 10.0f + 55.52f;
    laserQTOffset_[26] = 10.0f + 57.58f;
    laserQTOffset_[27] = 10.0f + 59.63f;
    laserQTOffset_[28] = 10.0f + 61.69f;
    laserQTOffset_[29] = 10.0f + 63.74f;
    laserQTOffset_[30] = 10.0f + 65.80f;
    laserQTOffset_[31] = 10.0f + 67.86f;

    laserQTOffset_[32] = 10.0f + 70.90f;
    laserQTOffset_[33] = 10.0f + 72.97f;
    laserQTOffset_[34] = 10.0f + 75.02f;
    laserQTOffset_[35] = 10.0f + 77.08f;
    laserQTOffset_[36] = 10.0f + 79.14f;
    laserQTOffset_[37] = 10.0f + 81.19f;
    laserQTOffset_[38] = 10.0f + 83.25f;
    laserQTOffset_[39] = 10.0f + 85.30f;
    laserQTOffset_[40] = 10.0f + 87.75f;
    laserQTOffset_[41] = 10.0f + 89.82f;
    laserQTOffset_[42] = 10.0f + 91.87f;
    laserQTOffset_[43] = 10.0f + 93.93f;
    laserQTOffset_[44] = 10.0f + 95.98f;
    laserQTOffset_[45] = 10.0f + 98.04f;
    laserQTOffset_[46] = 10.0f + 100.10f;
    laserQTOffset_[47] = 10.0f + 102.15f;

    laserQTOffset_[48] = 10.0f + 105.20f;
    laserQTOffset_[49] = 10.0f + 107.26f;
    laserQTOffset_[50] = 10.0f + 109.32f;
    laserQTOffset_[51] = 10.0f + 111.38f;
    laserQTOffset_[52] = 10.0f + 113.43f;
    laserQTOffset_[53] = 10.0f + 115.49f;
    laserQTOffset_[54] = 10.0f + 117.54f;
    laserQTOffset_[55] = 10.0f + 119.60f;
    laserQTOffset_[56] = 10.0f + 122.05f;
    laserQTOffset_[57] = 10.0f + 124.11f;
    laserQTOffset_[58] = 10.0f + 126.17f;
    laserQTOffset_[59] = 10.0f + 128.22f;
    laserQTOffset_[60] = 10.0f + 130.28f;
    laserQTOffset_[61] = 10.0f + 132.34f;
    laserQTOffset_[62] = 10.0f + 134.39f;
    laserQTOffset_[63] = 10.0f + 136.45f;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  laser_queue_.clear();
  laser_queue_.disable();
  if (nh_) {
    nh_->shutdown();
    delete nh_;
    nh_ = NULL;
  }
  callback_laser_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Load plugin
  RayPlugin::Load(_parent, _sdf);

  // Initialize Gazebo node
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  // Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
  parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
  parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
  if (!parent_ray_sensor_)
    gzthrow("GazeboRosVelodyne" << STR_Gpu << "Laser controller requires a " << STR_Gpu << "Ray Sensor as its parent");

  robot_namespace_ = "/";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Lidar plugin missing <frameName>, defaults to /world");
    frame_name_ = "/world";
  }
  else
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("min_range"))
  {
    ROS_INFO("Lidar plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  }
  else
    min_range_ = _sdf->GetElement("min_range")->Get<double>();

  if (!_sdf->HasElement("max_range"))
  {
    ROS_INFO("Lidar plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  }
  else
    max_range_ = _sdf->GetElement("max_range")->Get<double>();

  min_intensity_ = std::numeric_limits<double>::lowest();
  if (!_sdf->HasElement("min_intensity"))
    ROS_INFO("Lidar plugin missing <min_intensity>, defaults to no clipping");
  else
    min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Lidar plugin missing <topicName>, defaults to /points");
    topic_name_ = "/points";
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Lidar plugin missing <gaussianNoise>, defaults to 0.0");
    gaussian_noise_ = 0;
  }
  else
    gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

  if (!_sdf->HasElement("ignoreAzimuthAngleUp"))
  {
    ignoreAzimuthAngleUp_ = 360.0;
    ROS_INFO("Lidar plugin missing <ignoreAzimuthAngleUp>, defaults to %lf",ignoreAzimuthAngleUp_);
  }
  else
  {
    ignoreAzimuthAngleUp_ = _sdf->GetElement("ignoreAzimuthAngleUp")->Get<double>();
    ROS_INFO("Lidar plugin set <ignoreAzimuthAngleUp> to %lf!",ignoreAzimuthAngleUp_);
  }

  if (!_sdf->HasElement("ignoreAzimuthAngleLow"))
  {
    ignoreAzimuthAngleLow_ = 0.0;
    ROS_INFO("Lidar plugin missing <ignoreAzimuthAngleLow>, defaults to %lf",ignoreAzimuthAngleLow_);
  }
  else
  {
    ignoreAzimuthAngleLow_ = _sdf->GetElement("ignoreAzimuthAngleLow")->Get<double>();
    ROS_INFO("Lidar plugin set <ignoreAzimuthAngleLow> to %lf!",ignoreAzimuthAngleLow_);
  }

  if (!_sdf->HasElement("ignoreScanIDUp"))
  {
    ignoreScanIDUp_ = 63;
    ROS_INFO("Lidar plugin missing <ignoreScanIDUp>, defaults to %d",ignoreScanIDUp_);
  }
  else
  {
    ignoreScanIDUp_ = _sdf->GetElement("ignoreScanIDUp")->Get<int>();
    ROS_INFO("Lidar plugin set <ignoreScanIDUp> to %d!",ignoreScanIDUp_);
  }

  if (!_sdf->HasElement("ignoreScanIDLow"))
  {
    ignoreScanIDLow_ = 0;
    ROS_INFO("Lidar plugin missing <ignoreScanIDLow>, defaults to %d",ignoreScanIDLow_);
  }
  else
  {
    ignoreScanIDLow_ = _sdf->GetElement("ignoreScanIDLow")->Get<int>();
    ROS_INFO("Lidar plugin set <ignoreScanIDLow> to %d!",ignoreScanIDLow_);
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Create node handle
  nh_ = new ros::NodeHandle(robot_namespace_);

  // Resolve tf prefix
  std::string prefix;
  nh_->getParam(std::string("tf_prefix"), prefix);
  if (robot_namespace_ != "/") {
    prefix = robot_namespace_;
  }
  boost::trim_right_if(prefix, boost::is_any_of("/"));
  frame_name_ = tf::resolve(prefix, frame_name_);

  // Advertise publisher with a custom callback queue
  if (topic_name_ != "") {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
        topic_name_, 1,
        boost::bind(&GazeboRosVelodyneLaser::ConnectCb, this),
        boost::bind(&GazeboRosVelodyneLaser::ConnectCb, this),
        ros::VoidPtr(), &laser_queue_);
    pub_ = nh_->advertise(ao);
  }

  // Sensor generation off by default
  parent_ray_sensor_->SetActive(false);

  // Start custom queue for laser
  callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosVelodyneLaser::laserQueueThread,this ) );

#if GAZEBO_MAJOR_VERSION >= 7
  ROS_INFO("Lidar %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->VerticalRangeCount());
#else
  ROS_INFO("Lidar %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->GetVerticalRangeCount());
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Subscribe on-demand
void GazeboRosVelodyneLaser::ConnectCb()
{
  boost::lock_guard<boost::mutex> lock(lock_);
  if (pub_.getNumSubscribers()) {
    if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosVelodyneLaser::OnScan, this);
#else
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRosVelodyneLaser::OnScan, this);
#endif
    }
    parent_ray_sensor_->SetActive(true);
  } else {
#if GAZEBO_MAJOR_VERSION >= 7
    if (sub_) {
      sub_->Unsubscribe();
      sub_.reset();
    }
#endif
    parent_ray_sensor_->SetActive(false);
  }
}

void GazeboRosVelodyneLaser::OnScan(ConstLaserScanStampedPtr& _msg)
{
#if GAZEBO_MAJOR_VERSION >= 7
  const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
  const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

  const double maxRange = parent_ray_sensor_->RangeMax();
  const double minRange = parent_ray_sensor_->RangeMin();

  const int rayCount = parent_ray_sensor_->RayCount();
  const int rangeCount = parent_ray_sensor_->RangeCount();//水平方向，水平视场角除以水平角分辨率，如360/0.6=600

  const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();//垂直方向，如64线，128线等

  const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
  const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
#else
  math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

  const double maxRange = parent_ray_sensor_->GetRangeMax();
  const double minRange = parent_ray_sensor_->GetRangeMin();

  const int rayCount = parent_ray_sensor_->GetRayCount();
  const int rangeCount = parent_ray_sensor_->GetRangeCount();

  const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

  const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
  const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);
  const double MIN_INTENSITY = min_intensity_;

  // Populate message fields
  const uint32_t POINT_STEP = 48;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[3].count = 1;
  msg.fields[4].name = "timestamp";
  msg.fields[4].offset = 24;
  msg.fields[4].datatype = sensor_msgs::PointField::FLOAT64;
  msg.fields[4].count = 1;
  msg.fields[5].name = "ring";
  msg.fields[5].offset = 32;
  msg.fields[5].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[5].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  int i, j;
  uint8_t *ptr = msg.data.data();
  for (i = 0; i < rangeCount; i++)//rangeCount = 600
  {
    for (j = 0; j < verticalRangeCount; j++)//verticalRangeCount = 64
    {

      // Range
      double r = _msg->scan().ranges(i + j * rangeCount);
      // Intensity
      double intensity = _msg->scan().intensities(i + j * rangeCount);
      // Ignore points that lay outside range bands or optionally, beneath a
      // minimum intensity level.
      if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) )
        continue;

      // Noise
      if (gaussian_noise_ != 0.0)
        r += gaussianKernel(0,gaussian_noise_);

      // Get angles of ray to get xyz for point
      double yAngle;
      double pAngle;

      if (rangeCount > 1)
        yAngle = i * yDiff / (rangeCount -1) + minAngle.Radian();
      else
        yAngle = minAngle.Radian();

      if (verticalRayCount > 1)
        pAngle = j * pDiff / (verticalRangeCount -1) + verticalMinAngle.Radian();
      else
        pAngle = verticalMinAngle.Radian();

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE))
      {
        float x = r * cos(pAngle) * cos(yAngle);
        float y = r * cos(pAngle) * sin(yAngle);
        float z = r * sin(pAngle);

        // //剔除一定范围内的点
        // if( (x<1.0&&x>-1.2) && (y<0.8&&y>-1.0) && (z<0.0&&z>-1.5) )
        //   continue;
        //剔除一定方位角范围的点
        //水平方位角以绕Z轴由X轴转至Y轴为正方向，X轴正方向为起始，需要注意的是Gazebo仿真里面Lidar的角度以X轴负方向为起始，第一个点云的水平方位角应该是-180度
        // if((i<(ignoreAzimuthAngleUp_/360.0*rangeCount)&&i>(ignoreAzimuthAngleLow_/360.0*rangeCount))&&(j<ignoreScanIDUp_&&j>ignoreScanIDLow_))
        //   continue;
        // if(j>ignoreScanIDUp_||j<ignoreScanIDLow_)
        //   continue;

        if(((y<ignoreAzimuthAngleUp_)&&(y>ignoreAzimuthAngleLow_))&&(j<ignoreScanIDUp_&&j>ignoreScanIDLow_))
          continue;

        *((float*)(ptr + 0)) = x;
        *((float*)(ptr + 4)) = y;
#if GAZEBO_MAJOR_VERSION > 2
        *((float*)(ptr + 8)) = z;
#else
        *((float*)(ptr + 8)) = -z;
#endif
        *((uint8_t*)(ptr + 16)) = intensity;
        *((double*)(ptr + 24)) = _msg->time().sec() + 1.0e-9*_msg->time().nsec() + (laserQTOffset_[j] + 1.0e5/rangeCount*i)/1.0e6;
#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t*)(ptr + 32)) = j; // ring
#else
        *((uint16_t*)(ptr + 32)) = verticalRangeCount - 1 - j; // ring
#endif
        ptr += POINT_STEP;
      }
    }
  }

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step); // Shrink to actual size

  // Publish output
  pub_.publish(msg);
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void GazeboRosVelodyneLaser::laserQueueThread()
{
  while (nh_->ok()) {
    laser_queue_.callAvailable(ros::WallDuration(0.01));
  }
}

} // namespace gazebo

