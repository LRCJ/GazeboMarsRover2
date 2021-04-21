/*
 * Copyright 2012 Open Source Robotics Foundation
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
 *
*/

/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#ifndef GAZEBO_ROS_P3D_HH
#define GAZEBO_ROS_P3D_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class GazeboComputeSkidRate : public ModelPlugin
  {
    public: GazeboComputeSkidRate();

    public: virtual ~GazeboComputeSkidRate();

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected: virtual void UpdateChild();

private:
    void ComputeRelativeVel(const std::string &LinkName,\
                            const std::string &RefLinkName,\
                            ignition::math::Pose3d &LinkPose,\
                            ignition::math::Vector3d &LinkVelocity,\
                            ignition::math::Vector3d &LinkAngular);

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    private: physics::LinkPtr link_;

    private: physics::LinkPtr reference_link_;


    //link name
private:
    physics::LinkPtr WheelLink[6];//车轮link
    physics::LinkPtr WheelRotRateRefLink[6];//获取车轮转速的参考link
    physics::LinkPtr GlobalRefLink;//车轮全局速度的参考link

    std::string WheelLinkName[6];
    std::string WheelRotRateRefLinkName[6];
    std::string GlobalRefLinkName;


    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

    private: nav_msgs::Odometry pose_msg_;

    private: std::string link_name_;

    private: std::string topic_name_;

    private: std::string frame_name_;
    private: std::string tf_frame_name_;

    private: ignition::math::Pose3<double> offset_;

    private: boost::mutex lock;

    private: common::Time last_time_;
    private: ignition::math::Vector3<double> last_vpos_;
    private: ignition::math::Vector3<double> last_veul_;
    private: ignition::math::Vector3<double> apos_;
    private: ignition::math::Vector3<double> aeul_;
    private: ignition::math::Vector3<double> last_frame_vpos_;
    private: ignition::math::Vector3<double> last_frame_veul_;
    private: ignition::math::Vector3<double> frame_apos_;
    private: ignition::math::Vector3<double> frame_aeul_;

    // rate control
    private: double update_rate_;

    private: double gaussian_noise_;

    private: double GaussianKernel(double mu, double sigma);

    private: std::string robot_namespace_;

    private: ros::CallbackQueue p3d_queue_;
    private: void P3DQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
  };
}
#endif
