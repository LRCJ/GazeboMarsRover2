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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseArray.h>

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
    enum WheelIndex
    {
        RF=0,
        LF=1,
        RM=2,
        LM=3,
        RR=4,
        LR=5,
    };

    class GazeboComputeSkidRate : public ModelPlugin
    {
    public:
        GazeboComputeSkidRate();
        virtual ~GazeboComputeSkidRate();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
        virtual void UpdateChild();
    private:
        void ComputeRelativeVel(const std::string &LinkName,\
                                const std::string &RefLinkName,\
                                ignition::math::Vector3d &LinkVelocity,\
                                ignition::math::Vector3d &LinkAngular);
        void ComputeRelativePose(const std::string &LinkName,\
                                 const std::string &RefLinkName,\
                                 ignition::math::Pose3d &Pose);
        double ComputeSkidRate(const std::string &LinkName,\
                               const std::string &RefLinkName,\
                               WheelIndex wi);
        void QueueThread();


        // ros variable
        ros::NodeHandle* rosnode_;
        PubQueue<std_msgs::Float64MultiArray>::Ptr PubSkidRateQue_;
        PubQueue<geometry_msgs::PoseStamped>::Ptr PubWheelVelPoseQue_[6];
        PubQueue<geometry_msgs::PoseArray>::Ptr PubWheelPoseInBodyQue_;
        ros::Publisher SkidRatePub_;
        ros::Publisher WheelVelPosePub_[6];
        ros::Publisher WheelPoseInBodyPub_;
        std::string SkidRateTopicName_;
        std::string WheelVelPoseTopicName_;
        std::string WheelPoseInBodyTopicName_;

        // gazebo variable
        physics::WorldPtr world_;
        physics::ModelPtr model_;

        // data
        std_msgs::Float64MultiArray SkidRate_;
        double SkidRateBuf_[6];
        geometry_msgs::PoseStamped WheelVelPose_;
        geometry_msgs::PoseArray WheelPoseInBody_;


        //link name
        std::string WheelName_[6];
        std::string WheelRefLinkName_[6];
        // std::string GlobalRefLinkName;
        std::string BodyName_;

        boost::mutex lock;
        common::Time last_time_;


    // rate control
        double update_rate_;
        std::string robot_namespace_;
        ros::CallbackQueue ComputeSkidRate_queue_;
        boost::thread callback_queue_thread_;

    // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
        unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
        PubMultiQueue pmq;
  };
}
#endif
