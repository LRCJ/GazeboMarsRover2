/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include "ComputeSkidRate.h"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(GazeboComputeSkidRate);

    enum
    {
        RF=0,
        LF=1,
        RM=2,
        LM=3,
        RR=4,
        LR=5,
    };

    // Constructor
    GazeboComputeSkidRate::GazeboComputeSkidRate()
    {
        this->seed = 0;
    }

    // Destructor
    GazeboComputeSkidRate::~GazeboComputeSkidRate()
    {
        event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboComputeSkidRate::UpdateChild, this));
        // Finalize the controller
        this->rosnode_->shutdown();
        this->p3d_queue_.clear();
        this->p3d_queue_.disable();
        this->callback_queue_thread_.join();
        delete this->rosnode_;
    }

// Load the controller
    void GazeboComputeSkidRate::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Get the world name.
        this->world_ = _parent->GetWorld();
        this->model_ = _parent;

        // load parameters
        this->robot_namespace_ = "";
        if (_sdf->HasElement("robotNamespace"))
            this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

        if (!_sdf->HasElement("bodyName"))
        {
            ROS_FATAL("p3d plugin missing <bodyName>, cannot proceed");
            return;
        }
        else
            this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

        this->link_ = _parent->GetLink(this->link_name_);
        if (!this->link_)
        {
            ROS_FATAL("gazebo_ros_p3d plugin error: bodyName: %s does not exist\n",
            this->link_name_.c_str());
            return;
        }

        if (!_sdf->HasElement("topicName"))
        {
            ROS_FATAL("p3d plugin missing <topicName>, cannot proceed");
            return;
        }
        else
            this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

        if (!_sdf->HasElement("frameName"))
        {
            ROS_DEBUG("p3d plugin missing <frameName>, defaults to world");
            this->frame_name_ = "world";
        }
        else
            this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

        if (!_sdf->HasElement("xyzOffset"))
        {
            ROS_DEBUG("p3d plugin missing <xyzOffset>, defaults to 0s");
            this->offset_.Pos() = ignition::math::Vector3<double>(0, 0, 0);
        }
        else
            this->offset_.Pos() = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3<double>>();

        if (!_sdf->HasElement("rpyOffset"))
        {
            ROS_DEBUG("p3d plugin missing <rpyOffset>, defaults to 0s");
            this->offset_.Rot().Euler(ignition::math::Vector3<double>(0, 0, 0));
        }
        else
            this->offset_.Rot().Euler(_sdf->GetElement("rpyOffset")->Get<ignition::math::Vector3<double>>());

        if (!_sdf->HasElement("gaussianNoise"))
        {
            ROS_DEBUG("p3d plugin missing <gaussianNoise>, defaults to 0.0");
            this->gaussian_noise_ = 0;
        }
        else
            this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

        if (!_sdf->HasElement("updateRate"))
        {
            ROS_DEBUG("p3d plugin missing <updateRate>, defaults to 0.0"
        " (as fast as possible)");
            this->update_rate_ = 0;
        }
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        // publish multi queue
        this->pmq.startServiceThread();

        // resolve tf prefix
        std::string prefix;
        this->rosnode_->getParam(std::string("tf_prefix"), prefix);
        this->tf_frame_name_ = tf::resolve(prefix, this->frame_name_);

        if (this->topic_name_ != "")
        {
            this->pub_Queue = this->pmq.addPub<nav_msgs::Odometry>();
            this->pub_ =
            this->rosnode_->advertise<nav_msgs::Odometry>(this->topic_name_, 1);
        }

        this->last_time_ = this->world_->SimTime();
        // initialize body
        this->last_vpos_ = this->link_->WorldLinearVel();
        this->last_veul_ = this->link_->WorldAngularVel();
        this->apos_ = 0;
        this->aeul_ = 0;

        // if frameName specified is "/world", "world", "/map" or "map" report
        // back inertial values in the gazebo world
        if (this->frame_name_ != "/world" &&
        this->frame_name_ != "world" &&
        this->frame_name_ != "/map" &&
        this->frame_name_ != "map")
        {
            this->reference_link_ = this->model_->GetLink(this->frame_name_);
            if (!this->reference_link_)
            {
                ROS_ERROR("gazebo_ros_p3d plugin: frameName: %s does not exist, will"
        " not publish pose\n", this->frame_name_.c_str());
                return;
            }
        }

        // init reference frame state
        if (this->reference_link_)
        {
            ROS_DEBUG("got body %s", this->reference_link_->GetName().c_str());
            this->frame_apos_ = 0;
            this->frame_aeul_ = 0;
            this->last_frame_vpos_ = this->reference_link_->WorldLinearVel();
            this->last_frame_veul_ = this->reference_link_->WorldAngularVel();
        }


        // start custom queue for p3d
        this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboComputeSkidRate::P3DQueueThread, this));

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboComputeSkidRate::UpdateChild, this));
    }

void GazeboComputeSkidRate::ComputeRelativeVel(const std::string &LinkName,\
                                               const std::string &RefLinkName,\
                                               ignition::math::Pose3d &LinkPose,\
                                               ignition::math::Vector3d &LinkVelocity,\
                                               ignition::math::Vector3d &LinkAngular)
{
    // get links
    gazebo::physics::LinkPtr Link;
    gazebo::physics::LinkPtr RefLink;
    Link = this->model_->GetLink(LinkName);
    if(!Link)
    {
        ROS_ERROR("ComputeSkidRate plugin: '%s' link doesn't exist!\n", LinkName.c_str());
        return;  
    }


    // get the relative pose & vel between Link & GlobalLink(world)
    LinkVelocity = Link->WorldLinearVel();
    LinkAngular = Link->WorldAngularVel();
    LinkPose = Link->WorldPose();

    // if ref link is world, do not transform coordinate
    if(RefLinkName != "/world" &&
       RefLinkName != "world" &&
       RefLinkName != "/map" &&
       RefLinkName != "map")
    {
        RefLink = this->model_->GetLink(RefLinkName);
        if(!RefLink)
        {
            ROS_ERROR("ComputeSkidRate plugin: '%s' link doesn't exist!\n", RefLinkName.c_str());
            return;  
        }
        // get the relative pose & vel between RefLink & GlobalLink(world)
        auto RefLinkVelocity = RefLink->WorldLinearVel();
        auto RefLinkAngular = RefLink->WorldAngularVel();
        auto RefLinkPose = RefLink->WorldPose();

        // compute the relative pose & vel between Link & RefLink
        // convert to relative pose
        LinkPose.Pos() = LinkPose.Pos() - RefLinkPose.Pos();
        LinkPose.Pos() = RefLinkPose.Rot().RotateVectorReverse(LinkPose.Pos());
        LinkPose.Rot() *= RefLinkPose.Rot().Inverse();
        // convert to relative rates
        // source code is wrong, It should use RotateVectorReverse instead of RotateVector.
        LinkVelocity = RefLinkPose.Rot().RotateVectorReverse(LinkVelocity - RefLinkVelocity);
        LinkAngular = RefLinkPose.Rot().RotateVectorReverse(LinkAngular - RefLinkAngular);
    }
}

// Update the controller
void GazeboComputeSkidRate::UpdateChild()
{
    if (!this->link_)
        return;

    common::Time cur_time = this->world_->SimTime();

    // rate control
    if ( this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_) )
        return;

  if (this->pub_.getNumSubscribers() > 0)
  {
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
      this->lock.lock();

      if (this->topic_name_ != "")
      {
        // copy data into pose message
        this->pose_msg_.header.frame_id = this->tf_frame_name_;
        this->pose_msg_.header.stamp.sec = cur_time.sec;
        this->pose_msg_.header.stamp.nsec = cur_time.nsec;

        this->pose_msg_.child_frame_id = this->link_name_;

        ignition::math::Pose3<double> pose, frame_pose;
        ignition::math::Vector3<double> vpos;
        ignition::math::Vector3<double> veul;
        ignition::math::Vector3<double> frame_vpos;
        ignition::math::Vector3<double> frame_veul;

        // // get inertial Rates
        // vpos = this->link_->WorldLinearVel();
        // veul = this->link_->WorldAngularVel();

        // // Get Pose/Orientation
        // pose = this->link_->WorldPose();

        // // Apply Reference Frame
        // if (this->reference_link_)
        // {
        //   // convert to relative pose
        //   frame_pose = this->reference_link_->WorldPose();
        //   pose.Pos() = pose.Pos() - frame_pose.Pos();
        //   pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
        //   pose.Rot() *= frame_pose.Rot().Inverse();
        //   // convert to relative rates
        //   frame_vpos = this->reference_link_->WorldLinearVel();
        //   frame_veul = this->reference_link_->WorldAngularVel();
        //   // source code is wrong, It should use RotateVectorReverse instead of RotateVector.
        //   vpos = frame_pose.Rot().RotateVectorReverse(vpos - frame_vpos);
        //   veul = frame_pose.Rot().RotateVectorReverse(veul - frame_veul);
        // }

        ComputeRelativeVel(this->link_name_,this->frame_name_,pose,vpos,veul);


        // compute

        // Apply Constant Offsets
        // apply xyz offsets and get position and rotation components
        pose.Pos() = pose.Pos() + this->offset_.Pos();
        // apply rpy offsets
        pose.Rot() = this->offset_.Rot()*pose.Rot();
        pose.Rot().Normalize();

        // compute accelerations (not used)
        this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
        this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
        this->last_vpos_ = vpos;
        this->last_veul_ = veul;

        this->frame_apos_ = (this->last_frame_vpos_ - frame_vpos) / tmp_dt;
        this->frame_aeul_ = (this->last_frame_veul_ - frame_veul) / tmp_dt;
        this->last_frame_vpos_ = frame_vpos;
        this->last_frame_veul_ = frame_veul;

        // Fill out messages
        this->pose_msg_.pose.pose.position.x    = pose.Pos().X();
        this->pose_msg_.pose.pose.position.y    = pose.Pos().Y();
        this->pose_msg_.pose.pose.position.z    = pose.Pos().Z();

        this->pose_msg_.pose.pose.orientation.x = pose.Rot().X();
        this->pose_msg_.pose.pose.orientation.y = pose.Rot().Y();
        this->pose_msg_.pose.pose.orientation.z = pose.Rot().Z();
        this->pose_msg_.pose.pose.orientation.w = pose.Rot().W();

        this->pose_msg_.twist.twist.linear.x  = vpos.X() +
          this->GaussianKernel(0, this->gaussian_noise_);
        this->pose_msg_.twist.twist.linear.y  = vpos.Y() +
          this->GaussianKernel(0, this->gaussian_noise_);
        this->pose_msg_.twist.twist.linear.z  = vpos.Z() +
          this->GaussianKernel(0, this->gaussian_noise_);
        // pass euler angular rates
        this->pose_msg_.twist.twist.angular.x = veul.X() +
          this->GaussianKernel(0, this->gaussian_noise_);
        this->pose_msg_.twist.twist.angular.y = veul.Y()+
          this->GaussianKernel(0, this->gaussian_noise_);
        this->pose_msg_.twist.twist.angular.z = veul.Z() +
          this->GaussianKernel(0, this->gaussian_noise_);

        // fill in covariance matrix
        double gn2 = this->gaussian_noise_*this->gaussian_noise_;
        this->pose_msg_.pose.covariance[0] = gn2;
        this->pose_msg_.pose.covariance[7] = gn2;
        this->pose_msg_.pose.covariance[14] = gn2;
        this->pose_msg_.pose.covariance[21] = gn2;
        this->pose_msg_.pose.covariance[28] = gn2;
        this->pose_msg_.pose.covariance[35] = gn2;

        this->pose_msg_.twist.covariance[0] = gn2;
        this->pose_msg_.twist.covariance[7] = gn2;
        this->pose_msg_.twist.covariance[14] = gn2;
        this->pose_msg_.twist.covariance[21] = gn2;
        this->pose_msg_.twist.covariance[28] = gn2;
        this->pose_msg_.twist.covariance[35] = gn2;

        // publish to ros
        this->pub_Queue->push(this->pose_msg_, this->pub_);
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

// Utility for adding noise
double GazeboComputeSkidRate::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

// Put laser data to the interface
void GazeboComputeSkidRate::P3DQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
