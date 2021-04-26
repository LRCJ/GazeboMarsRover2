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
        this->ComputeSkidRate_queue_.clear();
        this->ComputeSkidRate_queue_.disable();
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
        if (!_sdf->HasElement("updateRate"))
        {
            ROS_DEBUG("ComputeSkidRate plugin missing <updateRate>, defaults to 0.0"
        " (as fast as possible)");
            this->update_rate_ = 0;
        }
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

        // get wheels name
        if (!_sdf->HasElement("WheelRF"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelRF>, cannot proceed");
            return;
        }
        else
            this->WheelName_[RF] = _sdf->GetElement("WheelRF")->Get<std::string>();
        if (!_sdf->HasElement("WheelLF"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelLF>, cannot proceed");
            return;
        }
        else
            this->WheelName_[LF] = _sdf->GetElement("WheelLF")->Get<std::string>();
        if (!_sdf->HasElement("WheelRM"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelRM>, cannot proceed");
            return;
        }
        else
            this->WheelName_[RM] = _sdf->GetElement("WheelRM")->Get<std::string>();
        if (!_sdf->HasElement("WheelLM"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelLM>, cannot proceed");
            return;
        }
        else
            this->WheelName_[LM] = _sdf->GetElement("WheelLM")->Get<std::string>();
        if (!_sdf->HasElement("WheelRR"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelRR>, cannot proceed");
            return;
        }
        else
            this->WheelName_[RR] = _sdf->GetElement("WheelRR")->Get<std::string>();
        if (!_sdf->HasElement("WheelLR"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelLR>, cannot proceed");
            return;
        }
        else
            this->WheelName_[LR] = _sdf->GetElement("WheelLR")->Get<std::string>();

        // and ref link name
        if (!_sdf->HasElement("WheelRF_RefLink"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelRF_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[RF] = _sdf->GetElement("WheelRF_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelLF_RefLink"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelLF_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[LF] = _sdf->GetElement("WheelLF_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelRM_RefLink"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelRM_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[RM] = _sdf->GetElement("WheelRM_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelLM_RefLink"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelLM_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[LM] = _sdf->GetElement("WheelLM_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelRR_RefLink"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelRR_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[RR] = _sdf->GetElement("WheelRR_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelLR_RefLink"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelLR_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[LR] = _sdf->GetElement("WheelLR_RefLink")->Get<std::string>();
        // get lidar name
        if (!_sdf->HasElement("Body"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <Body>, cannot proceed");
            return;
        }
        else
            this->BodyName_ = _sdf->GetElement("Body")->Get<std::string>();




        // get skid rate topic name
        if (!_sdf->HasElement("SkidRateTopicName"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <SkidRateTopicName>, cannot proceed");
            return;
        }
        else
            this->SkidRateTopicName_ = _sdf->GetElement("SkidRateTopicName")->Get<std::string>();
        if (!_sdf->HasElement("WheelVelPoseTopicName"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelVelPoseTopicName>, cannot proceed");
            return;
        }
        else
            this->WheelVelPoseTopicName_ = _sdf->GetElement("WheelVelPoseTopicName")->Get<std::string>();
        if (!_sdf->HasElement("WheelPoseInBodyTopicName"))
        {
            ROS_FATAL("ComputeSkidRate plugin missing <WheelPoseInBodyTopicName>, cannot proceed");
            return;
        }
        else
            this->WheelPoseInBodyTopicName_ = _sdf->GetElement("WheelPoseInBodyTopicName")->Get<std::string>();


        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'ComputeSkidRate_plugins.so' in the gazebo_ros package)");
            return;
        }

        this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        // publish multi queue
        this->pmq.startServiceThread();

        // publish wheels skid rate
        this->PubSkidRateQue_ = this->pmq.addPub<std_msgs::Float64MultiArray>();
        this->SkidRatePub_ = this->rosnode_->advertise<std_msgs::Float64MultiArray>(this->SkidRateTopicName_, 10);
        // and publish wheels velocity direction(pose)
        this->PubWheelVelPoseQue_[RF] = this->pmq.addPub<geometry_msgs::PoseStamped>();
        this->WheelVelPosePub_[RF] = this->rosnode_->advertise<geometry_msgs::PoseStamped>(std::string("/WRF_")+this->WheelVelPoseTopicName_, 10);
        this->PubWheelVelPoseQue_[LF] = this->pmq.addPub<geometry_msgs::PoseStamped>();
        this->WheelVelPosePub_[LF] = this->rosnode_->advertise<geometry_msgs::PoseStamped>(std::string("/WLF_")+this->WheelVelPoseTopicName_, 10);
        this->PubWheelVelPoseQue_[RM] = this->pmq.addPub<geometry_msgs::PoseStamped>();
        this->WheelVelPosePub_[RM] = this->rosnode_->advertise<geometry_msgs::PoseStamped>(std::string("/WRM_")+this->WheelVelPoseTopicName_, 10);
        this->PubWheelVelPoseQue_[LM] = this->pmq.addPub<geometry_msgs::PoseStamped>();
        this->WheelVelPosePub_[LM] = this->rosnode_->advertise<geometry_msgs::PoseStamped>(std::string("/WLM_")+this->WheelVelPoseTopicName_, 10);
        this->PubWheelVelPoseQue_[RR] = this->pmq.addPub<geometry_msgs::PoseStamped>();
        this->WheelVelPosePub_[RR] = this->rosnode_->advertise<geometry_msgs::PoseStamped>(std::string("/WRR_")+this->WheelVelPoseTopicName_, 10);
        this->PubWheelVelPoseQue_[LR] = this->pmq.addPub<geometry_msgs::PoseStamped>();
        this->WheelVelPosePub_[LR] = this->rosnode_->advertise<geometry_msgs::PoseStamped>(std::string("/WLR_")+this->WheelVelPoseTopicName_, 10);
        // publish wheels pose in LiDAR frame
        this->PubWheelPoseInBodyQue_ = this->pmq.addPub<geometry_msgs::PoseArray>();
        this->WheelPoseInBodyPub_ = this->rosnode_->advertise<geometry_msgs::PoseArray>(this->WheelPoseInBodyTopicName_,10);

        // initialize variable
        SkidRate_.data.resize(12,0.0);
        memset(&SkidRateBuf_,0,sizeof(SkidRateBuf_));
        WheelPoseInBody_.poses.resize(6);
        WheelPoseInBody_.header.frame_id = this->BodyName_;

        // start custom queue
        this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboComputeSkidRate::QueueThread, this));

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboComputeSkidRate::UpdateChild, this));
    }

void GazeboComputeSkidRate::ComputeRelativeVel(const std::string &LinkName,\
                                               const std::string &RefLinkName,\
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

    // get the relative vel between Link & GlobalLink(world)
    LinkVelocity = Link->WorldLinearVel();
    LinkAngular = Link->WorldAngularVel();

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
        // get the relative vel between RefLink & GlobalLink(world)
        auto RefLinkVelocity = RefLink->WorldLinearVel();
        auto RefLinkAngular = RefLink->WorldAngularVel();
        auto RefLinkPose = RefLink->WorldPose();
        // compute the relative pose & vel between Link & RefLink
        // convert to relative vel
        // source code is wrong, It should use RotateVectorReverse instead of RotateVector.
        LinkVelocity = RefLinkPose.Rot().RotateVectorReverse(LinkVelocity - RefLinkVelocity);
        LinkAngular = RefLinkPose.Rot().RotateVectorReverse(LinkAngular - RefLinkAngular);
    }
}

void GazeboComputeSkidRate::ComputeRelativePose(const std::string &LinkName,\
                                                const std::string &RefLinkName,\
                                                ignition::math::Pose3d &LinkPose)
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


    // get the relative pose between Link & GlobalLink(world)
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
        // get the relative pose between RefLink & GlobalLink(world)
        auto RefLinkPose = RefLink->WorldPose();

        // compute the relative pose between Link & RefLink
        // convert to relative pose
        LinkPose.Pos() = LinkPose.Pos() - RefLinkPose.Pos();
        LinkPose.Pos() = RefLinkPose.Rot().RotateVectorReverse(LinkPose.Pos());
        LinkPose.Rot() *= RefLinkPose.Rot().Inverse();
    }
}

double GazeboComputeSkidRate::ComputeSkidRate(const std::string &LinkName,\
                                              const std::string &RefLinkName,\
                                              WheelIndex wi,\
                                              double &rr)
{
    ignition::math::Pose3d pose;
    ignition::math::Vector3d vel,angular;
    // compute Wheel Rotation Rate
    ComputeRelativeVel(LinkName,RefLinkName,vel,angular);
    rr = angular.Y();
    // compute Wheel Velocity in world coordinate
    ComputeRelativeVel(LinkName,std::string("world"),vel,angular);
    double vel_mag = std::sqrt(vel.X()*vel.X()+vel.Y()*vel.Y()+vel.Z()*vel.Z());
    if( abs(vel_mag)<1.0e-6 || abs(rr)<1.0e-6 )
        return 0;
    //double SkidRate = (rr*0.15-vel_mag)/(rr*0.15);
    double SkidRate = (vel_mag)/(rr*0.15);
    if(SkidRate>2 || SkidRate<-2)
        return 0;

    // publish link vel pose
    WheelVelPose_.header.stamp = ros::Time::now();
    WheelVelPose_.header.frame_id = "world";

    // publish six wheel velocity pose(direction)
    if(vel.Length()>0.01)
    {
        ignition::math::Vector3d vx(vel);
        vx.Normalize();
        ComputeRelativePose(LinkName,std::string("world"),pose);
        ignition::math::Matrix3d m(pose.Rot());
        ignition::math::Vector3d vy(m(0,1),m(1,1),m(2,1));
        vy.Normalize();
        ignition::math::Vector3d vz = vx.Cross(vy);
        vz.Normalize();
        m.Axes(vx,vy,vz);
        ignition::math::Quaterniond q(m);

        WheelVelPose_.pose.position.x = pose.Pos().X();
        WheelVelPose_.pose.position.y = pose.Pos().Y();
        WheelVelPose_.pose.position.z = pose.Pos().Z();
        WheelVelPose_.pose.orientation.x = q.X();
        WheelVelPose_.pose.orientation.y = q.Y();
        WheelVelPose_.pose.orientation.z = q.Z();
        WheelVelPose_.pose.orientation.w = q.W(); 
    }
    
    this->PubWheelVelPoseQue_[wi]->push(WheelVelPose_,this->WheelVelPosePub_[wi]);

    return SkidRate;
}

// Update the controller
void GazeboComputeSkidRate::UpdateChild()
{
    common::Time cur_time = this->world_->SimTime();

    // rate control
    if ( this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_) )
        return;
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
        this->lock.lock();

        double rr;//wheel rotation rate
        // compute six wheel skid rate
        SkidRateBuf_[RF] = ComputeSkidRate(this->WheelName_[RF],this->WheelRefLinkName_[RF],RF,rr);
        abs(SkidRateBuf_[RF])<1.0e-6?:this->SkidRate_.data[RF] = SkidRateBuf_[RF];
        this->SkidRate_.data[RF+6] = rr;
        SkidRateBuf_[LF] = ComputeSkidRate(this->WheelName_[LF],this->WheelRefLinkName_[LF],LF,rr);
        abs(SkidRateBuf_[LF])<1.0e-6?:this->SkidRate_.data[LF] = SkidRateBuf_[LF];
        this->SkidRate_.data[LF+6] = rr;
        SkidRateBuf_[RM] = ComputeSkidRate(this->WheelName_[RM],this->WheelRefLinkName_[RM],RM,rr);
        abs(SkidRateBuf_[RM])<1.0e-6?:this->SkidRate_.data[RM] = SkidRateBuf_[RM];
        this->SkidRate_.data[RM+6] = rr;
        SkidRateBuf_[LM] = ComputeSkidRate(this->WheelName_[LM],this->WheelRefLinkName_[LM],LM,rr);
        abs(SkidRateBuf_[LM])<1.0e-6?:this->SkidRate_.data[LM] = SkidRateBuf_[LM];
        this->SkidRate_.data[LM+6] = rr;
        SkidRateBuf_[RR] = ComputeSkidRate(this->WheelName_[RR],this->WheelRefLinkName_[RR],RR,rr);
        abs(SkidRateBuf_[RR])<1.0e-6?:this->SkidRate_.data[RR] = SkidRateBuf_[RR];
        this->SkidRate_.data[RR+6] = rr;
        SkidRateBuf_[LR] = ComputeSkidRate(this->WheelName_[LR],this->WheelRefLinkName_[LR],LR,rr);
        abs(SkidRateBuf_[LR])<1.0e-6?:this->SkidRate_.data[LR] = SkidRateBuf_[LR];
        this->SkidRate_.data[LR+6] = rr;

        // publish the pose of six wheel in LiDAR(PandarQT) frame
        geometry_msgs::Pose this_pose;
        ignition::math::Pose3d that_pose;
        WheelPoseInBody_.header.stamp = ros::Time(this->world_->SimTime().sec,this->world_->SimTime().nsec);
        for(int i = 0;i < 6;i++)
        {
            ComputeRelativePose(this->WheelName_[i],this->BodyName_,that_pose);
            this_pose.position.x = that_pose.Pos().X();
            this_pose.position.y = that_pose.Pos().Y();
            this_pose.position.z = that_pose.Pos().Z();
            this_pose.orientation.w = that_pose.Rot().W();
            this_pose.orientation.x = that_pose.Rot().X();
            this_pose.orientation.y = that_pose.Rot().Y();
            this_pose.orientation.z = that_pose.Rot().Z();
            this->WheelPoseInBody_.poses[i] = this_pose;
        }


        this->lock.unlock();

        this->PubSkidRateQue_->push(this->SkidRate_, this->SkidRatePub_);
        this->PubWheelPoseInBodyQue_->push(this->WheelPoseInBody_, this->WheelPoseInBodyPub_);

        // save last time stamp
        this->last_time_ = cur_time;
    }
}

// Put laser data to the interface
void GazeboComputeSkidRate::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->ComputeSkidRate_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
