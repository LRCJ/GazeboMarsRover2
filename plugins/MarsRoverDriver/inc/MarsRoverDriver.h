/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_skid_steer_drive.h
 *
 * \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
 *
 * \author  Zdenek Materna (imaterna@fit.vutbr.cz) (modified by LR renliaocn@gmail.com)
 *
 * $ Id: 06/25/2013 11:23:40 AM materna $
 */

#ifndef GAZEBO_ROS_SKID_STEER_DRIVE_H_
#define GAZEBO_ROS_SKID_STEER_DRIVE_H_

//gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

//math
//#include <gazebo/math/gzmath.hh>
//#include <ignition/math4/ignition/math.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

//Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

//for keyboard input
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include "XBOX.h" 

namespace gazebo
{
    class Joint;
    class Entity;

    class GazeboRosSkidSteerDrive : public ModelPlugin
    {
        public:
            GazeboRosSkidSteerDrive();
            ~GazeboRosSkidSteerDrive();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        protected:
            virtual void UpdateChild();
            //virtual void FiniChild();

        private:
            void publishOdometry(double step_time);
            void getWheelVelocities();
            void scanKeyboard();
            void scanXBOX();

            physics::WorldPtr world;
            physics::ModelPtr parent;
            event::ConnectionPtr update_connection_;
            
            std::string RFWheelJointName;
            std::string LFWheelJointName;
            std::string RMWheelJointName;
            std::string LMWheelJointName;
            std::string RRWheelJointName;
            std::string LRWheelJointName;
            std::string RFBogieJointName;
            std::string LFBogieJointName;
            std::string RRBogieJointName;
            std::string LRBogieJointName;

            double wheel_separation_;
            double wheel_diameter_;
            double torque_;
            double wheel_base_;

            physics::JointPtr joints[10];

            // ROS STUFF
            ros::NodeHandle* rosnode_;
            ros::Publisher odometry_publisher_;
            ros::Publisher path_publisher_;
            ros::Subscriber cmd_vel_subscriber_;
            tf::TransformBroadcaster *transform_broadcaster_;
            nav_msgs::Odometry odom_;
            nav_msgs::Path path_;
            std::string tf_prefix_;
            bool broadcast_tf_;

            boost::mutex lock;

            std::string robot_namespace_;
            std::string command_topic_;
            std::string odometry_topic_;
            std::string path_topic_;
            std::string odometry_frame_;
            std::string robot_base_frame_;

            // Custom Callback Queue
            //ros::CallbackQueue queue_;
            //boost::thread callback_queue_thread_;
            //void QueueThread();

            // DiffDrive stuff
            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

            double x_,rot_;
            double wheel_speed_[6];
            double alpha_,beta_;
            //bool alive_;
            bool isCMD;

            // Update Rate
            double update_rate_;
            double update_period_;
            common::Time last_update_time_;

            double covariance_x_;
            double covariance_y_;
            double covariance_yaw_;

            Eigen::Isometry3d T_bp_ = Eigen::Isometry3d::Identity();

    };
}
#endif /* GAZEBO_ROS_SKID_STEER_DRIVE_H_ */
