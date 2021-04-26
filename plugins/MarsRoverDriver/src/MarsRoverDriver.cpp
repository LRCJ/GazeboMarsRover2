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
 * \file  gazebo_ros_skid_steer_drive.cpp
 *
 * \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
 *
 * \author  Zdenek Materna (imaterna@fit.vutbr.cz) (modified by LR renliaocn@gmail.com)
 *
 * $ Id: 06/25/2013 11:23:40 AM materna $
 */

#include "MarsRoverDriver.h"

namespace gazebo
{

    enum
    {
        WRF=0,
        WLF=1,
        WRM=2,
        WLM=3,
        WRR=4,
        WLR=5,
        BRF = 6,
        BLF = 7,
        BRR = 8,
        BLR = 9,
    };

    GazeboRosSkidSteerDrive::GazeboRosSkidSteerDrive()
    {
        // Initialize velocity stuff
        wheel_speed_[WRF] = 0;
        wheel_speed_[WLF] = 0;
        wheel_speed_[WRM] = 0;
        wheel_speed_[WLM] = 0;
        wheel_speed_[WRR] = 0;
        wheel_speed_[WLR] = 0;

        x_ = rot_ = alpha_ = beta_ = 0;
        //alive_ = true;
    }

    GazeboRosSkidSteerDrive::~GazeboRosSkidSteerDrive()
    {
        
        delete rosnode_;
        delete transform_broadcaster_;
    }

    //扫描键盘输入，控制小车前进转弯
    void GazeboRosSkidSteerDrive::scanKeyboard()
    {
        double x__,rot__;
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(0,&stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= ~ICANON;//关闭标准输入处理，即默认的回车和换行符之间的映射已经不存在了
        //new_settings.c_lflag &= ~ECHO;//关闭回显功能
        //以下两行用于设置键入字符后无需按下回车即可使getchar立刻返回
        new_settings.c_cc[VTIME] = 0;
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0,TCSANOW,&new_settings);
        while(true)
        {   int c = getchar();//阻塞式的
            isCMD = false;
            ros::param::get("Speed",x__);
            ros::param::get("RotationAngle",rot__);
            switch(c)
            {
                case 0x44:
                    beta_ = -rot__;
                    // //仅前轮转向
                    // alpha_ = -atan(1.0/(wheel_separation_/wheel_base_+1.0/tan(-beta_)));
                    //前后轮同时转向
                    alpha_ = -atan(1.0/(2.0*wheel_separation_/wheel_base_+1.0/tan(-beta_)));
                    ROS_INFO("TurnLeft!!!Steering angle is %f degree.\n",beta_/3.1415926*180.0);
                    break;
                case 0x43:
                    alpha_ = rot__;//右前轮转向架转向角度，从车体上方向下看，顺时针为正
                    // //仅前轮转向
                    // beta_ = atan(1.0/(wheel_separation_/wheel_base_+1.0/tan(alpha_)));//左前轮转向架转向角度
                    //前后轮同时转向
                    beta_ = atan(1.0/(2.0*wheel_separation_/wheel_base_+1.0/tan(alpha_)));//左前轮转向架转向角度
                    ROS_INFO("TurnRight!!!Steering angle is %f degree.\n",alpha_/3.1415926*180.0);
                    break;
                case 0x41:
                ROS_DEBUG("UP");
                    this->x_ = x__;
                    alpha_ = beta_ = 0;
                    ROS_INFO("Forward!!!Speed is %f m/s.",this->x_);
                    break;
                case 0x42:
                    this->x_ = -x__;
                    alpha_ = beta_ = 0;
                    ROS_INFO("BackOff!!!Speed is %f m/s.",this->x_);
                    break;
                case 0x53:
                    this->x_ = 0.0;
                    ROS_INFO("STOP!!!");
                    break;
            }

        }
    }

    //read XBOX input
    //扫描手柄输入，控制小车前进转弯及其速度
    void GazeboRosSkidSteerDrive::scanXBOX()
    {
        double x__,rot__;
        int xbox_fd,len;
        xbox_map_t map;
        ros::param::get("Speed",x__);
        ros::param::get("RotationAngle",rot__);
        while(1)
        {
            usleep(1000*1000);
            //ROS_INFO("started to open XBOX file!");
            xbox_fd = xbox_open("/dev/input/js0");
            if(xbox_fd!=-1)
            {
                //ROS_INFO("started to read XBOX input!");
            }
            else
            {
                //ROS_INFO("open XBOX file failed!");
                continue;
            }
            while(1)
            { 
                len = xbox_map_read(xbox_fd, &map);//阻塞式的
                isCMD = false;
                if (len == -1)  
                {
                    //ROS_INFO("read XBOX input failed!");
                    break;
                }
                this->x_ = map.ly/-32767.0*x__;
                if(map.rx<0)
                {
                    //左转
                    beta_ = map.rx/32767.0*rot__;
                    // //仅前轮转向
                    // alpha_ = -atan(1.0/(wheel_separation_/wheel_base_+1.0/tan(-beta_)));
                    //前后轮同时转向
                    alpha_ = -atan(1.0/(2.0*wheel_separation_/wheel_base_+1.0/tan(-beta_)));
                    //printf("map.rx = %d,beta = %f.\n", map.rx,beta_);
                    //ROS_INFO("TurnLeft!!!Steering angle is %f degree.\n",beta_/3.1415926*180.0);
                }
                else if(map.rx>0)
                {
                    //右转
                    alpha_ = map.rx/32767.0*rot__;
                    // //仅前轮转向
                    // beta_ = atan(1.0/(wheel_separation_/wheel_base_+1.0/tan(alpha_)));//左前轮转向架转向角度
                    //前后轮同时转向
                    beta_ = atan(1.0/(2.0*wheel_separation_/wheel_base_+1.0/tan(alpha_)));//左前轮转向架转向角度
                    //printf("map.rx = %d,alpha = %f.\n", map.rx,alpha_);
                    //ROS_INFO("TurnRight!!!Steering angle is %f.\n",alpha_/3.1415926*180.0);  
                }
                else
                {
                    alpha_ = beta_ = 0;
                }
            }
        }
        xbox_close(xbox_fd);
    }

    void GazeboRosSkidSteerDrive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        isCMD = true;
        // x_ = cmd_msg->linear.x;
        // if(cmd_msg->angular.z>0)
        //     alpha_ = cmd_msg->angular.z;
        // else 
        //     beta_ = cmd_msg->angular.z;
        wheel_speed_[WRF] = cmd_msg->linear.x;
        wheel_speed_[WLF] = -cmd_msg->linear.y;
        wheel_speed_[WRM] = cmd_msg->linear.z;
        wheel_speed_[WLM] = -cmd_msg->angular.x;
        wheel_speed_[WRR] = cmd_msg->angular.y;
        wheel_speed_[WLR] = -cmd_msg->angular.z;
    }

    // Load the controller
    void GazeboRosSkidSteerDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

        this->parent = _parent;
        this->world = _parent->GetWorld();

        // get setup parameters
        this->robot_namespace_ = "";
        if (!_sdf->HasElement("robotNamespace"))
            ROS_INFO("MarsRoverDriver Plugin missing <robotNamespace>, defaults to \"%s\"",this->robot_namespace_.c_str());
        else
            this->robot_namespace_ =_sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        this->broadcast_tf_ = false;
        if (!_sdf->HasElement("broadcastTF"))
        {
            if (!this->broadcast_tf_)
                ROS_INFO("MarsRoverDriver Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
            else 
                ROS_INFO("MarsRoverDriver Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
        }
        else
            this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
        this->update_rate_ = 100.0;
        if(!_sdf->HasElement("updateRate"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
                this->robot_namespace_.c_str(), this->update_rate_);
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();



        // get the name of drived joints
        //Right front wheel
        this->RFWheelJointName = "c_BogieRF_WheelRF";
        if(!_sdf->HasElement("RFWheelJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <RFWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RFWheelJointName.c_str());
        else 
            this->RFWheelJointName = _sdf->GetElement("RFWheelJoint")->Get<std::string>();
        //left front wheel
        this->LFWheelJointName = "c_BogieLF_WheelLF";
        if(!_sdf->HasElement("LFWheelJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LFWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LFWheelJointName.c_str());
        else
            this->LFWheelJointName = _sdf->GetElement("LFWheelJoint")->Get<std::string>();
        //right middle wheel
        this->RMWheelJointName = "c_RightViceRocket_WheelRM";
        if(!_sdf->HasElement("RMWheelJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <RMWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LFWheelJointName.c_str());
        else
            this->RMWheelJointName = _sdf->GetElement("RMWheelJoint")->Get<std::string>();
        //left middle wheel
        this->LMWheelJointName = "c_LeftViceRocket_WheelLM";
        if(!_sdf->HasElement("LMWheelJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LMWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LFWheelJointName.c_str());
        else
            this->LMWheelJointName = _sdf->GetElement("LMWheelJoint")->Get<std::string>();
        //right rear wheel
        this->RRWheelJointName = "c_BogieRR_WheelRR";
        if(!_sdf->HasElement("RRWheelJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <RRWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RRWheelJointName.c_str());
        else
            this->RRWheelJointName = _sdf->GetElement("RRWheelJoint")->Get<std::string>();
        //left rear wheel
        this->LRWheelJointName = "c_BogieLR_WheelLR";
        if(!_sdf->HasElement("LRWheelJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LRWheelJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->LRWheelJointName.c_str());
        else
            this->LRWheelJointName = _sdf->GetElement("LRWheelJoint")->Get<std::string>();
        //right front bogie
        this->RFBogieJointName = "c_RightMainRocket_BogieRF";
        if(!_sdf->HasElement("RFBogieJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <RFBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RFBogieJointName.c_str());
        else
            this->RFBogieJointName = _sdf->GetElement("RFBogieJoint")->Get<std::string>();
        //left front bogie
        this->LFBogieJointName = "c_LeftMainRocket_BogieLF";
        if(!_sdf->HasElement("LFBogieJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LFBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LFBogieJointName.c_str());
        else
            this->LFBogieJointName = _sdf->GetElement("LFBogieJoint")->Get<std::string>();
        //right rear bogie
        this->RRBogieJointName = "c_RightViceRocket_BogieRR";
        if(!_sdf->HasElement("RRBogieJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <RRBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RRBogieJointName.c_str());
        else
            this->RRBogieJointName = _sdf->GetElement("RRBogieJoint")->Get<std::string>();
        //left rear bogie
        this->LRBogieJointName = "c_LeftViceRocket_BogieLR";
        if(!_sdf->HasElement("LRBogieJoint"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LRBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LRBogieJointName.c_str());
        else
            this->LRBogieJointName = _sdf->GetElement("LRBogieJoint")->Get<std::string>();



        //get rover parameters
        // This assumes that front and rear wheel spacing is identical
        /*this->wheel_separation_ = this->parent->GetJoint(LFWheelJointName)->GetAnchor(0).Distance(
            this->parent->GetJoint(RFWheelJointName)->GetAnchor(0));*/
        this->wheel_separation_ = 0.4;
        if(!_sdf->HasElement("wheelSeparation"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
                this->robot_namespace_.c_str(), this->wheel_separation_);
        else
            this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();

        this->wheel_diameter_ = 0.3;
        if(!_sdf->HasElement("wheelDiameter"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                this->robot_namespace_.c_str(), this->wheel_diameter_);
        else
            this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();

        this->wheel_base_ = 1.55052;
        if(!_sdf->HasElement("wheelBase"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <wheelBase>, defaults to %f",
                this->robot_namespace_.c_str(), this->wheel_base_);
        else
            this->wheel_base_ = _sdf->GetElement("wheelBase")->Get<double>();

        this->torque_ = 5.0;
        if(!_sdf->HasElement("torque"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <torque>, defaults to %f",
                this->robot_namespace_.c_str(), this->torque_);
        else 
            this->torque_ = _sdf->GetElement("torque")->Get<double>();



        // get publish topic name and setup parameters
        this->command_topic_ = "cmd_vel";
        if(!_sdf->HasElement("commandTopic"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->command_topic_.c_str());
        else 
            this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();

        this->path_topic_ = "path";
        if(!_sdf->HasElement("PathTopic"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->path_topic_.c_str());
        else
            this->path_topic_ = _sdf->GetElement("PathTopic")->Get<std::string>();

        this->odometry_topic_ = "odometry";
        if(!_sdf->HasElement("odometryTopic"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
        else
            this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();

        this->odometry_frame_ = "odom";
        if(!_sdf->HasElement("odometryFrame"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
        else
            this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();

        this->robot_base_frame_ = "base_footprint";
        if(!_sdf->HasElement("robotBaseFrame"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
        else
            this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();



        // get the pose of Lidar relative to body
        double LidarPose_w = 1.0;
        if(!_sdf->HasElement("LiDARtoBody_w"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_w>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_w);
        else
            LidarPose_w = _sdf->GetElement("LiDARtoBody_w")->Get<double>();
        double LidarPose_x = 0.0;
        if(!_sdf->HasElement("LiDARtoBody_x"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_x>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_x);
        else
            LidarPose_x = _sdf->GetElement("LiDARtoBody_x")->Get<double>();
        double LidarPose_y = 0.0;
        if(!_sdf->HasElement("LiDARtoBody_y"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_w>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_y);
        else
            LidarPose_y = _sdf->GetElement("LiDARtoBody_y")->Get<double>();
        double LidarPose_z = 0.0;
        if(!_sdf->HasElement("LiDARtoBody_z"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_z>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_z);
        else
            LidarPose_z = _sdf->GetElement("LiDARtoBody_z")->Get<double>();
        double LidarPose_tx = 0.0;
        if(!_sdf->HasElement("LiDARtoBody_tx"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_tx>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_tx);
        else
            LidarPose_tx = _sdf->GetElement("LiDARtoBody_tx")->Get<double>();
        double LidarPose_ty = 0.0;
        if(!_sdf->HasElement("LiDARtoBody_ty"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_ty>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_ty);
        else
            LidarPose_ty = _sdf->GetElement("LiDARtoBody_ty")->Get<double>();
        double LidarPose_tz = 0.0;
        if(!_sdf->HasElement("LiDARtoBody_tz"))
            ROS_WARN("MarsRoverDriver Plugin (ns = %s) missing <LiDARtoBody_tz>, defaults to \"%lf\"",
                this->robot_namespace_.c_str(), LidarPose_tz);
        else
            LidarPose_tz = _sdf->GetElement("LiDARtoBody_tz")->Get<double>();
        //Eigen's Quaternion --- (w,x,y,z) ; ROS's Quaternion.msg --- (x,y,z,w)
        T_bp_.rotate(Eigen::Quaterniond(LidarPose_w, LidarPose_x, LidarPose_y, LidarPose_z));
        T_bp_.pretranslate(Eigen::Vector3d(LidarPose_tx, LidarPose_ty, LidarPose_tz));


        // Initialize update rate stuff
        if(this->update_rate_ > 0.0)
            this->update_period_ = 1.0 / this->update_rate_;//update_rate_ = 100,update_period_ = 0.01
        else
            this->update_period_ = 0.0;
        last_update_time_ = this->world->SimTime();

        // get the joint ptr for setting position/velocity/force
        joints[WRF] = this->parent->GetJoint(RFWheelJointName);
        joints[WLF] = this->parent->GetJoint(LFWheelJointName);
        joints[WRM] = this->parent->GetJoint(RMWheelJointName);
        joints[WLM] = this->parent->GetJoint(LMWheelJointName);
        joints[WRR] = this->parent->GetJoint(RRWheelJointName);
        joints[WLR] = this->parent->GetJoint(LRWheelJointName);
        joints[BRF] = this->parent->GetJoint(this->RFBogieJointName);
        joints[BLF] = this->parent->GetJoint(this->LFBogieJointName);
        joints[BRR] = this->parent->GetJoint(this->RRBogieJointName);
        joints[BLR] = this->parent->GetJoint(this->LRBogieJointName);

        if(!joints[WLF])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->LFWheelJointName.c_str());
            gzthrow(error);
        }

        if(!joints[WRF])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->RFWheelJointName.c_str());
            gzthrow(error);
        }

        if(!joints[WLR])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->LRWheelJointName.c_str());
            gzthrow(error);
       }

       if(!joints[WRR])
       {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->RRWheelJointName.c_str());
            gzthrow(error);
       }

        #if GAZEBO_MAJOR_VERSION > 2
            joints[WLF]->SetParam("fmax", 0, torque_);
            joints[WRF]->SetParam("fmax", 0, torque_);
            joints[WLR]->SetParam("fmax", 0, torque_);
            joints[WRR]->SetParam("fmax", 0, torque_);
        #else
            joints[WLF]->SetMaxForce(0, torque_);
            joints[WRF]->SetMaxForce(0, torque_);
            joints[WLR]->SetMaxForce(0, torque_);
            joints[WRR]->SetMaxForce(0, torque_);
        #endif

        // Make sure the ROS node for Gazebo has already been initialized
        if(!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load MarsRoverDriver plugin. ");
            return;
        }
        else
        {
            rosnode_ = new ros::NodeHandle(this->robot_namespace_);

            ROS_INFO("Starting MarsRoverDriver Plugin (ns = %s)!", this->robot_namespace_.c_str());

            transform_broadcaster_ = new tf::TransformBroadcaster();

            //publish pose relative to world frame
            odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 2);
            //continuous pose forms the trajectory,and is visualized in rviz
            path_publisher_ = rosnode_->advertise<nav_msgs::Path>(path_topic_, 2);
            path_.header.frame_id = robot_base_frame_;
            path_.header.stamp = ros::Time::now();

            //subscribe /cmd_vel topic，control rover by publishing topic
            // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
            //ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
            //    boost::bind(&GazeboRosSkidSteerDrive::cmdVelCallback, this, _1),ros::VoidPtr(), &queue_);
            cmd_vel_subscriber_ = rosnode_->subscribe<geometry_msgs::Twist>(command_topic_, 2, &GazeboRosSkidSteerDrive::cmdVelCallback, this);

            //create 2 new thread for processing keyboard and handle input
            //read key board input
            ROS_INFO("started to create a thread to process keyboard input!");
            boost::thread f1 = boost::thread(boost::bind(&GazeboRosSkidSteerDrive::scanKeyboard, this));
            f1.detach();
            //read XBOX controller input
            ROS_INFO("started to create a thread to process XBOX input!");
            boost::thread f2 = boost::thread(boost::bind(&GazeboRosSkidSteerDrive::scanXBOX, this));
            f2.detach();

            // start custom queue for diff drive
            //this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosSkidSteerDrive::QueueThread, this));

            // listen to the update event (broadcast every simulation iteration)
            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboRosSkidSteerDrive::UpdateChild, this));

        }
    }

  // Update the controller
    void GazeboRosSkidSteerDrive::UpdateChild()
    {
        common::Time current_time = this->world->SimTime();//这是Gazebo仿真时间，等于仿真步长乘以步数
        double seconds_since_last_update = (current_time - last_update_time_).Double();
        if(seconds_since_last_update > update_period_)
        {
            publishOdometry(seconds_since_last_update);
            // Update robot in case new velocities have been requested
            getWheelVelocities();
            #if GAZEBO_MAJOR_VERSION > 2
                joints[WRF]->SetParam("vel", 0, wheel_speed_[WRF] / (wheel_diameter_ / 2.0));
                joints[WLF]->SetParam("vel", 0, wheel_speed_[WLF] / (wheel_diameter_ / 2.0));
                joints[WRM]->SetParam("vel", 0, wheel_speed_[WRM] / (wheel_diameter_ / 2.0));
                joints[WLM]->SetParam("vel", 0, wheel_speed_[WLM] / (wheel_diameter_ / 2.0));
                joints[WRR]->SetParam("vel", 0, wheel_speed_[WRR] / (wheel_diameter_ / 2.0));
                joints[WLR]->SetParam("vel", 0, wheel_speed_[WLR] / (wheel_diameter_ / 2.0));
            #else
                joints[WRF]->SetVelocity(0, wheel_speed_[WRF] / (wheel_diameter_ / 2.0));
                joints[WLF]->SetVelocity(0, wheel_speed_[WLF] / (wheel_diameter_ / 2.0));
                joints[WRM]->SetVelocity(0, wheel_speed_[WRM] / (wheel_diameter_ / 2.0));
                joints[WLM]->SetVelocity(0, wheel_speed_[WLM] / (wheel_diameter_ / 2.0));
                joints[WRR]->SetVelocity(0, wheel_speed_[WRR] / (wheel_diameter_ / 2.0));
                joints[WLR]->SetVelocity(0, wheel_speed_[WLR] / (wheel_diameter_ / 2.0));
            #endif
            
            // //仅前轮转向
            // joints[BRF]->SetPosition(0,alpha_);
            // joints[BLF]->SetPosition(0,beta_);
            // joints[BRR]->SetPosition(0,0.0);
            // joints[BLR]->SetPosition(0,0.0);
            //前后轮同时转向
            joints[BRF]->SetPosition(0,alpha_);
            joints[BLF]->SetPosition(0,beta_);
            joints[BRR]->SetPosition(0,-alpha_);
            joints[BLR]->SetPosition(0,-beta_);

            last_update_time_+= common::Time(update_period_);
        }
    }
    /*
    void GazeboRosSkidSteerDrive::QueueThread()
    {
        static const double timeout = 0.01;
        while(alive_ && rosnode_->ok())
            queue_.callAvailable(ros::WallDuration(timeout));
    }
    // Finalize the controller
    void GazeboRosSkidSteerDrive::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        rosnode_->shutdown();
        callback_queue_thread_.join();
    }*/
    void GazeboRosSkidSteerDrive::getWheelVelocities()
    {
        boost::mutex::scoped_lock scoped_lock(lock);

        if(alpha_>0)
        {
            // //右转，仅前轮转向
            // double w = x_/(wheel_base_/tan(alpha_)+wheel_separation_/2);
            // wheel_speed_[WRF]= w * ( wheel_base_/sin(alpha_) );
            // wheel_speed_[WLF] = -w * ( wheel_base_/sin(beta_) );
            // wheel_speed_[WRR] = w * ( wheel_base_/tan(alpha_) );
            // wheel_speed_[WLR]  = -w * ( wheel_base_/tan(beta_) );


            //右转，前后轮同时转向
            double w = x_/(wheel_base_/2.0/tan(alpha_)+wheel_separation_/2.0);
            wheel_speed_[WRF] = w * ( wheel_base_/2.0/sin(alpha_) );
            wheel_speed_[WLF] = -w * ( wheel_base_/2.0/sin(beta_) );
            wheel_speed_[WRM] = w * ( wheel_base_/2.0/tan(alpha_) );
            wheel_speed_[WLM] = -w * ( wheel_base_/2.0/tan(alpha_)+wheel_separation_ );
            wheel_speed_[WRR] = wheel_speed_[WRF];
            wheel_speed_[WLR] = wheel_speed_[WLF];

        }
        else if(alpha_ < 0)
        {
            // //左转，仅前轮转向
            // double w = x_/(wheel_base_/tan(-beta_)+wheel_separation_/2);
            // wheel_speed_[WRR] = w * ( wheel_base_/tan(-alpha_) );
            // wheel_speed_[WLR]  = -w * ( wheel_base_/tan(-beta_) );
            // wheel_speed_[WRF]= w * ( wheel_base_/sin(-alpha_) );
            // wheel_speed_[WLF] = -w * ( wheel_base_/sin(-beta_) );

            //左转，前后轮同时转向
            double w = x_/(wheel_base_/2.0/tan(-beta_)+wheel_separation_/2.0);
            wheel_speed_[WRF] = w * ( wheel_base_/2.0/tan(-alpha_) );
            wheel_speed_[WLF] = -w * ( wheel_base_/2.0/tan(-beta_) );
            wheel_speed_[WRM] = w * ( wheel_base_/2.0/tan(-beta_)+wheel_separation_ );
            wheel_speed_[WLM] = -w * ( wheel_base_/2.0/tan(-beta_) );
            wheel_speed_[WRR] = wheel_speed_[WRF];
            wheel_speed_[WLR] = wheel_speed_[WLF];
        }
        else
        {
            //直行
            if(isCMD)
            {
                ;
            }
            else
            {
                wheel_speed_[WRR] = x_;
                wheel_speed_[WLR]  = -x_;
                wheel_speed_[WRF]= x_;
                wheel_speed_[WLF] = -x_; 
            }
        }
        

    }

    void GazeboRosSkidSteerDrive::publishOdometry(double step_time)
    {
        //这是ROS的时间，如果设置了参数use_sim_time为true，则ROS会使用/clock数据作为时间基准
        //此处/clock由gazebo publish，因此ROS时间与gazebo仿真时间同步
        //如果要获得真实时间，或者说操作系统时间，需要使用ros::WallTime::now()
        ros::Time current_time = ros::Time::now();

        // TODO create some non-perfect odometry!
        // getting data for robot_base_frame_ to odometry_frame_
        ignition::math::Pose3<double> pose = this->parent->WorldPose();

        //设置累计位姿信息，并使用“path_topic_”Topic发布累计位姿信息，rivz可使用该数据可视化运动轨迹
        //该位姿信息是车体body相对于世界坐标系world而言的，通过位姿转换可将轨迹转换为Lidar的轨迹
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.frame_id = "PandarQT";
        this_pose.header.stamp = current_time;
        //world->body
        Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
        T_wb.rotate(Eigen::Quaterniond(pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z()));
        T_wb.pretranslate(Eigen::Vector3d(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z()));
        //world->PandarQT
        Eigen::Isometry3d T_wp = T_wb*T_bp_;//右乘，T_bp_是在body坐标系下Lidar的位姿描述
        this_pose.pose.position.x =  T_wp.translation()(0);
        this_pose.pose.position.y =  T_wp.translation()(1);
        this_pose.pose.position.z =  T_wp.translation()(2);
        this_pose.pose.orientation.x = Eigen::Quaterniond(T_wp.rotation()).x();
        this_pose.pose.orientation.y = Eigen::Quaterniond(T_wp.rotation()).y();
        this_pose.pose.orientation.z = Eigen::Quaterniond(T_wp.rotation()).z();
        this_pose.pose.orientation.w = Eigen::Quaterniond(T_wp.rotation()).w();
        path_.poses.push_back(this_pose);

        path_publisher_.publish(path_);

        //将odometry_frame_和robot_base_frame_之间的位姿信息广播到tf tree
        tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        if(this->broadcast_tf_)
        {
            transform_broadcaster_->sendTransform(
            tf::StampedTransform(tf::Transform(qt, vt), current_time, robot_base_frame_, odometry_frame_));
        }

        //将odometry_frame_和robot_base_frame_之间的位姿信息利用“odometry_topic_”Topic发布出去
        odom_.header.stamp = current_time;
        odom_.header.frame_id = robot_base_frame_;
        odom_.child_frame_id = odometry_frame_;

        odom_.pose.pose.position.x = pose.Pos().X();
        odom_.pose.pose.position.y = pose.Pos().Y();
        odom_.pose.pose.position.z = pose.Pos().Z();
        odom_.pose.pose.orientation.x = pose.Rot().X();
        odom_.pose.pose.orientation.y = pose.Rot().Y();
        odom_.pose.pose.orientation.z = pose.Rot().Z();
        odom_.pose.pose.orientation.w = pose.Rot().W();

        ignition::math::Vector3<double> linear,angular;
        linear = this->parent->WorldLinearVel();
        angular = this->parent->WorldAngularVel();
        odom_.twist.twist.linear.x = linear.X();
        odom_.twist.twist.linear.y = linear.Y();
        odom_.twist.twist.linear.z = linear.Z();
        odom_.twist.twist.angular.x = angular.X();
        odom_.twist.twist.angular.y = angular.Y();
        odom_.twist.twist.angular.z = angular.Z();

        odometry_publisher_.publish(odom_);
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosSkidSteerDrive)
}
