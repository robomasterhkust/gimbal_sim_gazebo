/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <stdio.h>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <tf2/LinearMath/Quaternion.h>

namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief Sets velocity on a link or joint
  class GimbalControlPlugin : public ModelPlugin
  {
    public: physics::JointControllerPtr jointController;

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;

        //Initialization
        this->model->GetJoint("gimbal_yaw_joint")->SetPosition(0, yawPos);
        this->model->GetJoint("gimbal_pitch_joint")->SetPosition(0, pitchPos);

        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>
          (
              //this->model->GetName() + "/motion_cmd",
              // modify the following line to your own teleop topic name
              // attention:
              // different teleop may use different angular or linear velocity
              "/cmd_vel",
              1,
              boost::bind(&GimbalControlPlugin::OnRosMsg, this, _1),
              ros::VoidPtr(), &this->rosQueue
          );
        this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread = std::thread(std::bind(&GimbalControlPlugin::QueueThread, this));
    }

    /**
     *  @brief      Ros msg handler for controlling the simulated gimbal
     *  @param[in]  _msg control command in body frame position rotation or angular velocity
     */
    public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr &_msg)
    {
        //Get the delta-time
        static ros::Time prev;
        static double dt;

        static bool prev_isValid = false;

        //Attitude estimator
        tf2::Quaternion q;
        q.setEulerZYX(yawPos, pitchPos, 0);

        //The speed control mode, need to calculate dt
        if(_msg->angular.x == 1.0) //This indicates we are using speed command mode
        {

            if(prev_isValid)
            {
                ros::Duration duration = ros::Time::now() - prev;
                dt = duration.toSec();

                if(dt > 0.01)
                {
                    std::cerr<<"Too low frequency for vel command!"<<std::endl;
                    return;
                }
            }
            else
                dt = 0;

            prev = ros::Time::now();
            prev_isValid = true; //Only use prev time to calculate dt

//          //yawPos += _msg->angular.z * dt; //Yaw vel command
//          //pitchPos -= _msg->linear.x * dt; //Pitch vel command

            double yawVel = _msg->angular.z; //Yaw vel command
            double pitchVel = _msg->angular.y; //Pitch vel command

            //quaternion derivative
            tf2::Quaternion w = tf2::Quaternion(0, pitchVel, yawVel, 0);
            w *= q;

            tf2::Quaternion dq = w * (0.5 * dt);
            q += dq;
            q /= length(q);

            pitchPos = asin(2*(q.w() * q.y() - q.z() * q.x()));
            yawPos = atan2(2*(q.w() * q.z() + q.x() * q.y()),
                            1 - 2*(q.y() * q.y() + q.z() * q.z()));

        }
        else if(_msg->angular.x == 0.0) //This indicates we are using angle command mode
        {
            //double yaw_Ang = _msg->linear.z, pitch_Ang = _msg->linear.y;

            double yaw_Ang = 0, pitch_Ang = 0;
            if( _msg->angular.z > 0)
                yaw_Ang = 1.0;
            else if( _msg->angular.z < 0)
                yaw_Ang = -1.0;

            if( _msg->angular.y > 0)
                pitch_Ang = -0.5;
            else if( _msg->angular.y < 0)
                pitch_Ang = 0.5;

            //Prevent the turning angle to become too big
            if(
                yaw_Ang > 1.0 || yaw_Ang < -1.0 ||
                pitch_Ang > 1.0 || pitch_Ang < -1.0
                )
            {
                std::cerr<<"Too big turning angle!"<<std::endl;
                return;
            }

            static bool yaw_locked, pitch_locked, updated;
            if(yaw_locked) yaw_Ang = 0;
            if(pitch_locked) pitch_Ang = 0;
            if(_msg->angular.z != 0)
            {
                if(!yaw_locked)
                    updated = true;
                yaw_locked = true;
            }
            else yaw_locked = false;
            if(_msg->angular.y != 0)
            {
                if(!pitch_locked)
                    updated = true;
                pitch_locked = true;
            }
            else pitch_locked = false;

            double encoder_diff = (this->model->GetJoint("gimbal_pitch_joint")->GetAngle(0)).Radian();

            //====equivalent rotation calculation=====
            double theta_minus_pitch = asin(sin(-encoder_diff) * cos(pitch_Ang) * cos(yaw_Ang) -
                cos(encoder_diff) * sin(pitch_Ang));
            //These two are predicted motor rotation angle based on equivalent rotations
            double pitch_Axis = -encoder_diff - theta_minus_pitch;
            double yaw_Axis = asin(cos(pitch_Ang) * sin(yaw_Ang) / cos(theta_minus_pitch));
            //========================================
            tf2::Quaternion qRot_imag = tf2::Quaternion(
                                                    -sin(yaw_Ang/2)* sin(pitch_Ang/2),
                                                    cos(yaw_Ang/2)* sin(pitch_Ang/2),
                                                    sin(yaw_Ang/2)* cos(pitch_Ang/2),
                                                    cos(yaw_Ang/2)* cos(pitch_Ang/2)
                                                );
            tf2::Quaternion qImag = q;
            qImag *= qRot_imag;// This is an imagnary rotation on camera body axis

            tf2::Quaternion qRot = tf2::Quaternion(
                                                    -sin(yaw_Axis/2)* sin(pitch_Axis/2 + encoder_diff),
                                                    cos(yaw_Axis/2)* sin(pitch_Axis/2),
                                                    sin(yaw_Axis/2)* cos(pitch_Axis/2 + encoder_diff),
                                                    cos(yaw_Axis/2)* cos(pitch_Axis/2)
                                                );
            q *= qRot; //This is the real rotation performed by the motor axis

            pitchPos = asin(2*(q.w() * q.y() - q.z() * q.x()));
            yawPos = atan2(2*(q.w() * q.z() + q.x() * q.y()),
                            1 - 2*(q.y() * q.y() + q.z() * q.z()));

            if(updated)
            {
                double yaw_imag = atan2(2*(qImag.w() * qImag.z() + qImag.x() * qImag.y()),
                                1 - 2*(qImag.y() * qImag.y() + qImag.z() * qImag.z())),
                       pitch_imag = asin(2*(qImag.w() * qImag.y() - qImag.z() * qImag.x()));
                std::cout<<"ImagPos"<<pitch_imag<<' '<<yaw_imag<<std::endl;
                std::cout<<"RealPos"<<pitchPos<<' '<<yawPos<<std::endl;
                updated = false;
            }

            prev_isValid = false; //Invalidate the previous time stored
        }

        //Limit position
        if(yawPos > 1.5707)
            yawPos = 1.5707;
        if(yawPos < -1.5707)
            yawPos = -1.5707;

        if(pitchPos > 1.0)
            pitchPos = 1.0;
        if(pitchPos < -1.0)
            pitchPos = -1.0;

        this->model->GetJoint("gimbal_yaw_joint")->SetPosition(0, yawPos);
        this->model->GetJoint("gimbal_pitch_joint")->SetPosition(0, pitchPos);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    /// \brief a pointer to the model this plugin was loaded by
    public: physics::ModelPtr model;
    /// \brief object for callback connection

    private:
        double yawPos   = 0;
        double pitchPos = 0;
        int    rev      = 0; //Num of revolutions for the gimbal

        /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(GimbalControlPlugin)
}
