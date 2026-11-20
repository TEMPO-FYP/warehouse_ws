/*
Copyright (c) 2019-2020, Juan Miguel Jimeno

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <quadruped_controller.h>
#include <cmath>

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}

QuadrupedController::QuadrupedController():
    Node("quadruped_controller_node",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    knee_orientation_(">>"),
    body_controller_(base_),
    leg_controller_(base_, rosTimeToChampTime(this->now())),
    kinematics_(base_),
    publish_foot_contacts_(false),
    publish_joint_states_(false),
    publish_joint_control_(true),
    in_gazebo_(false),
    joint_command_time_(1.0 / 60.0)
{
    std::string joint_control_topic = "joint_group_position_controller/command";
    std::string urdf = "";

    req_vel_.linear.x = 0.0f;
    req_vel_.linear.y = 0.0f;
    req_vel_.linear.z = 0.0f;
    req_vel_.angular.x = 0.0f;
    req_vel_.angular.y = 0.0f;
    req_vel_.angular.z = 0.0f;
    req_pose_.position.x = 0.0f;
    req_pose_.position.y = 0.0f;
    req_pose_.position.z = 0.0f;
    req_pose_.orientation.roll = 0.0f;
    req_pose_.orientation.pitch = 0.0f;
    req_pose_.orientation.yaw = 0.0f;

    double loop_rate = 200.0;

    this->get_parameter("gait.pantograph_leg",         gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.swing_duration",         gait_config_.swing_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("gait.knee_orientation",       knee_orientation_);
    this->get_parameter("publish_foot_contacts",       publish_foot_contacts_);
    this->get_parameter("publish_joint_states",        publish_joint_states_);
    this->get_parameter("publish_joint_control",       publish_joint_control_);
    this->get_parameter("gazebo",                      in_gazebo_);
    this->get_parameter("joint_controller_topic",      joint_control_topic);
    this->get_parameter("loop_rate",                   loop_rate);
    this->get_parameter("urdf",                        urdf);
    if (!this->has_parameter("joint_command_time"))
    {
        this->declare_parameter("joint_command_time", joint_command_time_);
    }
    this->get_parameter("joint_command_time",          joint_command_time_);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&QuadrupedController::onSetParameters_, this, std::placeholders::_1));
    
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel/smooth", 10, std::bind(&QuadrupedController::cmdVelCallback_, this,  std::placeholders::_1));
    cmd_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "body_pose", 1,  std::bind(&QuadrupedController::cmdPoseCallback_, this,  std::placeholders::_1));
    
    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    if(publish_foot_contacts_ && !in_gazebo_)
    {
        foot_contacts_publisher_   = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);
    }

    gait_config_.knee_orientation = knee_orientation_.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);
    float crouch_joints[12] = {
        0.0f, 0.67f, -1.30f,
        0.0f, 0.67f, -1.30f,
        0.0f, 0.67f, -1.30f,
        0.0f, 0.67f, -1.30f
    };
    std::vector<double> init_joints;
    if (this->get_parameter("initial_joint_positions", init_joints) && init_joints.size() == 12)
    {
        for (size_t i = 0; i < 12; ++i)
        {
            crouch_joints[i] = static_cast<float>(init_joints[i]);
        }
    }
    base_.updateJointPositions(crouch_joints);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());
    loop_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration::from_seconds(1.0 / loop_rate),
        std::bind(&QuadrupedController::controlLoop_, this));
    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_()
{
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];
    bool foot_contacts[4] = {false, false, false, false};

    base_.getJointPositions(target_joint_positions);
    body_controller_.poseCommand(target_foot_positions, req_pose_);

    leg_controller_.velocityCommand(target_foot_positions, req_vel_, rosTimeToChampTime(this->now()));
    kinematics_.inverse(target_joint_positions, target_foot_positions);
    base_.updateJointPositions(target_joint_positions);

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{   
    
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.x = msg->position.x;
    req_pose_.position.y = msg->position.y;
    req_pose_.position.z = msg->position.z +  gait_config_.nominal_height;
}

void QuadrupedController::publishJoints_(float target_joints[12])
{   
    if(publish_joint_control_)
    {
        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = this->now();
        joints_cmd_msg.header.stamp.sec = 0;
        joints_cmd_msg.header.stamp.nanosec = 0;
        
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = rclcpp::Duration::from_seconds(joint_command_time_);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        sensor_msgs::msg::JointState joints_msg;

        joints_msg.header.stamp = this->now();

        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {    
            joints_msg.position[i]= target_joints[i];
        }

        joint_states_publisher_->publish(joints_msg);
    }
}

void QuadrupedController::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_ && !in_gazebo_)
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = this->now();
        contacts_msg.contacts.resize(4);
        
        std::string s2;
       for(size_t i = 0; i < 4; i++)
        {
            //This is only published when there's no feedback on the robot
            //that a leg is in contact with the ground
            //For such cases, we use the stance phase in the gait for foot contacts
            contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
            s2.append(std::to_string(contacts_msg.contacts[i]) + " ");
        }
        foot_contacts_publisher_->publish(contacts_msg);
    }
}

rcl_interfaces::msg::SetParametersResult QuadrupedController::onSetParameters_(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    double next_joint_command_time = joint_command_time_;
    champ::GaitConfig next_gait = gait_config_;
    bool gait_changed = false;
    bool nominal_height_changed = false;

    auto read_double = [&](const rclcpp::Parameter & p, double & out) -> bool {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            out = p.as_double();
        } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            out = static_cast<double>(p.as_int());
        } else {
            result.successful = false;
            result.reason = "Parameter '" + p.get_name() + "' must be numeric";
            return false;
        }
        if (!std::isfinite(out)) {
            result.successful = false;
            result.reason = "Parameter '" + p.get_name() + "' must be finite";
            return false;
        }
        return true;
    };

    for (const auto & p : parameters) {
        const std::string & name = p.get_name();
        double val = 0.0;
        if (name == "joint_command_time") {
            if (!read_double(p, val)) { return result; }
            if (val <= 0.0) {
                result.successful = false;
                result.reason = "joint_command_time must be > 0";
                return result;
            }
            next_joint_command_time = val;
        } else if (name == "gait.nominal_height") {
            if (!read_double(p, val)) { return result; }
            if (val <= 0.0) {
                result.successful = false;
                result.reason = "gait.nominal_height must be > 0";
                return result;
            }
            next_gait.nominal_height = static_cast<float>(val);
            gait_changed = true;
            nominal_height_changed = true;
        } else if (name == "gait.swing_height") {
            if (!read_double(p, val)) { return result; }
            next_gait.swing_height = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.com_x_translation") {
            if (!read_double(p, val)) { return result; }
            next_gait.com_x_translation = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.stance_depth") {
            if (!read_double(p, val)) { return result; }
            next_gait.stance_depth = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.stance_duration") {
            if (!read_double(p, val)) { return result; }
            if (val <= 0.0) {
                result.successful = false;
                result.reason = "gait.stance_duration must be > 0";
                return result;
            }
            next_gait.stance_duration = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.swing_duration") {
            if (!read_double(p, val)) { return result; }
            if (val <= 0.0) {
                result.successful = false;
                result.reason = "gait.swing_duration must be > 0";
                return result;
            }
            next_gait.swing_duration = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.max_linear_velocity_x") {
            if (!read_double(p, val)) { return result; }
            next_gait.max_linear_velocity_x = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.max_linear_velocity_y") {
            if (!read_double(p, val)) { return result; }
            next_gait.max_linear_velocity_y = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.max_angular_velocity_z") {
            if (!read_double(p, val)) { return result; }
            next_gait.max_angular_velocity_z = static_cast<float>(val);
            gait_changed = true;
        } else if (name == "gait.odom_scaler") {
            if (!read_double(p, val)) { return result; }
            next_gait.odom_scaler = static_cast<float>(val);
            gait_changed = true;
        }
    }

    joint_command_time_ = next_joint_command_time;
    if (gait_changed) {
        gait_config_ = next_gait;
        gait_config_.knee_orientation = knee_orientation_.c_str();
        base_.setGaitConfig(gait_config_);
        if (nominal_height_changed) {
            req_pose_.position.z = gait_config_.nominal_height;
        }
    }

    return result;
}
