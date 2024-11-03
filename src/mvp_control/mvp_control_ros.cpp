/*
    This file is part of MVP-Control program.

    MVP-Control is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MVP-Control is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MVP-Control.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022
    Author: Mingxi ZHou
    Email: mzhou@uri.edu
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/

#include "mvp_control/mvp_control_ros.hpp"
#include "mvp_control/exception.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/time.h"
#include "mvp_control/dictionary.hpp"

#include "boost/regex.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <functional>
#include <memory>


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using GetControlModes = mvp_msgs::srv::GetControlModes;
using SetControlPoint = mvp_msgs::srv::SetControlPoint;
using GetControlMode = mvp_msgs::srv::GetControlMode;
using namespace ctrl;

using namespace std::chrono_literals;

MvpControlROS::MvpControlROS(std::string name) : Node(name)
{
    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);


    m_process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    m_set_point = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    //ROS Params
    /**
     * Read basic configuration. Configuration regarding to thruster allocation
     * will be read later.
     */
    // // Read configuration: enabled
    this->declare_parameter(CONF_ENABLED, false);
    this->get_parameter(CONF_ENABLED, m_enabled);

    // // Read configuration: tf prefix
    std::string tf_prefix;
    this->declare_parameter(CONF_TF_PREFIX, CONF_TF_PREFIX_DEFAULT);
    this->get_parameter(CONF_TF_PREFIX, tf_prefix);
    m_tf_prefix = tf_prefix.empty() ? CONF_TF_PREFIX_DEFAULT : tf_prefix + "/";

    // Read configuration: center of gravity link
    std::string cg_link_id;
    this->declare_parameter(CONF_CG_LINK, "");
    this->get_parameter(CONF_CG_LINK, cg_link_id);
    m_cg_link_id = m_tf_prefix + cg_link_id;

    //delcare a child link for all velocity controls
    std::string child_link_id;
    this->declare_parameter(CONF_CHILD_LINK, CONF_CHILD_LINK_DEFAULT);
    this->get_parameter(CONF_CHILD_LINK, child_link_id);
    m_child_link_id = m_cg_link_id;//m_tf_prefix +s child_link_id;
    // m_child_link_id = m_tf_prefix + child_link_id;

    // Read configuration: world link
    this->declare_parameter(CONF_WORLD_LINK, CONF_WORLD_LINK_DEFAULT);
    this->get_parameter(CONF_WORLD_LINK, m_world_link_id);
    m_world_link_id = m_tf_prefix + m_world_link_id;

    // Read configuration: odometry topic id
    std::string odometry_topic;
    this->declare_parameter(CONF_ODOMETRY_SOURCE, CONF_ODOMETRY_SOURCE_DEFAULT);
    this->get_parameter(CONF_ODOMETRY_SOURCE, odometry_topic);

    this->declare_parameter(CONF_CONTROLLER_FREQUENCY, 10.0);
    this->get_parameter(CONF_CONTROLLER_FREQUENCY, m_controller_frequency);

    this->declare_parameter(CONF_NO_SETPOINT_TIMEOUT, 10.0);
    this->get_parameter(CONF_NO_SETPOINT_TIMEOUT, m_no_setpoint_timeout);

    //control config file location
    this->declare_parameter("config_file", "control.yaml");
    this->get_parameter("config_file", m_control_config_file);
    //End of ROS Params

    /**
     * Initialize Subscribers
     */
    m_odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
                                odometry_topic, 
                                100, 
                                std::bind(&MvpControlROS::f_cb_msg_odometry, this, _1) 
                                );

    m_set_point_subscriber = this->create_subscription<mvp_msgs::msg::ControlProcess>(
                                TOPIC_CONTROL_PROCESS_SET_POINT, 
                                100, 
                                std::bind(&MvpControlROS::f_cb_srv_set_point, this, _1) 
                                );

    /**
     * Initialize publishers
     */
    m_process_value_publisher = this->create_publisher<mvp_msgs::msg::ControlProcess>(TOPIC_CONTROL_PROCESS_VALUE, 100);
    m_process_error_publisher = this->create_publisher<mvp_msgs::msg::ControlProcess>(TOPIC_CONTROL_PROCESS_ERROR, 100);
    m_controller_state_publisher = this->create_publisher<std_msgs::msg::Bool>(TOPIC_CONTROLLER_STATE, 100);

    /**
     * Initialize services
     */
    m_get_control_modes_server = this->create_service<GetControlModes>(
        SERVICE_GET_CONTROL_MODES,
        std::bind(&MvpControlROS::f_cb_srv_get_control_modes, this, _1, _2)
        );

    m_set_control_point_server = this->create_service<SetControlPoint>(
        SERVICE_SET_CONTROL_POINT,
        std::bind(&MvpControlROS::f_cb_srv_set_control_point, this, _1, _2)
        );

    // m_enable_controller_server = this->create_service<std_srvs::srv::Empty>(
    //     SERVICE_CONTROL_ENABLE,
    //     std::bind(&MvpControlROS::f_cb_srv_enable, this, _1, _2)
    //     );

    // m_disable_controller_server = this->create_service<std_srvs::srv::Empty>(
    //     SERVICE_CONTROL_DISABLE,
    //     std::bind(&MvpControlROS::f_cb_srv_disable, this, _1, _2)
    //     );

    m_set_controller_server = this->create_service<std_srvs::srv::SetBool>(
        SERVICE_SET_CONTROLLER,
        std::bind(&MvpControlROS::f_cb_srv_set_controller, this, _1, _2)
    );

    m_get_controller_state_server = this->create_service<std_srvs::srv::Trigger>(
        SERVICE_GET_CONTROLLER_STATE,
        std::bind(&MvpControlROS::f_cb_srv_get_controller_state, this, _1, _2)
        );


    m_get_active_mode_server = this->create_service<GetControlMode>(
        SERVICE_GET_ACTIVE_MODE,
        std::bind(&MvpControlROS::f_cb_srv_get_active_mode, this, _1, _2)
        );

    // /**
    //  * Initialize the actual controller
    //  */
    m_mvp_control.reset(new MvpControl());

}


bool MvpControlROS::f_cb_srv_get_controller_state(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    resp->success = true;
    if(m_enabled){
         resp->message = "enabled";
    }
    else{
        resp->message = "disabled";
    }

    return true;
}


void MvpControlROS::f_generate_control_allocation_matrix() {

    // Read generator type
    std::string generator_type;

    this->declare_parameter(CONF_GENERATOR_TYPE, CONF_GENERATOR_TYPE_OPT_TF);
    this->get_parameter(CONF_GENERATOR_TYPE, generator_type);
    

    // Parse the control allocation generator type and save it as enum type
    if(generator_type == CONF_GENERATOR_TYPE_OPT_TF) {
        m_generator_type = GeneratorType::TF;
        f_generate_control_allocation_from_tf();
    } else if (generator_type == CONF_GENERATOR_TYPE_OPT_USER) {
        m_generator_type = GeneratorType::USER;
        throw control_ros_exception(
            "this option is depreciated please use the TF option"
        );
    } else {
        m_generator_type = GeneratorType::UNKNOWN;
        throw control_ros_exception(
            "control allocation generation method unspecified"
        );
    }

    // Conduct some checks to see if everything is ready to be initialized
    if(m_thrusters.empty()) {
        throw control_ros_exception("no thruster specified");
    }

    // Control allocation matrix is generated based on each thruster. Each
    // thruster must have equal number of elements in their contribution matrix.
    // Code below checks the validity of the contribution vectors for each
    // thruster.
    for(unsigned int i = 0 ; i < m_thrusters.size() - 1 ; i++ ) {
        if (m_thrusters[i]->get_contribution_vector().size() !=
            m_thrusters[i + 1]->get_contribution_vector().size()) {
            throw control_ros_exception(
                "contribution vector sizes doesn't match"
            );
        }
    }

    // Initialize the control allocation matrix based on zero matrix.
    // M by N matrix. M -> number of all controllable DOF, N -> number of
    // thrusters
    m_control_allocation_matrix = Eigen::MatrixXd::Zero(
        CONTROLLABLE_DOF_LENGTH, (int) m_thrusters.size()
    );

    // Until this point, all the allocation matrix related issued must be
    // solved or exceptions thrown.


    // Acquire DOF per actuator. Register it to control allocation matrix.
    // Only DOF::X, DOF::Y and DOF::Z are left unregistered. They are computed
    // online after each iteration.
    for (uint64_t i = 0; i < m_thrusters.size(); i++) {
        for(const auto& j :
            {DOF::ROLL, DOF::PITCH, DOF::YAW,
             DOF::U, DOF::V, DOF::W,
             DOF::P, DOF::Q, DOF::R
             })
        {
            m_control_allocation_matrix(j, i) =
                m_thrusters[i]->get_contribution_vector()(j);
        }
    }
    // std::cout<< m_control_allocation_matrix<<std::endl;

    // Finally, set the control allocation matrix for the controller object.
    m_mvp_control->set_control_allocation_matrix(m_control_allocation_matrix);

}


void MvpControlROS::f_generate_control_allocation_from_tf() {
    // double time; //used for tf lookup
    auto steady_clock = rclcpp::Clock();
    rclcpp::Time now = this->get_clock()->now();

    // For each thruster look up transformation
    for(const auto& t : m_thrusters) {
        now = this->get_clock()->now();
        Eigen::Isometry3d eigen_tf;
        try {
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_cg_link_id,
                t->get_link_id(),
                tf2::TimePointZero,
                10ms
                );

            eigen_tf = tf2::transformToEigen(tf_cg_thruster);
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't compute thruster tf between cg-thruster: ") + e.what());
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                         t->get_link_id().c_str(), m_cg_link_id.c_str(), e.what() ); 
          return;

        }

        Eigen::VectorXd contribution_vector(CONTROLLABLE_DOF_LENGTH);

        // thruster onlu use a axis (e.g., X-axis) for forece
        double Fx = eigen_tf.rotation()(0, 0);
        double Fy = eigen_tf.rotation()(1, 0);
        double Fz = eigen_tf.rotation()(2, 0);

        auto trans_xyz = eigen_tf.translation();

//         //! Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
        Eigen::Matrix3d ang_vel_tranform = Eigen::Matrix3d::Identity();
        now = this->get_clock()->now();
        try {
//             // Transform center of gravity to world
            geometry_msgs::msg::TransformStamped tf_torque = m_transform_buffer->lookupTransform(
                m_world_link_id,
                m_cg_link_id,
                tf2::TimePointZero,
                10ms
            );

            tf2::Quaternion quat;
            quat.setW(tf_torque.transform.rotation.w);
            quat.setX(tf_torque.transform.rotation.x);
            quat.setY(tf_torque.transform.rotation.y);
            quat.setZ(tf_torque.transform.rotation.z);

            Eigen::VectorXd process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
            tf2::Matrix3x3(quat).getRPY(
                process_values(DOF::ROLL),
                process_values(DOF::PITCH),
                process_values(DOF::YAW)
            );

            ang_vel_tranform = f_angular_velocity_transform(process_values);
        } catch(tf2::TransformException &e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't compute thruster tf between world-cg: ") + e.what());
            // printf("####Error frame %s to %s \r\n", m_world_link_id.c_str(), m_cg_link_id.c_str());
            return;
        }

        auto torque_pqr = trans_xyz.cross(Eigen::Vector3d{Fx, Fy, Fz});
        auto torque_rpy = ang_vel_tranform * torque_pqr;
        

        
        contribution_vector(DOF::U) = Fx;
        contribution_vector(DOF::V) = Fy;
        contribution_vector(DOF::W) = Fz;
        contribution_vector(DOF::ROLL) = torque_rpy(0);
        contribution_vector(DOF::PITCH) = torque_rpy(1);
        contribution_vector(DOF::YAW) = torque_rpy(2);
        // body frame p,q,r
        contribution_vector(DOF::P) = torque_pqr(0);
        contribution_vector(DOF::Q) = torque_pqr(1);
        contribution_vector(DOF::R) = torque_pqr(2);

        t->set_contribution_vector(contribution_vector);
    }
}

bool MvpControlROS::f_initial_tf_check(){
    auto steady_clock = rclcpp::Clock();
    
    RCLCPP_INFO_STREAM(this->get_logger(), "MVP_control_node initial TF checking");

    //check world link to cg link is up
    try {
            // Transform center of gravity to world
            geometry_msgs::msg::TransformStamped tf_torque = m_transform_buffer->lookupTransform(
                m_world_link_id,
                m_cg_link_id,
                tf2::TimePointZero,
                10ms
            );

        } catch(tf2::TransformException &e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't find TF between world and cg: ") + e.what());
            return false;
        }
    RCLCPP_INFO_STREAM(this->get_logger(), "world_link to cg_link found");

    //check thruster to cg_link is up.
    // For each thruster look up transformation
    for(const auto& t : m_thrusters) {
        try {
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_cg_link_id,
                t->get_link_id(),
                tf2::TimePointZero,
                10ms
                );

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't find TF for thrusters: ") + e.what());
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                         t->get_link_id().c_str(), m_cg_link_id.c_str(), e.what() ); 
          return false;
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "thrust to cg_link found");
    RCLCPP_INFO_STREAM(this->get_logger(), "MVP control initialized");
    return true;

}

void MvpControlROS::initialize() {

    // Read configured control modes from the ROS parameter server
    f_load_control_config();
    RCLCPP_INFO(this->get_logger(), "#### Control config file %s loaded ####", m_control_config_file.c_str());

    // Initialize thruster objects.
    std::for_each(m_thrusters.begin(),m_thrusters.end(),
        [](const ThrusterROS::Ptr& t){
            t->initialize();
        }
    );

    RCLCPP_INFO_STREAM(this->get_logger(), "#### Thruster initialized ####");

    // Generate thrusters with the given configuration
    while(!f_initial_tf_check())
    {
        sleep(1);
    };

    RCLCPP_INFO_STREAM(this->get_logger(), "#### TF checking done ####");

    // Generate control allocation matrix with defined method
    f_generate_control_allocation_matrix();

    RCLCPP_INFO_STREAM(this->get_logger(), "#### Allocation matrix generated ####");

    m_mvp_control->set_desired_state(m_set_point);

    m_mvp_control->set_system_state(m_process_values);

    m_controller_worker = std::thread([this] { f_control_loop(); });

    m_controller_worker.detach();

}

//only update earth frame related elements
bool MvpControlROS::f_update_control_allocation_matrix() {

    // update control allocation based on actuators as well
    rclcpp::Time now = this->get_clock()->now();

    try {
        // Transform center of gravity to world
        geometry_msgs::msg::TransformStamped cg_world = m_transform_buffer->lookupTransform(
            m_world_link_id,
            m_cg_link_id,
            tf2::TimePointZero,
            10ms
        );

        auto tf_eigen = tf2::transformToEigen(cg_world);

        tf2::Quaternion quat;
        quat.setW(cg_world.transform.rotation.w);
        quat.setX(cg_world.transform.rotation.x);
        quat.setY(cg_world.transform.rotation.y);
        quat.setZ(cg_world.transform.rotation.z);

        Eigen::VectorXd orientation = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
        tf2::Matrix3x3(quat).getRPY(
            orientation(DOF::ROLL),
            orientation(DOF::PITCH),
            orientation(DOF::YAW)
        );

        Eigen::Matrix3d ang_vel_tranform = Eigen::Matrix3d::Identity();

        // for each thruster compute contribution in earth frame
        for(int j = 0 ; j < m_control_allocation_matrix.cols() ; j++){
            Eigen::Vector3d uvw;
            uvw <<
                m_control_allocation_matrix(DOF::U, j),
                m_control_allocation_matrix(DOF::V, j),
                m_control_allocation_matrix(DOF::W, j);

            Eigen::Vector3d xyz = tf_eigen.rotation() * uvw;

            m_control_allocation_matrix(DOF::X, j) = xyz(0);
            m_control_allocation_matrix(DOF::Y, j) = xyz(1);
            m_control_allocation_matrix(DOF::Z, j) = xyz(2);
            
            // Convert prq to world_frame angular rate:
            //  Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
            Eigen::Vector3d pqr;
            pqr <<
                m_control_allocation_matrix(DOF::P, j),
                m_control_allocation_matrix(DOF::Q, j),
                m_control_allocation_matrix(DOF::R, j);                

            ang_vel_tranform = f_angular_velocity_transform(orientation);

            auto rpy = ang_vel_tranform * pqr;
            m_control_allocation_matrix(DOF::ROLL, j) = rpy(0);
            m_control_allocation_matrix(DOF::PITCH, j) = rpy(1);
            m_control_allocation_matrix(DOF::YAW, j) = rpy(2);             
        }

    } catch(tf2::TransformException& e) {
        auto steady_clock = rclcpp::Clock();
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't update control allocation matrix ") + e.what());
        return false;
    }

    m_mvp_control->update_control_allocation_matrix(
        m_control_allocation_matrix
    );

    Eigen::VectorXd upper_limit(m_thrusters.size());
    Eigen::VectorXd lower_limit(m_thrusters.size());

    for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
        upper_limit[i] = m_thrusters[i]->m_force_max;
        lower_limit[i] = m_thrusters[i]->m_force_min;
    }

    m_mvp_control->set_lower_limit(lower_limit);

    m_mvp_control->set_upper_limit(upper_limit);

    return true;
}

bool MvpControlROS::f_compute_process_values() {
    auto steady_clock = rclcpp::Clock();
    rclcpp::Time now = this->get_clock()->now();

    f_update_control_allocation_matrix();

    try {
        // Transform center of gravity to world
        geometry_msgs::msg::TransformStamped cg_world = m_transform_buffer->lookupTransform(
            m_world_link_id,
            m_cg_link_id,
            tf2::TimePointZero,
            10ms
        );

        tf2::Quaternion quat;
        quat.setW(cg_world.transform.rotation.w);
        quat.setX(cg_world.transform.rotation.x);
        quat.setY(cg_world.transform.rotation.y);
        quat.setZ(cg_world.transform.rotation.z);

        tf2::Matrix3x3(quat).getRPY(
            m_process_values(DOF::ROLL),
            m_process_values(DOF::PITCH),
            m_process_values(DOF::YAW)
        );

        m_process_values(DOF::X) = cg_world.transform.translation.x;
        m_process_values(DOF::Y) = cg_world.transform.translation.y;
        m_process_values(DOF::Z) = cg_world.transform.translation.z;

    } catch(tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't compute process values: ") + e.what());
        return false;
    }

        // Transform from odom to world
    try{
        std::scoped_lock lock(m_odom_lock);

        geometry_msgs::msg::TransformStamped cg_odom = m_transform_buffer->lookupTransform(
                m_cg_link_id,
                m_odometry_msg.child_frame_id,
                tf2::TimePointZero,
                10ms
        );

        auto cg_odom_eigen = tf2::transformToEigen(cg_odom);

        // angular velocity from odomteyr_child_frame to cg_link
        tf2::Quaternion quat;
        quat.setW(cg_odom.transform.rotation.w);
        quat.setX(cg_odom.transform.rotation.x);
        quat.setY(cg_odom.transform.rotation.y);
        quat.setZ(cg_odom.transform.rotation.z);

        Eigen::VectorXd orientation = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
        tf2::Matrix3x3(quat).getRPY(
            orientation(DOF::ROLL),
            orientation(DOF::PITCH),
            orientation(DOF::YAW)
        );

        // convert linear velocity from odomtery child frame to cg_link
        Eigen::Vector3d uvw;
        uvw(0) = m_odometry_msg.twist.twist.linear.x;
        uvw(1) = m_odometry_msg.twist.twist.linear.y;
        uvw(2) = m_odometry_msg.twist.twist.linear.z;

        uvw = cg_odom_eigen.rotation()  * uvw;

        m_process_values(DOF::U) = uvw(0);
        m_process_values(DOF::V) = uvw(1);
        m_process_values(DOF::W) = uvw(2);

        // convert angular velocity from odom_child_link to cg_link
        Eigen::Matrix3d ang_vel_transform = f_angular_velocity_transform(orientation);

        Eigen::Vector3d angular_rate;
        angular_rate(0) = m_odometry_msg.twist.twist.angular.x;
        angular_rate(1) = m_odometry_msg.twist.twist.angular.y;
        angular_rate(2) = m_odometry_msg.twist.twist.angular.z;

        angular_rate = ang_vel_transform * angular_rate;

        m_process_values(DOF::P) = angular_rate(0);
        m_process_values(DOF::Q) = angular_rate(1);
        m_process_values(DOF::R) = angular_rate(2);

    } catch(tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't compute process values!, check odometry!: ") + e.what());
        return false;
    }

    mvp_msgs::msg::ControlProcess s;
    s.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(),
    s.header.frame_id = m_world_link_id;
    s.control_mode = m_control_mode;
    s.child_frame_id = m_child_link_id;
    s.position.x = m_process_values(DOF::X);
    s.position.y = m_process_values(DOF::Y);
    s.position.z = m_process_values(DOF::Z);
    s.orientation.x = m_process_values(DOF::ROLL);
    s.orientation.y = m_process_values(DOF::PITCH);
    s.orientation.z = m_process_values(DOF::YAW);
    s.velocity.x = m_process_values(DOF::U);
    s.velocity.y = m_process_values(DOF::V);
    s.velocity.z = m_process_values(DOF::W);
    // body frame angular velocity pqr
    s.angular_rate.x = m_process_values(DOF::P);
    s.angular_rate.y = m_process_values(DOF::Q);
    s.angular_rate.z = m_process_values(DOF::R);

    m_mvp_control->set_system_state(m_process_values);

    m_process_value_publisher->publish(s);

    mvp_msgs::msg::ControlProcess e;

    Eigen::VectorXd error_state = m_mvp_control->get_state_error();
    e.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(),
    e.header.frame_id = m_world_link_id;
    e.control_mode = m_control_mode;
    e.child_frame_id = m_child_link_id;
    e.position.x = error_state(DOF::X);
    e.position.y = error_state(DOF::Y);
    e.position.z = error_state(DOF::Z);
    e.orientation.x = error_state(DOF::ROLL);
    e.orientation.y = error_state(DOF::PITCH);
    e.orientation.z = error_state(DOF::YAW);
    e.velocity.x = error_state(DOF::U);
    e.velocity.y = error_state(DOF::V);
    e.velocity.z = error_state(DOF::W);
    e.angular_rate.x = error_state(DOF::P);
    e.angular_rate.y = error_state(DOF::Q);
    e.angular_rate.z = error_state(DOF::R);

    m_process_error_publisher->publish(e);

    return true;
}


void MvpControlROS::f_control_loop() {

    double pt = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    // setpoint_timer = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds() / 1000000000.0;

    auto r = rclcpp::Rate(m_controller_frequency);

    while(rclcpp::ok()) {
        /**
         * Thread may not be able to sleep properly. This may happen using
         * simulated time.
         */
        /**
         * Record the time that loop ends. Later, it will feed the PID
         * controller.
         */

        if(!r.sleep()) {
            continue;
        }

        /**
         * Compute the state of the system. Continue on failure. This may
         * happen when transform tree is not ready.
         */
        if(not f_compute_process_values()) {
            continue;
        }
        /**
         * Check if controller is enabled or not.
         */
        
        double time_since_last_setpoint = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - setpoint_timer;
        // printf("setpoint_timer=%lf\r\n", setpoint_timer);
        // printf("time since last setpoint = %lf\r\n", time_since_last_setpoint);
        if(!m_enabled || time_since_last_setpoint > m_no_setpoint_timeout) {
             for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
                // m_thrusters.at(i)->command(0);
                std_msgs::msg::Float64 msg;
                msg.data = 0.0;
                m_thrusters.at(i)->m_thrust_publisher->publish(msg);
            }
            continue;
        }

        

        Eigen::VectorXd needed_forces;

        /**
         * Get time difference to feed PID controller
         */
        double dt = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - pt;
        
        /**
         * Calculate forces to be requested from thrusters. If operation fails,
         * do not send commands to thrusters.
         */
        if(m_mvp_control->calculate_needed_forces(&needed_forces, dt)) {
        
            for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
                std::vector<std::complex<double>> roots;
                std_msgs::msg::Float64 Nmsg;
                Nmsg.data = needed_forces(i);
                // printf("###force for thruster %d: %f\r\n", i, needed_forces(i));

                m_thrusters.at(i)->m_force_publisher->publish(Nmsg);
                
                if (m_thrusters.at(i)->request_force(needed_forces(i), roots)){
                    for(const auto& r : roots) {
                        if(r.imag() != 0){
                            continue;
                        }
                        if(r.real() >= 1 || r.real() < -1) {
                            continue;
                        }      
                        std_msgs::msg::Float64 msg;
                        msg.data = r.real();
                        m_thrusters.at(i)->m_thrust_publisher->publish(msg);
                        break;
                    }
                }
            }

        }

        // /**
        //  * Record the time that loop ends. Later, it will feed the PID
        //  * controller.
        //  */
        pt = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    }
}

void MvpControlROS::f_cb_msg_odometry(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::scoped_lock lock(m_odom_lock);
    m_odometry_msg = *msg;

}

void MvpControlROS::f_cb_srv_set_point(
    const mvp_msgs::msg::ControlProcess::SharedPtr msg) {

    // printf("got setpoint msgs\r\n");
    setpoint_timer = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    f_amend_set_point(msg);
}


void MvpControlROS::f_load_control_config()
{
    YAML::Node map = YAML::LoadFile(m_control_config_file);
    std::vector<std::string> modes;

    //load control modes
    if(map["control_modes"])
    {
        //parse control modes
        // printf("size = %d\r\n", map["control_modes"].size() );
        //iterate through control modes
        for(YAML::const_iterator it=map["control_modes"].begin();it != map["control_modes"].end(); ++it) 
        {
            std::string key = it->first.as<std::string>();       // <- key
            modes.push_back(key);
            
        }
        
        std::map<std::string, std::set<int>> mode_rules;

        //loop through modes
        for (const auto &mode: modes) 
        {
            mvp_msgs::msg::ControlMode m;

            m.name = mode;

            // printf("mode_name = %s\r\n", mode.c_str() );
            //loop through DOF to create ros params
            for(YAML::const_iterator it=map["control_modes"][mode].begin();it != map["control_modes"][mode].end(); ++it) 
            {
                std::string param_name;
                std::string dof_name = it->first.as<std::string>();
                // printf("    dof_name = %s\r\n", dof_name.c_str());
                //get PID values
                for(const auto& key : {"p", "i", "d", "i_max", "i_min"})
                {
                    param_name = "control_modes/" + mode + "/" + dof_name + "/" + key;
                    // printf("         param = %s\r\n", param_name.c_str());
                    this->declare_parameter( param_name, map["control_modes"][mode][dof_name][key].as<float>() );
                }

                if(dof_name.compare(CONF_DOF_X) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_x.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_x.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_x.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_x.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_x.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_Y) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_y.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_y.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_y.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_y.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_y.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_Z) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_z.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_z.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_z.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_z.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_z.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_ROLL) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_roll.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_roll.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_roll.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_roll.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_roll.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_PITCH) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_pitch.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_pitch.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_pitch.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_pitch.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_pitch.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_YAW) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_yaw.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_yaw.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_yaw.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_yaw.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_yaw.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_U) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_u.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_u.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_u.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_u.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_u.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_V) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_v.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_v.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_v.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_v.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_v.k_i_min);
                }

                if(dof_name.compare(CONF_DOF_P) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_p.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_p.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_p.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_p.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_p.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_Q) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_q.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_q.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_q.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_q.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_q.k_i_min);
                }
                if(dof_name.compare(CONF_DOF_R) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_r.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_r.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_r.kd);
                    this->get_parameter(param_name + CONF_PID_I_MAX, m.pid_r.k_i_max);
                    this->get_parameter(param_name + CONF_PID_I_MIN, m.pid_r.k_i_min);
                }
                //check dof enabled
                auto found =std::find_if(CONF_DOF_LOOKUP.begin(), CONF_DOF_LOOKUP.end(),
                    [dof_name](const std::pair<const char *, int> &t) -> bool {
                        return std::strcmp(dof_name.c_str(),t.first) == 0;
                    }
                );

                if (found != CONF_DOF_LOOKUP.end()) {
                    mode_rules[mode].insert(found->second);
                } else {
                    throw control_ros_exception(
                            "Unknown freedom name passed '" + dof_name + "'"
                            "Possible values are "
                        "'x, y, z, roll, pitch, yaw, surge, sway, heave"
                    );
                }

            }
            m.dofs = std::vector<int>(mode_rules[mode].begin(), mode_rules[mode].end());
            m_control_modes.modes.emplace_back(m);
        }

    }

    //load thruster params
    if(map["thruster_ids"])
    {
        std::vector<std::string> thruster_id_list;
        // printf("#######################################################\r\n");
        //load the thruster name
        for(YAML::const_iterator it=map["thruster_ids"].begin();it != map["thruster_ids"].end(); ++it) 
        {

            std::string t_name = it->first.as<std::string>();       // thruster_name
            // printf("###Thruster id =%s\r\n", t_name.c_str());
            ThrusterROS::Ptr t(new ThrusterROS());
            t->set_id(t_name);
            

            std::string param_name; 

            //get thruster parameters:
            std::string link_id;
            param_name = map["thruster_ids"][t_name]["control_tf"].as<std::string>();
            this->declare_parameter(std::string()+CONF_CONTROL_TF + "/" + t_name + "_thruster_link", param_name);
            this->get_parameter(std::string()+CONF_CONTROL_TF + "/" + t_name + "_thruster_link", link_id);
            t->set_link_id(m_tf_prefix + link_id);

            param_name = map["thruster_ids"][t_name]["command_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUST_COMMAND_TOPICS + "/" + t_name, param_name);
            t->set_thrust_command_topic_id(param_name);
            t->m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Thruster: %s, topic name: %s\r\n", t_name.c_str(), param_name.c_str());
            
            param_name = map["thruster_ids"][t_name]["force_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUSTER_FORCE_TOPICS + "/" + t_name, param_name);
            t->set_thrust_force_topic_id(param_name);
            t->m_force_publisher= this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Thruster: %s, force_topic name: %s\r\n", t_name.c_str(), param_name.c_str());


            std::vector<float> min_max;
            min_max = map["thruster_ids"][t_name]["limits"].as<std::vector<float> >();
            // printf("    MAX: %f to %f\r\n", min_max[0], min_max[1]);
            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, min_max[0]);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, t->m_force_min);

            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, min_max[1]);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, t->m_force_max);


            std::vector<double> poly_coef;
            poly_coef = map["thruster_ids"][t_name]["polynomials"].as<std::vector<double> >();
            // std::cout<<poly_coef<<std::endl;
            this->declare_parameter(std::string()+CONF_THRUSTER_POLY + "/" + t_name, poly_coef);
            t->get_poly_solver()->set_coeff(poly_coef);
 
            m_thrusters.emplace_back(t);
        }


    }

    f_amend_control_mode(*modes.begin());

}


bool MvpControlROS::f_cb_srv_get_control_modes(
    const std::shared_ptr<GetControlModes::Request> req,
    const std::shared_ptr<GetControlModes::Response> resp) {

    if(!m_control_modes.modes.empty()) {
        resp->modes = m_control_modes.modes;
        return true;
    } else {
        return false;
    }

}

bool MvpControlROS::f_cb_srv_set_control_point(
    const std::shared_ptr<SetControlPoint::Request> req,
    const std::shared_ptr<SetControlPoint::Response> resp) {
    
    mvp_msgs::msg::ControlProcess::SharedPtr msg = std::make_shared<mvp_msgs::msg::ControlProcess>(req->setpoint);

    return f_amend_set_point(msg);

}

// bool MvpControlROS::f_cb_srv_enable(
//     const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//     const std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
//     RCLCPP_INFO_STREAM(this->get_logger(), "Controller enabled!");
//     m_enabled = true;

//     return true;
// }

// bool MvpControlROS::f_cb_srv_disable(
//     const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//     const std::shared_ptr<std_srvs::srv::Empty::Response> resp) {

//     RCLCPP_INFO_STREAM(this->get_logger(), "Controller disabled!");
//     m_enabled = false;

//     return true;
// }

bool MvpControlROS::f_cb_srv_set_controller(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> resp){
                
    m_enabled = req->data;
    resp->success = true;
    resp->message = m_enabled ? "Controller enabled." : "Controller disabled.";

    std_msgs::msg::Bool msg;
    msg.data = m_enabled;

    // Publish the message
    m_controller_state_publisher->publish(msg);
    return true;
}

bool MvpControlROS::f_cb_srv_get_active_mode(
    const std::shared_ptr<GetControlMode::Request> req,
    const std::shared_ptr<GetControlMode::Response> resp) {

    auto found =
        std::find_if(
            m_control_modes.modes.begin(),
            m_control_modes.modes.end(),
            [this](const mvp_msgs::msg::ControlMode &t) -> bool {
                 if(this->m_control_mode == t.name) {
                     return true;
                 }
                 return false;
            }
        );

    resp->mode = *found;

    return true;
}

Eigen::MatrixXd MvpControlROS::f_angular_velocity_transform(const Eigen::VectorXd& orientation) {
    Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();

    // 85 < pitch < 95, -95 < pitch < -85 
    if( (orientation(DOF::PITCH) >  1.483529839 && orientation(DOF::PITCH) <  1.658062761) ||
        (orientation(DOF::PITCH) > -1.658062761 && orientation(DOF::PITCH) < -1.483529839) ) {
        transform(0,0) = 1.0;
        transform(0,1) = 0.0;
        transform(0,2) = 0.0;
        transform(1,0) = 0.0;
        transform(1,1) = cos(orientation(DOF::ROLL));
        transform(1,2) = -sin(orientation(DOF::ROLL));
        transform(2,0) = 0.0;
        transform(2,1) = 0.0;
        transform(2,2) = 0.0;
    }
    else {
        transform(0,0) = 1.0;
        transform(0,1) = sin(orientation(DOF::ROLL)) * tan(orientation(DOF::PITCH));
        transform(0,2) = cos(orientation(DOF::ROLL)) * tan(orientation(DOF::PITCH));
        transform(1,0) = 0.0;
        transform(1,1) = cos(orientation(DOF::ROLL));
        transform(1,2) = -sin(orientation(DOF::ROLL));
        transform(2,0) = 0.0;
        transform(2,1) = sin(orientation(DOF::ROLL)) / cos(orientation(DOF::PITCH));
        transform(2,2) = cos(orientation(DOF::ROLL)) / cos(orientation(DOF::PITCH));
    }    

    return transform;
}

bool MvpControlROS::f_amend_control_mode(std::string mode) {
    if(!mode.empty()) {
        if(mode == m_control_mode) {
            // nothing should change. Operation valid
            return true;
        }

        auto found = std::find_if(
                m_control_modes.modes.begin(),
                m_control_modes.modes.end(),
                [mode](const mvp_msgs::msg::ControlMode& m) -> bool {
                if(m.name == mode) {
                    return true;
                }
                return false;
            }
        );

        if(found == m_control_modes.modes.end()) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Requested mode [" << mode << "] doesn't exist. ");
            // mode doesn't exist. Operation invalid
            return false;
        }

        // update PID gains
        Eigen::VectorXd p(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd i(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd d(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd i_max(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd i_min(CONTROLLABLE_DOF_LENGTH);

        p <<
                found->pid_x.kp,
                found->pid_y.kp,
                found->pid_z.kp,
                found->pid_roll.kp,
                found->pid_pitch.kp,
                found->pid_yaw.kp,
                found->pid_u.kp,
                found->pid_v.kp,
                found->pid_w.kp,
                found->pid_p.kp,
                found->pid_q.kp,
                found->pid_r.kp;

        i <<
                found->pid_x.ki,
                found->pid_y.ki,
                found->pid_z.ki,
                found->pid_roll.ki,
                found->pid_pitch.ki,
                found->pid_yaw.ki,
                found->pid_u.ki,
                found->pid_v.ki,
                found->pid_w.ki,
                found->pid_p.ki,
                found->pid_q.ki,
                found->pid_r.ki;

        d <<
                found->pid_x.kd,
                found->pid_y.kd,
                found->pid_z.kd,
                found->pid_roll.kd,
                found->pid_pitch.kd,
                found->pid_yaw.kd,
                found->pid_u.kd,
                found->pid_v.kd,
                found->pid_w.kd,
                found->pid_p.kd,
                found->pid_q.kd,
                found->pid_r.kd;

        i_max <<
                found->pid_x.k_i_max,
                found->pid_y.k_i_max,
                found->pid_z.k_i_max,
                found->pid_roll.k_i_max,
                found->pid_pitch.k_i_max,
                found->pid_yaw.k_i_max,
                found->pid_u.k_i_max,
                found->pid_v.k_i_max,
                found->pid_w.k_i_max,
                found->pid_p.k_i_max,
                found->pid_q.k_i_max,
                found->pid_r.k_i_max;
        i_min <<
                found->pid_x.k_i_min,
                found->pid_y.k_i_min,
                found->pid_z.k_i_min,
                found->pid_roll.k_i_min,
                found->pid_pitch.k_i_min,
                found->pid_yaw.k_i_min,
                found->pid_u.k_i_min,
                found->pid_v.k_i_min,
                found->pid_w.k_i_min,
                found->pid_p.k_i_min,
                found->pid_q.k_i_min,
                found->pid_r.k_i_min;

        m_mvp_control->get_pid()->set_kp(p);
        m_mvp_control->get_pid()->set_ki(i);
        m_mvp_control->get_pid()->set_kd(d);
        m_mvp_control->get_pid()->set_i_max(i_max);
        m_mvp_control->get_pid()->set_i_min(i_min);

        m_control_mode = mode;

        m_mvp_control->update_freedoms(found->dofs);
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Controller mode changed to " << mode);

        // mode is not empty. mode is in the modes list. operation is valid.
        return true;
    } else {

        // its empty, operation valid.
        return true;
    }
}


bool MvpControlROS::f_amend_set_point(
    const mvp_msgs::msg::ControlProcess::SharedPtr set_point) {
    rclcpp::Time now = this->get_clock()->now();

    auto steady_clock = rclcpp::Clock();    

    if(!f_amend_control_mode(set_point->control_mode)) {
        return false;
    }

    if( set_point->header.frame_id.empty() || set_point->child_frame_id.empty() ) {
        // no decision can be made
        RCLCPP_WARN_STREAM(this->get_logger(), "no frame id provided for the setpoint!");

        return false;
    }

    Eigen::Vector3d p_world, rpy_world;
    try {
        // Transform the position of setpoint frame_id to world_link
        geometry_msgs::msg::TransformStamped tf_world_setpoint = m_transform_buffer->lookupTransform(
            m_world_link_id,
            set_point->header.frame_id,
            tf2::TimePointZero,
            10ms
        );

        auto tf_eigen = tf2::transformToEigen(tf_world_setpoint);

        p_world = tf_eigen.rotation() * 
                                  Eigen::Vector3d(set_point->position.x, set_point->position.y, set_point->position.z)
                                  + tf_eigen.translation();
        ///convert euler angle into a different frame
        ///find the rotation matrix from the set point frame to the desired pose.
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(set_point->orientation.z, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(set_point->orientation.y, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(set_point->orientation.x, Eigen::Vector3d::UnitX());
        //find rotation matrix from the world link to the setpoint frame.
        geometry_msgs::msg::TransformStamped tf_1 = m_transform_buffer->lookupTransform(
            set_point->header.frame_id,
            m_world_link_id,
            tf2::TimePointZero,
            10ms
        );
        auto tf_1_eigen = tf2::transformToEigen(tf_1);
        //find the total rotation matrix from the world link to the desired pose.
        Eigen::Matrix3d R_set_point =  tf_1_eigen.rotation() *R;

        rpy_world.y() = asin(-R_set_point(2, 0));

        // Calculate yaw (rotation about Z-axis)
        rpy_world.z() = atan2(R_set_point(1, 0), R_set_point(0, 0));

        // Calculate roll (rotation about X-axis)
        rpy_world.x() = atan2(R_set_point(2, 1), R_set_point(2, 2));

        //assume the set point uvw and pqr are in the m_cg_link_id

    } catch(tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't transform the p and rpy to the global!") + e.what());
        return false;
    }

    m_set_point(mvp_msgs::msg::ControlMode::DOF_X) = p_world.x();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_Y) = p_world.y();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_Z) = p_world.z();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_ROLL) = rpy_world.x();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_PITCH) = rpy_world.y();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_YAW) = rpy_world.z();

    Eigen::Vector3d vel_child, omega_child;
    
    //converts setpoint velocity into the controller child frame.
    try {
        // Transform the position of setpoint frame_id to world_link
        geometry_msgs::msg::TransformStamped tf_two_childs = m_transform_buffer->lookupTransform(
            m_child_link_id,
            set_point->child_frame_id,
            tf2::TimePointZero,
            10ms
        );

        auto tf_two_child_eigen = tf2::transformToEigen(tf_two_childs);
        //convert linear velocity
        vel_child = tf_two_child_eigen.rotation() * 
                                Eigen::Vector3d(set_point->velocity.x, set_point->velocity.y, set_point->velocity.z);

        //convert angular velocity
        tf2::Quaternion quat;
            quat.setW(tf_two_childs.transform.rotation.w);
            quat.setX(tf_two_childs.transform.rotation.x);
            quat.setY(tf_two_childs.transform.rotation.y);
            quat.setZ(tf_two_childs.transform.rotation.z);

            Eigen::VectorXd process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
            tf2::Matrix3x3(quat).getRPY(
                process_values(DOF::ROLL),
                process_values(DOF::PITCH),
                process_values(DOF::YAW)
            );

        Eigen::Matrix3d ang_vel_tranform = Eigen::Matrix3d::Identity();
        ang_vel_tranform = f_angular_velocity_transform(process_values);
        omega_child = ang_vel_tranform * 
                        Eigen::Vector3d(set_point->angular_rate.x, set_point->angular_rate.y, set_point->angular_rate.z);


    } catch(tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 10, std::string("Can't transform child links") + e.what());
        return false;
    }


    m_set_point(mvp_msgs::msg::ControlMode::DOF_U) = vel_child.x();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_V) = vel_child.y();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_W) = vel_child.z();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_P) = omega_child.x();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_Q) = omega_child.y();
    m_set_point(mvp_msgs::msg::ControlMode::DOF_R) = omega_child.z();

    m_mvp_control->update_desired_state(m_set_point);

    m_set_point_msg = *set_point;

    return true;
}
