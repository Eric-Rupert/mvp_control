
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "Eigen/Dense"
#include "mvp_control/polynomial_solver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "chrono"
#include "thread"
#include <fstream> 



namespace ctrl {

    class MvpControlROS;

    /** @brief Thruster class for managing data
     *
     */
    class FinROS {
    private:
        // rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logger_;
        friend MvpControlROS;
        // rclcpp::Node::SharedPtr node_;
        
        //! @brief FIn ID
        std::string m_id;

        //! @brief Fin joint state name
        std::string m_fin_joint_name;

        std::string m_fin_joint_state_topic_id;


        //! @brief fin link id
        std::string m_link_id;

        /** @brief Thruster contribution vector
         *
         * This vector defines a column in control allocation matrix.
         * Each element in the vector describes contribution on
         * vehicle motion of the thruster in each degree of freedom
         */
        Eigen::VectorXd m_contribution_vector;

        //! @brief Thrust publisher
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_fin_publisher;

        //! @brief Polynomial solver
        PolynomialSolver::Ptr m_poly_solver;

        //min and max angle of attack
        double m_aoa_max;
        double m_aoa_min;


    public:

        //! @brief Default constructor
        FinROS();

        /** @brief Thruster ROS class constructor.
         *
         * This constructor should be used in normal operation.
         * Initializes Thruster ID, Topic ID and contribution vector
         *
         * @param id
         * @param topic_id
         * @param contribution_vector
         */
        FinROS(std::string id, std::string joint_id,
                    Eigen::VectorXd contribution_vector);
        /** @brief Initializes publishers and subscribers
         *
         */
        void initialize();

        /** @brief Trivial getter for topic id
         *
         * @return #ThrusterROS::m_thrust_command_topic_id
         */
        auto
        get_joint_name() -> decltype(m_fin_joint_name);

        /** @brief Default Setter for topic id
         *
         * @param topic_id
         */
        void set_fin_joint_name(
            const decltype(m_fin_joint_name) &topic_id);

        void set_fin_joint_state_topic(const decltype(m_fin_joint_state_topic_id) &topic_id);  
        /** @brief Trivial getter for link id
         *
         * @return #FinROS link
         */
        auto get_link_id() -> decltype(m_link_id);

        /** @brief Trivial Setter for link id
         *
         * @param link_id
         */
        void set_link_id(const decltype(m_link_id) &link_id);


        /** @brief Trivial getter for thruster id
         *
         * @return #FinROS::m_id
         */
        auto get_id() -> decltype(m_id);

        /** @brief Trivial Setter for topic id
         *
         * @param fin_id
         */
        void set_id(const decltype(m_id) &fin_id);

        /** @brief Trivial getter for contribution vector
         *
         * @return #ThrusterROS::m_contribution_vector
         */
        auto get_contribution_vector() -> decltype(m_contribution_vector);

        /** @brief Trivial Setter for contribution vector
         *
         * @param contribution Contribution vector for the thruster
         */
        void set_contribution_vector(
            const decltype(m_contribution_vector) &contribution_vector);

        //! @brief Generic typedef for shared pointer
        typedef std::shared_ptr<FinROS> Ptr;

        /** @brief Publish thruster command
         *
         * Thuster command should be between -1 and 1
         *
         * @param cmd
         */
        void command(double cmd);

    };

}