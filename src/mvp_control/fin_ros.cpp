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
    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "mvp_control/fin_ros.hpp"

#include "utility"
#include "mvp_control/exception.hpp"
#include "mvp_control/dictionary.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace ctrl;

FinROS::FinROS(){
}

FinROS::FinROS(std::string id, std::string joint_id, Eigen::VectorXd contribution_vector) :
        m_id(std::move(id)),
        m_fin_joint_name(std::move(joint_id)),
        m_contribution_vector(std::move(contribution_vector))
{

}


auto FinROS::get_joint_name() -> decltype(m_fin_joint_name) {
    return m_fin_joint_name;
}

void FinROS::set_fin_joint_name(const decltype(m_fin_joint_name) &joint_id){
    m_fin_joint_name = joint_id;
}


void FinROS::set_fin_joint_state_topic(const decltype(m_fin_joint_state_topic_id) &topic_id) {
    m_fin_joint_state_topic_id = topic_id;
}


auto FinROS::get_id() -> decltype(m_id) {
    return m_id;
}

void FinROS::set_id(const decltype(m_id)& fin_id) {
    m_id = fin_id;
}


auto FinROS::get_contribution_vector() -> decltype(m_contribution_vector) {
    return m_contribution_vector;
}

void FinROS::set_contribution_vector(const decltype(m_contribution_vector)& contribution_vector) {
    m_contribution_vector = contribution_vector;
}

void FinROS::initialize() {
    if(!m_fin_joint_name.empty()) {
    } else {
        throw control_ros_exception("empty fin joint name");
    }

}

auto FinROS::get_link_id() -> decltype(m_link_id) {
    return m_link_id;
}

void FinROS::set_link_id(const decltype(m_link_id)& link_id) {
    m_link_id = link_id;
}

void FinROS::command(double cmd) {
    std_msgs::msg::Float64 msg;
    msg.data = cmd;
}