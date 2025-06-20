/**
(C) Copyright 2011-2025 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.
         The DQ_CoppeliaSimRobotZMQ class is partially based on the DQ_SerialVrepRobot class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h)

*/

#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h>


namespace DQ_robotics
{

/**
 * @brief DQ_CoppeliaSimRobotZMQ::_get_interface_sptr gets the smartpointer of the ZMQ-based CoppeliaSim interface
 * @return The smartpointer of the CoppeliaSim interface.
 */
std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> DQ_CoppeliaSimRobotZMQ::_get_interface_sptr() const
{
    if(!cs_zmq_)
        throw std::runtime_error("DQ_CoppeliaSimInterfaceZMQ::_get_interface_sptr invalid interface pointer");
    return cs_zmq_;
}

/**
 * @brief DQ_CoppeliaSimRobotZMQ::_get_jointnames gets the robot joint names used in CoppeliaSim
 * @return the robot joint names.
 */
std::vector<std::string> DQ_CoppeliaSimRobotZMQ::_get_jointnames() const
{
    return jointnames_;
}


/**
 * @brief DQ_CoppeliaSimRobotZMQ::DQ_CoppeliaSimRobotZMQ constructor of the class.
 * @param robot_name The name of the robot used in CoppeliaSim
 * @param interface_sptr The CoppeliaSim Interface
 */
DQ_CoppeliaSimRobotZMQ::DQ_CoppeliaSimRobotZMQ(const std::string &robot_name,
                                               const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &interface_sptr)
    :DQ_CoppeliaSimRobot()
{
    if (!interface_sptr)
        throw std::runtime_error("DQ_CoppeliaSimRobotZMQ:::DQ_CoppeliaSimRobotZMQ Invalid DQ_CoppeliaSimInterfaceZMQ pointer!");
    robot_name_ = robot_name;
    cs_zmq_ = interface_sptr;
    jointnames_ = cs_zmq_->get_jointnames_from_object(robot_name_);
}

/**
 * @brief DQ_CoppeliaSimRobotZMQ::set_configuration sets the robot configuration in the CoppeliaSim scene.
                                 It is required a dynamics disabled scene.
 * @param configuration The robot configuration
 */
void DQ_CoppeliaSimRobotZMQ::set_configuration(const VectorXd &configuration)
{
    cs_zmq_->set_joint_positions(jointnames_, configuration);
}

/**
 * @brief DQ_CoppeliaSimRobotZMQ::set_target_configuration This method sets the target robot configuration in the
                                 CoppeliaSim scene. It requires a dynamics-enabled scene
 * @param target_configuration The target configuration
 */
void DQ_CoppeliaSimRobotZMQ::set_target_configuration(const VectorXd &target_configuration)
{
    cs_zmq_->set_joint_target_positions(jointnames_, target_configuration);
}

/**
 * @brief DQ_CoppeliaSimRobotZMQ::set_target_configuration_velocities This method sets the target robot configuration velocities in the CoppeliaSim scene.
                                It requires a dynamics-enabled scene.
 * @param target_configuration_velocities  The target configuration velocities.
 */
void DQ_CoppeliaSimRobotZMQ::set_target_configuration_velocities(const VectorXd &target_configuration_velocities)
{
    cs_zmq_->set_joint_target_velocities(jointnames_, target_configuration_velocities);
}


/**
 * @brief DQ_CoppeliaSimRobotZMQ::set_target_configuration_forces This method sets the target robot configuration forces in the CoppeliaSim scene.
                                 It requires a dynamics-enabled scene.
 * @param target_configuration_forces The target configuration forces.
 */
void DQ_CoppeliaSimRobotZMQ::set_target_configuration_forces(const VectorXd &target_configuration_forces)
{
   cs_zmq_->set_joint_target_forces(jointnames_, target_configuration_forces);
}

/**
 * @brief DQ_CoppeliaSimRobotZMQ::get_configuration This method returns the robot configuration in the CoppeliaSim scene.
 * @return The robot configuration
 */
VectorXd DQ_CoppeliaSimRobotZMQ::get_configuration()
{
    return  cs_zmq_->get_joint_positions(jointnames_);
}


/**
 * @brief DQ_CoppeliaSimRobotZMQ::get_configuration_velocities This method returns the robot configuration velocities in the CoppeliaSim scene.
 * @return The configuration velocities.
 */
VectorXd DQ_CoppeliaSimRobotZMQ::get_configuration_velocities()
{
    return  cs_zmq_->get_joint_velocities(jointnames_);
}


/**
 * @brief DQ_CoppeliaSimRobotZMQ::get_configuration_forces This method returns the robot configuration forces in the CoppeliaSim scene.
 * @return The configuration forces.
 */
VectorXd DQ_CoppeliaSimRobotZMQ::get_configuration_forces()
{
    return  cs_zmq_->get_joint_forces(jointnames_);
}

}
