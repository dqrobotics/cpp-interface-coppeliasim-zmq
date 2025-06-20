/**
(C) Copyright 2024 DQ Robotics Developers

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

#pragma once
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>

namespace DQ_robotics
{
class DQ_CoppeliaSimRobotZMQ: public DQ_CoppeliaSimRobot
{
protected:
    std::vector<std::string> jointnames_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_zmq_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> _get_interface_sptr() const;
    std::vector<std::string> _get_jointnames() const;
public:
    DQ_CoppeliaSimRobotZMQ(const std::string& robot_name,
                           const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& interface_sptr);

    void set_configuration(const VectorXd& configuration) override;
    void set_target_configuration(const VectorXd& target_configuration) override;
    void set_target_configuration_velocities(const VectorXd& target_configuration_velocities) override;
    void set_target_configuration_forces(const VectorXd& target_configuration_forces) override;
    VectorXd get_configuration() override;
    VectorXd get_configuration_velocities() override;
    VectorXd get_configuration_forces() override;
};

}


