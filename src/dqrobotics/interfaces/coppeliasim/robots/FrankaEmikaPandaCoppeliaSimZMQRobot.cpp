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
         This class is based on the FrankaEmikaPandaVrepRobot class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/robots/FrankaEmikaPandaVrepRobot.h)

*/

#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include<dqrobotics/robots/FrankaEmikaPandaRobot.h>

namespace DQ_robotics
{



/**
 * @brief FrankaEmikaPandaCoppeliaSimZMQRobot::FrankaEmikaPandaCoppeliaSimZMQRobot
 * @param robot_name
 * @param coppeliasim_interface_sptr
 */
FrankaEmikaPandaCoppeliaSimZMQRobot::FrankaEmikaPandaCoppeliaSimZMQRobot(const std::string &robot_name,
                                                                         const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &coppeliasim_interface_sptr)
    :DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface_sptr)
{
    base_frame_name_ = _get_jointnames().at(0);
}

/**
 * @brief FrankaEmikaPandaCoppeliaSimZMQRobot::kinematics
 * @return
 */
DQ_SerialManipulatorMDH FrankaEmikaPandaCoppeliaSimZMQRobot::kinematics()
{
    DQ_SerialManipulatorMDH kin = FrankaEmikaPandaRobot::kinematics();
    kin.set_reference_frame(_get_interface_sptr()->get_object_pose(base_frame_name_));
    kin.set_base_frame(_get_interface_sptr()->get_object_pose(base_frame_name_));
    return kin;
}

}
