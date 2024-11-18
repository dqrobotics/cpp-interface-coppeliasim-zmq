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

DQ Robotics website: dqrobotics.github.io

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.
         The DQ_CoppeliaSimInterface class is partially based on the DQ_VrepInterface class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_VrepInterface.h)

*/

#pragma once
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <thread>
#include <atomic>
#include <memory>
#include <unordered_map>


using namespace DQ_robotics;
using namespace Eigen;


class DQ_CoppeliaSimZmqInterface : public DQ_CoppeliaSimInterface
{
public:
    enum class REFERENCE
    {
        BODY_FRAME,
        ABSOLUTE_FRAME
    };
    enum class JOINT_MODE
    {
        KINEMATIC,
        DYNAMIC,
        DEPENDENT
    };
    enum class ENGINE
    {
        BULLET,
        ODE,
        VORTEX,
        NEWTON,
        MUJOCO
    };
    enum class JOINT_CONTROL_MODE
    {
        FREE,
        FORCE,
        VELOCITY,
        POSITION,
        SPRING,
        CUSTOM,
        TORQUE
    };
    enum class PRIMITIVE {
        PLANE,
        DISC,
        CUBOID,
        SPHEROID,
        CYLINDER,
        CONE,
        CAPSULE
    };
    enum class SHAPE_TYPE{
        DYNAMIC,
        STATIC,
        ANY
    };

    DQ_CoppeliaSimZmqInterface();
    ~DQ_CoppeliaSimZmqInterface();

    //-----------Override from DQ_CoppeliaSimInterface ---------------------------------------
    bool connect(const std::string& host = "localhost",
                 const int& port = 23000,
                 const int&TIMEOUT_IN_MILISECONDS = 300) override;

    void   start_simulation() const override;
    void   stop_simulation()  const override;
    void   set_stepping_mode(const bool& flag) const override;
    void   trigger_next_simulation_step() const override;

    int    get_object_handle(const std::string& objectname) override;
    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames) override;
    DQ   get_object_translation(const std::string& objectname) override;
    void set_object_translation(const std::string& objectname, const DQ& t) override;
    DQ   get_object_rotation(const std::string& objectname) override;
    void set_object_rotation(const std::string& objectname, const DQ& r) override;
    void set_object_pose(const std::string& objectname, const DQ& h) override;
    DQ   get_object_pose(const std::string& objectname) override;

    VectorXd get_joint_positions(const std::vector<std::string>& jointnames) override;
    void     set_joint_positions(const std::vector<std::string>& jointnames,
                                 const VectorXd& angles_rad) override;
    void     set_joint_target_positions(const std::vector<std::string>& jointnames,
                                        const VectorXd& angles_rad) override;
    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) override;
    void     set_joint_target_velocities(const std::vector<std::string>& jointnames,
                                     const VectorXd& angles_rad_dot) override;
    void     set_joint_torques(const std::vector<std::string>& jointnames,
                               const VectorXd& torques) override;
    VectorXd get_joint_torques(const std::vector<std::string>& jointnames) override;

    //-----------------------------------------------------------------------------------------------------//
    //-----------Deprecated methods------------------------------------------------------------------------//
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect();
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect_all();
    [[deprecated("The synchronous mode is now called stepping mode. Consider using set_stepping_mode(flag) instead.")]]
    void set_synchronous(const bool& flag);
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    int wait_for_simulation_step_to_end();

    // For backward compatibility
    [[deprecated("This signature with MAX_TRY_COUNT is not required with ZeroMQ remote API. "
                 "If you use the port 19997, this signature will change it to 23000.")]]
    bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);

    // For backward compatibility
    [[deprecated("This signature with MAX_TRY_COUNT is not required with ZeroMQ remote API. "
                 "If you use the port 19997, this signature will change it to 23000.")]]
    bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);


    //-----------------------------------------------------------------------------------------------------//
    //---------------Experimental methods------------------------------------------------------------------//
    class experimental
    {
    private:
        std::shared_ptr<DQ_CoppeliaSimZmqInterface> smptr_;
    public:
        [[deprecated("This unstable class aims to test concepts and prototypes. Do not use it in production.")]]
        experimental(const std::shared_ptr<DQ_CoppeliaSimZmqInterface>& smptr);

        void plot_reference_frame(const std::string& name,
                                  const DQ& pose,
                                  const double& scale = 1,
                                  const std::vector<double>& thickness_and_length = {0.005, 0.1});

        void plot_plane(const std::string& name,
                        const DQ& normal_to_the_plane,
                        const DQ& location,
                        const std::vector<double>& sizes = {0.2,0.2},
                        const std::vector<double>& rgba_color = {1,0,0,0.5},
                        const bool& add_normal = true,
                        const double& normal_scale = 1);

        void plot_line(const std::string& name,
                       const DQ& line_direction,
                       const DQ& location,
                       const std::vector<double>& thickness_and_length = {0.01,1.5},
                       const std::vector<double>& rgba_color = {1,0,0,0.5},
                       const bool& add_arrow = true,
                       const double& arrow_scale = 1);

        void plot_cylinder(const std::string& name,
                           const DQ& direction,
                           const DQ& location,
                           const std::vector<double>& width_and_length = {0.2,1.0},
                           const std::vector<double>& rgba_color = {1,0,0,0.5},
                           const bool& add_line = true,
                           const double& line_scale = 1);

        void plot_sphere(const std::string& name,
                         const DQ& location,
                         const double& size = 0.2,
                         const std::vector<double> rgba_color = {1,0,0,0.5});

        bool load_model(const std::string& path_to_filename,
                        const std::string& desired_model_name,
                        const bool& load_model_only_if_missing = true,
                        const bool& remove_child_script = true);

        bool load_from_model_browser(const std::string& path_to_filename,
                                     const std::string& desired_model_name,
                                     const bool& load_model_only_if_missing = true,
                                     const bool& remove_child_script = true);

        int add_primitive(const PRIMITIVE& primitive,
                          const std::string& name,
                          const std::vector<double>& sizes);

        bool object_exist_on_scene(const std::string& objectname) const;

        void remove_object(const std::string& objectname,
                            const bool& remove_children = false);

        void draw_trajectory(const std::string& objectname,
                             const double& size = 2,
                             const std::vector<double>& rgb_color = {1,0,1},
                             const int& max_item_count = 1000);

        bool check_collision(const std::string& objectname1, const std::string& objectname2);
        std::vector<double> get_bounding_box_size(const std::string& objectname);

        //-------Mujoco settings-------------------------------------------------------------
        void set_mujoco_joint_stiffness(const std::string& jointname, const double& stiffness);
        void set_mujoco_joint_stiffnesses(const std::vector<std::string>& jointnames,
                                          const double& stiffness);

        void set_mujoco_joint_damping(const std::string& jointname, const double& damping);
        void set_mujoco_joint_dampings(const std::vector<std::string>& jointnames,
                                       const double& damping);

        void set_mujoco_joint_armature(const std::string& jointname, const double& armature);
        void set_mujoco_joint_armatures(const std::vector<std::string>& jointnames,
                                        const double& armature);

        void set_mujoco_body_friction(const std::string& bodyname, const std::vector<double>& friction);
        void set_mujoco_body_frictions(const std::vector<std::string>& bodynames,
                                       const std::vector<double>& friction);

        void   set_joint_mode(const std::string& jointname, const JOINT_MODE& joint_mode);
        void   set_joint_modes(const std::vector<std::string>& jointnames, const JOINT_MODE& joint_mode);
        void   set_joint_control_mode(const std::string& jointname, const JOINT_CONTROL_MODE& joint_control_mode);
        void   set_joint_control_modes(const std::vector<std::string>& jointnames, const JOINT_CONTROL_MODE& joint_control_mode);
        void   enable_dynamics(const bool& flag);
        double get_simulation_time_step() const;
        void   set_simulation_time_step(const double& time_step);
        double get_physics_time_step() const;
        void   set_physics_time_step(const double& time_step) const;
        void   set_engine(const ENGINE& engine);

        std::vector<std::string> get_jointnames_from_parent_object(const std::string& parent_objectname);
        std::vector<std::string> get_shapenames_from_parent_object(const std::string& parent_objectname,
                                                                    const SHAPE_TYPE& shape_type = SHAPE_TYPE::ANY);

        void load_scene(const std::string& path_to_filename) const;
        void save_scene(const std::string& path_to_filename) const;
        void close_scene() const;
    };


protected:

    std::string host_{"localhost"};
    int rpcPort_{23000};


private:
    int cntPort_{-1};
    int verbose_{-1};

    enum class AXIS{i,j,k};
    enum class UPDATE_MAP{ADD, REMOVE};

    bool _connect(const std::string& host,
                 const int& rpcPort,
                 const int& MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION,
                 const int& cntPort,
                 const int& verbose);

    int _get_port_from_deprecated_default_port(const int& port);

    std::atomic<bool> client_created_{false};

    // If true, the class will accept names without a slash in the first character.
    bool enable_deprecated_name_compatibility_{true};

    void _check_client() const;
    [[noreturn]] void _throw_runtime_error(const std::string& msg) const;

    int MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_{300};
    double elapsed_time_ {0};
    std::thread chronometer_thread_;
    void _join_if_joinable_chronometer_thread();
    void _start_chronometer();
    void _check_connection();
    //-------------------map zone--------------------------------------------
    std::string _map_simulation_state(const int& state);
    std::unordered_map<std::string, int> handles_map_;
    void _update_map(const std::string& objectname, const int& handle, const UPDATE_MAP& mode = UPDATE_MAP::ADD);
    int _get_handle_from_map(const std::string& objectname);

    //------------------------------------------------------------------------
    std::string _remove_first_slash_from_string(const std::string& str) const;
    bool _start_with_slash(const std::string& str) const;
    std::string _get_standard_name(const std::string& str) const;

    ENGINE _get_engine();
    std::unordered_map<ENGINE, int> engines_ = {{ENGINE::BULLET, 0},
                                                {ENGINE::ODE,    1},
                                                {ENGINE::VORTEX, 2},
                                                {ENGINE::NEWTON, 3},
                                                {ENGINE::MUJOCO, 4}};
    std::unordered_map<int,ENGINE> engines_invmap = {{0,ENGINE::BULLET},
                                                     {1,ENGINE::ODE},
                                                     {2,ENGINE::VORTEX},
                                                     {3,ENGINE::NEWTON},
                                                     {4,ENGINE::MUJOCO}};


    std::unordered_map<int, std::string> simulation_status_ = {{0, "simulation stopped"},
                                                               {8, "simulation paused"},
                                                               {17,"simulation advancing running"},
                                                               {22, "simulation advancing last before stop"},
                                                               {19, "simulation advancing last before pause"},
                                                               {16, "simulation advancing first after stop or simulation advancing"},
                                                               {20, "simulation advancing first after pause"},
                                                               {21, "simulation advancing about to stop"}};

    std::vector<int> _get_velocity_const_params() const;

    std::string _get_resources_path() const;
    bool _load_model(const std::string& path_to_filename,
                     const std::string& desired_model_name,
                     const bool& remove_child_script);

    MatrixXd _get_transformation_matrix(const std::vector<double>& coeff_vector) const;
    MatrixXd _get_rotation_matrix(const DQ& r) const;

    DQ _get_pose_from_direction(const DQ& direction, const DQ& point = DQ(1));

    int _add_primitive(const PRIMITIVE& primitive,
                      const std::string& name,
                      const std::vector<double>& sizes) const;


    std::vector<std::string> _create_static_axis_at_origin(const int& parent_handle,
                                                           const std::string& parent_name,
                                                           const std::vector<double>& sizes,
                                                           const AXIS& axis,
                                                           const double& alpha_color = 1) const;

    void _set_static_object_properties(const std::string& name,
                                       const std::string& parent_name,
                                       const DQ& pose,
                                       const std::vector<double>& rgba_color);


    void _set_static_object_properties(const int& handle,
                                       const int& parent_handle,
                                       const DQ& pose,
                                       const std::vector<double>& rgba_color) const;

    void _create_reference_frame(const std::string& name,
                                 const double& scale = 1,
                                 const std::vector<double>& thickness_and_length = {0.005, 0.1}) const;

    void _create_plane(const std::string& name,
                       const std::vector<double>& sizes = {0.2,0.2},
                       const std::vector<double>& rgba_color = {1,0,0,0.5},
                       const bool& add_normal = true,
                       const double& normal_scale = 1) const;

    void _create_line(const std::string& name,
                        const std::vector<double>& thickness_and_length = {0.01,1.5},
                        const std::vector<double>& rgba_color = {1,0,0,0.5},
                        const bool& add_arrow = true,
                        const double& arrow_scale = 1) const;

    void _create_cylinder(const std::string& name,
                          const std::vector<double>& width_and_length = {0.2,1.0},
                          const std::vector<double>& rgba_color = {1,0,0,0.5},
                          const bool& add_line = true,
                          const double& line_scale = 1) const;

    void _merge_shapes(const int& parent_handle) const;

    std::tuple<DQ, MatrixXd> _get_center_of_mass_and_inertia_matrix(const int& handle) const;


    template <typename T, typename U>
    void _check_sizes(const T &v1,
                      const U &v2,
                      const std::string& error_message) const
    {
        if (static_cast<std::size_t>(v1.size()) != static_cast<std::size_t>(v2.size()))
            throw std::runtime_error(error_message);
    }
    int _get_primitive_identifier(const PRIMITIVE& primitive) const;

    void _set_object_parent(const int& handle, const int& parent_handle, const bool& move_child_to_parent_pose) const;
    void _set_object_parent(const std::string& objectname, const std::string& parent_object_name,
                           const bool& move_child_to_parent_pose = true);

    void _remove_child_script_from_object(const std::string& objectname, const std::string& script_name = "/Script");

    bool _object_exist_on_scene(const std::string& objectname) const;

    void _set_object_name(const int& handle,
                         const std::string& new_object_name) const;
    void _set_object_name(const std::string& current_object_name,
                         const std::string& new_object_name);

    void _set_object_color(const int& handle,
                          const std::vector<double>& rgba_color) const;

    void _set_object_color(const std::string& objectname,
                          const std::vector<double>& rgba_color);

    void _set_object_as_respondable(const int& handle,
                                   const bool& respondable_object = true) const;

    void _set_object_as_respondable(const std::string& objectname,
                                   const bool& respondable_object = true);

    void _set_object_as_static(const int& handle,
                              const bool& static_object = true) const;

    void _set_object_as_static(const std::string& objectname,
                              const bool& static_object = true);


    bool _check_collision(const int& handle1, const int& handle2) const;
    bool _check_collision(const std::string& objectname1, const std::string& objectname2);
    std::tuple<double, DQ, DQ> _check_distance(const int& handle1, const int& handle2, const double& threshold = 0) const;
    std::tuple<double, DQ, DQ> _check_distance(const std::string& objectname1, const std::string& objectname2, const double& threshold = 0);


    double _compute_distance(const int& handle1,
                            const int& handle2,
                            const double& threshold = 0) const;
    double _compute_distance(const std::string& objectname1,
                            const std::string& objectname2,
                            const double& threshold = 0);


    void _draw_permanent_trajectory(const DQ& point,
                                   const double& size = 2,
                                   const std::vector<double>& color = {1,0,0},
                                   const int& max_item_count = 1000);

    int _add_simulation_lua_script(const std::string& script_name,
                                  const std::string& script_code);

    void _draw_trajectory(const std::string& objectname,
                         const double& size = 2,
                         const std::vector<double>& rgb_color = {1,0,1},
                         const int& max_item_count = 1000);

    void _remove_object(const std::string& objectname,
                       const bool& remove_children = false);

    std::vector<double> _get_bounding_box_size(const int& handle) const;
    std::vector<double> _get_bounding_box_size(const std::string& objectname);




    //-----------------------------------------------------------------------
    // Mujoco settings

    bool _mujoco_is_used();
    void _set_mujoco_global_impratio(const double& impratio);
    void _set_mujoco_global_wind(const std::vector<double>& wind);
    void _set_mujoco_global_density(const double& density);
    void _set_mujoco_global_viscosity(const double& viscosity);
    void _set_mujoco_global_boundmass(const double& boundmass);
    void _set_mujoco_global_boundinertia(const double& boundinertia);
    void _set_mujoco_global_overridemargin(const double& overridemargin);
    void _set_mujoco_global_overridesolref(const std::vector<double>& overridesolref);
    void _set_mujoco_global_overridesolimp(const std::vector<double>& overridesolimp);
    void _set_mujoco_global_iterations(const int& iterations);
    void _set_mujoco_global_integrator(const int& integrator);
    void _set_mujoco_global_solver(const int& solver);
    void _set_mujoco_global_njmax(const int& njmax);
    void _set_mujoco_global_nstack(const int& nstack);
    void _set_mujoco_global_nconmax(const int& nconmax);
    void _set_mujoco_global_cone(const int& cone);
    void _set_mujoco_global_overridekin(const int& overridekin);
    //void set_mujoco_global_rebuildcondition(const int& rebuildcondition);
    void _set_mujoco_global_computeinertias(const bool& computeinertias);
    void _set_mujoco_global_multithreaded(const bool& multithreaded);
    void _set_mujoco_global_multiccd(const bool& multiccd);
    void _set_mujoco_global_balanceinertias(const bool& balanceinertias);
    void _set_mujoco_global_overridecontacts(const bool& overridecontacts);


    void _set_mujoco_joint_stiffness(const std::string& jointname, const double& stiffness);
    void _set_mujoco_joint_stiffnesses(const std::vector<std::string>& jointnames,
                                      const double& stiffness);

    void _set_mujoco_joint_damping(const std::string& jointname, const double& damping);
    void _set_mujoco_joint_dampings(const std::vector<std::string>& jointnames,
                                   const double& damping);

    void _set_mujoco_joint_armature(const std::string& jointname, const double& armature);
    void _set_mujoco_joint_armatures(const std::vector<std::string>& jointnames,
                                    const double& armature);

    void _set_mujoco_body_friction(const std::string& bodyname, const std::vector<double>& friction);
    void _set_mujoco_body_frictions(const std::vector<std::string>& bodynames,
                                   const std::vector<double>& friction);


    void   _pause_simulation() const;
    double _get_simulation_time() const;

    bool   _is_simulation_running() const;
    int    _get_simulation_state() const;
    void   _set_status_bar_message(const std::string& message) const;


    std::string _get_object_name(const int& handle);

    template<typename T>
    std::vector<std::string> _get_object_names(const T& handles);

    std::vector<std::string> _get_jointnames_from_parent_object(const std::string& parent_objectname);
    std::vector<std::string> _get_shapenames_from_parent_object(const std::string& parent_objectname,
                                                               const SHAPE_TYPE& shape_type = SHAPE_TYPE::ANY);

    VectorXd _get_angular_and_linear_velocities(const int& handle,
                                               const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;

    VectorXd _get_angular_and_linear_velocities(std::string& objectname,
                                               const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);

    void _set_angular_and_linear_velocities(const int& handle,
                                           const DQ& w,
                                           const DQ& p_dot,
                                           const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;
    void _set_angular_and_linear_velocities(std::string& objectname,
                                           const DQ& w,
                                           const DQ& p_dot,
                                           const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);
    void _set_twist(const int& handle,
                   const DQ& twist,
                   const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;
    void _set_twist(const std::string& objectname,
                   const DQ& twist, const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);
    DQ   _get_twist(const int& handle,
                 const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;
    DQ   _get_twist(const std::string& objectname,
                 const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);

    double _get_mass(const int& handle) const;
    double _get_mass(const std::string& object_name);

    DQ     _get_center_of_mass(const int& handle, const REFERENCE& reference_frame=REFERENCE::ABSOLUTE_FRAME) const;
    DQ     _get_center_of_mass(const std::string& object_name, const REFERENCE& reference_frame=REFERENCE::ABSOLUTE_FRAME);

    MatrixXd _get_inertia_matrix(const int& handle, const REFERENCE& reference_frame=REFERENCE::BODY_FRAME);
    MatrixXd _get_inertia_matrix(const std::string& link_name, const REFERENCE& reference_frame=REFERENCE::BODY_FRAME);

    void   _set_joint_mode(const std::string& jointname, const JOINT_MODE& joint_mode);
    void   _set_joint_modes(const std::vector<std::string>& jointnames, const JOINT_MODE& joint_mode);
    void   _set_joint_control_mode(const std::string& jointname, const JOINT_CONTROL_MODE& joint_control_mode);
    void   _set_joint_control_modes(const std::vector<std::string>& jointnames, const JOINT_CONTROL_MODE& joint_control_mode);
    void   _enable_dynamics(const bool& flag);
    double _get_simulation_time_step() const;
    void   _set_simulation_time_step(const double& time_step);
    double _get_physics_time_step() const;
    void   _set_physics_time_step(const double& time_step) const;
    void   _set_engine(const ENGINE& engine);

    std::string get_engine();
    void   _set_gravity(const DQ& gravity=-9.81*k_);
    DQ     _get_gravity() const;

    void _load_scene(const std::string& path_to_filename) const;
    void _save_scene(const std::string& path_to_filename) const;
    void _close_scene() const;


    //------------------Additional methods-------------------------------------------------------------//
    DQ   _get_object_translation(const int& handle) const;
    void _set_object_translation(const int& handle, const DQ& t);
    DQ   _get_object_rotation(const int& handle) const;
    void _set_object_rotation(const int& handle, const DQ& r);
    DQ   _get_object_pose(const int& handle) const;
    void _set_object_pose(const int& handle, const DQ& h) const;

    double   _get_joint_position(const int& handle) const;
    double   _get_joint_position(const std::string& jointname);
    VectorXd _get_joint_positions(const std::vector<int>& handles) const;

    void     _set_joint_position(const int& handle, const double& angle_rad) const;
    void     _set_joint_position(const std::string& jointname, const double& angle_rad);
    void     _set_joint_positions(const std::vector<int>& handles, const VectorXd& angles_rad) const ;

    void     _set_joint_target_position(const int& handle, const double& angle_rad) const;
    void     _set_joint_target_position(const std::string& jointname, const double& angle_rad);
    void     _set_joint_target_positions(const std::vector<int>& handles, const VectorXd& angles_rad) const;

    double   _get_joint_velocity(const int& handle) const;
    double   _get_joint_velocity(const std::string& jointname);
    VectorXd _get_joint_velocities(const std::vector<int>& handles) const;

    void     _set_joint_target_velocity(const int& handle, const double& angle_rad_dot) const;
    void     _set_joint_target_velocity(const std::string& jointname, const double& angle_rad_dot);
    void     _set_joint_target_velocities(const std::vector<int>& handles, const VectorXd& angles_rad_dot) const;

    void     _set_joint_torque(const int& handle, const double& torque) const;
    void     _set_joint_torque(const std::string& jointname, const double& torque);
    void     _set_joint_torques(const std::vector<int>& handles, const VectorXd& torques) const;

    double   _get_joint_torque(const int& handle) const;
    double   _get_joint_torque(const std::string& jointname);
    VectorXd _get_joint_torques(const std::vector<int>& handles) const;

};





