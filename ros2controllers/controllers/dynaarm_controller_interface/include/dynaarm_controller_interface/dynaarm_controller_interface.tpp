template <typename ParamListener, typename Param>
DynaarmControllerInterface<ParamListener, Param>::DynaarmControllerInterface() : dynaarm_controller_interface_base::DynaarmControllerInterfaceBase() {}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::on_configure_interface()
{
    try
    {
        param_listener_ = std::make_shared<ParamListener>(get_node());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Error initializing param listener");
        return false;
    }
    params_ = param_listener_->get_params();

    std::vector<std::string> joint_state_interface_types{"position", "velocity", "effort"};
    std::vector<std::string> joint_command_interface_types{"position", "velocity", "effort", "p_gain", "i_gain", "d_gain", "command_freeze_mode"};
    std::vector<std::string> gpio_state_interfaces{"joystick_axis_0", "joystick_axis_1", "joystick_axis_2", "joystick_axis_3", "joystick_axis_4", "joystick_axis_5",
                                                   "joystick_button_0", "joystick_button_1", "joystick_button_2", "joystick_button_3", "joystick_button_4", "joystick_button_5", "joystick_button_6", "joystick_button_7", "joystick_button_8",
                                                   "joystick_button_9", "joystick_button_10", "joystick_button_11", "joystick_button_12", "joystick_button_13", "joystick_button_14", "joystick_button_15", "joystick_button_16", "joystick_button_17",
                                                   "joystick_button_18", "joystick_button_19", "joystick_button_20", "twist_linear_x", "twist_linear_y", "twist_linear_z", "twist_angular_x", "twist_angular_y", "twist_angular_z"};
    std::string gpio_dynaarm_interface_name = "gpio_dynaarm_interfaces";
    for (std::string &gpio_state_interface : gpio_state_interfaces)
    {
        gpio_state_interface = gpio_dynaarm_interface_name + "/" + gpio_state_interface;
    }

    std::string urdf_string = get_robot_description();
    if (!dynaarm_state_.init(urdf_string))
    {
        return false;
    }
    if (!dynaarm_command_.init(urdf_string))
    {
        return false;
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "<<<<<<<>>>>>>>: ");
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "dynaarm state joint names: ");
    for (const std::string &joint_name : dynaarm_state_.joint_names)
    {
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "joint name: " << joint_name);
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "dynaarm command joint names: ");
    for (const std::string &joint_name : dynaarm_command_.joint_names)
    {
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "joint name: " << joint_name);
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "<<<<<<<>>>>>>>: ");

    if (dynaarm_state_.joint_names.size() != dynaarm_command_.joint_names.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Did not get the same joint names in state and command");
        return false;
    }

    if (dynaarm_state_.joint_names.size() == 0)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Did not found any joints in the urdf");
        return false;
    }

    if (dynaarm_state_.joint_names.size() != params_.p_gain.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "p_gain vector not the right size");
        return false;
    }

    if (dynaarm_state_.joint_names.size() != params_.i_gain.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "i_gain vector not the right size");
        return false;
    }

    if (dynaarm_state_.joint_names.size() != params_.d_gain.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "d_gain vector not the right size");
        return false;
    }

    if (dynaarm_state_.joint_names.size() != params_.command_freeze_mode.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "command_freeze_mode vector not the right size");
        return false;
    }

    initialize_interfaces(dynaarm_state_.joint_names, joint_state_interface_types, dynaarm_command_.joint_names, joint_command_interface_types, gpio_state_interfaces, std::vector<std::string>());

    pinocchio::urdf::buildModelFromXML(urdf_string, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    if (!on_configure_controller())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "on_configure_controller Failed");
        return false;
    }

    return true;
}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::on_activate_interface()
{
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();

    for (int i = 0; i < static_cast<int>(dynaarm_state_.joint_names.size()); i++)
    {
        dynaarm_state_.joint_position(i) = get_state_interface_value("position", dynaarm_state_.joint_names[i]);
        dynaarm_state_.joint_velocity(i) = get_state_interface_value("velocity", dynaarm_state_.joint_names[i]);
        dynaarm_state_.joint_effort(i) = get_state_interface_value("effort", dynaarm_state_.joint_names[i]);
    }

    dynaarm_command_.joint_position = dynaarm_state_.joint_position;
    dynaarm_command_.joint_velocity.setZero();
    dynaarm_command_.joint_effort.setZero();

    for (int i = 0; i < static_cast<int>(dynaarm_state_.joint_names.size()); i++)
    {
        dynaarm_command_.p_gain(i) = params_.p_gain[i];
        dynaarm_command_.i_gain(i) = params_.i_gain[i];
        dynaarm_command_.d_gain(i) = params_.d_gain[i];
        dynaarm_command_.command_freeze_mode(i) = params_.command_freeze_mode[i];
    }

    if (add_gravity_compensation_effort_)
    {
        RCLCPP_WARN(get_node()->get_logger(), "ATTENTION: Adding Gravity compensation efforts to the commanded effort!");
    }

    if (!on_activate_controller())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "on_activate_controller Failed");
        return false;
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "successfully activated controller ");
    return true;
}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::on_deactivate_interface()
{
    if (!on_deactivate_controller())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "on_deactivate_controller Failed");
        return false;
    }

    dynaarm_command_.joint_position = dynaarm_state_.joint_position;
    dynaarm_command_.joint_velocity.setZero();
    dynaarm_command_.joint_effort.setZero();

    for (int i = 0; static_cast<int>(dynaarm_command_.joint_names.size()); i++)
    {
        dynaarm_command_.command_freeze_mode(i) = 1.0;
    }

    update_interface_command();

    return true;
}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::update_interface_state()
{
    for (int i = 0; i < static_cast<int>(dynaarm_state_.joint_names.size()); i++)
    {
        dynaarm_state_.joint_position(i) = get_state_interface_value("position", dynaarm_state_.joint_names[i]);
        dynaarm_state_.joint_velocity(i) = get_state_interface_value("velocity", dynaarm_state_.joint_names[i]);
        dynaarm_state_.joint_effort(i) = get_state_interface_value("effort", dynaarm_state_.joint_names[i]);
    }

    for (int i = 0; i < dynaarm_state_.joystick_axes.size(); i++)
    {
        dynaarm_state_.joystick_axes(i) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/joystick_axis_" + std::to_string(i));
    }

    for (int i = 0; i < dynaarm_state_.joystick_buttons.size(); i++)
    {
        dynaarm_state_.joystick_buttons(i) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/joystick_button_" + std::to_string(i));
    }

    dynaarm_state_.twist(0) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/twist_linear_x");
    dynaarm_state_.twist(1) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/twist_linear_y");
    dynaarm_state_.twist(2) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/twist_linear_z");
    dynaarm_state_.twist(3) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/twist_angular_x");
    dynaarm_state_.twist(4) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/twist_angular_y");
    dynaarm_state_.twist(5) = get_state_interface_value("gpio", "gpio_dynaarm_interfaces/twist_angular_z");

    return true;
}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::update_interface()
{
    if (add_gravity_compensation_effort_)
    {
        forwardKinematics(pinocchio_model_, pinocchio_data_, dynaarm_state_.joint_position, dynaarm_state_.joint_velocity);
        dynaarm_command_.joint_effort_grav_comp = pinocchio::rnea(pinocchio_model_, pinocchio_data_, dynaarm_state_.joint_position, dynaarm_state_.joint_velocity, Eigen::VectorXd::Zero(pinocchio_model_.nv));
    }

    if (!update_controller())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "update_controller Failed");
        return false;
    }

    return true;
}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::update_interface_command()
{
    for (int i = 0; i < static_cast<int>(dynaarm_command_.joint_names.size()); i++)
    {
        dynaarm_command_.joint_position(i) = std::clamp(dynaarm_command_.joint_position(i), dynaarm_command_.joint_limits[i].lower_limit, dynaarm_command_.joint_limits[i].upper_limit);
        set_command_interface(dynaarm_command_.joint_position(i), "position", dynaarm_command_.joint_names[i]);

        dynaarm_command_.joint_velocity(i) = std::clamp(dynaarm_command_.joint_velocity(i), -dynaarm_command_.joint_limits[i].velocity_limit, dynaarm_command_.joint_limits[i].velocity_limit);
        set_command_interface(dynaarm_command_.joint_velocity(i), "velocity", dynaarm_command_.joint_names[i]);

        dynaarm_command_.joint_effort(i) = std::clamp(dynaarm_command_.joint_effort(i), -dynaarm_command_.joint_limits[i].effort_limit, dynaarm_command_.joint_limits[i].effort_limit);
        if (add_gravity_compensation_effort_)
        {
            dynaarm_command_.joint_effort(i) += dynaarm_command_.joint_effort_grav_comp(i);
        }
        set_command_interface(dynaarm_command_.joint_effort(i), "effort", dynaarm_command_.joint_names[i]);

        set_command_interface(dynaarm_command_.p_gain(i), "p_gain", dynaarm_command_.joint_names[i]);
        set_command_interface(dynaarm_command_.i_gain(i), "i_gain", dynaarm_command_.joint_names[i]);
        set_command_interface(dynaarm_command_.d_gain(i), "d_gain", dynaarm_command_.joint_names[i]);
        set_command_interface(dynaarm_command_.command_freeze_mode(i), "command_freeze_mode", dynaarm_command_.joint_names[i]);
    }

    return true;
}

template <typename ParamListener, typename Param>
bool DynaarmControllerInterface<ParamListener, Param>::publish_ros()
{
    if (!publish_ros_controller())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "publish_ros_controller Failed");
        return false;
    }

    return true;
}
