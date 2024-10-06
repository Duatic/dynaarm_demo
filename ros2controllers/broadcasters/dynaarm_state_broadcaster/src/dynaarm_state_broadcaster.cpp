#include "dynaarm_state_broadcaster/dynaarm_state_broadcaster.hpp"

namespace dynaarm_state_broadcaster
{
    DynaarmStateBroadcaster::DynaarmStateBroadcaster() : dynaarm_controller_interface_base::DynaarmControllerInterfaceBase() {}

    bool DynaarmStateBroadcaster::on_configure_interface()
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

        std::vector<std::string> joint_interface_types{"position", "velocity", "effort",
                                                       "position_commanded", "velocity_commanded", "effort_commanded", "p_gain_commanded", "i_gain_commanded", "d_gain_commanded",
                                                       "motor_position", "motor_velocity", "motor_effort", "motor_position_commanded", "motor_velocity_commanded", "motor_effort_commanded",
                                                       "motor_temperature_system", "motor_temperature_coil_A", "motor_temperature_coil_B", "motor_temperature_coil_C", "motor_bus_voltage",
                                                       "command_freeze_mode"};
        initialize_interfaces(params_.joints, joint_interface_types, std::vector<std::string>(), std::vector<std::string>(), std::vector<std::string>(), std::vector<std::string>());

        auto custom_qos = rclcpp::SystemDefaultsQoS();
        custom_qos.best_effort();
        custom_qos.durability_volatile();

        auto state_publisher = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", custom_qos);
        rt_joint_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(state_publisher);
        auto command_publisher = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/joint_commands", custom_qos);
        rt_joint_command_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(command_publisher);

        return true;
    }

    bool DynaarmStateBroadcaster::on_activate_interface()
    {
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();

        rt_joint_state_publisher_->lock();
        rt_joint_state_publisher_->msg_.header.stamp = update_time_;
        rt_joint_state_publisher_->msg_.name = params_.joints;
        rt_joint_state_publisher_->msg_.position.resize(params_.joints.size());
        rt_joint_state_publisher_->msg_.velocity.resize(params_.joints.size());
        rt_joint_state_publisher_->msg_.effort.resize(params_.joints.size());
        rt_joint_state_publisher_->unlockAndPublish();

        rt_joint_command_publisher_->lock();
        rt_joint_command_publisher_->msg_.header.stamp = update_time_;
        rt_joint_command_publisher_->msg_.name = params_.joints;
        rt_joint_command_publisher_->msg_.position.resize(params_.joints.size());
        rt_joint_command_publisher_->msg_.velocity.resize(params_.joints.size());
        rt_joint_command_publisher_->msg_.effort.resize(params_.joints.size());
        rt_joint_command_publisher_->unlockAndPublish();

        return true;
    }

    bool DynaarmStateBroadcaster::on_deactivate_interface()
    {
        return true;
    }

    bool DynaarmStateBroadcaster::update_interface_state()
    {
        return true;
    }

    bool DynaarmStateBroadcaster::update_interface()
    {
        return true;
    }

    bool DynaarmStateBroadcaster::update_interface_command()
    {
        return true;
    }

    bool DynaarmStateBroadcaster::publish_ros()
    {
        for (int i = 0; i < static_cast<int>(params_.joints.size()); i++)
        {
            rt_joint_state_publisher_->lock();
            rt_joint_state_publisher_->msg_.header.stamp = update_time_;
            rt_joint_state_publisher_->msg_.position[i] = get_state_interface_value("position", params_.joints[i]);
            rt_joint_state_publisher_->msg_.velocity[i] = get_state_interface_value("velocity", params_.joints[i]);
            rt_joint_state_publisher_->msg_.effort[i] = get_state_interface_value("effort", params_.joints[i]);
            rt_joint_state_publisher_->unlockAndPublish();

            rt_joint_command_publisher_->lock();
            rt_joint_command_publisher_->msg_.header.stamp = update_time_;
            rt_joint_command_publisher_->msg_.position[i] = get_state_interface_value("position_commanded", params_.joints[i]);
            rt_joint_command_publisher_->msg_.velocity[i] = get_state_interface_value("velocity_commanded", params_.joints[i]);
            rt_joint_command_publisher_->msg_.effort[i] = get_state_interface_value("effort_commanded", params_.joints[i]);
            rt_joint_command_publisher_->unlockAndPublish();
        }

        return true;
    }

} // namespace dynaarm_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_state_broadcaster::DynaarmStateBroadcaster,
                       controller_interface::ControllerInterface)