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

        if (!get_joint_names_from_urdf())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Did not get the joint names from the urdf");
            return false;
        }
        else
        {
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "<<<<<<<>>>>>>>: ");
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "DynaarmStateBroadcaster joint names: ");
            for (const std::string &joint_name : joint_names_)
            {
                RCLCPP_INFO_STREAM(get_node()->get_logger(), "joint name: " << joint_name);
            }
        }

        initialize_interfaces(joint_names_, joint_interface_types, std::vector<std::string>(), std::vector<std::string>(), std::vector<std::string>(), std::vector<std::string>());

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
        rt_joint_state_publisher_->msg_.name = joint_names_;
        rt_joint_state_publisher_->msg_.position.resize(joint_names_.size());
        rt_joint_state_publisher_->msg_.velocity.resize(joint_names_.size());
        rt_joint_state_publisher_->msg_.effort.resize(joint_names_.size());
        rt_joint_state_publisher_->unlockAndPublish();

        rt_joint_command_publisher_->lock();
        rt_joint_command_publisher_->msg_.header.stamp = update_time_;
        rt_joint_command_publisher_->msg_.name = joint_names_;
        rt_joint_command_publisher_->msg_.position.resize(joint_names_.size());
        rt_joint_command_publisher_->msg_.velocity.resize(joint_names_.size());
        rt_joint_command_publisher_->msg_.effort.resize(joint_names_.size());
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
        for (int i = 0; i < static_cast<int>(joint_names_.size()); i++)
        {
            rt_joint_state_publisher_->lock();
            rt_joint_state_publisher_->msg_.header.stamp = update_time_;
            rt_joint_state_publisher_->msg_.position[i] = get_state_interface_value("position", joint_names_[i]);
            rt_joint_state_publisher_->msg_.velocity[i] = get_state_interface_value("velocity", joint_names_[i]);
            rt_joint_state_publisher_->msg_.effort[i] = get_state_interface_value("effort", joint_names_[i]);
            rt_joint_state_publisher_->unlockAndPublish();

            rt_joint_command_publisher_->lock();
            rt_joint_command_publisher_->msg_.header.stamp = update_time_;
            rt_joint_command_publisher_->msg_.position[i] = get_state_interface_value("position_commanded", joint_names_[i]);
            rt_joint_command_publisher_->msg_.velocity[i] = get_state_interface_value("velocity_commanded", joint_names_[i]);
            rt_joint_command_publisher_->msg_.effort[i] = get_state_interface_value("effort_commanded", joint_names_[i]);
            rt_joint_command_publisher_->unlockAndPublish();
        }

        return true;
    }

    bool DynaarmStateBroadcaster::get_joint_names_from_urdf()
    {
        // Create a URDF model from the URDF string
        urdf::Model model;
        if (!model.initString(get_robot_description()))
        {
            return false;
        }

        // Iterate through all joints in the model
        for (const auto &joint_pair : model.joints_)
        {
            urdf::JointConstSharedPtr joint = joint_pair.second;

            // Ensure the joint has limits
            if (joint->limits && joint->type == urdf::Joint::REVOLUTE)
            {
                joint_names_.push_back(joint->name);
            }
        }

        return true;
    }

} // namespace dynaarm_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_state_broadcaster::DynaarmStateBroadcaster,
                       controller_interface::ControllerInterface)