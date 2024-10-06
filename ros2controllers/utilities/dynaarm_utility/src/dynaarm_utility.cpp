#include "dynaarm_utility/dynaarm_utility.hpp"

namespace dynaarm_utility
{
    DynaarmUtility::DynaarmUtility() : dynaarm_controller_interface_base::DynaarmControllerInterfaceBase(), rt_joy_subscriber_(), rt_twist_subscriber_() {}

    bool DynaarmUtility::on_configure_interface()
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

        std::vector<std::string> gpio_state_interfaces{"joystick_axis_0", "joystick_axis_1", "joystick_axis_2", "joystick_axis_3", "joystick_axis_4", "joystick_axis_5",
                                                       "joystick_button_0", "joystick_button_1", "joystick_button_2", "joystick_button_3", "joystick_button_4", "joystick_button_5", "joystick_button_6", "joystick_button_7", "joystick_button_8",
                                                       "joystick_button_9", "joystick_button_10", "joystick_button_11", "joystick_button_12", "joystick_button_13", "joystick_button_14", "joystick_button_15", "joystick_button_16", "joystick_button_17",
                                                       "joystick_button_18", "joystick_button_19", "joystick_button_20", "twist_linear_x", "twist_linear_y", "twist_linear_z", "twist_angular_x", "twist_angular_y", "twist_angular_z"};
        std::string gpio_dynaarm_interface_name = "gpio_dynaarm_interfaces";
        for (std::string &gpio_state_interface : gpio_state_interfaces)
        {
            gpio_state_interface = gpio_dynaarm_interface_name + "/" + gpio_state_interface;
        }
        std::vector<std::string> gpio_command_interfaces = gpio_state_interfaces;
        initialize_interfaces(std::vector<std::string>(), std::vector<std::string>(), std::vector<std::string>(), std::vector<std::string>(), gpio_state_interfaces, gpio_command_interfaces);

        auto custom_qos = rclcpp::SystemDefaultsQoS();
        custom_qos.keep_last(1);
        custom_qos.best_effort();
        custom_qos.durability_volatile();

        joy_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Joy>("/joy", custom_qos, [this](const sensor_msgs::msg::Joy::SharedPtr msg)
                                                                                 { rt_joy_subscriber_.writeFromNonRT(msg); });
        twist_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>("/twist", custom_qos, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
                                                                                              { rt_twist_subscriber_.writeFromNonRT(msg); });

        return true;
    }

    bool DynaarmUtility::on_activate_interface()
    {
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();
        return true;
    }

    bool DynaarmUtility::on_deactivate_interface()
    {
        return true;
    }

    bool DynaarmUtility::update_interface_state()
    {
        return true;
    }

    bool DynaarmUtility::update_interface()
    {
        return true;
    }

    bool DynaarmUtility::update_interface_command()
    {
        auto joy_commands = rt_joy_subscriber_.readFromRT();

        // this works and checks if new messages are here.
        // Not worth trying to make this easier, it was tried before
        if (!(!joy_commands || !(*joy_commands)))
        {
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "got joy_commands!!!!!");
            set_command_interface((*joy_commands)->axes[0], "gpio", "joystick_axis_0");
            set_command_interface((*joy_commands)->axes[1], "gpio", "joystick_axis_1");
            set_command_interface((*joy_commands)->axes[2], "gpio", "joystick_axis_2");
            set_command_interface((*joy_commands)->axes[3], "gpio", "joystick_axis_3");
            set_command_interface((*joy_commands)->axes[4], "gpio", "joystick_axis_4");
            set_command_interface((*joy_commands)->axes[5], "gpio", "joystick_axis_5");

            set_command_interface((*joy_commands)->buttons[0], "gpio", "joystick_button_0");
            set_command_interface((*joy_commands)->buttons[1], "gpio", "joystick_button_1");
            set_command_interface((*joy_commands)->buttons[2], "gpio", "joystick_button_2");
            set_command_interface((*joy_commands)->buttons[3], "gpio", "joystick_button_3");
            set_command_interface((*joy_commands)->buttons[4], "gpio", "joystick_button_4");
            set_command_interface((*joy_commands)->buttons[5], "gpio", "joystick_button_5");
            set_command_interface((*joy_commands)->buttons[6], "gpio", "joystick_button_6");
            set_command_interface((*joy_commands)->buttons[7], "gpio", "joystick_button_7");
            set_command_interface((*joy_commands)->buttons[8], "gpio", "joystick_button_8");
            set_command_interface((*joy_commands)->buttons[9], "gpio", "joystick_button_9");
            set_command_interface((*joy_commands)->buttons[10], "gpio", "joystick_button_10");
            set_command_interface((*joy_commands)->buttons[11], "gpio", "joystick_button_11");
            set_command_interface((*joy_commands)->buttons[12], "gpio", "joystick_button_12");
            set_command_interface((*joy_commands)->buttons[13], "gpio", "joystick_button_13");
            set_command_interface((*joy_commands)->buttons[14], "gpio", "joystick_button_14");
            set_command_interface((*joy_commands)->buttons[15], "gpio", "joystick_button_15");
            set_command_interface((*joy_commands)->buttons[16], "gpio", "joystick_button_16");
            set_command_interface((*joy_commands)->buttons[17], "gpio", "joystick_button_17");
            set_command_interface((*joy_commands)->buttons[18], "gpio", "joystick_button_18");
            set_command_interface((*joy_commands)->buttons[19], "gpio", "joystick_button_19");
            set_command_interface((*joy_commands)->buttons[20], "gpio", "joystick_button_20");
        }

        auto twist_commands = rt_twist_subscriber_.readFromRT();

        if (!(!twist_commands || !(*twist_commands)))
        {
            set_command_interface((*twist_commands)->twist.linear.x, "gpio", "twist_linear_x");
            set_command_interface((*twist_commands)->twist.linear.y, "gpio", "twist_linear_y");
            set_command_interface((*twist_commands)->twist.linear.z, "gpio", "twist_linear_z");
            set_command_interface((*twist_commands)->twist.angular.x, "gpio", "twist_angular_x");
            set_command_interface((*twist_commands)->twist.angular.y, "gpio", "twist_angular_y");
            set_command_interface((*twist_commands)->twist.angular.z, "gpio", "twist_angular_z");
        }

        return true;
    }

    bool DynaarmUtility::publish_ros()
    {
        return true;
    }

} // namespace dynaarm_utility

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_utility::DynaarmUtility, controller_interface::ControllerInterface)