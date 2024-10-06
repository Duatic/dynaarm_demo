#include "dynaarm_controller_interface_base/dynaarm_controller_interface_base.hpp"

namespace dynaarm_controller_interface_base
{
    DynaarmControllerInterfaceBase::DynaarmControllerInterfaceBase() : ControllerInterface() {}

    void DynaarmControllerInterfaceBase::initialize_interfaces(std::vector<std::string> state_joints, std::vector<std::string> state_joint_interface_types, std::vector<std::string> command_joints, std::vector<std::string> command_joint_interface_types, std::vector<std::string> state_gpio, std::vector<std::string> command_gpio)
    {
        interface_collection_.initialized = true;
        interface_collection_.state_joints = state_joints;
        interface_collection_.state_joint_interface_types = state_joint_interface_types;

        interface_collection_.command_joints = command_joints;
        interface_collection_.command_joint_interface_types = command_joint_interface_types;

        interface_collection_.state_gpio = state_gpio;
        interface_collection_.command_gpio = command_gpio;
    }

    controller_interface::InterfaceConfiguration DynaarmControllerInterfaceBase::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration interface_config;
        interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const std::string &interface_type : interface_collection_.state_joint_interface_types)
        {
            for (const std::string &joint : interface_collection_.state_joints)
            {
                interface_config.names.push_back(joint + "/" + interface_type);
            }
        }
        for (const std::string &gpio_state : interface_collection_.state_gpio)
        {
            interface_config.names.push_back(gpio_state);
        }

        return interface_config;
    }

    controller_interface::InterfaceConfiguration DynaarmControllerInterfaceBase::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration interface_config;
        interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const std::string &interface_type : interface_collection_.command_joint_interface_types)
        {
            for (const std::string &joint : interface_collection_.command_joints)
            {
                interface_config.names.push_back(joint + "/" + interface_type);
            }
        }
        for (const std::string &gpio_command : interface_collection_.command_gpio)
        {
            interface_config.names.push_back(gpio_command);
        }

        return interface_config;
    }

    controller_interface::CallbackReturn DynaarmControllerInterfaceBase::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!on_configure_interface())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_configure_interface Failed");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!interface_collection_.initialized)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "interface_collection_ not initialized. Please call initialize_interfaces in configure method");
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DynaarmControllerInterfaceBase::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!build_state_interface_reference_map())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "build_state_interface_reference_map Failed");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!build_command_interface_reference_map())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "build_command_interface_reference_map Failed");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!on_activate_interface())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_activate_interface Failed");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DynaarmControllerInterfaceBase::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!on_deactivate_interface())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_deactivate_interface Failed");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DynaarmControllerInterfaceBase::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        dt_ = period.seconds();
        update_time_ = time;

        if (!update_interface_state())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "update_interface_state Failed");
            return controller_interface::return_type::ERROR;
        }

        if (!update_interface())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "update_interface Failed");
            return controller_interface::return_type::ERROR;
        }

        publish_ros();

        if (!update_interface_command())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "update_interface_command Failed");
            return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
    }

    bool DynaarmControllerInterfaceBase::build_state_interface_reference_map()
    {
        for (const std::string &interface_type : interface_collection_.state_joint_interface_types)
        {
            std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> inner_map;
            for (const std::string &joint : interface_collection_.state_joints)
            {
                bool found;
                for (hardware_interface::LoanedStateInterface &interface : state_interfaces_)
                {
                    if ((interface_type == interface.get_interface_name() && joint == interface.get_prefix_name()) ||
                        (joint + "/" + interface_type == interface.get_name()) || (joint == interface.get_name()))
                    {
                        inner_map.emplace(joint, std::ref(interface));
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Did not find joint %s with type %s", joint.c_str(), interface_type.c_str());
                    return false;
                }
            }
            state_interface_map_[interface_type] = inner_map;
        }

        std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> inner_map;
        for (const std::string &gpio : interface_collection_.state_gpio)
        {
            bool found;
            for (hardware_interface::LoanedStateInterface &interface : state_interfaces_)
            {
                if (gpio == interface.get_name())
                {
                    inner_map.emplace(gpio, std::ref(interface));
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Did not find gpio %s", gpio.c_str());
                return false;
            }
        }
        state_interface_map_["gpio"] = inner_map;

        return true;
    }

    bool DynaarmControllerInterfaceBase::build_command_interface_reference_map()
    {
        for (const std::string &interface_type : interface_collection_.command_joint_interface_types)
        {
            std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> inner_map;
            for (const std::string &joint : interface_collection_.command_joints)
            {
                bool found;
                for (hardware_interface::LoanedCommandInterface &interface : command_interfaces_)
                {
                    if ((interface_type == interface.get_interface_name() && joint == interface.get_prefix_name()) ||
                        (joint + "/" + interface_type == interface.get_name()) || (joint == interface.get_name()))
                    {
                        inner_map.emplace(joint, std::ref(interface));
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Did not find joint %s with type %s", joint.c_str(), interface_type.c_str());
                    return false;
                }
            }
            command_interface_map_[interface_type] = inner_map;
        }

        std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> inner_map;
        for (const std::string &gpio : interface_collection_.command_gpio)
        {
            bool found;
            for (hardware_interface::LoanedCommandInterface &interface : command_interfaces_)
            {
                if (gpio == interface.get_name())
                {
                    inner_map.emplace(gpio, std::ref(interface));
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Did not find gpio %s", gpio.c_str());
                return false;
            }
        }
        command_interface_map_["gpio"] = inner_map;

        return true;
    }

    double DynaarmControllerInterfaceBase::get_state_interface_value(const std::string interface_type, const std::string interface_name)
    {
        return state_interface_map_.at(interface_type).at(interface_name).get().get_value();
    }

    void DynaarmControllerInterfaceBase::set_command_interface(const double value, const std::string interface_type, const std::string interface_name)
    {
        command_interface_map_.at(interface_type).at(interface_name).get().set_value(value);
    }

} // namespace dynaarm_controller_interface_base
