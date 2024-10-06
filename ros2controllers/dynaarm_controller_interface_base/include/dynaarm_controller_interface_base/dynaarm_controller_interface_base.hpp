#pragma once

#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/visibility_control.h"
#include "hardware_interface/handle.hpp"

namespace dynaarm_controller_interface_base
{
    struct InterfaceCollection
    {
        bool initialized = false;

        std::vector<std::string> state_joint_interface_types;
        std::vector<std::string> state_joints;

        std::vector<std::string> command_joint_interface_types;
        std::vector<std::string> command_joints;

        std::vector<std::string> state_gpio;
        std::vector<std::string> command_gpio;
    };

    class DynaarmControllerInterfaceBase : public controller_interface::ControllerInterface
    {
    public:
        DynaarmControllerInterfaceBase(/* args */);
        virtual ~DynaarmControllerInterfaceBase() = default;

        using StateInterfaceReferenceMap = std::map<std::string, std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>>;
        using CommandInterfaceReferenceMap = std::map<std::string, std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>>;

        void initialize_interfaces(std::vector<std::string> state_joints, std::vector<std::string> state_joint_interface_types, std::vector<std::string> command_joints, std::vector<std::string> command_joint_interface_types, std::vector<std::string> state_gpio, std::vector<std::string> command_gpio);

        controller_interface::CallbackReturn on_init() override final { return controller_interface::CallbackReturn::SUCCESS; }

        controller_interface::InterfaceConfiguration state_interface_configuration() const override final;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override final;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        virtual bool on_configure_interface() = 0;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        virtual bool on_activate_interface() = 0;

        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        virtual bool on_deactivate_interface() = 0;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        virtual bool update_interface_state() = 0;
        virtual bool update_interface() = 0;
        virtual bool update_interface_command() = 0;

        virtual bool publish_ros() = 0;

        bool build_state_interface_reference_map();
        bool build_command_interface_reference_map();

        double get_state_interface_value(const std::string interface_type, const std::string interface_name);
        void set_command_interface(const double value, const std::string interface_type, const std::string interface_name);

    protected:
        double dt_;
        rclcpp::Time update_time_;

        InterfaceCollection interface_collection_;

        StateInterfaceReferenceMap state_interface_map_;
        CommandInterfaceReferenceMap command_interface_map_;
    };

}