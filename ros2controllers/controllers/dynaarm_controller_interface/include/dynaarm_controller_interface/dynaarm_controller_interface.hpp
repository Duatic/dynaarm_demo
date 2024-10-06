#pragma once

#include <string>
#include <vector>
#include <map>

#include "dynaarm_controller_interface_base/dynaarm_controller_interface_base.hpp"

namespace dynaarm_controller_interface
{
    class DynaarmControllerInterface : public dynaarm_controller_interface_base::DynaarmControllerInterfaceBase
    {
    public:
        DynaarmControllerInterface(/* args */);
        virtual ~DynaarmControllerInterface() = default;

        bool on_configure_interface() override;
        virtual bool on_configure_controller() = 0;
        bool on_activate_interface() override;
        virtual bool on_activate_controller() = 0;
        bool on_deactivate_interface() override;
        virtual bool on_deactivate_controller() = 0;
        bool update_interface_state() override;
        bool update_interface() override;
        virtual bool update_controller() = 0;
        bool update_interface_command() override;
        bool publish_ros() override;
        virtual bool publish_ros_controller() = 0;
    };

}