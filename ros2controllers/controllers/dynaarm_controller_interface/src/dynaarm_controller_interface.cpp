#include "dynaarm_controller_interface/dynaarm_controller_interface.hpp"

namespace dynaarm_controller_interface
{
    DynaarmControllerInterface::DynaarmControllerInterface() : dynaarm_controller_interface_base::DynaarmControllerInterfaceBase() {}

    bool DynaarmControllerInterface::on_configure_interface()
    {
        return true;
    }

    bool DynaarmControllerInterface::on_activate_interface()
    {
        return true;
    }

    bool DynaarmControllerInterface::on_deactivate_interface()
    {
        return true;
    }

    bool DynaarmControllerInterface::update_interface_state()
    {
        return true;
    }

    bool DynaarmControllerInterface::update_interface()
    {
        return true;
    }

    bool DynaarmControllerInterface::update_interface_command()
    {
        return true;
    }

    bool DynaarmControllerInterface::publish_ros()
    {
        return true;
    }

} // namespace dynaarm_controller_interface
