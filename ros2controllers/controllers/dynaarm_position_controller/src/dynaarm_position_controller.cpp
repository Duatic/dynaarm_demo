#include "dynaarm_position_controller/dynaarm_position_controller.hpp"

namespace dynaarm_position_controller
{
    DynaarmPositionController::DynaarmPositionController() : DynaarmControllerInterface() {}

    bool DynaarmPositionController::on_configure_controller()
    {
        add_gravity_compensation_effort_ = true;
        return true;
    }

    bool DynaarmPositionController::on_activate_controller()
    {
        return true;
    }

    bool DynaarmPositionController::on_deactivate_controller()
    {
        return true;
    }

    bool DynaarmPositionController::update_controller()
    {
        return true;
    }

    bool DynaarmPositionController::publish_ros_controller()
    {
        return true;
    }

} // namespace dynaarm_position_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(dynaarm_position_controller::DynaarmPositionController, controller_interface::ControllerInterface)