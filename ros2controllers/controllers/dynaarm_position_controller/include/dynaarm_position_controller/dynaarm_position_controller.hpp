#pragma once

#include "dynaarm_controller_interface/dynaarm_controller_interface.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include <rclcpp/clock.hpp>
#include <rclcpp/time_source.hpp>
#include "dynaarm_position_controller_parameters.hpp"

namespace dynaarm_position_controller
{
    using DynaarmControllerInterface = dynaarm_controller_interface::DynaarmControllerInterface<::dynaarm_position_controller::ParamListener, ::dynaarm_position_controller::Params>;

    class DynaarmPositionController final : public DynaarmControllerInterface
    {
    public:
        DynaarmPositionController();

        bool on_configure_controller() override;
        bool on_activate_controller() override;
        bool on_deactivate_controller() override;
        bool update_controller() override;
        bool publish_ros_controller() override;
    };
}
