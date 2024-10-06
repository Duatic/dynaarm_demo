#pragma once

#include "dynaarm_controller_interface_base/dynaarm_controller_interface_base.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include <rclcpp/clock.hpp>
#include <rclcpp/time_source.hpp>
#include "dynaarm_utility_parameters.hpp"

namespace dynaarm_utility
{
    class DynaarmUtility : public dynaarm_controller_interface_base::DynaarmControllerInterfaceBase
    {
    public:
        DynaarmUtility();

        bool on_configure_interface() override;
        bool on_activate_interface() override;
        bool on_deactivate_interface() override;
        bool update_interface_state() override;
        bool update_interface() override;
        bool update_interface_command() override;
        bool publish_ros() override;

    private:
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::Joy>> rt_joy_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>> rt_twist_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    };
}
