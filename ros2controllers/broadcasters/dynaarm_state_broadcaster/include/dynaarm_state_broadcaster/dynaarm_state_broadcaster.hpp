#pragma once

#include "dynaarm_controller_interface_base/dynaarm_controller_interface_base.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include <rclcpp/clock.hpp>
#include <rclcpp/time_source.hpp>
#include "dynaarm_state_broadcaster_parameters.hpp"

namespace dynaarm_state_broadcaster
{
    class DynaarmStateBroadcaster : public dynaarm_controller_interface_base::DynaarmControllerInterfaceBase
    {
    public:
        DynaarmStateBroadcaster();

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
        std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_joint_state_publisher_;
        std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_joint_command_publisher_;
    };
}
