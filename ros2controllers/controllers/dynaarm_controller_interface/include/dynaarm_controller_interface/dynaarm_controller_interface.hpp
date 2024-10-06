#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <Eigen/Dense>

#include "dynaarm_controller_interface_base/dynaarm_controller_interface_base.hpp"
#include "dynaarm_controller_interface/types.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf/model.h>

namespace dynaarm_controller_interface
{
    template <typename ParamListener, typename Param>
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

    protected:
        std::shared_ptr<ParamListener> param_listener_;
        Param params_;
        DynaarmState dynaarm_state_;
        DynaarmCommand dynaarm_command_;

        bool add_gravity_compensation_effort_{false};
        pinocchio::Model pinocchio_model_;
        pinocchio::Data pinocchio_data_;
    };

#include "dynaarm_controller_interface/dynaarm_controller_interface.tpp"

}