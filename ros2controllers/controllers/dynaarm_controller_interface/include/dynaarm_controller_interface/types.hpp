#include <string>
#include <vector>
#include <Eigen/Dense>
#include <urdf/model.h>

namespace dynaarm_controller_interface
{
    // const std::map<std::string, int> JointN = {
    //     {"SH_ROT", 0},
    //     {"SH_FLE", 1},
    //     {"EL_FLE", 2},
    //     {"FA_ROT", 3},
    //     {"WRIST_1", 4},
    //     {"WRIST_2", 5},
    // }

    struct DynaarmState
    {
        std::vector<std::string> joint_names;
        Eigen::VectorXd joint_position;
        Eigen::VectorXd joint_velocity;
        Eigen::VectorXd joint_effort;
        Eigen::VectorXd joystick_axes;
        Eigen::VectorXd joystick_buttons;
        Eigen::VectorXd twist;

        bool init(const std::string &urdf_string)
        {
            if (!init_joint_names_from_urdf_in_order(urdf_string))
            {
                return false;
            }
            joint_position.setZero(joint_names.size());
            joint_velocity.setZero(joint_names.size());
            joint_effort.setZero(joint_names.size());
            joystick_axes.setZero(6);
            joystick_buttons.setZero(21);
            twist.setZero(6);
            return true;
        }

        void traverse_links(const urdf::Model &model, urdf::LinkConstSharedPtr link, std::vector<std::string> &joint_names)
        {
            // Iterate over the child joints
            for (const auto &joint : link->child_joints)
            {
                if (joint->limits && joint->type == urdf::Joint::REVOLUTE)
                {
                    joint_names.push_back(joint->name);
                }
                // Recursively process the child link
                urdf::LinkConstSharedPtr child_link = model.getLink(joint->child_link_name);

                // Recursively traverse the child link if it exists
                if (child_link)
                {
                    traverse_links(model, child_link, joint_names);
                }
            }
        }

        bool init_joint_names_from_urdf_in_order(const std::string &urdf_string)
        {
            // Create a URDF model from the URDF string
            urdf::Model model;
            if (!model.initString(urdf_string))
            {
                return false;
            }

            // Get the root link of the model
            urdf::LinkConstSharedPtr root_link = model.getRoot();
            if (!root_link)
            {
                return false;
            }

            // Traverse the tree from the root link
            traverse_links(model, root_link, joint_names);

            return true;
        }
    };

    struct JointLimits
    {
        std::string joint_name;
        double lower_limit;
        double upper_limit;
        double effort_limit;
        double velocity_limit;
    };

    struct DynaarmCommand
    {
        std::vector<std::string> joint_names;
        Eigen::VectorXd joint_position;
        Eigen::VectorXd joint_velocity;
        Eigen::VectorXd joint_effort;
        Eigen::VectorXd joint_effort_grav_comp;
        Eigen::VectorXd p_gain;
        Eigen::VectorXd i_gain;
        Eigen::VectorXd d_gain;
        Eigen::VectorXd command_freeze_mode;
        std::vector<JointLimits> joint_limits;

        bool init(const std::string urdf_string)
        {
            if (!init_joint_names_and_limits_from_urdf_in_order(urdf_string))
            {
                return false;
            }
            joint_position.setZero(joint_names.size());
            joint_velocity.setZero(joint_names.size());
            joint_effort.setZero(joint_names.size());
            joint_effort.setZero(joint_names.size());
            p_gain.setZero(joint_names.size());
            i_gain.setZero(joint_names.size());
            d_gain.setZero(joint_names.size());
            command_freeze_mode.setZero(joint_names.size());

            return true;
        }

        void traverse_links(const urdf::Model &model, urdf::LinkConstSharedPtr link, std::vector<std::string> &joint_names)
        {
            // Iterate over the child joints
            for (const auto &joint : link->child_joints)
            {
                if (joint->limits && joint->type == urdf::Joint::REVOLUTE)
                {
                    JointLimits limits;
                    limits.joint_name = joint->name;
                    limits.lower_limit = joint->limits->lower;
                    limits.upper_limit = joint->limits->upper;
                    limits.effort_limit = joint->limits->effort;
                    limits.velocity_limit = joint->limits->velocity;

                    joint_names.push_back(joint->name);
                    joint_limits.push_back(limits);
                }
                // Recursively process the child link
                urdf::LinkConstSharedPtr child_link = model.getLink(joint->child_link_name);

                // Recursively traverse the child link if it exists
                if (child_link)
                {
                    traverse_links(model, child_link, joint_names);
                }
            }
        }

        bool init_joint_names_and_limits_from_urdf_in_order(const std::string &urdf_string)
        {
            // Create a URDF model from the URDF string
            urdf::Model model;
            if (!model.initString(urdf_string))
            {
                return false;
            }

            // Get the root link of the model
            urdf::LinkConstSharedPtr root_link = model.getRoot();
            if (!root_link)
            {
                return false;
            }

            // Traverse the tree from the root link
            traverse_links(model, root_link, joint_names);

            return true;
        }
    };
}