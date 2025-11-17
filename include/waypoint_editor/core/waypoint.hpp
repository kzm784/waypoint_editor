#ifndef WAYPOINT_EDITOR__CORE__WAYPOINT_HPP_
#define WAYPOINT_EDITOR__CORE__WAYPOINT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace waypoint_editor
{

struct Waypoint
{
    geometry_msgs::msg::PoseStamped pose;
    std::string function_command;
};

}  // namespace waypoint_editor

#endif  // WAYPOINT_EDITOR__CORE__WAYPOINT_HPP_
