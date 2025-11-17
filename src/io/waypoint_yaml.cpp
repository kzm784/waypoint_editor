#include "waypoint_editor/io/waypoint_yaml.hpp"

#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace waypoint_editor::io
{

namespace
{

std::string EscapeYamlString(const std::string &input)
{
    std::ostringstream oss;
    for (const auto ch : input) {
        if (ch == '"') {
            oss << "\\\"";
        } else {
            oss << ch;
        }
    }
    return oss.str();
}

void WritePose(std::ostream &os, const geometry_msgs::msg::Pose &pose, const std::string &indent)
{
    os << indent << "position:\n";
    os << indent << "  x: " << pose.position.x << "\n";
    os << indent << "  y: " << pose.position.y << "\n";
    os << indent << "  z: " << pose.position.z << "\n";
    os << indent << "orientation:\n";
    os << indent << "  x: " << pose.orientation.x << "\n";
    os << indent << "  y: " << pose.orientation.y << "\n";
    os << indent << "  z: " << pose.orientation.z << "\n";
    os << indent << "  w: " << pose.orientation.w << "\n";
}

}  // namespace

bool WaypointYaml::Save(const std::vector<Waypoint> &waypoints, const std::string &path, std::string &error)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        error = "Failed to open file for writing: " + path;
        return false;
    }

    ofs << "waypoints:\n";
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const auto &wp = waypoints[i];
        ofs << "  - id: " << i << "\n";
        ofs << "    frame_id: " << wp.pose.header.frame_id << "\n";
        ofs << "    pose:\n";
        WritePose(ofs, wp.pose.pose, "      ");
        if (!wp.function_command.empty()) {
            ofs << "    command: \"" << EscapeYamlString(wp.function_command) << "\"\n";
        }
    }

    error.clear();
    return true;
}

namespace
{

bool ReadPoseComponent(const YAML::Node &parent, const char *key, double &value)
{
    const auto node = parent[key];
    if (!node) {
        return false;
    }
    try {
        value = node.as<double>();
        return true;
    } catch (const YAML::Exception &) {
        return false;
    }
}

bool PopulatePose(const YAML::Node &pose_node, geometry_msgs::msg::Pose &pose)
{
    const auto position = pose_node["position"];
    const auto orientation = pose_node["orientation"];
    if (!position || !orientation) {
        return false;
    }

    return ReadPoseComponent(position, "x", pose.position.x) &&
           ReadPoseComponent(position, "y", pose.position.y) &&
           ReadPoseComponent(position, "z", pose.position.z) &&
           ReadPoseComponent(orientation, "x", pose.orientation.x) &&
           ReadPoseComponent(orientation, "y", pose.orientation.y) &&
           ReadPoseComponent(orientation, "z", pose.orientation.z) &&
           ReadPoseComponent(orientation, "w", pose.orientation.w);
}

}  // namespace

bool WaypointYaml::Load(const std::string &path, std::vector<Waypoint> &waypoints, std::string &error)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception &ex) {
        error = "Failed to parse yaml: " + std::string(ex.what());
        return false;
    }

    const auto node = root["waypoints"];
    if (!node || !node.IsSequence()) {
        error = "Missing 'waypoints' list in yaml";
        return false;
    }

    std::vector<Waypoint> parsed;
    for (const auto &entry : node) {
        if (!entry.IsMap()) {
            continue;
        }

        const auto pose_node = entry["pose"];
        if (!pose_node || !pose_node.IsMap()) {
            continue;
        }

        Waypoint waypoint;
        if (!PopulatePose(pose_node, waypoint.pose.pose)) {
            continue;
        }

        if (auto frame_node = entry["frame_id"]; frame_node && frame_node.IsScalar()) {
            waypoint.pose.header.frame_id = frame_node.as<std::string>();
        } else {
            waypoint.pose.header.frame_id = "map";
        }

        if (auto command_node = entry["command"]; command_node && command_node.IsScalar()) {
            waypoint.function_command = command_node.as<std::string>();
        } else {
            waypoint.function_command.clear();
        }

        parsed.emplace_back(std::move(waypoint));
    }

    waypoints = std::move(parsed);
    error.clear();
    return true;
}

}  // namespace waypoint_editor::io
