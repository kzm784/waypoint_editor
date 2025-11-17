#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_common/display_context.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QString>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "waypoint_editor/io/waypoint_csv.hpp"
#include "waypoint_editor/io/waypoint_yaml.hpp"
#include "waypoint_editor/rviz/waypoint_editor_tool.hpp"

using namespace std::placeholders;

namespace waypoint_editor
{

WaypointEditorTool::WaypointEditorTool() : rviz_default_plugins::tools::PoseTool() {}
WaypointEditorTool::~WaypointEditorTool() {}

void WaypointEditorTool::onInitialize()
{
    PoseTool::onInitialize();

    setName("Add Waypoint");

    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server",
        nh_,
        rclcpp::SystemDefaultsQoS(),
        rclcpp::SystemDefaultsQoS()
    );
    save_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "save_waypoints",
        std::bind(&WaypointEditorTool::handleSaveWaypoints, this, _1, _2)
    );
    undo_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "undo_waypoints",
        std::bind(&WaypointEditorTool::handleUndoWaypoints, this, _1, _2)
    );
    redo_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "redo_waypoints",
        std::bind(&WaypointEditorTool::handleRedoWaypoints, this, _1, _2)
    );
    load_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "load_waypoints",
        std::bind(&WaypointEditorTool::handleLoadWaypoints, this, _1, _2)
    );

    line_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("waypoint_line", 10);
    total_wp_dist_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("total_wp_dist", 10);
    last_wp_dist_pub_  = nh_->create_publisher<std_msgs::msg::Float64>("last_wp_dist", 10);

    waypoint_sequence_.clear();
    pose_dirty_ = false;
    updateLastDistanceFromWaypoint(0);
    publishRangeMetrics();
}

void WaypointEditorTool::onPoseSet(double x, double y, double theta)
{
    Waypoint wp;
    wp.pose.header.frame_id = "map";
    wp.pose.header.stamp = nh_->now();
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.y = y;
    wp.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    wp.pose.pose.orientation.x = 0.0;
    wp.pose.pose.orientation.y = 0.0;
    wp.pose.pose.orientation.z = q.z();
    wp.pose.pose.orientation.w = q.w();

    wp.function_command.clear();

    const int new_id = waypoint_sequence_.appendWaypoint(std::move(wp));
    auto int_marker = createWaypointMarker(new_id);
    server_->insert(int_marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
    server_->applyChanges();
    RCLCPP_INFO(nh_->get_logger(), "Added waypoint %d", new_id);
    waypoint_sequence_.snapshotHistory();
    pose_dirty_ = false;
    updateLastDistanceFromWaypoint(new_id);
    publishRangeMetrics();

    deactivate();
}

void WaypointEditorTool::updateWaypointMarker()
{
    server_->clear();
    for (size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        auto int_marker = createWaypointMarker(static_cast<int>(i));
        server_->insert(int_marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
    }

    server_->applyChanges();
}

visualization_msgs::msg::InteractiveMarker WaypointEditorTool::createWaypointMarker(const int id)
{
    const auto & wp = waypoint_sequence_.at(id);

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = wp.pose.header.frame_id;
    int_marker.name = std::to_string(id);
    int_marker.description = waypoint_sequence_.at(id).function_command;
    int_marker.scale = 1.0;
    int_marker.pose.position = wp.pose.pose.position;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = wp.pose.pose.orientation.z;
    int_marker.pose.orientation.w = wp.pose.pose.orientation.w;

    // Position control (sphere)
    visualization_msgs::msg::InteractiveMarkerControl pos_control;
    pos_control.name = "move_position";
    pos_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    pos_control.always_visible = true;
    pos_control.orientation.w = 0.7071;
    pos_control.orientation.x = 0.0;
    pos_control.orientation.y = 0.7071;
    pos_control.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker sphere;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        sphere.color.r = 0.0;
        sphere.color.g = 1.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;
        pos_control.markers.push_back(sphere);
    }
    int_marker.controls.push_back(pos_control);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_default;
    rot_control_default.name = "rotate_yaw_default";
    rot_control_default.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rot_control_default.orientation.w = 0.7071;
    rot_control_default.orientation.x = 0.0;
    rot_control_default.orientation.y = -0.7071;
    rot_control_default.orientation.z = 0.0;
    rot_control_default.always_visible = true;
    int_marker.controls.push_back(rot_control_default);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_arrow;
    rot_control_arrow.name = "rotate_yaw_arrow";
    rot_control_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    rot_control_arrow.orientation.w = 0.7071;
    rot_control_arrow.orientation.x = 0.0;
    rot_control_arrow.orientation.y = -0.7071;
    rot_control_arrow.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker arrow;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = 0.4;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        rot_control_arrow.markers.push_back(arrow);
    }
    rot_control_arrow.always_visible = true;
    int_marker.controls.push_back(rot_control_arrow);

    // Text control
    visualization_msgs::msg::InteractiveMarkerControl text_control;
    text_control.name = "display_text";
    text_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    text_control.always_visible = true;
    {
        visualization_msgs::msg::Marker text;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.scale.z = 0.2;
        text.color.r = 0.0;
        text.color.g = 0.0;
        text.color.b = 0.0;
        text.color.a = 1.0;
        std::string id_text = "ID:" + std::to_string(id) + "\n" + waypoint_sequence_.at(id).function_command;
        text.text = id_text;
        text.pose.position.x = -0.3;
        text.pose.position.y = -0.3;
        text.pose.position.z = 0.3;
        text_control.markers.push_back(text);
    }
    int_marker.controls.push_back(text_control);

    visualization_msgs::msg::InteractiveMarkerControl menu_control;
    menu_control.name              = "menu";
    menu_control.always_visible    = true;
    menu_control.interaction_mode  = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    int_marker.controls.push_back(menu_control);
    
    visualization_msgs::msg::MenuEntry delete_entry;
    delete_entry.id = 1;
    delete_entry.parent_id = 0;
    delete_entry.title = "Delete Waypoint";
    int_marker.menu_entries.push_back(delete_entry);

    visualization_msgs::msg::MenuEntry change_id_entry;
    change_id_entry.id = 2;
    change_id_entry.parent_id = 0;
    change_id_entry.title = "Change Waypoint ID";
    int_marker.menu_entries.push_back(change_id_entry);
    
    visualization_msgs::msg::MenuEntry add_function_command_entry;
    add_function_command_entry.id = 3;
    add_function_command_entry.parent_id = 0;
    add_function_command_entry.title = "Edit Function Command";
    int_marker.menu_entries.push_back(add_function_command_entry);

    return int_marker;
}

void WaypointEditorTool::processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb)
{
    int id = std::stoi(fb->marker_name);

    switch (fb->event_type)
    {
        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            geometry_msgs::msg::Pose new_pose = fb->pose;
            new_pose.orientation.x = 0.0;
            new_pose.orientation.y = 0.0;
            new_pose.orientation.z = fb->pose.orientation.z;
            new_pose.orientation.w = fb->pose.orientation.w;
            waypoint_sequence_.updatePose(id, new_pose);
            pose_dirty_ = true;

            server_->setPose(fb->marker_name, fb->pose);
            server_->applyChanges();
            updateLastDistanceFromWaypoint(id);
            publishRangeMetrics();
            break;
        }

        case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
        {
            if (pose_dirty_) {
                waypoint_sequence_.snapshotHistory();
                pose_dirty_ = false;
            }
            break;
        }

        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        {
            processMenuControl(fb);
            break;
        }

        default:
            break;
    }
}

void WaypointEditorTool::processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb)
{
    if (fb->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) { return; }

    int id = std::stoi(fb->marker_name);
    if (id < 0 || id >= static_cast<int>(waypoint_sequence_.size())) { return; }

    switch (fb->menu_entry_id) {
      
        // Delete Waypoint
        case 1:
            waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
            updateWaypointMarker();
            waypoint_sequence_.snapshotHistory();
            pose_dirty_ = false;
            updateLastDistanceFromWaypoint(id);
            publishRangeMetrics();
            RCLCPP_INFO(nh_->get_logger(), "Deleted waypoint %d", id);
            break;

        // Change Waypoint ID
        case 2:
        {
         bool ok = false;
            QString current = QString::fromStdString(std::to_string(id));
            QString text = QInputDialog::getText(
                nullptr,
                tr("Change Waypoint ID"),
                tr("Enter New Waypoint ID: %1").arg(id),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                int insert_id = text.toInt();
                if (0 <= insert_id && insert_id < static_cast<int>(waypoint_sequence_.size())) {
                    Waypoint waypoint = waypoint_sequence_.at(static_cast<std::size_t>(id));
                    waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
                    waypoint_sequence_.insertWaypoint(static_cast<std::size_t>(insert_id), std::move(waypoint));
                    updateWaypointMarker();
                    waypoint_sequence_.snapshotHistory();
                    pose_dirty_ = false;
                    updateLastDistanceFromWaypoint(insert_id);
                    publishRangeMetrics();
                    RCLCPP_INFO(nh_->get_logger(), "Changed waypoint id %d to %d", id, insert_id);
                } else {
                    const int max_index = std::max(0, static_cast<int>(waypoint_sequence_.size()) - 1);
                    QMessageBox::warning(
                        nullptr,
                        tr("Invalid Waypoint ID"),
                        tr("Waypoint ID %1 is out of range.\n"
                        "Please enter a value between %2 and %3.")
                        .arg(insert_id)
                        .arg(0)
                        .arg(max_index)
                    );
                }
            }
        }
            break;

        // Add / Edit Function Command
        case 3:
        {
            bool ok = false;
            QString current = QString::fromStdString(waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Edit Function Command"),
                tr("Enter command for waypoint ID: %1").arg(id),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command = text.toStdString();
                updateWaypointMarker();
                waypoint_sequence_.snapshotHistory();
                pose_dirty_ = false;
                updateLastDistanceFromWaypoint(id);
                publishRangeMetrics();
                RCLCPP_INFO(nh_->get_logger(), "Updated command of waypoint %d to '%s'", id, waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command.c_str());
            }
        }
            break;

      default:
            break;
    }
}

double WaypointEditorTool::computeSegmentDistance(std::size_t first, std::size_t second) const
{
    const auto &waypoints = waypoint_sequence_.waypoints();
    if (first >= waypoints.size() || second >= waypoints.size()) {
        return 0.0;
    }
    const auto &p0 = waypoints[first].pose.pose.position;
    const auto &p1 = waypoints[second].pose.pose.position;
    return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

void WaypointEditorTool::updateLastDistanceFromWaypoint(int waypoint_index)
{
    const auto size = waypoint_sequence_.size();
    if (size < 2) {
        last_displayed_distance_ = 0.0;
        return;
    }

    const int max_index = static_cast<int>(size) - 1;
    const int clamped = std::max(0, std::min(waypoint_index, max_index));
    const std::size_t idx = static_cast<std::size_t>(clamped);

    if (idx > 0) {
        last_displayed_distance_ = computeSegmentDistance(idx - 1, idx);
    } else if (idx + 1 < size) {
        last_displayed_distance_ = computeSegmentDistance(idx, idx + 1);
    } else {
        last_displayed_distance_ = computeSegmentDistance(size - 2, size - 1);
    }
}

void WaypointEditorTool::publishLineMarker()
{
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = nh_->now();
    line.ns = "waypoint_lines";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action  = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.025f;
    line.color.r = 0.0f;
    line.color.g  = 1.0f;
    line.color.b  = 0.0f;
    line.color.a  = 1.0f;

    const auto &waypoints = waypoint_sequence_.waypoints();
    for (size_t i = 1; i < waypoints.size(); ++i) {
        geometry_msgs::msg::Point p0 = waypoints[i-1].pose.pose.position;
        geometry_msgs::msg::Point p1 = waypoints[i].pose.pose.position;
        line.points.push_back(p0);
        line.points.push_back(p1);
    }
    line_pub_->publish(line);
}

void WaypointEditorTool::publishTotalWpsDist()
{
    std_msgs::msg::Float64 msg;
    msg.data = waypoint_sequence_.totalDistance();
    total_wp_dist_pub_->publish(msg);
}

void WaypointEditorTool::publishLastWpsDist()
{
    std_msgs::msg::Float64 msg;
    msg.data = last_displayed_distance_;
    last_wp_dist_pub_->publish(msg);
}

void WaypointEditorTool::publishRangeMetrics()
{
    publishLineMarker();
    publishTotalWpsDist();
    publishLastWpsDist();
}

bool WaypointEditorTool::requestFilePathForSaving(std::string &path, bool &save_as_yaml)
{
    QString selected_filter;
    QString qpath = QFileDialog::getSaveFileName(
        nullptr,
        tr("Save Waypoints As"),
        "",
        tr("CSV Files (*.csv);;YAML Files (*.yaml)"),
        &selected_filter
    );

    if (qpath.isEmpty()) {
        return false;
    }

    auto lower = qpath.toLower();
    if (lower.endsWith(".yaml")) {
        save_as_yaml = true;
    } else if (lower.endsWith(".csv")) {
        save_as_yaml = false;
    } else {
        // Fallback to selected filter when no extension given.
        if (selected_filter.contains("yaml", Qt::CaseInsensitive)) {
            qpath += ".yaml";
            save_as_yaml = true;
        } else {
            qpath += ".csv";
            save_as_yaml = false;
        }
    }

    path = qpath.toStdString();
    return true;
}

bool WaypointEditorTool::requestFilePathForLoading(std::string &path, bool &load_yaml)
{
    QString selected_filter;
    QString qpath = QFileDialog::getOpenFileName(
        nullptr,
        tr("Open Waypoints"),
        "",
        tr("CSV Files (*.csv);;YAML Files (*.yaml)"),
        &selected_filter
    );
    if (qpath.isEmpty()) {
        return false;
    }

    auto lower = qpath.toLower();
    if (lower.endsWith(".yaml")) {
        load_yaml = true;
    } else if (lower.endsWith(".csv")) {
        load_yaml = false;
    } else {
        load_yaml = selected_filter.contains("yaml", Qt::CaseInsensitive);
    }

    path = qpath.toStdString();
    return true;
}

void WaypointEditorTool::handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string path;
    bool save_as_yaml = false;
    if (!requestFilePathForSaving(path, save_as_yaml)) {
        res->success = false;
        res->message = "Save canceled by user";
        return;
    }

    std::string error;
    bool ok = false;
    if (save_as_yaml) {
        ok = io::WaypointYaml::Save(waypoint_sequence_.waypoints(), path, error);
    } else {
        ok = io::WaypointCsv::Save(waypoint_sequence_.waypoints(), path, error);
    }

    if (!ok) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Cannot open file:\n%1").arg(QString::fromStdString(path)));
        res->success = false;
        res->message = error;
        return;
    }

    res->success = true;
    res->message = "Saved " + std::to_string(waypoint_sequence_.size()) + " waypoints to " + path;
}

void WaypointEditorTool::handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string path;
    bool load_yaml = false;
    if (!requestFilePathForLoading(path, load_yaml)) {
        res->success = false;
        res->message = "Load canceled by user";
        return;
    }

    std::vector<Waypoint> loaded;
    std::string error;
    bool ok = false;
    if (load_yaml) {
        ok = io::WaypointYaml::Load(path, loaded, error);
    } else {
        ok = io::WaypointCsv::Load(path, loaded, error);
    }

    if (!ok) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Cannot open file:\n%1").arg(QString::fromStdString(path)));
        res->success = false;
        res->message = error;
        return;
    }

    for (auto &wp : loaded) {
        if (wp.pose.header.frame_id.empty()) {
            wp.pose.header.frame_id = "map";
        }
        wp.pose.header.stamp = nh_->now();
    }
    waypoint_sequence_.assign(std::move(loaded));
    waypoint_sequence_.snapshotHistory();
    pose_dirty_ = false;
    updateWaypointMarker();
    updateLastDistanceFromWaypoint(static_cast<int>(waypoint_sequence_.size()) - 1);
    publishRangeMetrics();

    res->success = true;
    res->message = "Loaded " + std::to_string(waypoint_sequence_.size()) + " waypoints from " + path;
}

void WaypointEditorTool::handleUndoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!waypoint_sequence_.undo()) {
        res->success = false;
        res->message = "No more actions to undo";
        return;
    }
    pose_dirty_ = false;
    updateWaypointMarker();
    updateLastDistanceFromWaypoint(static_cast<int>(waypoint_sequence_.size()) - 1);
    publishRangeMetrics();
    res->success = true;
    res->message = "Undid waypoint change";
}

void WaypointEditorTool::handleRedoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!waypoint_sequence_.redo()) {
        res->success = false;
        res->message = "No more actions to redo";
        return;
    }
    pose_dirty_ = false;
    updateWaypointMarker();
    updateLastDistanceFromWaypoint(static_cast<int>(waypoint_sequence_.size()) - 1);
    publishRangeMetrics();
    res->success = true;
    res->message = "Redid waypoint change";
}

void WaypointEditorTool::activate() {}
void WaypointEditorTool::deactivate()
{
    PoseTool::deactivate();
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorTool, rviz_common::Tool)
