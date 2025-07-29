#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <nav2_rviz_plugins/goal_common.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>
#include <fstream>

#include "waypoint_editer/waypoint_editer_tool.hpp"

using namespace std::placeholders;

namespace waypoint_editer
{

WaypointEditerTool::WaypointEditerTool() : rviz_default_plugins::tools::PoseTool() {}
WaypointEditerTool::~WaypointEditerTool() {}

void WaypointEditerTool::onInitialize()
{
    PoseTool::onInitialize();

    setName("Add Waypoint");

    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server",
        nh_,
        rclcpp::SystemDefaultsQoS(),
        rclcpp::SystemDefaultsQoS()
    );
    line_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("waypoint_line", 10);
    line_timer_ = nh_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&WaypointEditerTool::publishLineMarker, this)
    );
    save_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "save_waypoints",
        std::bind(&WaypointEditerTool::handleSaveWaypoints, this, _1, _2)
    );
    load_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "load_waypoints",
        std::bind(&WaypointEditerTool::handleLoadWaypoints, this, _1, _2)
    );

    waypoints_.clear();
}

void WaypointEditerTool::onPoseSet(double x, double y, double theta)
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

    waypoints_.push_back(std::move(wp));
    
    int new_id = static_cast<int>(waypoints_.size() - 1);
    auto int_marker = createWaypointMarker(new_id);
    server_->insert(int_marker, std::bind(&WaypointEditerTool::processFeedback, this, _1));
    server_->applyChanges();
    RCLCPP_INFO(nh_->get_logger(), "Added waypoint %d", new_id);
    
    deactivate();
}

void WaypointEditerTool::updateWaypointMarker()
{
    server_->clear();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        auto int_marker = createWaypointMarker(static_cast<int>(i));
        server_->insert(int_marker, std::bind(&WaypointEditerTool::processFeedback, this, _1));
    }

    server_->applyChanges();
}

visualization_msgs::msg::InteractiveMarker WaypointEditerTool::createWaypointMarker(const int id)
{
    const auto & wp = waypoints_[id];

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = wp.pose.header.frame_id;;
    int_marker.name = std::to_string(id);
    int_marker.description = waypoints_.at(id).function_command;
    int_marker.scale = 1.0;
    int_marker.pose.position = wp.pose.pose.position;
    int_marker.pose.orientation = wp.pose.pose.orientation;

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
        std::string id_text = "ID:" + std::to_string(id) + "\n" + waypoints_.at(id).function_command;;
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

void WaypointEditerTool::processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb)
{
    int id = std::stoi(fb->marker_name);

    switch (fb->event_type)
    {
        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
            waypoints_[id].pose.pose.position = fb->pose.position;
            waypoints_[id].pose.pose.orientation.z = fb->pose.orientation.z;
            waypoints_[id].pose.pose.orientation.w = fb->pose.orientation.w;

            server_->setPose(fb->marker_name, fb->pose);
            server_->applyChanges();
            break;

        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
            processMenuControl(fb);
            break;

        default:
            break;
    }    
}

void WaypointEditerTool::processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb)
{
    if (fb->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) { return; }

    int idx = std::stoi(fb->marker_name);
    if (idx < 0 || idx >= static_cast<int>(waypoints_.size())) { return; }

    switch (fb->menu_entry_id) {
      
        // Delete Waypoint
        case 1:
            waypoints_.erase(waypoints_.begin() + idx);
            updateWaypointMarker();
            RCLCPP_INFO(nh_->get_logger(), "Deleted waypoint %d", idx);
            break;

        // Change Waypoint ID
        case 2:
        {
         bool ok = false;
            QString current = QString::fromStdString(std::to_string(idx));
            QString text = QInputDialog::getText(
                nullptr,
                tr("Change Waypoint ID"),
                tr("Enter New Waypoint ID: %1").arg(idx),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                int insert_id = text.toInt();
                if (0 <= insert_id && insert_id < static_cast<int>(waypoints_.size())) {
                    auto tmp_wp = waypoints_[idx];
                    waypoints_.erase(waypoints_.begin() + idx);
                    waypoints_.insert(waypoints_.begin() + insert_id, tmp_wp);
                    updateWaypointMarker();
                    RCLCPP_INFO(nh_->get_logger(), "Changed waypoint id %d to %d", idx, insert_id);
                } else {
                    QMessageBox::warning(
                        nullptr,
                        tr("Invalid Waypoint ID"),
                        tr("Waypoint ID %1 is out of range.\n"
                        "Please enter a value between %2 and %3.")
                        .arg(insert_id)
                        .arg(0)
                        .arg(static_cast<int>(waypoints_.size() - 1))
                    );
                }
            }
        }
            break;

        // Add / Edit Function Command
        case 3:
        {
            bool ok = false;
            QString current = QString::fromStdString(waypoints_[idx].function_command);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Edit Function Command"),
                tr("Enter command for waypoint ID: %1").arg(idx),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                waypoints_[idx].function_command = text.toStdString();
                updateWaypointMarker();
                RCLCPP_INFO(nh_->get_logger(), "Updated command of waypoint %d to '%s'", idx, waypoints_[idx].function_command.c_str());
            }
        }
            break;

      default:
            break;
    }
}

void WaypointEditerTool::publishLineMarker()
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

    for (size_t i = 1; i < waypoints_.size(); ++i) {
        geometry_msgs::msg::Point p0 = waypoints_[i-1].pose.pose.position;
        geometry_msgs::msg::Point p1 = waypoints_[i].pose.pose.position;
        line.points.push_back(p0);
        line.points.push_back(p1);
    }

    line_pub_->publish(line);
}

void WaypointEditerTool::handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    QString qpath = QFileDialog::getSaveFileName(
        nullptr,
        tr("Save Waypoints As"),
        "",
        tr("CSV Files (*.csv)")
    );
    if (qpath.isEmpty()) {
        res->success = false;
        res->message = "Save canceled by user";
        return;
    }

    if (!qpath.endsWith(".csv", Qt::CaseInsensitive)) {
        qpath += ".csv";
    }
    const std::string path = qpath.toStdString();

    std::ofstream ofs(path);
    if (!ofs) {
        QMessageBox::warning(nullptr, tr("Error"),
            tr("Cannot open file:\n%1").arg(qpath));
        res->success = false;
        res->message = "Failed to open file: " + path;
        return;
    }

    std::vector<std::vector<std::string>> all_cmds;
    all_cmds.reserve(waypoints_.size());
    size_t max_cmds = 0;
    for (const auto & wp : waypoints_) {
        std::vector<std::string> parts;
        std::istringstream ss(wp.function_command);
        std::string token;
        while (std::getline(ss, token, ',')) {
            parts.push_back(token);
        }
        max_cmds = std::max(max_cmds, parts.size());
        all_cmds.push_back(std::move(parts));
    }

    ofs << "id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command";
    for (size_t j = 1; j < max_cmds; ++j) {
        ofs << ",";
    }
    ofs << ",\n";

    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const auto & p = waypoints_[i].pose.pose;
        ofs
          << i << ","
          << p.position.x << "," << p.position.y << "," << p.position.z << ","
          << p.orientation.x << "," << p.orientation.y << ","
          << p.orientation.z << "," << p.orientation.w;

        const auto & parts = all_cmds[i];
        for (size_t j = 0; j < max_cmds; ++j) {
            ofs << ",";
            if (j < parts.size()) {
                ofs << parts[j];
            }
        }
        ofs << ",\n";
    }

    ofs.close();
    res->success = true;
    res->message = "Saved " + std::to_string(waypoints_.size()) + " waypoints to " + path;
    return;
}

void WaypointEditerTool::handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    QString qpath = QFileDialog::getOpenFileName(
        nullptr,
        tr("Open Waypoints"),
        "",
        tr("CSV Files (*.csv)"));
    if (qpath.isEmpty()) {
        res->success = false;
        res->message = "Load canceled by user";
    return;
    }

    QFile file(qpath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(nullptr, tr("Error"),
        tr("Cannot open file:\n%1").arg(qpath));
    res->success = false;
    res->message = "Failed to open file: " + qpath.toStdString();
    return;
    }

    QTextStream in(&file);
    QString header = in.readLine();

    waypoints_.clear();

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;

        QStringList cols = line.split(',');
        if (cols.size() < 9) continue;

        Waypoint wp;
        wp.pose.header.frame_id = "map";
        wp.pose.header.stamp = nh_->now();
        bool ok = false;
        wp.pose.pose.position.x    = cols[1].toDouble(&ok);  if (!ok) continue;
        wp.pose.pose.position.y    = cols[2].toDouble(&ok);  if (!ok) continue;
        wp.pose.pose.position.z    = cols[3].toDouble(&ok);  if (!ok) continue;
        wp.pose.pose.orientation.x = cols[4].toDouble(&ok);  if (!ok) continue;
        wp.pose.pose.orientation.y = cols[5].toDouble(&ok);  if (!ok) continue;
        wp.pose.pose.orientation.z = cols[6].toDouble(&ok);  if (!ok) continue;
        wp.pose.pose.orientation.w = cols[7].toDouble(&ok);  if (!ok) continue;

        QStringList cmdParts = cols.mid(8);
        for (int i = cmdParts.size() - 1; i >= 0; --i) {
            if (cmdParts[i].trimmed().isEmpty()) {
                cmdParts.removeAt(i);
            }
        }

        QString cmd = cmdParts.join(',');
        if (cmd.startsWith('"') && cmd.endsWith('"') && cmd.size() >= 2) {
            cmd = cmd.mid(1, cmd.size() - 2);
        }
        wp.function_command = cmd.toStdString();
        waypoints_.push_back(std::move(wp));
    }
    file.close();

    updateWaypointMarker();

    res->success = true;
    res->message = "Loaded " + std::to_string(waypoints_.size()) + " waypoints from " + qpath.toStdString();
}

void WaypointEditerTool::activate() {}
void WaypointEditerTool::deactivate()
{
    PoseTool::deactivate();
}

} // namespace waypoint_editer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editer::WaypointEditerTool, rviz_common::Tool)