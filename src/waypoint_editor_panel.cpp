#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <QLabel>

#include "waypoint_editor/waypoint_editor_panel.hpp"

namespace waypoint_editor
{

WaypointEditorPanel::WaypointEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{
    QVBoxLayout * status_layout = new QVBoxLayout;
    QLabel * status = new QLabel("Status:", this);
    QLabel * last_wp_dist = new QLabel("Last WP Distance:", this);
    QLabel * total_wp_dist = new QLabel("Total WP Distance:", this);
    status_layout->addWidget(status);
    status_layout->addWidget(last_wp_dist);
    status_layout->addWidget(total_wp_dist);

    QLabel * logo_label = new QLabel(this);
    QPixmap logo = rviz_common::loadPixmap("package://waypoint_editor/icons/classes/waypoint_editor_logo.png");
    QPixmap small_logo = logo.scaledToWidth(120, Qt::SmoothTransformation);
    logo_label->setPixmap(small_logo);
    logo_label->setFixedSize(small_logo.size());
    logo_label->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

    QHBoxLayout * top_layout = new QHBoxLayout;
    top_layout->addLayout(status_layout, /*stretch=*/1);
    top_layout->addWidget(logo_label, /*stretch=*/0, Qt::AlignRight);

    QHBoxLayout * button_layout = new QHBoxLayout;
    load_2d_map_button_    = new QPushButton("Load 2D Map", this);
    load_waypoints_button_ = new QPushButton("Load WPs", this);
    save_waypoints_button_ = new QPushButton("Save WPs", this);
    button_layout->addWidget(load_2d_map_button_);
    button_layout->addWidget(load_waypoints_button_);
    button_layout->addWidget(save_waypoints_button_);

    layout_ = new QVBoxLayout;
    layout_->addLayout(top_layout);
    layout_->addSpacing(12);
    layout_->addLayout(button_layout);
    layout_->addStretch();
    setLayout(layout_);

    connect(load_2d_map_button_, &QPushButton::clicked,
            this, &WaypointEditorPanel::onLoad2DMap);
    connect(load_waypoints_button_, &QPushButton::clicked,
            this, &WaypointEditorPanel::onLoadWaypointsButtonClick);
    connect(save_waypoints_button_, &QPushButton::clicked,
            this, &WaypointEditorPanel::onSaveWaypointsButtonClick);
}

WaypointEditorPanel::~WaypointEditorPanel() {}

void WaypointEditorPanel::onInitialize()
{
    nh_           = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    load_map_client_  = nh_->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    load_client_      = nh_->create_client<std_srvs::srv::Trigger>("load_waypoints");
    save_client_      = nh_->create_client<std_srvs::srv::Trigger>("save_waypoints");    
}

void WaypointEditorPanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
}

void WaypointEditorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void WaypointEditorPanel::onLoad2DMap()
{
    QString qpath = QFileDialog::getOpenFileName(
        this,
        tr("Open 2D Map YAML"),
        "",
        tr("YAML Files (*.yaml)"));

    if (qpath.isEmpty()) {
        return;
    }

    if (!load_map_client_->wait_for_service(std::chrono::seconds(2))) {
        return;
    }
    auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    req->map_url = qpath.toStdString();

    load_map_client_->async_send_request(req,
        [](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
            // handle response if needed
        });
}

void WaypointEditorPanel::onLoadWaypointsButtonClick()
{
    if (!load_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    load_client_->async_send_request(req,
        [](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            // handle response if needed
        });
}

void WaypointEditorPanel::onSaveWaypointsButtonClick()
{
    if (!save_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    save_client_->async_send_request(req,
        [](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            // handle response if needed
        });
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorPanel, rviz_common::Panel)
