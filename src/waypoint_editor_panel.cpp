
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/srv/load_map.hpp>

#include "waypoint_editor/waypoint_editor_panel.hpp"

namespace waypoint_editor
{

WaypointEditorPanel::WaypointEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{
    load_2d_map_button_ = new QPushButton("Load 2D Map", this);
    load_waypoints_button_ = new QPushButton("Load Waypoints", this);
    save_waypoints_button_ = new QPushButton("Save Waypoints", this);

    button_layout_ = new QHBoxLayout;
    button_layout_->addWidget(load_2d_map_button_);
    button_layout_->addWidget(load_waypoints_button_);
    button_layout_->addWidget(save_waypoints_button_);

    status_label_ = new QLabel("No file loaded", this);
    status_label_->setWordWrap(false);
    status_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

    status_scroll_area_ = new QScrollArea(this);
    status_scroll_area_->setWidget(status_label_);
    status_scroll_area_->setWidgetResizable(true);
    status_scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    status_scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    status_scroll_area_->setFrameStyle(QFrame::StyledPanel);
    status_scroll_area_->setFixedHeight(60);

    layout_ = new QVBoxLayout;
    layout_->addLayout(button_layout_);
    layout_->addWidget(status_scroll_area_);
    setLayout(layout_);

    connect(load_2d_map_button_, SIGNAL(clicked()), this, SLOT(onLoad2DMap()));
    connect(load_waypoints_button_, SIGNAL(clicked()), this, SLOT(onLoadWaypointsButtonClick()));
    connect(save_waypoints_button_, SIGNAL(clicked()), this, SLOT(onSaveWaypointsButtonClick()));    
}

WaypointEditorPanel::~WaypointEditorPanel() {}

void WaypointEditorPanel::onInitialize()
{
    nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    load_map_client_ = nh_->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    load_client_ = nh_->create_client<std_srvs::srv::Trigger>("load_waypoints");
    save_client_ = nh_->create_client<std_srvs::srv::Trigger>("save_waypoints");    
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
        status_label_->setText("Map load canceled");
        return;
    }

    if (!load_map_client_->wait_for_service(std::chrono::seconds(2))) {
        status_label_->setText("LoadMap service unavailable");
        return;
    }
    auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    req->map_url = qpath.toStdString();

    load_map_client_->async_send_request(req,
        [this, qpath](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
        auto res = future.get();
        bool ok = (res->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);

        QString status = QString("%1: %2")
            .arg(ok ? "Map loaded" : "Map load failed")
            .arg(qpath);
        QMetaObject::invokeMethod(status_label_, "setText",
            Qt::QueuedConnection,
            Q_ARG(QString, status));
        });
}

void WaypointEditorPanel::onLoadWaypointsButtonClick()
{
    if (!load_client_->wait_for_service(std::chrono::seconds(1))) {
        status_label_->setText("Load service unavailable");
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = load_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto res = future.get();
        QMetaObject::invokeMethod(status_label_, "setText",
            Qt::QueuedConnection,
            Q_ARG(QString, QString::fromStdString(res->message)));
        });
}

void WaypointEditorPanel::onSaveWaypointsButtonClick()
{
    if (!save_client_->wait_for_service(std::chrono::seconds(1))) {
        status_label_->setText("Save service unavailable");
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = save_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto res = future.get();
        QMetaObject::invokeMethod(status_label_, "setText",
            Qt::QueuedConnection,
            Q_ARG(QString, QString::fromStdString(res->message)));
        });
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorPanel, rviz_common::Panel)
