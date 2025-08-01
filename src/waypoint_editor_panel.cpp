#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/srv/load_map.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QPixmap>
#include <QFont>
#include <QFontMetrics>
#include <QSizePolicy>

#include "waypoint_editor/waypoint_editor_panel.hpp"

namespace waypoint_editor
{

WaypointEditorPanel::WaypointEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{
    // Fonts
    QFont smallFont = this->font();
    smallFont.setPointSize(10);
    QFont boldFont = smallFont;
    boldFont.setBold(true);

    // Text labels (Bold Font)
    status_text_label_       = new QLabel("Status :", this);
    last_wp_dist_text_label_ = new QLabel("Last WP Distance :", this);
    total_wp_dist_text_label_= new QLabel("Total WP Distance :", this);

    for (auto *lbl : {status_text_label_, last_wp_dist_text_label_, total_wp_dist_text_label_}) {
        lbl->setFont(boldFont);
        lbl->setContentsMargins(0, 0, 0, 0);
    }

    // Value labels (Normal Font)
    status_value_label_       = new QLabel("Start Waypoint Editor!", this);
    last_wp_dist_value_label_ = new QLabel("0.0", this);
    total_wp_dist_value_label_= new QLabel("0.0", this);

    for (auto *lbl : {status_value_label_, last_wp_dist_value_label_, total_wp_dist_value_label_}) {
        lbl->setFont(smallFont);
        lbl->setContentsMargins(0, 0, 0, 0);
    }

    // Row helper
    auto makeRow = [&](QLabel *text, QLabel *value) {
        QHBoxLayout *row = new QHBoxLayout;
        row->setContentsMargins(0, 8, 0, 0);
        row->setSpacing(4);
        row->addWidget(text);
        row->addWidget(value);
        row->addStretch();
        return row;
    };

    // Build rows
    QHBoxLayout *status_row        = makeRow(status_text_label_,        status_value_label_);
    QHBoxLayout *last_wp_dist_row  = makeRow(last_wp_dist_text_label_,  last_wp_dist_value_label_);
    QHBoxLayout *total_wp_dist_row = makeRow(total_wp_dist_text_label_, total_wp_dist_value_label_);

    // Combine rows into container
    QVBoxLayout *status_container = new QVBoxLayout;
    status_container->setContentsMargins(0, 0, 0, 0);
    status_container->setSpacing(0);
    status_container->addLayout(status_row);
    status_container->addLayout(last_wp_dist_row);
    status_container->addLayout(total_wp_dist_row);

    // Setup logo
    QLabel *logo_label = new QLabel(this);
    QPixmap logo = rviz_common::loadPixmap("package://waypoint_editor/icons/classes/waypoint_editor_logo.png");
    QPixmap small_logo = logo.scaledToWidth(120, Qt::SmoothTransformation);
    logo_label->setPixmap(small_logo);
    logo_label->setFixedSize(small_logo.size());
    logo_label->setAlignment(Qt::AlignCenter);
    logo_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    // Logo layout (centered vertically)
    QVBoxLayout *logo_layout = new QVBoxLayout;
    logo_layout->setContentsMargins(0, 0, 0, 0);
    logo_layout->setSpacing(0);
    logo_layout->addStretch();
    logo_layout->addWidget(logo_label);
    logo_layout->addStretch();

    // Top layout: status on left, logo on right
    QHBoxLayout *top_layout = new QHBoxLayout;
    top_layout->setContentsMargins(0, 0, 0, 0);
    top_layout->setSpacing(6);
    top_layout->addLayout(status_container);
    top_layout->addLayout(logo_layout);
    top_layout->setAlignment(status_container, Qt::AlignVCenter);
    top_layout->setAlignment(logo_layout,      Qt::AlignVCenter);

    // Buttons
    load_2d_map_button_    = new QPushButton("Load Map", this);
    load_waypoints_button_ = new QPushButton("Load WPs", this);
    save_waypoints_button_ = new QPushButton("Save WPs", this);

    QHBoxLayout *button_layout = new QHBoxLayout;
    button_layout->setContentsMargins(0, 0, 0, 0);
    button_layout->setSpacing(8);
    button_layout->addWidget(load_2d_map_button_);
    button_layout->addWidget(load_waypoints_button_);
    button_layout->addWidget(save_waypoints_button_);

    // Main layout
    layout_ = new QVBoxLayout;
    layout_->setContentsMargins(6, 6, 6, 6);
    layout_->setSpacing(8);

    layout_->addLayout(top_layout);
    layout_->addSpacing(8);
    layout_->addLayout(button_layout);
    layout_->addStretch();

    setLayout(layout_);

    // Connect signals
    connect(load_2d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoadMapButtonClick);
    connect(load_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoadWaypointsButtonClick);
    connect(save_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onSaveWaypointsButtonClick);
}

WaypointEditorPanel::~WaypointEditorPanel() {}

void WaypointEditorPanel::onInitialize()
{
    nh_               = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    load_map_client_  = nh_->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    load_client_      = nh_->create_client<std_srvs::srv::Trigger>("load_waypoints");
    save_client_      = nh_->create_client<std_srvs::srv::Trigger>("save_waypoints");

    last_wp_dist_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "last_wp_dist", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            QMetaObject::invokeMethod(
                last_wp_dist_value_label_, "setText",
                Qt::QueuedConnection,
                Q_ARG(QString, QString::number(msg->data, 'f', 3) + " m")
        );
    });

    total_wp_dist_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "total_wp_dist", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
        QMetaObject::invokeMethod(
            total_wp_dist_value_label_, "setText",
            Qt::QueuedConnection,
            Q_ARG(QString, QString::number(msg->data, 'f', 3) + " m")
        );
    });
    
}

void WaypointEditorPanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
}

void WaypointEditorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void WaypointEditorPanel::onLoadMapButtonClick()
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
        [this](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = true;
            } catch (const std::exception &e) {
                ok = false;
            }
            QMetaObject::invokeMethod(status_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, ok ? tr("Loaded Map") : tr("Failed to load Map")));
        });
}

void WaypointEditorPanel::onLoadWaypointsButtonClick()
{
    if (!load_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    load_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            QMetaObject::invokeMethod(status_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, ok ? tr("Loaded WPs") : tr("Failed to load WPs")));
        });
}

void WaypointEditorPanel::onSaveWaypointsButtonClick()
{
    if (!save_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    save_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            QMetaObject::invokeMethod(status_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, ok ? tr("Saved WPs") : tr("Failed to save WPs")));
        });
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorPanel, rviz_common::Panel)
