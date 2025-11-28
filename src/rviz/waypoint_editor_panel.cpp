#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QPixmap>
#include <QFont>
#include <QFontMetrics>
#include <QSizePolicy>
#include <QDoubleSpinBox>
#include <QSignalBlocker>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>

#include <chrono>

#include "waypoint_editor/rviz/waypoint_editor_panel.hpp"

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
    status_value_label_->setWordWrap(true);
    status_value_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

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
    QHBoxLayout *status_row        = new QHBoxLayout;
    status_row->setContentsMargins(0, 0, 0, 0);
    status_row->setSpacing(4);
    status_text_label_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    status_value_label_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    status_row->addWidget(status_text_label_);
    status_row->addWidget(status_value_label_, /*stretch=*/1);
    status_row->addStretch();
    QHBoxLayout *last_wp_dist_row  = makeRow(last_wp_dist_text_label_,  last_wp_dist_value_label_);
    QHBoxLayout *total_wp_dist_row = makeRow(total_wp_dist_text_label_, total_wp_dist_value_label_);

    // Combine rows into container
    QVBoxLayout *status_container = new QVBoxLayout;
    status_container->setContentsMargins(6, 6, 6, 6);
    status_container->setSpacing(6);
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
    // Status group
    QGroupBox *status_group = new QGroupBox(tr("Status"), this);
    status_group->setFont(smallFont);
    status_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    status_group->setLayout(status_container);
    // Nudge logo down to align visually with group box height (account for title bar)
    logo_layout->setContentsMargins(0, status_group->fontMetrics().height(), 0, 0);
    top_layout->addWidget(status_group);
    top_layout->addLayout(logo_layout);
    top_layout->setAlignment(status_group, Qt::AlignVCenter);
    top_layout->setAlignment(logo_layout,  Qt::AlignVCenter);

    // Buttons
    load_2d_map_button_    = new QPushButton("Load 2D Map", this);
    load_3d_map_button_    = new QPushButton("Load 3D Map", this);
    load_waypoints_button_ = new QPushButton("Load WPs", this);
    save_waypoints_button_ = new QPushButton("Save WPs", this);
    undo_button_           = new QPushButton("Undo", this);
    redo_button_           = new QPushButton("Redo", this);
    clear_button_          = new QPushButton("Clear All", this);

    // Button sizing/styling to break monotony
    for (auto *btn : {load_2d_map_button_, load_3d_map_button_, load_waypoints_button_, save_waypoints_button_}) {
        btn->setMinimumWidth(90);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    }
    for (auto *btn : {undo_button_, redo_button_}) {
        btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        btn->setMinimumWidth(60);
    }
    clear_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    clear_button_->setMinimumWidth(80);
    clear_button_->setStyleSheet(""); // match other buttons

    // Map section
    QHBoxLayout *map_row = new QHBoxLayout;
    map_row->setContentsMargins(6, 6, 6, 6);
    map_row->setSpacing(8);
    map_row->addWidget(load_2d_map_button_, /*stretch=*/1);
    map_row->addWidget(load_3d_map_button_, /*stretch=*/1);
    QGroupBox *map_group = new QGroupBox(tr("Map"), this);
    map_group->setFont(smallFont);
    map_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    map_group->setLayout(map_row);
    map_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    // Waypoints section
    QHBoxLayout *wp_row1 = new QHBoxLayout;
    wp_row1->setContentsMargins(6, 6, 6, 0);
    wp_row1->setSpacing(8);
    wp_row1->addWidget(undo_button_, /*stretch=*/1);
    wp_row1->addWidget(redo_button_, /*stretch=*/1);
    wp_row1->addWidget(clear_button_, /*stretch=*/1);

    QHBoxLayout *wp_row2 = new QHBoxLayout;
    wp_row2->setContentsMargins(6, 0, 6, 6);
    wp_row2->setSpacing(8);
    wp_row2->addWidget(load_waypoints_button_, /*stretch=*/1);
    wp_row2->addWidget(save_waypoints_button_, /*stretch=*/1);

    QVBoxLayout *wp_layout = new QVBoxLayout;
    wp_layout->setContentsMargins(0, 0, 0, 0);
    wp_layout->setSpacing(4);
    wp_layout->addLayout(wp_row1);
    wp_layout->addLayout(wp_row2);

    QGroupBox *action_group = new QGroupBox(tr("Waypoints"), this);
    action_group->setFont(smallFont);
    action_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    action_group->setLayout(wp_layout);
    action_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    QLabel *auto_label = new QLabel("Auto Δd :", this);
    auto_label->setFont(boldFont);
    auto_label->setContentsMargins(0, 0, 0, 0);

    auto_distance_spin_ = new QDoubleSpinBox(this);
    auto_distance_spin_->setDecimals(2);
    auto_distance_spin_->setRange(0.01, 1000.0);
    auto_distance_spin_->setSingleStep(0.10);
    auto_distance_spin_->setValue(1.00);
    auto_distance_spin_->setSuffix(" m");
    auto_distance_spin_->setFont(smallFont);
    auto_distance_spin_->setMaximumWidth(90);
    auto_distance_spin_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    QLabel *auto_topic_label = new QLabel("Topic", this);
    auto_topic_label->setFont(boldFont);
    auto_topic_label->setContentsMargins(0, 0, 0, 0);
    auto_topic_edit_ = new QLineEdit("/amcl_pose", this);
    auto_topic_edit_->setFont(smallFont);
    auto_topic_edit_->setMinimumWidth(140);

    QLabel *auto_type_label = new QLabel("Type", this);
    auto_type_label->setFont(boldFont);
    auto_type_label->setContentsMargins(0, 0, 0, 0);
    auto_type_combo_ = new QComboBox(this);
    auto_type_combo_->setFont(smallFont);
    auto_type_combo_->addItem("PoseWithCovarianceStamped");
    auto_type_combo_->addItem("PoseStamped");

    auto_toggle_button_ = new QPushButton("Start Auto Capture", this);
    auto_toggle_button_->setCheckable(true);
    auto_toggle_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QHBoxLayout *auto_row1 = new QHBoxLayout;
    auto_row1->setContentsMargins(0, 0, 0, 0);
    auto_row1->setSpacing(6);
    auto_row1->addWidget(auto_label);
    auto_row1->addWidget(auto_distance_spin_, /*stretch=*/0);
    auto_row1->addStretch();
    auto_row1->addWidget(auto_toggle_button_, /*stretch=*/0);

    QHBoxLayout *auto_row2 = new QHBoxLayout;
    auto_row2->setContentsMargins(0, 0, 0, 0);
    auto_row2->setSpacing(6);
    auto_row2->addWidget(auto_topic_label);
    auto_row2->addWidget(auto_topic_edit_, /*stretch=*/1);
    auto_row2->addWidget(auto_type_label);
    auto_row2->addWidget(auto_type_combo_, /*stretch=*/0);

    QVBoxLayout *auto_layout = new QVBoxLayout;
    auto_layout->setContentsMargins(6, 6, 6, 6);
    auto_layout->setSpacing(6);
    auto_layout->addLayout(auto_row1);
    auto_layout->addLayout(auto_row2);

    QGroupBox *auto_group = new QGroupBox(tr("Auto Capture"), this);
    auto_group->setFont(smallFont);
    auto_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    auto_group->setLayout(auto_layout);
    auto_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    // Main layout
    layout_ = new QVBoxLayout;
    layout_->setContentsMargins(6, 6, 6, 6);
    layout_->setSpacing(8);
    layout_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    layout_->setSizeConstraint(QLayout::SetMinimumSize);

    layout_->addLayout(top_layout);
    layout_->addSpacing(8);
    layout_->addWidget(map_group);
    layout_->addSpacing(6);
    layout_->addWidget(action_group);
    layout_->addSpacing(6);
    layout_->addWidget(auto_group);
    layout_->addStretch();

    setLayout(layout_);
    setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

    // Connect signals
    connect(load_2d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad2DMapButtonClick);
    connect(load_3d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad3DMapButtonClick);
    connect(load_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoadWaypointsButtonClick);
    connect(save_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onSaveWaypointsButtonClick);
    connect(undo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onUndoWaypointsButtonClick);
    connect(redo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRedoWaypointsButtonClick);
    connect(clear_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onClearWaypointsButtonClick);
    connect(auto_toggle_button_, &QPushButton::toggled, this, &WaypointEditorPanel::onAutoToggle);
}

WaypointEditorPanel::~WaypointEditorPanel() {}

void WaypointEditorPanel::onInitialize()
{
    nh_               = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    load_map_client_  = nh_->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    load_client_      = nh_->create_client<std_srvs::srv::Trigger>("load_waypoints");
    save_client_      = nh_->create_client<std_srvs::srv::Trigger>("save_waypoints");
    undo_client_      = nh_->create_client<std_srvs::srv::Trigger>("undo_waypoints");
    redo_client_      = nh_->create_client<std_srvs::srv::Trigger>("redo_waypoints");
    clear_client_     = nh_->create_client<std_srvs::srv::Trigger>("clear_waypoints");
    auto_start_client_= nh_->create_client<std_srvs::srv::Trigger>("start_auto_waypoints");
    auto_stop_client_ = nh_->create_client<std_srvs::srv::Trigger>("stop_auto_waypoints");
    auto_distance_pub_= nh_->create_publisher<std_msgs::msg::Float64>("auto_waypoint_min_distance", rclcpp::QoS(1).transient_local());

    last_wp_dist_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "last_wp_dist", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            QMetaObject::invokeMethod(last_wp_dist_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(msg->data, 'f', 3) + " m"));
        }
    );

    total_wp_dist_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "total_wp_dist", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            QMetaObject::invokeMethod(total_wp_dist_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(msg->data, 'f', 3) + " m"));
        }
    );
    
    cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void WaypointEditorPanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
}

void WaypointEditorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void WaypointEditorPanel::onAutoToggle(bool checked)
{
    if (!auto_start_client_ || !auto_stop_client_) {
        return;
    }

    // Push latest threshold before (re)starting.
    std_msgs::msg::Float64 msg;
    msg.data = auto_distance_spin_->value();
    auto_distance_pub_->publish(msg);

    // Push topic/type parameters.
    std::string topic = auto_topic_edit_->text().toStdString();
    std::string type = auto_type_combo_->currentText().toStdString();
    std::vector<rclcpp::Parameter> params;
    params.emplace_back("auto_pose_topic", topic);
    params.emplace_back("auto_pose_type", type);
    auto results = nh_->set_parameters(params);
    for (const auto &res : results) {
        if (!res.successful) {
        postStatusMessage(QString::fromStdString(res.reason));
        QSignalBlocker blocker(auto_toggle_button_);
        auto_toggle_button_->setChecked(false);
        auto_toggle_button_->setText(tr("Start Auto"));
        setAutoControlsEnabled(true);
        return;
        }
    }

    auto client = checked ? auto_start_client_ : auto_stop_client_;
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
        postStatusMessage(tr("Auto service unavailable"));
        QSignalBlocker blocker(auto_toggle_button_);
        auto_toggle_button_->setChecked(false);
        auto_toggle_button_->setText(tr("Start Auto"));
        setAutoControlsEnabled(true);
        return;
    }

    auto_toggle_button_->setText(checked ? tr("Stop Auto Capture") : tr("Start Auto Capture"));
    if (checked) {
        setAutoControlsEnabled(false);
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(req,
        [this, checked](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            std::string message = checked ? "Auto capture start failed" : "Auto capture stop failed";
            try {
                auto response = future.get();
                ok = response->success;
                message = response->message;
            } catch (...) {
                ok = false;
            }

            postStatusMessage(QString::fromStdString(message));
            if (!ok) {
                QMetaObject::invokeMethod(auto_toggle_button_, [this]() {
                    QSignalBlocker blocker(auto_toggle_button_);
                    auto_toggle_button_->setChecked(false);
                    auto_toggle_button_->setText(tr("Start Auto Capture"));
                }, Qt::QueuedConnection);
                QMetaObject::invokeMethod(this, [this]() {
                    setAutoControlsEnabled(true);
                }, Qt::QueuedConnection);
            } else {
                QMetaObject::invokeMethod(this, [this, checked]() {
                    setAutoControlsEnabled(!checked);
                }, Qt::QueuedConnection);
            }
        }
    );
}

void WaypointEditorPanel::setAutoControlsEnabled(bool enabled)
{
    auto_distance_spin_->setEnabled(enabled);
    auto_topic_edit_->setEnabled(enabled);
    auto_type_combo_->setEnabled(enabled);
}

void WaypointEditorPanel::postStatusMessage(const QString &msg)
{
    QMetaObject::invokeMethod(
        status_value_label_,
        "setText",
        Qt::QueuedConnection,
        Q_ARG(QString, msg));
}

void WaypointEditorPanel::onLoad2DMapButtonClick()
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
            postStatusMessage(ok ? tr("Loaded 2D Map") : tr("Failed to load 2D Map"));
        });
}

void WaypointEditorPanel::onLoad3DMapButtonClick()
{
    QString qpath = QFileDialog::getOpenFileName(
        this,
        tr("Open 3D Map PCD"),
        "",
        tr("PCD Files (*.pcd)"));

    if (qpath.isEmpty()) {
        return;
    }

    const auto path = qpath.toStdString();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    int ret = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    if (ret != 0 || cloud->empty()) {
        postStatusMessage(tr("Failed to load 3D Map"));
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = "map";
    cloud_pub_->publish(msg);
    postStatusMessage(tr("Loaded 3D Map"));
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
            postStatusMessage(ok ? tr("Loaded WPs") : tr("Failed to load WPs"));
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
            postStatusMessage(ok ? tr("Saved WPs") : tr("Failed to save WPs"));
        });
}

void WaypointEditorPanel::onUndoWaypointsButtonClick()
{
    if (!undo_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    undo_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            postStatusMessage(ok ? tr("Reverted change") : tr("Nothing to undo"));
        });
}

void WaypointEditorPanel::onRedoWaypointsButtonClick()
{
    if (!redo_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    redo_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            postStatusMessage(ok ? tr("Reapplied change") : tr("Nothing to redo"));
        });
}

void WaypointEditorPanel::onClearWaypointsButtonClick()
{
    if (!clear_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    clear_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            postStatusMessage(ok ? tr("Cleared all waypoints") : tr("Failed to clear waypoints"));
        });
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorPanel, rviz_common::Panel)
