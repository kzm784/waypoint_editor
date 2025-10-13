#ifndef WAYPOINT_EDITOR__WAYPOINT_EDITOR_PANEL_HPP_
#define WAYPOINT_EDITOR__WAYPOINT_EDITOR_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace waypoint_editor
{

class WaypointEditorPanel : public rviz_common::Panel
{
    Q_OBJECT
    
public:
    explicit WaypointEditorPanel(QWidget *parent = nullptr);
    ~WaypointEditorPanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config &config) override;
    void save(rviz_common::Config config) const override;

protected Q_SLOTS:
    void onLoadMapButtonClick();
    void onLoadWaypointsButtonClick();
    void onSaveWaypointsButtonClick();

private:
    QLabel *status_text_label_;
    QLabel *status_value_label_;
    QLabel *last_wp_dist_text_label_;
    QLabel *last_wp_dist_value_label_;
    QLabel *total_wp_dist_text_label_;
    QLabel *total_wp_dist_value_label_;
    QVBoxLayout *layout_;
    QHBoxLayout *logo_layout_;
    QHBoxLayout *button_layout_;
    QPushButton *load_2d_map_button_;
    QPushButton *load_waypoints_button_;
    QPushButton *save_waypoints_button_;

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr last_wp_dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr total_wp_dist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};
    
} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__WAYPOINT_EDITOR_PANEL_HPP_
