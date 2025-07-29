#ifndef WAYPOINT_EDITER__WAYPOINT_EDITER_PANEL_HPP_
#define WAYPOINT_EDITER__WAYPOINT_EDITER_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QtWidgets>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/srv/load_map.hpp>

namespace waypoint_editer
{

class WaypointEditerPanel : public rviz_common::Panel
{
    Q_OBJECT
    
    public:
        explicit WaypointEditerPanel(QWidget *parent = nullptr);
        ~WaypointEditerPanel() override;

        void onInitialize() override;
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

    protected Q_SLOTS:
        void onLoad2DMap();
        void onLoadWaypointsButtonClick();
        void onSaveWaypointsButtonClick();

    private:
        QVBoxLayout *layout_;
        QHBoxLayout *button_layout_;
        QPushButton *load_2d_map_button_;
        QPushButton *load_waypoints_button_;
        QPushButton *save_waypoints_button_;
        QPushButton *save_waypoints_as_button_;
        QScrollArea *status_scroll_area_;
        QLabel *status_label_;

        rclcpp::Node::SharedPtr nh_;
        rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
};
    
} // namespace waypoint_editer

#endif // WAYPOINT_EDITER__WAYPOINT_EDITER_PANEL_HPP_