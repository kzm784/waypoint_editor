#ifndef WAYPOINT_EDITOR__WAYPOINT_EDITOR_TOOL_HPP_
#define WAYPOINT_EDITOR__WAYPOINT_EDITOR_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <nav2_rviz_plugins/goal_common.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_default_plugins/visibility_control.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <QObject>

namespace waypoint_editor
{

struct Waypoint
{
    geometry_msgs::msg::PoseStamped pose;
    std::string function_command;
};

class WaypointEditorTool : public rviz_default_plugins::tools::PoseTool
{
    Q_OBJECT

public:
    WaypointEditorTool();
    virtual ~WaypointEditorTool();

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    void onPoseSet(double x, double y, double theta) override;
    void updateWaypointMarker();
    visualization_msgs::msg::InteractiveMarker createWaypointMarker(const int id);
    void processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb);
    void processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb);
    void handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void publishLineMarker();


private Q_SLOTS:  

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::TimerBase::SharedPtr line_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;

    std::vector<Waypoint> waypoints_;
};
    
} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__WAYPOINT_EDITOR_TOOL_HPP_