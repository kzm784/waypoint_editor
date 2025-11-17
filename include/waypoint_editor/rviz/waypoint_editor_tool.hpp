#ifndef WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_
#define WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>

#include "waypoint_editor/core/waypoint_sequence.hpp"

namespace waypoint_editor
{

class WaypointEditorTool : public rviz_default_plugins::tools::PoseTool
{
public:
    WaypointEditorTool();
    ~WaypointEditorTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    void onPoseSet(double x, double y, double theta) override;
    void updateWaypointMarker();
    visualization_msgs::msg::InteractiveMarker createWaypointMarker(int id);
    void processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb);
    void processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb);
    void handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleUndoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleRedoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void publishLineMarker();
    void publishTotalWpsDist();
    void publishLastWpsDist();
    void publishRangeMetrics();
    bool requestFilePathForSaving(std::string &path, bool &save_as_yaml);
    bool requestFilePathForLoading(std::string &path, bool &load_yaml);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_wp_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr last_wp_dist_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr redo_service_;

    WaypointSequence waypoint_sequence_;
    bool pose_dirty_{false};
};

} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_
