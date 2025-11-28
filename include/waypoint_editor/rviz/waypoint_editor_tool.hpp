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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
    void handleClearWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void publishLineMarker();
    void publishTotalWpsDist();
    void publishLastWpsDist();
    void publishRangeMetrics();
    bool requestFilePathForSaving(std::string &path, bool &save_as_yaml);
    bool requestFilePathForLoading(std::string &path, bool &load_yaml);

private:
    int appendWaypointAndRefresh(Waypoint wp);
    bool transformToMapFrame(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output) const;
    void refreshAutoPoseSubscription();
    void handleAutoPose(const geometry_msgs::msg::PoseStamped &pose);

    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_wp_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr last_wp_dist_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr redo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_stop_service_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr auto_distance_sub_;
    std::shared_ptr<rclcpp::SubscriptionBase> auto_pose_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string auto_pose_topic_;
    std::string auto_pose_type_;
    double auto_min_distance_m_{1.0};
    bool auto_enabled_{false};

    WaypointSequence waypoint_sequence_;
    bool pose_dirty_{false};
    double last_displayed_distance_{0.0};

    void updateLastDistanceFromWaypoint(int waypoint_index);
    double computeSegmentDistance(std::size_t first, std::size_t second) const;
};

} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_
