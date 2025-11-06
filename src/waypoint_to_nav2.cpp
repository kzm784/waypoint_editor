#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class WaypointToNav2 : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    WaypointToNav2() : Node("waypoint_to_nav2")
    {
        // Declare parameters
        this->declare_parameter<std::string>("waypoint_file", "");
        this->declare_parameter<std::string>("frame_id", "map");

        // Create service to trigger waypoint following
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_waypoint_following",
            std::bind(&WaypointToNav2::handleStartWaypointFollowing, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Create action client for Nav2 waypoint following
        follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints");

        RCLCPP_INFO(this->get_logger(), "Waypoint to Nav2 bridge node started");
        RCLCPP_INFO(this->get_logger(), "Call service /start_waypoint_following to send waypoints to Nav2");
    }

private:
    void handleStartWaypointFollowing(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string waypoint_file;
        this->get_parameter("waypoint_file", waypoint_file);

        if (waypoint_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Waypoint file not specified. Set 'waypoint_file' parameter.");
            response->success = false;
            response->message = "Waypoint file not specified";
            return;
        }

        // Load waypoints from CSV
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (!loadWaypointsFromCSV(waypoint_file, waypoints)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from: %s", waypoint_file.c_str());
            response->success = false;
            response->message = "Failed to load waypoints from file";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints.size(), waypoint_file.c_str());

        // Wait for action server
        if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 follow_waypoints action server not available");
            response->success = false;
            response->message = "Nav2 follow_waypoints action server not available";
            return;
        }

        // Send waypoints to Nav2
        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = waypoints;

        RCLCPP_INFO(this->get_logger(), "Sending %zu waypoints to Nav2...", waypoints.size());

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointToNav2::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointToNav2::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointToNav2::resultCallback, this, std::placeholders::_1);

        follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);

        response->success = true;
        response->message = "Waypoints sent to Nav2";
    }

    bool loadWaypointsFromCSV(const std::string& filename, std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        std::string frame_id;
        this->get_parameter("frame_id", frame_id);

        std::string line;
        // Skip header line
        std::getline(file, line);

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            // CSV format: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command...
            if (tokens.size() < 8) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid line: %s", line.c_str());
                continue;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = rclcpp::Time(0);  // Use time 0 to let Nav2 use current time

            try {
                // Parse position
                pose.pose.position.x = std::stod(tokens[1]);
                pose.pose.position.y = std::stod(tokens[2]);
                pose.pose.position.z = std::stod(tokens[3]);

                // Parse orientation
                pose.pose.orientation.x = std::stod(tokens[4]);
                pose.pose.orientation.y = std::stod(tokens[5]);
                pose.pose.orientation.z = std::stod(tokens[6]);
                pose.pose.orientation.w = std::stod(tokens[7]);

                waypoints.push_back(pose);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s, error: %s", line.c_str(), e.what());
                continue;
            }
        }

        file.close();
        return !waypoints.empty();
    }

    void goalResponseCallback(const GoalHandleFollowWaypoints::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2, robot will start following waypoints");
        }
    }

    void feedbackCallback(
        GoalHandleFollowWaypoints::SharedPtr,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current waypoint: %u", feedback->current_waypoint);
    }

    void resultCallback(const GoalHandleFollowWaypoints::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "All waypoints reached successfully!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Waypoint following was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Waypoint following was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointToNav2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
