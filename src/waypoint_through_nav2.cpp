#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class WaypointThroughNav2 : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNTP = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    WaypointThroughNav2() : Node("waypoint_through_nav2")
    {
        this->declare_parameter<std::string>("waypoint_file", "");
        this->declare_parameter<std::string>("frame_id", "map");

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_waypoint_through",
            std::bind(&WaypointThroughNav2::handleStart, this,
                      std::placeholders::_1, std::placeholders::_2));

        nav_through_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
            this, "navigate_through_poses");

        RCLCPP_INFO(this->get_logger(), "Waypoint through Nav2 bridge node started");
        RCLCPP_INFO(this->get_logger(), "Call service /start_waypoint_through to navigate through poses");
    }

private:
    void handleStart(
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

        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (!loadWaypointsFromCSV(waypoint_file, waypoints)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from: %s", waypoint_file.c_str());
            response->success = false;
            response->message = "Failed to load waypoints from file";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints.size(), waypoint_file.c_str());

        if (!nav_through_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 navigate_through_poses action server not available");
            response->success = false;
            response->message = "Nav2 navigate_through_poses action server not available";
            return;
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses = waypoints;

        RCLCPP_INFO(this->get_logger(), "Sending %zu poses to Nav2 (navigate through)...", waypoints.size());

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointThroughNav2::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointThroughNav2::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointThroughNav2::resultCallback, this, std::placeholders::_1);

        nav_through_client_->async_send_goal(goal_msg, send_goal_options);

        response->success = true;
        response->message = "Poses sent to Nav2 (navigate through)";
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
        std::getline(file, line); // skip header

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() < 8) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid line: %s", line.c_str());
                continue;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = rclcpp::Time(0);

            try {
                pose.pose.position.x = std::stod(tokens[1]);
                pose.pose.position.y = std::stod(tokens[2]);
                pose.pose.position.z = std::stod(tokens[3]);

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

    void goalResponseCallback(const GoalHandleNTP::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2, robot will navigate through poses");
        }
    }

    void feedbackCallback(
        GoalHandleNTP::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
                     "Distance remaining: %.2f, poses remaining: %d",
                     feedback->distance_remaining,
                     feedback->number_of_poses_remaining);
    }

    void resultCallback(const GoalHandleNTP::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigate through poses completed successfully!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigate through poses was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigate through poses was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointThroughNav2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
