#include "waypoint_editor/core/waypoint_sequence.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace waypoint_editor
{

namespace
{
constexpr double kZeroDistance = 0.0;
}

int WaypointSequence::appendWaypoint(Waypoint waypoint)
{
    waypoints_.push_back(std::move(waypoint));
    if (waypoints_.size() >= 2) {
        last_segment_distance_ = computeSegment(waypoints_.size() - 2);
        total_distance_ += last_segment_distance_;
    } else {
        last_segment_distance_ = kZeroDistance;
        total_distance_ = kZeroDistance;
    }
    return static_cast<int>(waypoints_.size() - 1);
}

void WaypointSequence::insertWaypoint(std::size_t index, Waypoint waypoint)
{
    if (index > waypoints_.size()) {
        index = waypoints_.size();
    }
    waypoints_.insert(waypoints_.begin() + static_cast<std::ptrdiff_t>(index), std::move(waypoint));
    recalcDistances();
}

void WaypointSequence::eraseWaypoint(std::size_t index)
{
    if (index >= waypoints_.size()) {
        return;
    }
    waypoints_.erase(waypoints_.begin() + static_cast<std::ptrdiff_t>(index));
    recalcDistances();
}

void WaypointSequence::assign(Container waypoints)
{
    waypoints_ = std::move(waypoints);
    recalcDistances();
}

void WaypointSequence::clear()
{
    waypoints_.clear();
    total_distance_ = kZeroDistance;
    last_segment_distance_ = kZeroDistance;
}

Waypoint &WaypointSequence::at(std::size_t index)
{
    return waypoints_.at(index);
}

const Waypoint &WaypointSequence::at(std::size_t index) const
{
    return waypoints_.at(index);
}

void WaypointSequence::updatePose(std::size_t index, const geometry_msgs::msg::Pose &pose)
{
    if (index >= waypoints_.size()) {
        return;
    }
    waypoints_[index].pose.pose = pose;
    recalcDistances();
}

double WaypointSequence::computeSegment(std::size_t first_index) const
{
    if (first_index + 1 >= waypoints_.size()) {
        return kZeroDistance;
    }
    const auto &p0 = waypoints_[first_index].pose.pose.position;
    const auto &p1 = waypoints_[first_index + 1].pose.pose.position;
    return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

void WaypointSequence::recalcDistances()
{
    total_distance_ = kZeroDistance;
    if (waypoints_.size() >= 2) {
        for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
            total_distance_ += computeSegment(i);
        }
        last_segment_distance_ = computeSegment(waypoints_.size() - 2);
    } else {
        last_segment_distance_ = kZeroDistance;
    }
}

}  // namespace waypoint_editor
