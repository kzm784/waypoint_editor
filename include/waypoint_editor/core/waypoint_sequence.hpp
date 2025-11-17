#ifndef WAYPOINT_EDITOR__CORE__WAYPOINT_SEQUENCE_HPP_
#define WAYPOINT_EDITOR__CORE__WAYPOINT_SEQUENCE_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <cstddef>
#include <vector>

#include "waypoint_editor/core/waypoint.hpp"

namespace waypoint_editor
{

class WaypointSequence
{
public:
    using Container = std::vector<Waypoint>;

    WaypointSequence() = default;

    int appendWaypoint(Waypoint waypoint);
    void insertWaypoint(std::size_t index, Waypoint waypoint);
    void eraseWaypoint(std::size_t index);
    void assign(Container waypoints);
    void clear();

    std::size_t size() const noexcept { return waypoints_.size(); }
    bool empty() const noexcept { return waypoints_.empty(); }

    Waypoint &at(std::size_t index);
    const Waypoint &at(std::size_t index) const;

    Container &waypoints() { return waypoints_; }
    const Container &waypoints() const { return waypoints_; }

    void updatePose(std::size_t index, const geometry_msgs::msg::Pose &pose);

    double totalDistance() const noexcept { return total_distance_; }
    double lastSegmentDistance() const noexcept { return last_segment_distance_; }

    void resetHistory();
    void snapshotHistory();
    bool undo();
    bool redo();

private:
    double computeSegment(std::size_t first_index) const;
    void recalcDistances();

    Container waypoints_;
    double total_distance_{0.0};
    double last_segment_distance_{0.0};
    std::vector<Container> history_;
    std::size_t history_index_{0};
};

}  // namespace waypoint_editor

#endif  // WAYPOINT_EDITOR__CORE__WAYPOINT_SEQUENCE_HPP_
