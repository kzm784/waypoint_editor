#include "waypoint_editor/io/waypoint_csv.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace waypoint_editor::io
{
namespace
{

std::string Trim(const std::string &text)
{
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) { return std::isspace(c); });
    if (first == text.end()) {
        return {};
    }
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    return std::string(first, last);
}

std::vector<std::string> SplitCsvRow(const std::string &line)
{
    std::vector<std::string> columns;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        columns.emplace_back(std::move(token));
    }
    return columns;
}

std::vector<std::string> SplitFunctionCommand(const std::string &command)
{
    std::vector<std::string> tokens;
    std::stringstream ss(command);
    std::string chunk;
    while (std::getline(ss, chunk, ',')) {
        tokens.emplace_back(std::move(chunk));
    }
    return tokens;
}

bool ParseDouble(const std::string &text, double &value)
{
    try {
        size_t processed = 0U;
        const auto trimmed = Trim(text);
        value = std::stod(trimmed, &processed);
        return processed == trimmed.size();
    } catch (const std::exception &) {
        return false;
    }
}

}  // namespace

bool WaypointCsv::Save(const std::vector<Waypoint> &waypoints, const std::string &path, std::string &error)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        error = "Failed to open file for writing: " + path;
        return false;
    }

    std::vector<std::vector<std::string>> command_columns;
    command_columns.reserve(waypoints.size());
    std::size_t max_columns = 0U;
    for (const auto &wp : waypoints) {
        auto tokens = SplitFunctionCommand(wp.function_command);
        max_columns = std::max(max_columns, tokens.size());
        command_columns.push_back(std::move(tokens));
    }

    ofs << "id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command";
    for (std::size_t column = 1; column < max_columns; ++column) {
        ofs << ",";
    }
    ofs << ",\n";

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const auto &pose = waypoints[i].pose.pose;
        ofs << i << ","
            << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
            << pose.orientation.x << "," << pose.orientation.y << ","
            << pose.orientation.z << "," << pose.orientation.w;

        const auto &tokens = command_columns[i];
        for (std::size_t column = 0; column < max_columns; ++column) {
            ofs << ",";
            if (column < tokens.size()) {
                ofs << tokens[column];
            }
        }
        ofs << ",\n";
    }

    error.clear();
    return true;
}

bool WaypointCsv::Load(const std::string &path, std::vector<Waypoint> &waypoints, std::string &error)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        error = "Failed to open file for reading: " + path;
        return false;
    }

    std::string line;
    std::getline(ifs, line);  // skip header

    std::vector<Waypoint> parsed;
    while (std::getline(ifs, line)) {
        if (Trim(line).empty()) {
            continue;
        }
        auto columns = SplitCsvRow(line);
        if (columns.size() < 9U) {
            continue;
        }

        Waypoint waypoint;
        if (!ParseDouble(columns[1], waypoint.pose.pose.position.x)) { continue; }
        if (!ParseDouble(columns[2], waypoint.pose.pose.position.y)) { continue; }
        if (!ParseDouble(columns[3], waypoint.pose.pose.position.z)) { continue; }
        if (!ParseDouble(columns[4], waypoint.pose.pose.orientation.x)) { continue; }
        if (!ParseDouble(columns[5], waypoint.pose.pose.orientation.y)) { continue; }
        if (!ParseDouble(columns[6], waypoint.pose.pose.orientation.z)) { continue; }
        if (!ParseDouble(columns[7], waypoint.pose.pose.orientation.w)) { continue; }

        std::vector<std::string> command_parts(columns.begin() + 8, columns.end());
        while (!command_parts.empty() && Trim(command_parts.back()).empty()) {
            command_parts.pop_back();
        }

        std::ostringstream command_stream;
        for (std::size_t idx = 0; idx < command_parts.size(); ++idx) {
            if (idx > 0U) {
                command_stream << ",";
            }
            command_stream << command_parts[idx];
        }

        auto command = Trim(command_stream.str());
        if (command.size() >= 2U && command.front() == '"' && command.back() == '"') {
            command = command.substr(1, command.size() - 2);
        }
        waypoint.function_command = command;
        parsed.push_back(std::move(waypoint));
    }

    waypoints = std::move(parsed);
    error.clear();
    return true;
}

}  // namespace waypoint_editor::io
