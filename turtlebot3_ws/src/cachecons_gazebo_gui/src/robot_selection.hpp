#pragma once
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/pose_v.pb.h>

struct GuiArguments
{
  std::string config;
  std::string selectionTopic{"/cachecons/selected_robot"};
  std::map<std::string, std::string> robots;
};

inline GuiArguments parseGuiArguments(const std::vector<std::string> &args)
{
  if (args.empty())
    throw std::invalid_argument("A GUI configuration path is required");
  GuiArguments result;
  result.config = args[0];
  std::set<std::string> entities;
  for (size_t i = 1; i < args.size(); i += 2)
  {
    if (i + 1 == args.size())
      throw std::invalid_argument("Missing value for " + args[i]);
    if (args[i] == "--selection-topic")
    {
      if (args[i + 1].empty() || args[i + 1][0] != '/')
        throw std::invalid_argument("Selection topic must be absolute");
      result.selectionTopic = args[i + 1];
    }
    else if (args[i] == "--robot")
    {
      const auto &mapping = args[i + 1];
      const auto split = mapping.find('=');
      if (split == 0 || split == std::string::npos || split + 1 == mapping.size() ||
          mapping.find('=', split + 1) != std::string::npos)
        throw std::invalid_argument("Robot mapping must be namespace=entity");
      const auto name = mapping.substr(0, split);
      const auto entity = mapping.substr(split + 1);
      if (!result.robots.emplace(name, entity).second || !entities.insert(entity).second)
        throw std::invalid_argument("Robot namespaces and entities must be unique");
    }
    else
      throw std::invalid_argument("Unknown option: " + args[i]);
  }
  return result;
}

struct SelectedPosition { double x, y, z; };

class RobotSelection
{
public:
  explicit RobotSelection(std::map<std::string, std::string> robots)
    : robots_(std::move(robots))
  {
    for (const auto &entry : robots_)
      entities_.insert(entry.second);
  }

  void select(const std::string &name)
  {
    const auto it = robots_.find(name);
    selected_ = it == robots_.end() ? "" : it->second;
  }

  void update(const gz::msgs::Pose_V &message)
  {
    for (const auto &pose : message.pose())
    {
      // Link/visual entries use local coordinates. Only mapped top-level model
      // names are accepted, matching SceneBroadcaster's world-coordinate poses.
      if (entities_.count(pose.name()) == 0)
        continue;
      const auto &p = pose.position();
      if (pose.has_position() && std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()))
        poses_[pose.name()] = {p.x(), p.y(), p.z()};
      else
        poses_.erase(pose.name());
    }
  }

  std::optional<SelectedPosition> position() const
  {
    const auto it = poses_.find(selected_);
    return it == poses_.end() ? std::nullopt : std::optional<SelectedPosition>(it->second);
  }

private:
  std::map<std::string, std::string> robots_;
  std::set<std::string> entities_;
  std::map<std::string, SelectedPosition> poses_;
  std::string selected_;
};

inline gz::msgs::Marker selectionMarker(const std::optional<SelectedPosition> &position)
{
  gz::msgs::Marker marker;
  marker.set_ns("cachecons_selected_robot");
  marker.set_id(1);
  marker.set_action(position ? gz::msgs::Marker::ADD_MODIFY : gz::msgs::Marker::DELETE_MARKER);
  if (!position)
    return marker;
  marker.set_type(gz::msgs::Marker::TRIANGLE_LIST);
  marker.mutable_pose()->mutable_position()->set_x(position->x);
  marker.mutable_pose()->mutable_position()->set_y(position->y);
  marker.mutable_pose()->mutable_position()->set_z(position->z + .035);
  marker.mutable_pose()->mutable_orientation()->set_w(1.0);
  auto *material = marker.mutable_material();
  material->set_lighting(false);
  for (auto *colour : {material->mutable_ambient(), material->mutable_diffuse(), material->mutable_emissive()})
  {
    colour->set_r(1.0); colour->set_g(.85); colour->set_b(0.0); colour->set_a(1.0);
  }
  // A selection ring around the robot.
  constexpr int segments = 48;
  constexpr double inner = .21, outer = .25, twoPi = 6.2831853071795864769;
  auto point = [&marker](double radius, double angle)
  {
    auto *p = marker.add_point();
    p->set_x(radius * std::cos(angle));
    p->set_y(radius * std::sin(angle));
    p->set_z(0.0);
  };
  for (int i = 0; i < segments; ++i)
  {
    const auto a = twoPi * i / segments, b = twoPi * (i + 1) / segments;
    point(inner, a); point(outer, a); point(outer, b);
    point(inner, a); point(outer, b); point(inner, b);
  }
  return marker;
}
