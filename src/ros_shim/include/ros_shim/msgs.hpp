#pragma once

// Plain-struct stand-ins for the ROS 2 messages referenced by the vendored Nav2 code.
// Field names follow the ROS message definitions.

// Real ROS message headers transitively provide these; vendored code relies on that.
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

namespace std_msgs
{
namespace msg
{
struct Header
{
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
  bool operator==(const Header & o) const {return stamp == o.stamp && frame_id == o.frame_id;}
};

struct ColorRGBA
{
  float r{0.0f};
  float g{0.0f};
  float b{0.0f};
  float a{0.0f};
};
}  // namespace msg
}  // namespace std_msgs

namespace geometry_msgs
{
namespace msg
{
struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool operator==(const Point & o) const {return x == o.x && y == o.y && z == o.z;}
  bool operator!=(const Point & o) const {return !(*this == o);}
};

struct Point32
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
  bool operator==(const Quaternion & o) const
  {
    return x == o.x && y == o.y && z == o.z && w == o.w;
  }
};

struct Pose
{
  Point position;
  Quaternion orientation;
  bool operator==(const Pose & o) const
  {
    return position == o.position && orientation == o.orientation;
  }
};

struct PoseStamped
{
  std_msgs::msg::Header header;
  Pose pose;
  bool operator==(const PoseStamped & o) const {return header == o.header && pose == o.pose;}
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct TwistStamped
{
  std_msgs::msg::Header header;
  Twist twist;
};

struct Accel
{
  Vector3 linear;
  Vector3 angular;
};

struct AccelStamped
{
  std_msgs::msg::Header header;
  Accel accel;
};

struct Polygon
{
  std::vector<Point32> points;
};

struct PolygonStamped
{
  std_msgs::msg::Header header;
  Polygon polygon;
};
}  // namespace msg
}  // namespace geometry_msgs

namespace nav_msgs
{
namespace msg
{
struct Path
{
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

struct MapMetaData
{
  builtin_interfaces::msg::Time map_load_time;
  float resolution{0.0f};
  uint32_t width{0};
  uint32_t height{0};
  geometry_msgs::msg::Pose origin;
};

struct OccupancyGrid
{
  std_msgs::msg::Header header;
  MapMetaData info;
  std::vector<int8_t> data;
};

struct TrajectoryPoint
{
  std_msgs::msg::Header header;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist velocity;
  geometry_msgs::msg::Accel acceleration;
};

struct Trajectory
{
  std_msgs::msg::Header header;
  std::vector<TrajectoryPoint> points;
};
}  // namespace msg
}  // namespace nav_msgs

namespace visualization_msgs
{
namespace msg
{
struct Marker
{
  static constexpr int32_t ARROW = 0;
  static constexpr int32_t CUBE = 1;
  static constexpr int32_t SPHERE = 2;
  static constexpr int32_t CYLINDER = 3;
  static constexpr int32_t LINE_STRIP = 4;
  static constexpr int32_t LINE_LIST = 5;
  static constexpr int32_t CUBE_LIST = 6;
  static constexpr int32_t SPHERE_LIST = 7;
  static constexpr int32_t POINTS = 8;
  static constexpr int32_t ADD = 0;
  static constexpr int32_t MODIFY = 0;
  static constexpr int32_t DELETE = 2;
  static constexpr int32_t DELETEALL = 3;

  std_msgs::msg::Header header;
  std::string ns;
  int32_t id{0};
  int32_t type{0};
  int32_t action{0};
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 scale;
  std_msgs::msg::ColorRGBA color;
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<std_msgs::msg::ColorRGBA> colors;
};

struct MarkerArray
{
  std::vector<Marker> markers;
};
}  // namespace msg
}  // namespace visualization_msgs

namespace nav2_msgs
{
namespace msg
{
struct CriticsStats
{
  builtin_interfaces::msg::Time stamp;
  std::vector<std::string> critics;
  std::vector<bool> changed;
  std::vector<float> costs_sum;
};

struct WaypointStatus
{
  static constexpr uint8_t PENDING = 0;
  static constexpr uint8_t COMPLETED = 1;
  static constexpr uint8_t SKIPPED = 2;
  static constexpr uint8_t FAILED = 3;
  uint8_t waypoint_status{PENDING};
  uint32_t waypoint_index{0};
  geometry_msgs::msg::PoseStamped waypoint_pose;
  int32_t error_code{0};
  std::string error_msg;
};
}  // namespace msg
}  // namespace nav2_msgs
