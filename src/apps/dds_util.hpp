#pragma once

// Conversions between g1d types and unitree_sdk2 (CycloneDDS-CXX) message types.
// Only built on the robot (G1D_WITH_UNITREE=ON).

#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/PointField_.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/idl/ros2/Time_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "g1d/common/geometry.hpp"

namespace g1d::bus
{

namespace ros2_time = ::builtin_interfaces::msg::dds_;
using PointCloud2 = ::sensor_msgs::msg::dds_::PointCloud2_;
using PointField = ::sensor_msgs::msg::dds_::PointField_;
using Odometry = ::nav_msgs::msg::dds_::Odometry_;
using String = ::std_msgs::msg::dds_::String_;

inline double toSec(const ros2_time::Time_ & t) {return t.sec() + 1e-9 * t.nanosec();}

inline void setStamp(ros2_time::Time_ & t, double s)
{
  const double sec = std::floor(s);
  t.sec(static_cast<int32_t>(sec));
  t.nanosec(static_cast<uint32_t>(std::llround((s - sec) * 1e9)) % 1000000000u);
}

/// x, y, z float32 cloud (point_step 12), little endian.
inline PointCloud2 makeCloudXYZ(
  const std::vector<Eigen::Vector3f> & pts, const std::string & frame, double stamp)
{
  PointCloud2 m;
  setStamp(m.header().stamp(), stamp);
  m.header().frame_id(frame);
  m.height(1);
  m.width(static_cast<uint32_t>(pts.size()));
  const char * names[3] = {"x", "y", "z"};
  for (uint32_t i = 0; i < 3; ++i) {
    PointField f;
    f.name(names[i]);
    f.offset(4 * i);
    f.datatype(7);  // FLOAT32
    f.count(1);
    m.fields().push_back(f);
  }
  m.is_bigendian(false);
  m.point_step(12);
  m.row_step(12 * m.width());
  m.data().resize(12 * pts.size());
  if (!pts.empty()) {
    for (size_t i = 0; i < pts.size(); ++i) {
      std::memcpy(&m.data()[12 * i], pts[i].data(), 12);
    }
  }
  m.is_dense(true);
  return m;
}

/// Reads float32 x/y/z fields (any point_step / extra fields). Returns false on layout errors.
inline bool readCloudXYZ(const PointCloud2 & m, std::vector<Eigen::Vector3f> & out)
{
  int ox = -1, oy = -1, oz = -1;
  for (const auto & f : m.fields()) {
    if (f.datatype() != 7) {continue;}
    if (f.name() == "x") {ox = static_cast<int>(f.offset());}
    if (f.name() == "y") {oy = static_cast<int>(f.offset());}
    if (f.name() == "z") {oz = static_cast<int>(f.offset());}
  }
  if (ox < 0 || oy < 0 || oz < 0 || m.point_step() == 0) {return false;}
  const size_t n = m.data().size() / m.point_step();
  out.clear();
  out.reserve(n);
  const uint8_t * d = m.data().data();
  for (size_t i = 0; i < n; ++i) {
    float x, y, z;
    std::memcpy(&x, d + i * m.point_step() + ox, 4);
    std::memcpy(&y, d + i * m.point_step() + oy, 4);
    std::memcpy(&z, d + i * m.point_step() + oz, 4);
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {out.emplace_back(x, y, z);}
  }
  return true;
}

inline Eigen::Isometry3d poseOf(const Odometry & m)
{
  const auto & p = m.pose().pose();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position().x(), p.position().y(), p.position().z());
  T.linear() = Eigen::Quaterniond(
    p.orientation().w(), p.orientation().x(), p.orientation().y(), p.orientation().z())
    .normalized().toRotationMatrix();
  return T;
}

inline Odometry makeOdometry(
  const Eigen::Isometry3d & T, const Eigen::Vector3d & lin_vel, const Eigen::Vector3d & ang_vel,
  const std::string & frame, const std::string & child, double stamp)
{
  Odometry m;
  setStamp(m.header().stamp(), stamp);
  m.header().frame_id(frame);
  m.child_frame_id(child);
  const Eigen::Quaterniond q(T.linear());
  auto & p = m.pose().pose();
  p.position().x(T.translation().x());
  p.position().y(T.translation().y());
  p.position().z(T.translation().z());
  p.orientation().x(q.x());
  p.orientation().y(q.y());
  p.orientation().z(q.z());
  p.orientation().w(q.w());
  m.twist().twist().linear().x(lin_vel.x());
  m.twist().twist().linear().y(lin_vel.y());
  m.twist().twist().linear().z(lin_vel.z());
  m.twist().twist().angular().x(ang_vel.x());
  m.twist().twist().angular().y(ang_vel.y());
  m.twist().twist().angular().z(ang_vel.z());
  return m;
}

inline String makeString(const std::string & s)
{
  String m;
  m.data(s);
  return m;
}

/// Latest-value mailbox filled from DDS callback threads and drained by the main loop.
template<typename T>
class Mailbox
{
public:
  void put(T v)
  {
    std::lock_guard<std::mutex> lk(m_);
    q_.push_back(std::move(v));
    while (q_.size() > max_) {q_.pop_front();}
  }
  std::vector<T> drain()
  {
    std::lock_guard<std::mutex> lk(m_);
    std::vector<T> out(std::make_move_iterator(q_.begin()), std::make_move_iterator(q_.end()));
    q_.clear();
    return out;
  }
  explicit Mailbox(size_t max = 16)
  : max_(max) {}

private:
  std::mutex m_;
  std::deque<T> q_;
  size_t max_;
};

template<typename MSG>
using Subscriber = unitree::robot::ChannelSubscriber<MSG>;
template<typename MSG>
using Publisher = unitree::robot::ChannelPublisher<MSG>;

/// Creates and initialises a subscriber whose handler receives the typed message.
template<typename MSG>
std::shared_ptr<Subscriber<MSG>> subscribe(
  const std::string & topic, std::function<void(const MSG &)> handler, int64_t queue = 1)
{
  auto sub = std::make_shared<Subscriber<MSG>>(topic);
  sub->InitChannel([handler](const void * msg) {handler(*static_cast<const MSG *>(msg));}, queue);
  return sub;
}

template<typename MSG>
std::shared_ptr<Publisher<MSG>> advertise(const std::string & topic)
{
  auto pub = std::make_shared<Publisher<MSG>>(topic);
  pub->InitChannel();
  return pub;
}

}  // namespace g1d::bus
