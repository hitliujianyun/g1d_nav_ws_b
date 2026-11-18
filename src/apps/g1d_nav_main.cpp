// g1d_nav: state estimation + costmaps + NavFn + MPPI + goal state machine (unitree_sdk2 DDS).
//
// Subscribes: rt/nav/lio_odom, rt/nav/lio_health, rt/nav/registered_cloud, rt/nav/chassis_scan,
//             rt/slamware_ros_sdk_server_node/odom, rt/hispeed_state, rt/lowstate,
//             rt/waistpitchstate, rt/nav/goal, rt/nav/estop, rt/nav/guard_state
//             rt/nav/param_set (JSON {"target": "nav", "name", "value", "id"})
//             rt/nav/person_points (PointCloud2 xyz in odom_lio: late moving-object evidence)
// registered_cloud points with dynamic = 1 mark only the short-lived dynamic costmap layer.
// Publishes:  rt/nav/cmd_vel, rt/nav/status, rt/nav/base_odom, rt/nav/obstacle_points,
//             rt/nav/param_state, debug: rt/nav/local_costmap, rt/nav/global_costmap,
//             rt/nav/debug (JSON: T_odom_lio, goal, paths, MPPI trajectory)
// The chassis LiDAR arrives as rt/nav/chassis_scan (PointCloud2 in base_link, republished by
// g1d_base_guard) so that no LaserScan type support is needed in C++.

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <limits>
#include <map>
#include <thread>

#include <nlohmann/json.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/MotorState_.hpp>
#include <unitree/idl/ros2/OccupancyGrid_.hpp>
#include <unitree/idl/ros2/Point32_.hpp>
#include <unitree/idl/ros2/Twist_.hpp>

#include "dds_util.hpp"
#include "g1d/common/config.hpp"
#include "g1d/common/time.hpp"
#include "g1d/nav/nav_core.hpp"

using namespace g1d;
using namespace g1d::nav;

namespace
{
std::atomic<bool> g_run{true};
void onSignal(int) {g_run = false;}

struct LioOdomMsg
{
  double stamp;
  Eigen::Isometry3d T;
  Eigen::Vector3d v;
};
struct CloudMsg
{
  double stamp;
  std::vector<Eigen::Vector3f> pts;
  std::vector<Eigen::Vector3f> dynamic;
};
struct WheelMsg
{
  double stamp;
  Pose2D pose;
  double v, w;
};
}  // namespace

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "config/g1d_nav.yaml";
  const YAML::Node cfg = loadYamlFile(config_path);
  const std::string net_if = get(cfg, "system.net_if", std::string("eth0"));
  const int domain = get(cfg, "system.dds_domain", 0);
  const bool publish_costmap = get(cfg, "debug.publish_costmap", true);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  unitree::robot::ChannelFactory::Instance()->Init(domain, net_if);
  const NavConfig nav_cfg = NavConfig::fromYaml(cfg);
  NavCore core(nav_cfg);
  const BodyKinematics kin(nav_cfg.kinematics);
  const Eigen::Isometry3d T_imu_lidar = nav_cfg.kinematics.T_livox_imu.inverse();

  bus::Mailbox<LioOdomMsg> lio_box(32);
  bus::Mailbox<CloudMsg> cloud_box(4);
  bus::Mailbox<CloudMsg> scan_box(4);
  bus::Mailbox<WheelMsg> wheel_box(64);
  bus::Mailbox<std::string> goal_box(8);
  bus::Mailbox<std::string> estop_box(8);
  bus::Mailbox<std::string> guard_box(4);
  std::mutex joint_mutex;
  JointState joints;
  std::atomic<bool> lio_health_ok{false};
  std::atomic<double> lio_health_stamp{0.0};

  using bus::subscribe;
  std::vector<std::shared_ptr<void>> subs;
  subs.push_back(subscribe<bus::Odometry>("rt/nav/lio_odom", [&](const bus::Odometry & m) {
      const auto & t = m.twist().twist().linear();
      lio_box.put({bus::toSec(m.header().stamp()), bus::poseOf(m), Eigen::Vector3d(t.x(), t.y(), t.z())});
    }, 32));
  subs.push_back(subscribe<bus::String>("rt/nav/lio_health", [&](const bus::String & m) {
      try {
        const auto j = nlohmann::json::parse(m.data());
        lio_health_ok = j.value("ok", false);
        lio_health_stamp = j.value("stamp", 0.0);
      } catch (...) {
        lio_health_ok = false;
      }
    }));
  subs.push_back(subscribe<bus::PointCloud2>("rt/nav/registered_cloud", [&](const bus::PointCloud2 & m) {
      CloudMsg c;
      c.stamp = bus::toSec(m.header().stamp());
      if (bus::readCloudXYZL(m, c.pts, c.dynamic)) {cloud_box.put(std::move(c));}
    }));
  bus::Mailbox<CloudMsg> person_box(8);
  subs.push_back(subscribe<bus::PointCloud2>("rt/nav/person_points", [&](const bus::PointCloud2 & m) {
      CloudMsg c;
      c.stamp = bus::toSec(m.header().stamp());
      if (bus::readCloudXYZ(m, c.pts)) {person_box.put(std::move(c));}
    }, 8));
  subs.push_back(subscribe<bus::PointCloud2>("rt/nav/chassis_scan", [&](const bus::PointCloud2 & m) {
      CloudMsg c;
      c.stamp = nowSec();  // republished with receipt time by the guard
      if (bus::readCloudXYZ(m, c.pts)) {scan_box.put(std::move(c));}
    }));
  subs.push_back(subscribe<bus::Odometry>("rt/slamware_ros_sdk_server_node/odom", [&](const bus::Odometry & m) {
      // SLAMWARE stamps run ~181 s behind the host clock: use receipt time.
      const auto T = bus::poseOf(m);
      wheel_box.put({nowSec(), toPose2D(T), m.twist().twist().linear().x(), m.twist().twist().angular().z()});
    }, 8));
  subs.push_back(subscribe<::geometry_msgs::msg::dds_::Point32_>("rt/hispeed_state",
    [&](const ::geometry_msgs::msg::dds_::Point32_ & m) {
      std::lock_guard<std::mutex> lk(joint_mutex);
      joints.lift_height = m.y();
      joints.lift_speed = m.z();
      joints.has_lift = true;
      joints.stamp = nowSec();
    }));
  subs.push_back(subscribe<::unitree_hg::msg::dds_::LowState_>("rt/lowstate",
    [&](const ::unitree_hg::msg::dds_::LowState_ & m) {
      std::lock_guard<std::mutex> lk(joint_mutex);
      joints.waist_yaw = m.motor_state()[12].q();
      for (size_t i = 0; i < joints.arms.size(); ++i) {joints.arms[i] = m.motor_state()[15 + i].q();}
      joints.has_lowstate = true;
    }));
  subs.push_back(subscribe<::unitree_hg::msg::dds_::MotorState_>("rt/waistpitchstate",
    [&](const ::unitree_hg::msg::dds_::MotorState_ & m) {
      std::lock_guard<std::mutex> lk(joint_mutex);
      joints.waist_pitch = m.q();
      joints.has_waist_pitch = true;
    }));
  subs.push_back(subscribe<bus::String>("rt/nav/goal", [&](const bus::String & m) {goal_box.put(m.data());}, 8));
  subs.push_back(subscribe<bus::String>("rt/nav/estop", [&](const bus::String & m) {estop_box.put(m.data());}, 8));
  subs.push_back(subscribe<bus::String>("rt/nav/guard_state", [&](const bus::String & m) {guard_box.put(m.data());}));
  bus::Mailbox<std::string> param_box(16);
  subs.push_back(subscribe<bus::String>("rt/nav/param_set", [&](const bus::String & m) {param_box.put(m.data());}, 16));

  auto cmd_pub = bus::advertise<::geometry_msgs::msg::dds_::Twist_>("rt/nav/cmd_vel");
  auto status_pub = bus::advertise<bus::String>("rt/nav/status");
  auto odom_pub = bus::advertise<bus::Odometry>("rt/nav/base_odom");
  auto obst_pub = bus::advertise<bus::PointCloud2>("rt/nav/obstacle_points");
  auto costmap_pub = bus::advertise<::nav_msgs::msg::dds_::OccupancyGrid_>("rt/nav/local_costmap");
  auto global_costmap_pub = bus::advertise<::nav_msgs::msg::dds_::OccupancyGrid_>("rt/nav/global_costmap");
  auto debug_pub = bus::advertise<bus::String>("rt/nav/debug");
  auto param_pub = bus::advertise<bus::String>("rt/nav/param_state");
  auto publishGrid = [](auto & pub, const costmap::LayeredCostmap & cm, double stamp) {
      const auto g = cm.toOccupancyGrid();
      ::nav_msgs::msg::dds_::OccupancyGrid_ m;
      bus::setStamp(m.header().stamp(), stamp);
      m.header().frame_id("odom");
      m.info().resolution(g.info.resolution);
      m.info().width(g.info.width);
      m.info().height(g.info.height);
      m.info().origin().position().x(g.info.origin.position.x);
      m.info().origin().position().y(g.info.origin.position.y);
      m.info().origin().orientation().w(1.0);
      m.data().assign(g.data.begin(), g.data.end());
      pub->Write(m);
    };
  auto publishParams = [&](const nlohmann::json & ack) {
      nlohmann::json j;
      j["target"] = "nav";
      j["params"] = core.paramsJson();
      if (!ack.is_null()) {j["ack"] = ack;}
      param_pub->Write(bus::makeString(j.dump()));
    };

  std::deque<LioOdomMsg> lio_hist;
  Eigen::Vector3d last_lidar_origin_lio = Eigen::Vector3d::Zero();
  double last_obst_stamp = 0.0;
  double last_costmap_pub = 0.0;
  double last_global_costmap_pub = 0.0;
  double last_debug_pub = 0.0;
  double last_param_pub = 0.0;
  Rate rate(nav_cfg.controller.controller_frequency);
  std::fprintf(stderr, "[g1d_nav] running (domain %d, if %s)\n", domain, net_if.c_str());

  while (g_run) {
    const double now = nowSec();
    {
      std::lock_guard<std::mutex> lk(joint_mutex);
      core.onJointState(joints);
    }
    for (auto & w : wheel_box.drain()) {core.onWheelOdom(w.stamp, w.pose, w.v, w.w);}
    for (auto & o : lio_box.drain()) {
      lio_hist.push_back(o);
      while (lio_hist.size() > 30) {lio_hist.pop_front();}
    }
    // Pair registered clouds with the LIO pose of the same scan.
    for (auto & c : cloud_box.drain()) {
      for (auto it = lio_hist.rbegin(); it != lio_hist.rend(); ++it) {
        if (std::abs(it->stamp - c.stamp) < 1e-3) {
          LioInput in;
          in.stamp = it->stamp;
          in.T_lio_imu = it->T;
          in.velocity_lio = it->v;
          in.lidar_origin_lio = (it->T * T_imu_lidar).translation();
          in.ok = lio_health_ok && std::abs(lio_health_stamp - it->stamp) < 0.5;
          in.cloud_lio = std::move(c.pts);
          in.dynamic_lio = std::move(c.dynamic);
          last_lidar_origin_lio = in.lidar_origin_lio;
          core.onLio(in);
          break;
        }
      }
    }
    for (auto & c : person_box.drain()) {
      DynamicPointsInput in;
      in.stamp = c.stamp;
      in.origin_lio = last_lidar_origin_lio;
      in.points_lio = std::move(c.pts);
      core.onDynamicPoints(in, now);
    }
    for (auto & s : scan_box.drain()) {
      // Points are in base_link; convert to a pseudo scan around the scanner origin.
      ScanData sd;
      sd.stamp = s.stamp;
      const Pose2D sp = toPose2D(nav_cfg.T_base_scanner);
      const int beams = 720;
      sd.angle_min = static_cast<float>(-M_PI);
      sd.angle_increment = static_cast<float>(2 * M_PI / beams);
      sd.range_min = 0.05f;
      sd.range_max = 30.0f;
      sd.ranges.assign(beams, std::numeric_limits<float>::infinity());
      for (const auto & p : s.pts) {
        const Eigen::Vector2d q = sp.inverse().transformPoint(Eigen::Vector2d(p.x(), p.y()));
        const double a = std::atan2(q.y(), q.x());
        int k = static_cast<int>(std::floor((a + M_PI) / sd.angle_increment));
        k = std::clamp(k, 0, beams - 1);
        sd.ranges[k] = std::min(sd.ranges[k], static_cast<float>(q.norm()));
      }
      core.onScan(sd);
    }
    for (auto & g : goal_box.drain()) {core.onGoalJson(g, now);}
    for (auto & e : estop_box.drain()) {core.onEstop(e == "on", now);}
    for (auto & gs : guard_box.drain()) {
      try {
        const auto j = nlohmann::json::parse(gs);
        core.onGuardState(!j.value("blocked", std::string()).empty(), j.value("estop", false),
          j.value("manual", false));
      } catch (...) {
      }
    }

    const TickOutput out = core.tick(now);
    if (out.publish_cmd) {
      ::geometry_msgs::msg::dds_::Twist_ tw;
      tw.linear().x(out.cmd.v);
      tw.angular().z(out.cmd.w);
      cmd_pub->Write(tw);
    }
    if (out.status_json) {status_pub->Write(bus::makeString(*out.status_json));}
    if (out.state.valid) {
      odom_pub->Write(bus::makeOdometry(toIsometry(out.state.pose),
        Eigen::Vector3d(out.state.velocity.v, 0, 0), Eigen::Vector3d(0, 0, out.state.velocity.w),
        "odom", "base_link", now));
    }
    if (core.lastObstaclePointsStamp() > last_obst_stamp) {
      last_obst_stamp = core.lastObstaclePointsStamp();
      obst_pub->Write(bus::makeCloudXYZ(core.obstaclePointsBase(), "base_link", last_obst_stamp));
    }
    if (publish_costmap && now - last_costmap_pub > 0.5) {
      last_costmap_pub = now;
      publishGrid(costmap_pub, core.localCostmap(), now);
    }
    if (publish_costmap && now - last_global_costmap_pub > 1.0) {
      last_global_costmap_pub = now;
      publishGrid(global_costmap_pub, core.globalCostmap(), now);
    }
    if (now - last_debug_pub > 0.2) {
      last_debug_pub = now;
      debug_pub->Write(bus::makeString(core.debugJson(now).dump()));
    }
    for (auto & ps : param_box.drain()) {
      nlohmann::json ack;
      try {
        const auto j = nlohmann::json::parse(ps);
        if (j.value("target", std::string()) != "nav") {continue;}
        std::string err;
        const bool ok = core.setParam(j.at("name").get<std::string>(), j.at("value").get<double>(), &err);
        ack = {{"id", j.value("id", std::string())}, {"name", j.value("name", std::string())},
          {"ok", ok}, {"error", err}};
      } catch (const std::exception & e) {
        ack = {{"ok", false}, {"error", e.what()}};
      }
      publishParams(ack);
      last_param_pub = now;
    }
    if (now - last_param_pub > 1.0) {
      last_param_pub = now;
      publishParams(nullptr);
    }
    rate.sleep();
  }
  // Leave the base stopped: one last zero command (the guard also times out on its own).
  ::geometry_msgs::msg::dds_::Twist_ zero;
  cmd_pub->Write(zero);
  return 0;
}
