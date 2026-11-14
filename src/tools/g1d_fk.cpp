// Prints the configured body chain (base_link -> head -> Mid-360S -> IMU) for given joint values,
// to compare with tools/calib_floor.py measurements and the URDF.
//
//   g1d_fk [config/g1d_nav.yaml] [lift_y=0] [waist_yaw=0] [waist_pitch=0]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "g1d/common/config.hpp"
#include "g1d/nav/body_kinematics.hpp"

using namespace g1d;

namespace
{
void print(const char * name, const Eigen::Isometry3d & T)
{
  const Eigen::Vector3d t = T.translation();
  const Eigen::Matrix3d R = T.linear();
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const Eigen::Vector3d up = R.transpose() * Eigen::Vector3d::UnitZ();
  std::printf("%-14s xyz [%.4f, %.4f, %.4f]  rpy [%.4f, %.4f, %.4f]  floor-up in frame [%.3f, %.3f, %.3f]\n",
    name, t.x(), t.y(), t.z(), roll, pitch, yaw, up.x(), up.y(), up.z());
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "config/g1d_nav.yaml";
  const nav::KinematicsParams kp = nav::KinematicsParams::fromYaml(loadYamlFile(config_path));
  nav::JointState q;
  q.lift_height = argc > 2 ? std::atof(argv[2]) : 0.0;
  q.waist_yaw = argc > 3 ? std::atof(argv[3]) : 0.0;
  q.waist_pitch = argc > 4 ? std::atof(argv[4]) : 0.0;
  q.has_lift = q.has_lowstate = q.has_waist_pitch = true;
  const nav::BodyKinematics kin(kp);
  std::printf("joints: lift_y=%.3f waist_yaw=%.3f waist_pitch=%.3f\n", q.lift_height, q.waist_yaw,
    q.waist_pitch);
  print("base->head", kin.baseToHead(q));
  print("base->livox", kin.baseToLivox(q));
  print("base->imu", kin.baseToImu(q));
  return 0;
}
