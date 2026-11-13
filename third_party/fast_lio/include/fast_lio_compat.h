#ifndef FAST_LIO_COMPAT_H
#define FAST_LIO_COMPAT_H

// g1d: replacements for the ROS1 types/macros used by FAST-LIO2, so the estimator code can be
// built without ROS. Only the members FAST-LIO actually touches are provided.

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#else
inline double omp_get_wtime()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline void omp_set_num_threads(int) {}
#endif

#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif

#define ROS_INFO(...) do {std::fprintf(stderr, "[fast_lio][INFO] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n");} while (0)
#define ROS_WARN(...) do {std::fprintf(stderr, "[fast_lio][WARN] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n");} while (0)
#define ROS_ERROR(...) do {std::fprintf(stderr, "[fast_lio][ERROR] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n");} while (0)
#define ROS_ASSERT(cond) do {if (!(cond)) {throw std::runtime_error("fast_lio assertion failed: " #cond);}} while (0)

namespace fast_lio
{
// Same fields as fast_lio/msg/Pose6D.msg.
struct Pose6D
{
  double offset_time{0.0};
  double acc[3]{0, 0, 0};
  double gyr[3]{0, 0, 0};
  double vel[3]{0, 0, 0};
  double pos[3]{0, 0, 0};
  double rot[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
};
}  // namespace fast_lio

namespace sensor_msgs
{
struct ImuStamp
{
  double sec{0.0};
  double toSec() const {return sec;}
};
struct ImuHeader
{
  ImuStamp stamp;
};
struct ImuVector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};
struct Imu
{
  ImuHeader header;
  ImuVector3 angular_velocity;     // rad/s, IMU frame
  ImuVector3 linear_acceleration;  // any unit; FAST-LIO rescales by the initial mean norm
  using Ptr = std::shared_ptr<Imu>;
  using ConstPtr = std::shared_ptr<const Imu>;
};
using ImuPtr = Imu::Ptr;
using ImuConstPtr = Imu::ConstPtr;
}  // namespace sensor_msgs

#endif  // FAST_LIO_COMPAT_H
