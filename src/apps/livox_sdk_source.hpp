#pragma once

#include <functional>
#include <string>

#include "g1d/lio/livox_frame.hpp"

namespace g1d::apps
{

/// Direct Mid-360S connection through Livox-SDK2 (>= 1.3.1). Callbacks run on SDK threads.
class LivoxSdkSource
{
public:
  using PacketFn = std::function<void(const g1d::lio::LivoxPacket &)>;
  using ImuFn = std::function<void(double t_sensor, const double gyr[3], const double acc[3])>;

  LivoxSdkSource(PacketFn on_packet, ImuFn on_imu);
  ~LivoxSdkSource();
  /// `sdk_config`: Livox JSON (e.g. config/mid360s_config.json); `host_ip`: PC2 address.
  bool start(const std::string & sdk_config, const std::string & host_ip);
  void stop();

  // SDK trampolines
  static void onPoints(uint32_t handle, uint8_t dev_type, void * packet, void * self);
  static void onImu(uint32_t handle, uint8_t dev_type, void * packet, void * self);
  static void onInfoChange(uint32_t handle, const void * info, void * self);

private:
  PacketFn on_packet_;
  ImuFn on_imu_;
  bool started_{false};
};

}  // namespace g1d::apps
