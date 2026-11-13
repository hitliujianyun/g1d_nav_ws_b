#include "livox_sdk_source.hpp"

#include <cstdio>
#include <cstring>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace g1d::apps
{
namespace
{
uint64_t packetTimestamp(const LivoxLidarEthernetPacket * p)
{
  uint64_t t = 0;
  std::memcpy(&t, p->timestamp, sizeof(t));
  return t;
}

void pointCb(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket * data, void * self)
{
  LivoxSdkSource::onPoints(handle, dev_type, data, self);
}
void imuCb(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket * data, void * self)
{
  LivoxSdkSource::onImu(handle, dev_type, data, self);
}
void infoCb(const uint32_t handle, const LivoxLidarInfo * info, void * self)
{
  LivoxSdkSource::onInfoChange(handle, info, self);
}
void workModeCb(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * resp, void *)
{
  std::fprintf(stderr, "[livox] work mode set: status=%d handle=%u ret=%d\n", status, handle,
    resp ? resp->ret_code : -1);
}
}  // namespace

LivoxSdkSource::LivoxSdkSource(PacketFn on_packet, ImuFn on_imu)
: on_packet_(std::move(on_packet)), on_imu_(std::move(on_imu)) {}

LivoxSdkSource::~LivoxSdkSource() {stop();}

bool LivoxSdkSource::start(const std::string & sdk_config, const std::string & host_ip)
{
  if (!LivoxLidarSdkInit(sdk_config.c_str(), host_ip.c_str())) {
    std::fprintf(stderr, "[livox] LivoxLidarSdkInit failed (%s)\n", sdk_config.c_str());
    LivoxLidarSdkUninit();
    return false;
  }
  SetLivoxLidarPointCloudCallBack(pointCb, this);
  SetLivoxLidarImuDataCallback(imuCb, this);
  SetLivoxLidarInfoChangeCallback(infoCb, this);
  started_ = true;
  return true;
}

void LivoxSdkSource::stop()
{
  if (started_) {
    LivoxLidarSdkUninit();
    started_ = false;
  }
}

void LivoxSdkSource::onInfoChange(uint32_t handle, const void * info, void *)
{
  const auto * i = static_cast<const LivoxLidarInfo *>(info);
  std::fprintf(stderr, "[livox] lidar online: handle=%u sn=%s ip=%s type=%u\n", handle,
    i ? i->sn : "?", i ? i->lidar_ip : "?", i ? i->dev_type : 0);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, workModeCb, nullptr);
}

void LivoxSdkSource::onPoints(uint32_t, uint8_t, void * packet, void * self)
{
  auto * src = static_cast<LivoxSdkSource *>(self);
  const auto * p = static_cast<const LivoxLidarEthernetPacket *>(packet);
  if (!p || p->data_type != kLivoxLidarCartesianCoordinateHighData) {return;}
  g1d::lio::LivoxPacket pkt;
  pkt.timestamp_ns = packetTimestamp(p);
  pkt.time_interval = p->time_interval;
  pkt.points.resize(p->dot_num);
  const auto * pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(p->data);
  for (uint32_t i = 0; i < p->dot_num; ++i) {
    pkt.points[i].x_mm = pts[i].x;
    pkt.points[i].y_mm = pts[i].y;
    pkt.points[i].z_mm = pts[i].z;
    pkt.points[i].reflectivity = pts[i].reflectivity;
    pkt.points[i].tag = pts[i].tag;
  }
  src->on_packet_(pkt);
}

void LivoxSdkSource::onImu(uint32_t, uint8_t, void * packet, void * self)
{
  auto * src = static_cast<LivoxSdkSource *>(self);
  const auto * p = static_cast<const LivoxLidarEthernetPacket *>(packet);
  if (!p || p->data_type != kLivoxLidarImuData) {return;}
  const auto * imu = reinterpret_cast<const LivoxLidarImuRawPoint *>(p->data);
  const double t = static_cast<double>(packetTimestamp(p)) * 1e-9;
  const double gyr[3] = {imu->gyro_x, imu->gyro_y, imu->gyro_z};
  const double acc[3] = {imu->acc_x, imu->acc_y, imu->acc_z};  // g
  src->on_imu_(t, gyr, acc);
}

}  // namespace g1d::apps
