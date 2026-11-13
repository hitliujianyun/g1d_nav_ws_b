#pragma once

#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/logger.hpp"

namespace rclcpp
{
namespace detail
{

enum class LogLevel : int { Debug = 10, Info = 20, Warn = 30, Error = 40, Fatal = 50 };

using LogHandler = std::function<void(LogLevel, const std::string &, const std::string &)>;

inline LogLevel parse_level(const char * s)
{
  if (!s) {return LogLevel::Info;}
  if (!std::strcmp(s, "DEBUG") || !std::strcmp(s, "debug")) {return LogLevel::Debug;}
  if (!std::strcmp(s, "WARN") || !std::strcmp(s, "warn")) {return LogLevel::Warn;}
  if (!std::strcmp(s, "ERROR") || !std::strcmp(s, "error")) {return LogLevel::Error;}
  if (!std::strcmp(s, "FATAL") || !std::strcmp(s, "fatal")) {return LogLevel::Fatal;}
  return LogLevel::Info;
}

inline LogLevel & min_level()
{
  static LogLevel level = parse_level(std::getenv("G1D_LOG_LEVEL"));
  return level;
}

inline void default_handler(LogLevel level, const std::string & name, const std::string & msg)
{
  static std::mutex m;
  const char * tag = "INFO";
  switch (level) {
    case LogLevel::Debug: tag = "DEBUG"; break;
    case LogLevel::Warn: tag = "WARN"; break;
    case LogLevel::Error: tag = "ERROR"; break;
    case LogLevel::Fatal: tag = "FATAL"; break;
    default: break;
  }
  std::lock_guard<std::mutex> lk(m);
  std::fprintf(stderr, "[%s] [%s] %s\n", tag, name.c_str(), msg.c_str());
}

inline LogHandler & log_handler()
{
  static LogHandler handler = default_handler;
  return handler;
}

inline bool enabled(LogLevel level) {return static_cast<int>(level) >= static_cast<int>(min_level());}

inline std::string vformat(const char * fmt, va_list ap)
{
  char buf[1024];
  va_list ap2;
  va_copy(ap2, ap);
  int n = std::vsnprintf(buf, sizeof(buf), fmt, ap2);
  va_end(ap2);
  if (n < 0) {return std::string();}
  if (static_cast<size_t>(n) < sizeof(buf)) {return std::string(buf, static_cast<size_t>(n));}
  std::string out(static_cast<size_t>(n) + 1, '\0');
  std::vsnprintf(&out[0], out.size(), fmt, ap);
  out.resize(static_cast<size_t>(n));
  return out;
}

inline void logf(LogLevel level, const Logger & logger, const char * fmt, ...)
{
  if (!enabled(level)) {return;}
  va_list ap;
  va_start(ap, fmt);
  std::string msg = vformat(fmt, ap);
  va_end(ap);
  while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')) {msg.pop_back();}
  log_handler()(level, logger.get_name(), msg);
}

inline void logs(LogLevel level, const Logger & logger, const std::string & msg)
{
  if (!enabled(level)) {return;}
  log_handler()(level, logger.get_name(), msg);
}

}  // namespace detail
}  // namespace rclcpp

#define G1D_RCLCPP_LOG(level, logger, ...) \
  ::rclcpp::detail::logf(::rclcpp::detail::LogLevel::level, (logger), __VA_ARGS__)

#define G1D_RCLCPP_LOG_ONCE(level, logger, ...) \
  do { \
    static bool g1d_once_flag_ = false; \
    if (!g1d_once_flag_) {g1d_once_flag_ = true; G1D_RCLCPP_LOG(level, logger, __VA_ARGS__);} \
  } while (0)

#define G1D_RCLCPP_LOG_STREAM(level, logger, stream_arg) \
  do { \
    if (::rclcpp::detail::enabled(::rclcpp::detail::LogLevel::level)) { \
      std::stringstream g1d_ss_; g1d_ss_ << stream_arg; \
      ::rclcpp::detail::logs(::rclcpp::detail::LogLevel::level, (logger), g1d_ss_.str()); \
    } \
  } while (0)

#define RCLCPP_DEBUG(logger, ...) G1D_RCLCPP_LOG(Debug, logger, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) G1D_RCLCPP_LOG(Info, logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) G1D_RCLCPP_LOG(Warn, logger, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) G1D_RCLCPP_LOG(Error, logger, __VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) G1D_RCLCPP_LOG(Fatal, logger, __VA_ARGS__)

#define RCLCPP_DEBUG_ONCE(logger, ...) G1D_RCLCPP_LOG_ONCE(Debug, logger, __VA_ARGS__)
#define RCLCPP_INFO_ONCE(logger, ...) G1D_RCLCPP_LOG_ONCE(Info, logger, __VA_ARGS__)
#define RCLCPP_WARN_ONCE(logger, ...) G1D_RCLCPP_LOG_ONCE(Warn, logger, __VA_ARGS__)
#define RCLCPP_ERROR_ONCE(logger, ...) G1D_RCLCPP_LOG_ONCE(Error, logger, __VA_ARGS__)

#define RCLCPP_DEBUG_STREAM(logger, s) G1D_RCLCPP_LOG_STREAM(Debug, logger, s)
#define RCLCPP_INFO_STREAM(logger, s) G1D_RCLCPP_LOG_STREAM(Info, logger, s)
#define RCLCPP_WARN_STREAM(logger, s) G1D_RCLCPP_LOG_STREAM(Warn, logger, s)
#define RCLCPP_ERROR_STREAM(logger, s) G1D_RCLCPP_LOG_STREAM(Error, logger, s)

#define RCLCPP_DEBUG_THROTTLE(logger, clock, ms, ...) G1D_RCLCPP_LOG(Debug, logger, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) G1D_RCLCPP_LOG(Info, logger, __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) G1D_RCLCPP_LOG(Warn, logger, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) G1D_RCLCPP_LOG(Error, logger, __VA_ARGS__)
