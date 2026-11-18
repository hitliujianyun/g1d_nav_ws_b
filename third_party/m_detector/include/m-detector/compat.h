// g1d port of M-detector: replacements for the ROS / TBB facilities used upstream.
#ifndef M_DETECTOR_COMPAT_H
#define M_DETECTOR_COMPAT_H

#include <chrono>
#include <functional>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#else
inline double omp_get_wtime()
{
    using clk = std::chrono::steady_clock;
    return std::chrono::duration<double>(clk::now().time_since_epoch()).count();
}
#endif

namespace m_detector
{

/// Stand-in for ros::NodeHandle::param(): `lookup(name, value)` returns true and sets `value`
/// when the key exists (names keep the upstream "dyn_obj/..." prefix).
class ParamSource
{
public:
    using Lookup = std::function<bool(const std::string &, double &)>;
    ParamSource() = default;
    explicit ParamSource(Lookup lookup) : lookup_(std::move(lookup)) {}

    template <typename T>
    void param(const std::string & name, T & value, T fallback) const
    {
        double v = 0.0;
        if (lookup_ && lookup_(name, v)) value = static_cast<T>(v);
        else value = fallback;
    }

private:
    Lookup lookup_;
};

/// String parameters (frame ids, debug file names) are not configurable in the port.
template <>
inline void ParamSource::param<std::string>(
    const std::string &, std::string & value, std::string fallback) const
{
    value = std::move(fallback);
}

template <typename Fn>
inline void parallel_for(int n, Fn && fn)
{
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
    for (int i = 0; i < n; ++i) fn(i);
}

}  // namespace m_detector

#endif
