#pragma once

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pluginlib
{

class PluginlibException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

template<typename T>
using UniquePtr = std::unique_ptr<T>;

namespace impl
{
template<typename Base>
std::map<std::string, std::function<Base *()>> & registry()
{
  static std::map<std::string, std::function<Base *()>> r;
  return r;
}

template<typename Derived, typename Base>
struct Registrar
{
  explicit Registrar(const char * name)
  {
    registry<Base>()[name] = []() -> Base * {return new Derived();};
  }
};
}  // namespace impl

/// Static-registry replacement for pluginlib::ClassLoader (no dynamic library loading).
template<typename Base>
class ClassLoader
{
public:
  ClassLoader(const std::string & /*package*/, const std::string & /*base_class*/) {}

  std::shared_ptr<Base> createSharedInstance(const std::string & name)
  {
    return std::shared_ptr<Base>(create(name));
  }
  UniquePtr<Base> createUniqueInstance(const std::string & name)
  {
    return UniquePtr<Base>(create(name));
  }
  Base * createUnmanagedInstance(const std::string & name) {return create(name);}

  bool isClassAvailable(const std::string & name) const
  {
    return impl::registry<Base>().count(name) > 0;
  }
  std::vector<std::string> getDeclaredClasses() const
  {
    std::vector<std::string> out;
    for (const auto & kv : impl::registry<Base>()) {out.push_back(kv.first);}
    return out;
  }

private:
  Base * create(const std::string & name)
  {
    auto & r = impl::registry<Base>();
    auto it = r.find(name);
    if (it == r.end()) {
      throw PluginlibException("plugin not registered: " + name);
    }
    return it->second();
  }
};

}  // namespace pluginlib
