#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace nav2
{
class LifecycleNode;
}

namespace g1d
{

/// Loads a YAML file; throws std::runtime_error with the path on failure.
YAML::Node loadYamlFile(const std::string & path);

/// Returns the node at a dotted path ("costmap.local.size"), or an undefined node.
YAML::Node findPath(const YAML::Node & root, const std::string & dotted);

template<typename T>
T get(const YAML::Node & root, const std::string & dotted, const T & fallback)
{
  const YAML::Node n = findPath(root, dotted);
  if (!n || n.IsNull()) {return fallback;}
  try {
    return n.as<T>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("config '" + dotted + "': " + e.what());
  }
}

/// Copies every scalar / sequence under `subtree` into the node's parameter table as
/// `<prefix>.<key>` (nested maps are flattened with '.'), the way ROS parameter files are.
void loadParameters(
  nav2::LifecycleNode & node, const YAML::Node & subtree, const std::string & prefix);

}  // namespace g1d
