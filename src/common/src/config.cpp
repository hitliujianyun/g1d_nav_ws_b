#include "g1d/common/config.hpp"

#include <sstream>

#include "nav2_ros_common/lifecycle_node.hpp"

namespace g1d
{

YAML::Node loadYamlFile(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("failed to load config '" + path + "': " + e.what());
  }
}

YAML::Node findPath(const YAML::Node & root, const std::string & dotted)
{
  YAML::Node cur = YAML::Clone(root);
  std::stringstream ss(dotted);
  std::string key;
  while (std::getline(ss, key, '.')) {
    if (!cur || !cur.IsMap()) {return YAML::Node(YAML::NodeType::Undefined);}
    YAML::Node next = cur[key];
    if (!next) {return YAML::Node(YAML::NodeType::Undefined);}
    cur = next;
  }
  return cur;
}

namespace
{

rclcpp::ParameterValue scalarValue(const YAML::Node & n)
{
  const std::string s = n.Scalar();
  if (n.Tag() == "!" ) {return rclcpp::ParameterValue(s);}  // quoted string
  if (s == "true" || s == "True" || s == "TRUE") {return rclcpp::ParameterValue(true);}
  if (s == "false" || s == "False" || s == "FALSE") {return rclcpp::ParameterValue(false);}
  try {
    size_t idx = 0;
    const long long v = std::stoll(s, &idx);
    if (idx == s.size()) {return rclcpp::ParameterValue(static_cast<int64_t>(v));}
  } catch (...) {
  }
  try {
    size_t idx = 0;
    const double v = std::stod(s, &idx);
    if (idx == s.size()) {return rclcpp::ParameterValue(v);}
  } catch (...) {
  }
  return rclcpp::ParameterValue(s);
}

rclcpp::ParameterValue sequenceValue(const YAML::Node & n)
{
  if (n.size() == 0) {return rclcpp::ParameterValue(std::vector<std::string>{});}
  bool all_int = true, all_num = true, all_bool = true;
  for (const auto & e : n) {
    const auto v = scalarValue(e);
    all_int &= v.get_type() == rclcpp::PARAMETER_INTEGER;
    all_num &= v.get_type() == rclcpp::PARAMETER_INTEGER || v.get_type() == rclcpp::PARAMETER_DOUBLE;
    all_bool &= v.get_type() == rclcpp::PARAMETER_BOOL;
  }
  if (all_bool) {
    std::vector<bool> out;
    for (const auto & e : n) {out.push_back(scalarValue(e).as_bool());}
    return rclcpp::ParameterValue(out);
  }
  if (all_int) {
    std::vector<int64_t> out;
    for (const auto & e : n) {out.push_back(scalarValue(e).as_int());}
    return rclcpp::ParameterValue(out);
  }
  if (all_num) {
    std::vector<double> out;
    for (const auto & e : n) {out.push_back(scalarValue(e).as_double());}
    return rclcpp::ParameterValue(out);
  }
  std::vector<std::string> out;
  for (const auto & e : n) {out.push_back(e.Scalar());}
  return rclcpp::ParameterValue(out);
}

}  // namespace

void loadParameters(
  nav2::LifecycleNode & node, const YAML::Node & subtree, const std::string & prefix)
{
  if (!subtree) {return;}
  if (subtree.IsMap()) {
    for (const auto & kv : subtree) {
      const std::string key = kv.first.as<std::string>();
      loadParameters(node, kv.second, prefix.empty() ? key : prefix + "." + key);
    }
  } else if (subtree.IsSequence()) {
    node.set_parameter(prefix, sequenceValue(subtree));
  } else if (subtree.IsScalar()) {
    node.set_parameter(prefix, scalarValue(subtree));
  }
}

}  // namespace g1d
