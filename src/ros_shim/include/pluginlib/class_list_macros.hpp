#pragma once

#include "pluginlib/class_loader.hpp"

#define G1D_PLUGINLIB_CAT_IMPL(a, b) a ## b
#define G1D_PLUGINLIB_CAT(a, b) G1D_PLUGINLIB_CAT_IMPL(a, b)

// Registers Derived under its fully-qualified name, e.g. "mppi::critics::CostCritic".
#define PLUGINLIB_EXPORT_CLASS(Derived, Base) \
  namespace { \
  static ::pluginlib::impl::Registrar<Derived, Base> \
  G1D_PLUGINLIB_CAT(g1d_pluginlib_registrar_, __LINE__)(#Derived); \
  }
