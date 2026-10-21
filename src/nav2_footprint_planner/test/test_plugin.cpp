#include <gtest/gtest.h>
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"
TEST(Plugin,LoadsThroughInstalledNav2RegistryWithoutStartingNodes)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  EXPECT_TRUE(loader.isClassAvailable("nav2_footprint_planner/StrictHybridPlanner"));
  EXPECT_NE(loader.createSharedInstance("nav2_footprint_planner/StrictHybridPlanner"),nullptr);
}
