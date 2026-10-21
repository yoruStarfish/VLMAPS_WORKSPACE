#include "nav2_footprint_planner/core.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
using namespace nav2_footprint_planner;
namespace
{
Grid grid(unsigned n=100,double r=.05)
{Grid g;g.width=g.height=n;g.resolution=r;g.name="global";g.costs.resize(n*n);return g;}
std::vector<Point> square(double r=.27)
{return {{r,r},{-r,r},{-r,-r},{r,-r}};}
void block(Grid & g,unsigned x,unsigned y,uint8_t c=254) {g.costs[y*g.width+x]=c;}
}
TEST(Collision,ChecksInteriorNotOnlyBoundary)
{
  auto g=grid();block(g,40,40);CollisionChecker c({g},square(),false);
  Failure f;EXPECT_FALSE(c.poseValid({2,2,0},&f));EXPECT_EQ(f.grid,"global");
  EXPECT_EQ(f.cell_x,40);EXPECT_EQ(f.cell_y,40);
}
TEST(Collision,SoftInscribedIsNotAnObstacleByItself)
{
  auto g=grid();block(g,40,40,253);CollisionChecker c({g},square(),false);
  EXPECT_TRUE(c.poseValid({2,2,0}));EXPECT_DOUBLE_EQ(c.softCost({2.01,2.01,0}),1);
}
TEST(Collision,UnknownAndBounds)
{
  auto g=grid();block(g,40,40,255);
  EXPECT_FALSE(CollisionChecker({g},square(),false).poseValid({2,2,0}));
  EXPECT_TRUE(CollisionChecker({g},square(),true).poseValid({2,2,0}));
  EXPECT_FALSE(CollisionChecker({g},square(),true).poseValid({.1,2,0}));
}
TEST(Collision,DetectsObstacleBetweenValidEndpoints)
{
  auto g=grid();block(g,40,40);CollisionChecker c({g},square(.1),false);
  EXPECT_TRUE(c.poseValid({1,2,0}));EXPECT_TRUE(c.poseValid({3,2,0}));
  EXPECT_FALSE(c.pathValid({{1,2,0},{3,2,0}}));
}
TEST(Collision,DetectsCornerDuringRotation)
{
  auto g=grid(200,.01);block(g,135,100);CollisionChecker c({g},square(),false);
  EXPECT_TRUE(c.poseValid({1,1,0}));EXPECT_TRUE(c.poseValid({1,1,1.57079632679}));
  EXPECT_FALSE(c.pathValid({{1,1,0},{1,1,1.57079632679}}));
}
TEST(Collision,LocalTransformAndPartialWindow)
{
  auto g=grid(),l=grid(20,.05);l.name="local";l.partial=true;l.from_global={-1,-1,0};
  block(l,10,10);
  CollisionChecker c({g,l},square(.1),false);
  EXPECT_FALSE(c.poseValid({1.5,1.5,0}));
  EXPECT_TRUE(c.poseValid({3,3,0}));
  EXPECT_TRUE(c.poseValid({1,1,0})); // partially overlaps the local window's free cells
}
TEST(Search,FreeHeadingAndContinuousValidation)
{
  auto g=grid();for(unsigned x=5;x<95;++x) {block(g,x,33);block(g,x,46);}
  CollisionChecker c({g},square(),false);
  const auto r=search(c,{1,2,0},{4,2,1.57079632679},Options{});
  ASSERT_EQ(r.status,"SUCCESS");EXPECT_TRUE(c.pathValid(r.path));
  EXPECT_NEAR(r.path.back().x,4,.025);EXPECT_NEAR(r.path.back().y,2,.025);
  EXPECT_NEAR(r.path.back().yaw,0,1e-6); // requested 90 degrees cannot fit
}
TEST(Search,FindsAlternativeInsteadOfRejectingStraightPath)
{
  auto g=grid();for(unsigned x=42;x<49;++x) {for(unsigned y=35;y<66;++y) {block(g,x,y);}}
  CollisionChecker c({g},square(.15),false);
  Options o;o.max_planning_time=3;
  EXPECT_FALSE(c.pathValid({{1,2.5,0},{4,2.5,0}}));
  const auto r=search(c,{1,2.5,0},{4,2.5,0},o);
  ASSERT_EQ(r.status,"SUCCESS");EXPECT_TRUE(c.pathValid(r.path));
  double deviation=0;for(const auto & p:r.path) {deviation=std::max(deviation,std::abs(p.y-2.5));}
  EXPECT_GT(deviation,.6);
}
TEST(Search,RefusesCollidingStartAndBlockedGoal)
{
  auto g=grid();block(g,40,40);CollisionChecker c({g},square(),false);
  EXPECT_EQ(search(c,{2,2,0},{3,2,0},Options{}).status,"START_COLLISION");
  EXPECT_EQ(search(c,{3,2,0},{2,2,0},Options{}).status,"GOAL_COLLISION");
}
TEST(Search,RejectsSealedCorridor)
{
  auto g=grid(30,.05);for(unsigned y=0;y<30;++y) {block(g,15,y);}
  CollisionChecker c({g},square(.15),false);Options o;o.max_planning_time=.1;
  const auto r=search(c,{.4,.75,0},{1.1,.75,0},o);
  EXPECT_TRUE(r.path.empty());EXPECT_NE(r.status,"SUCCESS");
}
TEST(Search,RejectsInvalidParameters)
{
  CollisionChecker c({grid()},square(),false);Options o;o.search_resolution=0;
  EXPECT_THROW(search(c,{1,1,0},{2,1,0},o),std::invalid_argument);
}
TEST(Collision,RotatedLocalFrame)
{
  auto g=grid(),l=grid(100,.05);l.name="local";l.partial=true;
  // global (2,1) maps to local (1,2) under a 90-degree transform plus translation.
  l.from_global={2,0,1.5707963267948966};block(l,20,40);
  Failure f;CollisionChecker c({g,l},square(.1),false);
  EXPECT_FALSE(c.poseValid({2,1,0},&f));EXPECT_EQ(f.grid,"local");
  EXPECT_TRUE(c.poseValid({2,2,0}));
}
TEST(Collision,FreshSnapshotCanInvalidatePreviouslyClearPath)
{
  auto g=grid();const std::vector<Pose> path{{1,2,0},{3,2,0}};
  EXPECT_TRUE(CollisionChecker({g},square(),false).pathValid(path));
  block(g,40,40);
  EXPECT_FALSE(CollisionChecker({g},square(),false).pathValid(path));
}
TEST(Collision,TouchingLethalAtLocalWindowBoundaryIsBlocked)
{
  auto g=grid(),l=grid(20,.05);l.name="local";l.partial=true;l.from_global={-1,-1,0};
  block(l,19,10);CollisionChecker c({g,l},square(.1),false);
  EXPECT_FALSE(c.poseValid({2.1,1.525,0}));
  EXPECT_TRUE(c.poseValid({2.101,1.525,0}));
}
TEST(Search,FindsDetourInInflatedFieldWithinExpansionBudget)
{
  auto g=grid();std::fill(g.costs.begin(),g.costs.end(),180);
  for(unsigned x=42;x<49;++x) {for(unsigned y=35;y<66;++y) {block(g,x,y);}}
  CollisionChecker c({g},square(.15),false);
  Options o;o.max_planning_time=3;o.max_expansions=60000;
  // A straight reference intersects the obstacle. Matching h's soft-cost
  // scale to g must find a real detour before the fixed expansion budget.
  EXPECT_FALSE(c.pathValid({{1,2.5,0},{4,2.5,0}}));
  const auto r=search(c,{1,2.5,0},{4,2.5,0},o);
  ASSERT_EQ(r.status,"SUCCESS");EXPECT_TRUE(c.pathValid(r.path));
  EXPECT_LT(r.expansions,60000u);
}
