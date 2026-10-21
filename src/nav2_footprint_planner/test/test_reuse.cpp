#include "nav2_footprint_planner/core.hpp"
#include <gtest/gtest.h>
#include <cmath>
using namespace nav2_footprint_planner;
namespace
{
Grid grid(unsigned n=100,double r=.05)
{Grid g;g.width=g.height=n;g.resolution=r;g.name="global";g.costs.resize(n*n);return g;}
std::vector<Point> square(double r=.1) {return {{r,r},{-r,r},{-r,-r},{r,-r}};}
std::vector<Pose> line(double a=1,double b=3,double y=2)
{
  std::vector<Pose> p;const unsigned n=std::lround(std::abs(b-a)/.01);
  for(unsigned i=0;i<=n;++i) {p.push_back({a+(b-a)*i/n,y,0});}return p;
}
std::vector<int> directions(const std::vector<Pose> & path)
{
  std::vector<int> result;
  for(size_t i=1;i<path.size();++i) {
    const auto a=path[i-1],b=path[i];
    if(std::hypot(b.x-a.x,b.y-a.y)<1e-9) {continue;}
    const int d=(b.x-a.x)*std::cos(a.yaw)+(b.y-a.y)*std::sin(a.yaw)<0?-1:1;
    if(result.empty() || result.back()!=d) {result.push_back(d);}
  }
  return result;
}
}
TEST(Reuse,TrimsPassedPrefixAndStartsAtActualPose)
{
  CollisionChecker c({grid()},square(),false);const Pose start{1.12,2,0};
  const auto r=reusePath(c,start,line(),Options{});
  ASSERT_EQ(r.reason,"VALID");ASSERT_FALSE(r.path.empty());
  EXPECT_DOUBLE_EQ(r.path.front().x,start.x);EXPECT_DOUBLE_EQ(r.path.front().y,start.y);
  EXPECT_DOUBLE_EQ(r.path.front().yaw,start.yaw);EXPECT_NEAR(r.path.back().x,3,1e-9);
  EXPECT_GE(r.nearest_index,11u);EXPECT_TRUE(c.pathValid(r.path));
  EXPECT_EQ(directions(r.path),std::vector<int>({1}));
}
TEST(Reuse,SmallOffsetUsesCurvatureBoundedForwardConnection)
{
  CollisionChecker c({grid()},square(),false);Options o;
  const auto r=reusePath(c,{1.10,2.01,0},line(),o);
  ASSERT_EQ(r.reason,"VALID");EXPECT_GT(r.connection_length,0);
  EXPECT_LE(r.connection_length,.30);EXPECT_TRUE(c.pathValid(r.path));
  EXPECT_EQ(directions(r.path),std::vector<int>({1}));
  for(size_t i=1;i<r.path.size();++i) {
    const auto a=r.path[i-1],b=r.path[i];const double ds=std::hypot(b.x-a.x,b.y-a.y);
    if(ds<1e-8) {continue;}
    const double turn=std::abs(std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw)));
    EXPECT_LE(turn/ds,1/o.minimum_turning_radius*1.001);
  }
}
TEST(Reuse,ReversePathGetsReverseOnlyConnector)
{
  CollisionChecker c({grid()},square(),false);
  const auto r=reusePath(c,{2.90,2.01,0},line(3,1),Options{});
  ASSERT_EQ(r.reason,"VALID");EXPECT_TRUE(c.pathValid(r.path));
  EXPECT_EQ(directions(r.path),std::vector<int>({-1}));EXPECT_NEAR(r.path.back().x,1,1e-9);
}
TEST(Reuse,RejectsNearbyPositionWithIncompatibleHeading)
{
  CollisionChecker c({grid()},square(),false);
  const auto r=reusePath(c,{1.10,2,3.141592653589793},line(),Options{});
  EXPECT_EQ(r.reason,"NO_VALID_CONNECTION");EXPECT_TRUE(r.path.empty());
}
TEST(Reuse,NewObstacleOnRemainingPathInvalidatesCache)
{
  auto g=grid();const auto old=line();
  ASSERT_TRUE(CollisionChecker({g},square(),false).pathValid(old));
  g.costs[40*g.width+44]=254;
  const auto r=reusePath(CollisionChecker({g},square(),false),{1.1,2,0},old,Options{});
  EXPECT_EQ(r.reason,"CACHED_PATH_COLLISION");EXPECT_EQ(r.failure.grid,"global");
  EXPECT_TRUE(r.path.empty());
}
TEST(Reuse,ObstacleOnAlreadyPassedPrefixDoesNotForceReturn)
{
  auto g=grid();g.costs[40*g.width+20]=254;
  CollisionChecker c({g},square(.05),false);
  EXPECT_FALSE(c.pathValid(line()));
  const auto r=reusePath(c,{1.30,2,0},line(),Options{});
  EXPECT_EQ(r.reason,"VALID");EXPECT_TRUE(c.pathValid(r.path));
}
TEST(Reuse,ConnectorCannotCrossObstacleOffTheOldPath)
{
  auto g=grid(400,.01);for(unsigned x=80;x<151;++x) {g.costs[205*g.width+x]=254;}
  CollisionChecker c({g},square(.02),false);const auto old=line();
  ASSERT_TRUE(c.pathValid(old));ASSERT_TRUE(c.poseValid({1,2.09,0}));
  const auto r=reusePath(c,{1,2.09,0},old,Options{});
  EXPECT_EQ(r.reason,"NO_VALID_CONNECTION");EXPECT_TRUE(r.path.empty());
}
TEST(Reuse,DoesNotSkipPendingReversalsOnAnOverlappingPath)
{
  auto old=line(1,1.1);auto back=line(1.1,.9);auto forward=line(.9,1.2);
  old.insert(old.end(),back.begin()+1,back.end());
  old.insert(old.end(),forward.begin()+1,forward.end());
  CollisionChecker c({grid()},square(.05),false);
  const auto r=reusePath(c,{1.04,2,0},old,Options{});
  ASSERT_EQ(r.reason,"VALID");EXPECT_LT(r.nearest_index,10u);
  EXPECT_EQ(directions(r.path),std::vector<int>({1,-1,1}));
}
TEST(Reuse,RejectsDistantOrCollidingCurrentPose)
{
  auto g=grid();CollisionChecker c({g},square(),false);
  EXPECT_EQ(reusePath(c,{1,2.11,0},line(),Options{}).reason,"DEVIATED_FROM_CACHED_PATH");
  g.costs[40*g.width+22]=254;
  const auto r=reusePath(CollisionChecker({g},square(),false),{1.1,2,0},line(),Options{});
  EXPECT_EQ(r.reason,"CURRENT_POSE_COLLISION");EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(reusePath(c,{1,2,0},{},Options{}).reason,"NO_CACHED_PATH");
}
TEST(Cache,GoalPositionFrameAndClockChangesInvalidate)
{
  PathCache cache;const Pose goal{3,2,0};
  EXPECT_EQ(cache.eligibility(goal,"map",10,3),"NO_CACHED_PATH");
  cache.remember(line(),goal,"map",10,false);
  EXPECT_EQ(cache.eligibility({3,2,3.14},"map",11,3),"ELIGIBLE"); // free goal yaw
  EXPECT_EQ(cache.eligibility({3.01,2,0},"map",11,3),"GOAL_CHANGED");
  EXPECT_EQ(cache.eligibility(goal,"new_map",11,3),"FRAME_CHANGED");
  EXPECT_EQ(cache.eligibility(goal,"map",9,3),"CLOCK_CHANGED");
  cache.clear();EXPECT_EQ(cache.eligibility(goal,"map",11,3),"NO_CACHED_PATH");
}
TEST(Cache,ReuseDoesNotRefreshLifetime)
{
  PathCache cache;const Pose goal{3,2,0};cache.remember(line(),goal,"map",10,false);
  cache.remember(line(1.1,3),goal,"map",12,true);
  EXPECT_DOUBLE_EQ(cache.age(12),2);
  EXPECT_EQ(cache.eligibility(goal,"map",13,3),"CACHE_EXPIRED");
  cache.remember(line(1.2,3),goal,"map",14,false);
  EXPECT_EQ(cache.eligibility(goal,"map",15,3),"ELIGIBLE");
  EXPECT_THROW(cache.remember({},goal,"map",15,false),std::invalid_argument);
}
