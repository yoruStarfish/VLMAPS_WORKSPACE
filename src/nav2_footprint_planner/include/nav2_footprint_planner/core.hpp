#ifndef NAV2_FOOTPRINT_PLANNER__CORE_HPP_
#define NAV2_FOOTPRINT_PLANNER__CORE_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace nav2_footprint_planner
{
struct Point {double x{}, y{};};
struct Pose {double x{}, y{}, yaw{};};
struct Grid
{
  unsigned width{}, height{};
  double resolution{}, origin_x{}, origin_y{};
  std::vector<uint8_t> costs;
  // Transform a global planning pose into this grid's coordinate frame.
  Pose from_global{};
  std::string name;
  bool partial{false};  // A local window only constrains its covered cells.
};
struct Failure
{
  std::string reason, grid;
  Pose pose;
  int cell_x{-1}, cell_y{-1};
};
class CollisionChecker
{
public:
  CollisionChecker(std::vector<Grid> grids, std::vector<Point> footprint, bool allow_unknown);
  bool poseValid(const Pose & pose, Failure * failure = nullptr) const;
  // Validate the convex sweep of two rigid footprints plus an arc sagitta bound.
  bool sweepValid(const Pose & a, const Pose & b, double bulge,
    Failure * failure = nullptr) const;
  bool pathValid(const std::vector<Pose> & path, Failure * failure = nullptr) const;
  double softCost(const Pose & pose) const;
  double footprintRadius() const {return radius_;}
  const Grid & globalGrid() const {return grids_.front().grid;}
private:
  struct Prepared {Grid grid; std::vector<unsigned> occupied_integral;};
  std::vector<Prepared> grids_;
  std::vector<Point> footprint_;
  double radius_{};
  bool polygonValid(const std::vector<Point> & polygon, double margin,
    const Pose & pose, Failure * failure) const;
  std::vector<Point> vertices(const Pose & pose) const;
};
struct Options
{
  double search_resolution{0.0125};
  unsigned angle_bins{72};
  double minimum_turning_radius{0.2};
  double primitive_length{0.05};
  double validation_step{0.01};
  double goal_tolerance{0.025};
  double max_planning_time{1.0};
  unsigned max_expansions{100000};
  unsigned analytic_interval{12};
  unsigned goal_heading_samples{72};
  double reverse_penalty{1.2};
  double change_penalty{5.0};
  double cost_penalty{2.0};
};
struct Result
{
  std::vector<Pose> path;
  std::string status;
  Failure failure;
  unsigned expansions{}, rejected_edges{};
  double elapsed{};
};
// Cache lifetime is measured from the last *fresh search*, not each reuse.
class PathCache
{
public:
  std::string eligibility(const Pose & goal, const std::string & frame,
    double now, double max_age) const;
  void remember(std::vector<Pose> path, const Pose & goal, const std::string & frame,
    double now, bool reused);
  void clear();
  double age(double now) const;
  const std::vector<Pose> & path() const {return path_;}
private:
  std::vector<Pose> path_;
  Pose goal_;
  std::string frame_;
  double created_at_{};
};
struct ReuseResult
{
  std::vector<Pose> path;
  std::string reason;
  Failure failure;
  unsigned nearest_index{}, join_index{};
  double connection_length{}, elapsed{};
};
// Reattach to a previously validated kinematic path with a short, same-direction
// Dubins connection. Never bypasses collision checks or inserts new cusps.
ReuseResult reusePath(const CollisionChecker & checker, const Pose & start,
  const std::vector<Pose> & previous, const Options & options);
// Hybrid A*: continuous forward/reverse arcs, free terminal heading, strict swept geometry.
Result search(const CollisionChecker & checker, const Pose & start, const Pose & goal,
  const Options & options);
}  // namespace nav2_footprint_planner
#endif
