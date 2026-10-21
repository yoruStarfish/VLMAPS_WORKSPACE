#include "nav2_footprint_planner/core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace nav2_footprint_planner
{
namespace
{
constexpr double pi = 3.14159265358979323846;
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
bool finite(const Pose & p)
{return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);}
Point apply(const Point & p, const Pose & t)
{return {t.x + std::cos(t.yaw)*p.x - std::sin(t.yaw)*p.y,
    t.y + std::sin(t.yaw)*p.x + std::cos(t.yaw)*p.y};}
double cross(Point a, Point b, Point c)
{return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
std::vector<Point> hull(std::vector<Point> p)
{
  std::sort(p.begin(), p.end(), [](Point a, Point b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);});
  p.erase(std::unique(p.begin(), p.end(), [](Point a, Point b) {
    return a.x == b.x && a.y == b.y;}), p.end());
  if (p.size() < 3) {return p;}
  std::vector<Point> h(2*p.size()); size_t k=0;
  for (auto v:p) {while (k>=2 && cross(h[k-2],h[k-1],v)<=0) {--k;} h[k++]=v;}
  const size_t lower=k+1;
  for (int i=static_cast<int>(p.size())-2;i>=0;--i) {
    while (k>=lower && cross(h[k-2],h[k-1],p[i])<=0) {--k;} h[k++]=p[i];}
  h.resize(k-1); return h;
}
// All separating axes of a convex polygon and an axis-aligned obstacle cell.
bool intersects(const std::vector<Point> & p, double x, double y, double half, double margin)
{
  auto separated=[&](Point axis) {
      const double norm=std::hypot(axis.x,axis.y);
      if (norm<1e-12) {return false;}
      double lo=std::numeric_limits<double>::infinity(), hi=-lo;
      for (auto v:p) {const double t=v.x*axis.x+v.y*axis.y;
        lo=std::min(lo,t); hi=std::max(hi,t);}
      const double mid=x*axis.x+y*axis.y;
      const double extent=half*(std::abs(axis.x)+std::abs(axis.y))+margin*norm;
      return mid+extent<lo-1e-10 || mid-extent>hi+1e-10;
    };
  if (separated({1,0}) || separated({0,1})) {return false;}
  for (size_t i=0;i<p.size();++i) {
    const auto a=p[i], b=p[(i+1)%p.size()];
    if (separated({a.y-b.y,b.x-a.x})) {return false;}
  }
  return true;
}
Pose advance(Pose p, double distance, double curvature)
{
  const double turn=distance*curvature;
  if (std::abs(curvature)<1e-12) {
    p.x+=distance*std::cos(p.yaw); p.y+=distance*std::sin(p.yaw);
  } else {
    p.x+=(std::sin(p.yaw+turn)-std::sin(p.yaw))/curvature;
    p.y+=(-std::cos(p.yaw+turn)+std::cos(p.yaw))/curvature;
  }
  p.yaw=wrap(p.yaw+turn); return p;
}
bool arc(const CollisionChecker & checker, Pose & p, double distance, double curvature,
  double step, std::vector<Pose> * output, Failure * failure)
{
  // Limit each swept arc to <= 0.1 rad; this makes the sagitta bound tight.
  unsigned n=std::max(1u, static_cast<unsigned>(std::ceil(std::max(
      std::abs(distance)/step, std::abs(distance*curvature)/0.1))));
  const double ds=distance/n, angle=std::abs(ds*curvature);
  const double radius=std::abs(curvature)<1e-12 ? 0 : 1/std::abs(curvature);
  const double bulge=angle==0 ? 0 :
    (radius+checker.footprintRadius())*(1-std::cos(angle/2));
  for (unsigned i=0;i<n;++i) {
    const Pose next=advance(p,ds,curvature);
    if (!checker.sweepValid(p,next,bulge,failure)) {return false;}
    p=next; if (output) {output->push_back(p);}
  }
  return true;
}
}  // namespace

CollisionChecker::CollisionChecker(std::vector<Grid> grids, std::vector<Point> footprint,
  bool allow_unknown):footprint_(std::move(footprint))
{
  if (grids.empty() || grids.front().partial || footprint_.size()<3) {
    throw std::invalid_argument("A bounded global grid and convex footprint are required");}
  double area=0, sign=0;
  for (size_t i=0;i<footprint_.size();++i) {
    const auto a=footprint_[i], b=footprint_[(i+1)%footprint_.size()];
    const auto c=footprint_[(i+2)%footprint_.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y)) {
      throw std::invalid_argument("Non-finite footprint");}
    area+=a.x*b.y-a.y*b.x; radius_=std::max(radius_,std::hypot(a.x,a.y));
    const double z=cross(a,b,c);
    if (std::abs(z)>1e-12) {
      if (sign*z<0) {throw std::invalid_argument("Footprint must be convex and ordered");}
      sign=z;
    }
  }
  if (std::abs(area)<1e-9) {throw std::invalid_argument("Degenerate footprint");}
  for (auto & g:grids) {
    if (!g.width || !g.height || !std::isfinite(g.resolution) || g.resolution<=0 ||
      !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y) || !finite(g.from_global) ||
      g.costs.size()!=static_cast<size_t>(g.width)*g.height)
    {throw std::invalid_argument("Invalid grid geometry/data");}
    Prepared p; p.grid=std::move(g);
    p.occupied_integral.assign(static_cast<size_t>(p.grid.width+1)*(p.grid.height+1),0);
    const unsigned stride=p.grid.width+1;
    for (unsigned y=0;y<p.grid.height;++y) {
      unsigned row=0;
      for (unsigned x=0;x<p.grid.width;++x) {
        const auto c=p.grid.costs[y*p.grid.width+x];
        row+=(c==254 || (c==255 && !allow_unknown));
        p.occupied_integral[(y+1)*stride+x+1]=p.occupied_integral[y*stride+x+1]+row;
      }
    }
    grids_.push_back(std::move(p));
  }
}
std::vector<Point> CollisionChecker::vertices(const Pose & p) const
{
  std::vector<Point> result; result.reserve(footprint_.size());
  for (auto v:footprint_) {result.push_back(apply(v,p));} return result;
}
bool CollisionChecker::polygonValid(const std::vector<Point> & poly, double margin,
  const Pose & pose, Failure * failure) const
{
  for (const auto & prepared:grids_) {
    const auto & g=prepared.grid;
    std::vector<Point> p; p.reserve(poly.size());
    double xmin=std::numeric_limits<double>::infinity(), xmax=-xmin, ymin=xmin, ymax=-xmin;
    for (auto v:poly) {
      v=apply(v,g.from_global); p.push_back(v);
      xmin=std::min(xmin,v.x); xmax=std::max(xmax,v.x);
      ymin=std::min(ymin,v.y); ymax=std::max(ymax,v.y);
    }
    xmin-=margin; xmax+=margin; ymin-=margin; ymax+=margin;
    const double gxmax=g.origin_x+g.width*g.resolution;
    const double gymax=g.origin_y+g.height*g.resolution;
    if (!g.partial && (xmin<g.origin_x || ymin<g.origin_y || xmax>=gxmax || ymax>=gymax)) {
      if (failure) {*failure={"OUTSIDE_GLOBAL",g.name,pose,-1,-1};} return false;
    }
    if (xmax<g.origin_x || ymax<g.origin_y || xmin>gxmax || ymin>gymax) {continue;}
    // Include boundary cells even when a polygon edge falls exactly on a grid line.
    const int x0=std::max(0,static_cast<int>(std::floor((xmin-g.origin_x)/g.resolution-1e-9)));
    const int y0=std::max(0,static_cast<int>(std::floor((ymin-g.origin_y)/g.resolution-1e-9)));
    const int x1=std::min(static_cast<int>(g.width)-1,
      static_cast<int>(std::floor((xmax-g.origin_x)/g.resolution+1e-9)));
    const int y1=std::min(static_cast<int>(g.height)-1,
      static_cast<int>(std::floor((ymax-g.origin_y)/g.resolution+1e-9)));
    const auto & sum=prepared.occupied_integral; const unsigned stride=g.width+1;
    const unsigned count=sum[(y1+1)*stride+x1+1]-sum[y0*stride+x1+1]-
      sum[(y1+1)*stride+x0]+sum[y0*stride+x0];
    if (!count) {continue;}
    for (int y=y0;y<=y1;++y) {for (int x=x0;x<=x1;++x) {
      // Determine whether this cell was marked blocked in the integral image.
      const unsigned n=sum[(y+1)*stride+x+1]-sum[y*stride+x+1]-
        sum[(y+1)*stride+x]+sum[y*stride+x];
      if (n && intersects(p,g.origin_x+(x+.5)*g.resolution,
        g.origin_y+(y+.5)*g.resolution,g.resolution/2,margin))
      {if (failure) {*failure={"FOOTPRINT_COLLISION",g.name,pose,x,y};} return false;}
    }}
  }
  return true;
}
bool CollisionChecker::poseValid(const Pose & p, Failure * f) const
{
  if (!finite(p)) {if (f) {*f={"INVALID_POSE","",p,-1,-1};} return false;}
  return polygonValid(vertices(p),0,p,f);
}
bool CollisionChecker::sweepValid(const Pose & a, const Pose & b, double bulge, Failure * f) const
{
  if (!finite(a) || !finite(b) || !std::isfinite(bulge) || bulge<0) {return false;}
  auto p=vertices(a), q=vertices(b); p.insert(p.end(),q.begin(),q.end());
  return polygonValid(hull(std::move(p)),bulge,b,f);
}
bool CollisionChecker::pathValid(const std::vector<Pose> & path, Failure * f) const
{
  if (path.empty()) {return false;}
  if (!poseValid(path.front(),f)) {return false;}
  for (size_t i=1;i<path.size();++i) {
    // For linearly interpolated position and shortest-angle rotation, each
    // footprint vertex deviates from its endpoint chord by <= r*(1-cos(dyaw/2)).
    const double angle=std::abs(wrap(path[i].yaw-path[i-1].yaw));
    if (!sweepValid(path[i-1],path[i],radius_*(1-std::cos(angle/2)),f)) {return false;}
  }
  return true;
}
double CollisionChecker::softCost(const Pose & pose) const
{
  double cost=0;
  for (const auto & p:grids_) {
    const auto & g=p.grid; const auto v=apply({pose.x,pose.y},g.from_global);
    const int x=std::floor((v.x-g.origin_x)/g.resolution);
    const int y=std::floor((v.y-g.origin_y)/g.resolution);
    if (x>=0 && y>=0 && x<static_cast<int>(g.width) && y<static_cast<int>(g.height)) {
      const auto c=g.costs[y*g.width+x];
      if (c<254) {cost=std::max(cost,c/253.0);}
    }
  }
  return cost;
}

std::string PathCache::eligibility(const Pose & goal, const std::string & frame,
  double now, double max_age) const
{
  if (!std::isfinite(max_age) || max_age<=0) {throw std::invalid_argument("Invalid cache age");}
  if (path_.empty()) {return "NO_CACHED_PATH";}
  if (frame!=frame_) {return "FRAME_CHANGED";}
  if (!finite(goal) || std::hypot(goal.x-goal_.x,goal.y-goal_.y)>1e-6) {return "GOAL_CHANGED";}
  if (!std::isfinite(now) || now<created_at_) {return "CLOCK_CHANGED";}
  return now-created_at_>=max_age?"CACHE_EXPIRED":"ELIGIBLE";
}
void PathCache::remember(std::vector<Pose> path, const Pose & goal, const std::string & frame,
  double now, bool reused)
{
  if (path.empty() || !finite(goal) || !std::isfinite(now)) {
    throw std::invalid_argument("Invalid cache update");}
  if (reused && (path_.empty() || frame!=frame_ || now<created_at_ ||
    std::hypot(goal.x-goal_.x,goal.y-goal_.y)>1e-6)) {
    throw std::invalid_argument("Cannot reuse an unrelated cache");}
  if (!reused) {created_at_=now;}
  path_=std::move(path);goal_=goal;frame_=frame;
}
void PathCache::clear() {path_.clear();frame_.clear();created_at_=0;}
double PathCache::age(double now) const {return path_.empty()?-1:now-created_at_;}

ReuseResult reusePath(const CollisionChecker & checker, const Pose & start,
  const std::vector<Pose> & previous, const Options & options)
{
  using Clock=std::chrono::steady_clock;
  const auto begun=Clock::now();ReuseResult result;
  auto finish=[&](const std::string & reason) {
      result.reason=reason;result.elapsed=std::chrono::duration<double>(Clock::now()-begun).count();
      if (reason=="VALID") {result.failure=Failure{};} else {result.path.clear();}
      return result;};
  // Deliberately local reattachment, not another global search. These bounds
  // limit both computation and the amount of old route that can be skipped.
  constexpr double max_deviation=.10, max_progress=.50, max_ahead=.25;
  constexpr double max_connection=.30, max_seconds=.05;
  auto expired=[&]() {return std::chrono::duration<double>(Clock::now()-begun).count()>max_seconds;};
  if (!std::isfinite(options.minimum_turning_radius) || options.minimum_turning_radius<=0 ||
    !std::isfinite(options.validation_step) || options.validation_step<=0) {
    throw std::invalid_argument("Invalid connection parameters");}
  if (!checker.poseValid(start,&result.failure)) {return finish("CURRENT_POSE_COLLISION");}
  if (previous.empty()) {return finish("NO_CACHED_PATH");}
  double travelled=0, best=std::numeric_limits<double>::infinity(), best_angle=best;
  size_t nearest=0;
  for (size_t i=0;i<previous.size();++i) {
    if (!finite(previous[i])) {return finish("INVALID_CACHED_PATH");}
    if (i) {travelled+=std::hypot(previous[i].x-previous[i-1].x,previous[i].y-previous[i-1].y);}
    if (travelled>max_progress) {break;}
    const double distance=std::hypot(start.x-previous[i].x,start.y-previous[i].y);
    const double angle=std::abs(wrap(start.yaw-previous[i].yaw));
    if (distance<best-1e-9 || (std::abs(distance-best)<1e-9 && angle<best_angle-1e-9)) {
      nearest=i;best=distance;best_angle=angle;}
  }
  result.nearest_index=nearest;
  if (best>max_deviation) {return finish("DEVIATED_FROM_CACHED_PATH");}
  const std::vector<Pose> remaining(previous.begin()+nearest,previous.end());
  if (!checker.pathValid(remaining,&result.failure)) {return finish("CACHED_PATH_COLLISION");}
  if (expired()) {return finish("REUSE_CHECK_TIMEOUT");}
  auto direction=[&](size_t i) {
      const auto & a=previous[i];const auto & b=previous[i+1];
      const double dot=(b.x-a.x)*std::cos(a.yaw)+(b.y-a.y)*std::sin(a.yaw);
      return dot<0?-1:1;};
  const int desired=nearest+1<previous.size()?direction(nearest):
    (nearest?direction(nearest-1):1);
  std::vector<size_t> candidates{nearest};
  travelled=0;double last_candidate=0;
  for (size_t i=nearest;i+1<previous.size();++i) {
    if (direction(i)!=desired) {break;}  // Never jump over a pending reversal.
    travelled+=std::hypot(previous[i+1].x-previous[i].x,previous[i+1].y-previous[i].y);
    if (travelled>max_ahead+1e-9) {break;}
    if (travelled-last_candidate>=.02-1e-9 || i+2==previous.size()) {
      candidates.push_back(i+1);last_candidate=travelled;}
  }
  // Dubins is forward-only. For a reverse suffix, rotate both body headings
  // by pi and integrate negative distance / opposite curvature. This finds a
  // same-direction connector even when the shortest Reeds-Shepp curve has cusps.
  ompl::base::DubinsStateSpace space(options.minimum_turning_radius);
  const auto free_state=[&](ompl::base::State * p) {space.freeState(p);};
  std::unique_ptr<ompl::base::State,decltype(free_state)> owner_a(space.allocState(),free_state);
  std::unique_ptr<ompl::base::State,decltype(free_state)> owner_b(space.allocState(),free_state);
  auto * a=owner_a->as<ompl::base::SE2StateSpace::StateType>();
  auto * b=owner_b->as<ompl::base::SE2StateSpace::StateType>();
  const double offset=desired<0?pi:0;
  a->setXY(start.x,start.y);a->setYaw(wrap(start.yaw+offset));
  for (const size_t join:candidates) {
    if (expired()) {return finish("REUSE_CHECK_TIMEOUT");}
    const auto target=previous[join];std::vector<Pose> prefix{start};Pose p=start;
    double length=0;bool valid=true;
    if (std::hypot(target.x-start.x,target.y-start.y)>1e-8 ||
      std::abs(wrap(target.yaw-start.yaw))>1e-8) {
      b->setXY(target.x,target.y);b->setYaw(wrap(target.yaw+offset));
      const auto connection=space.dubins(a,b);
      length=connection.length()*options.minimum_turning_radius;
      if (!std::isfinite(length) || length>max_connection) {continue;}
      for (unsigned j=0;j<3 && valid;++j) {
        const double ds=desired*connection.length_[j]*options.minimum_turning_radius;
        if (std::abs(ds)<1e-12) {continue;}
        double curvature=0;
        if ((*connection.type_)[j]==ompl::base::DubinsStateSpace::DUBINS_LEFT) {
          curvature=desired/options.minimum_turning_radius;
        } else if ((*connection.type_)[j]==ompl::base::DubinsStateSpace::DUBINS_RIGHT) {
          curvature=-desired/options.minimum_turning_radius;}
        valid=arc(checker,p,ds,curvature,options.validation_step,&prefix,&result.failure);
      }
    }
    if (!valid || std::hypot(p.x-target.x,p.y-target.y)>1e-6 ||
      std::abs(wrap(p.yaw-target.yaw))>1e-6) {continue;}
    prefix.back()=target;  // Eliminate roundoff at the exact position/yaw splice.
    prefix.front()=start;
    prefix.insert(prefix.end(),previous.begin()+join+1,previous.end());
    if (!checker.pathValid(prefix,&result.failure)) {continue;}
    if (expired()) {return finish("REUSE_CHECK_TIMEOUT");}
    result.path=std::move(prefix);result.join_index=join;result.connection_length=length;
    return finish("VALID");
  }
  return finish("NO_VALID_CONNECTION");
}

Result search(const CollisionChecker & checker, const Pose & start, const Pose & goal,
  const Options & o)
{
  using Clock=std::chrono::steady_clock;
  const auto begun=Clock::now(); Result result;
  auto finish=[&](std::string status) {
      result.status=std::move(status);
      if (result.status=="SUCCESS") {result.failure=Failure{};}
      result.elapsed=
        std::chrono::duration<double>(Clock::now()-begun).count(); return result;};
  if (!finite(start) || !finite(goal) || !std::isfinite(o.search_resolution) ||
    o.search_resolution<=0 || o.angle_bins<8 || !std::isfinite(o.minimum_turning_radius) ||
    o.minimum_turning_radius<=0 || !std::isfinite(o.validation_step) || o.validation_step<=0 ||
    !std::isfinite(o.primitive_length) || o.primitive_length<=0 ||
    !std::isfinite(o.goal_tolerance) || o.goal_tolerance<0 ||
    !std::isfinite(o.max_planning_time) || o.max_planning_time<=0 || !o.max_expansions ||
    !o.analytic_interval || o.goal_heading_samples<4 ||
    !std::isfinite(o.reverse_penalty) || o.reverse_penalty<1 ||
    !std::isfinite(o.change_penalty) || o.change_penalty<0 ||
    !std::isfinite(o.cost_penalty) || o.cost_penalty<0)
  {throw std::invalid_argument("Invalid strict search options");}
  if (!checker.poseValid(start,&result.failure)) {return finish("START_COLLISION");}
  ompl::base::ReedsSheppStateSpace rs(o.minimum_turning_radius);
  std::vector<double> headings;
  // Try broad approach directions first, then refine the free terminal angle.
  for (unsigned i=0;i<8;++i) {headings.push_back(2*pi*i/8);}
  headings.push_back(start.yaw);
  headings.push_back(std::atan2(goal.y-start.y,goal.x-start.x));
  for (unsigned i=0;i<o.goal_heading_samples;++i) {
    const double a=2*pi*i/o.goal_heading_samples;
    if (std::none_of(headings.begin(),headings.end(),[&](double h) {
      return std::abs(wrap(a-h))<1e-8;})) {headings.push_back(a);}
  }
  headings.erase(std::remove_if(headings.begin(),headings.end(),[&](double h) {
    return !checker.poseValid({goal.x,goal.y,h});}),headings.end());
  if (headings.empty()) {result.failure={"NO_VALID_GOAL_HEADING","",goal,-1,-1};
    return finish("GOAL_COLLISION");}
  // A center-cell Dijkstra guide avoids wasting the entire budget pushing
  // against a wall. It guides expansion only; every accepted edge still uses
  // the full swept polygon in both grids. Use the same soft-cost scale as g:
  // distance-only h vastly underestimates long inflated routes and expands
  // thousands of equivalent states near the start. No collision rule changes.
  const auto & guide=checker.globalGrid();
  const double inf=std::numeric_limits<double>::infinity();
  std::vector<double> distance(guide.costs.size(),inf);
  auto cell=[&](Pose p) {
      const auto v=apply({p.x,p.y},guide.from_global);
      return std::pair<int,int>{std::floor((v.x-guide.origin_x)/guide.resolution),
        std::floor((v.y-guide.origin_y)/guide.resolution)};};
  const auto target=cell(goal);
  using GuideEntry=std::pair<double,unsigned>;
  std::priority_queue<GuideEntry,std::vector<GuideEntry>,std::greater<GuideEntry>> queue;
  if (target.first>=0 && target.second>=0 && target.first<static_cast<int>(guide.width) &&
    target.second<static_cast<int>(guide.height)) {
    const unsigned idx=target.second*guide.width+target.first;
    distance[idx]=0;queue.push({0,idx});
  }
  while (!queue.empty()) {
    const auto [d,i]=queue.top();queue.pop();if (d>distance[i]) {continue;}
    const int x=i%guide.width,y=i/guide.width;
    for (int dy=-1;dy<=1;++dy) {for (int dx=-1;dx<=1;++dx) {
      if (!dx && !dy) {continue;}
      const int nx=x+dx,ny=y+dy;
      if (nx<0 || ny<0 || nx>=static_cast<int>(guide.width) || ny>=static_cast<int>(guide.height)) {continue;}
      const unsigned j=ny*guide.width+nx;
      if (guide.costs[j]==254) {continue;}
      const double from_cost=guide.costs[i]<254?guide.costs[i]/253.0:0;
      const double to_cost=guide.costs[j]<254?guide.costs[j]/253.0:0;
      const double nd=d+guide.resolution*std::hypot(dx,dy)*
        (1+o.cost_penalty*(from_cost+to_cost)/2);
      if (nd<distance[j]) {distance[j]=nd;queue.push({nd,j});}
    }}
    if (std::chrono::duration<double>(Clock::now()-begun).count()>o.max_planning_time) {
      return finish("TIMEOUT");}
  }
  auto goalDistance=[&](Pose p) {return std::hypot(p.x-goal.x,p.y-goal.y);};
  auto heuristic=[&](Pose p) {
      const double euclidean=goalDistance(p);const auto c=cell(p);
      if (c.first<0 || c.second<0 || c.first>=static_cast<int>(guide.width) ||
        c.second>=static_cast<int>(guide.height)) {return euclidean;}
      const double d=distance[c.second*guide.width+c.first];
      return std::isfinite(d)?std::max(euclidean,d-2*std::sqrt(2.)*guide.resolution):euclidean;};
  struct Key {long x,y; unsigned angle; int direction;
    bool operator==(const Key & b) const {return x==b.x&&y==b.y&&angle==b.angle&&direction==b.direction;}};
  struct Hash {size_t operator()(const Key & k) const {
    size_t h=std::hash<long>()(k.x);h^=std::hash<long>()(k.y)+0x9e3779b9+(h<<6)+(h>>2);
    h^=k.angle+0x9e3779b9+(h<<6)+(h>>2);return h^(k.direction+1);}};
  auto key=[&](Pose p,int direction) {
    const long bin=std::lround(wrap(p.yaw)*o.angle_bins/(2*pi));
    return Key{std::lround((p.x-start.x)/o.search_resolution),
      std::lround((p.y-start.y)/o.search_resolution),
      static_cast<unsigned>((bin%o.angle_bins+o.angle_bins)%o.angle_bins),direction};};
  struct Node {Pose p; double g; int parent; double ds,curvature; int direction;};
  struct Entry {double score; int node;
    bool operator<(const Entry & b) const {return score>b.score;}};
  std::vector<Node> nodes; nodes.reserve(10000); nodes.push_back({start,0,-1,0,0,1});
  std::priority_queue<Entry> open; open.push({heuristic(start),0});
  std::unordered_map<Key,double,Hash> best; best[key(start,1)]=0;
  auto reconstruct=[&](int idx) {
      std::vector<int> chain;
      for (int n=idx;n>=0;n=nodes[n].parent) {chain.push_back(n);}
      std::reverse(chain.begin(),chain.end()); std::vector<Pose> path{start}; Pose p=start;
      for (size_t i=1;i<chain.size();++i) {
        const auto & n=nodes[chain[i]];
        if (!arc(checker,p,n.ds,n.curvature,o.validation_step,&path,&result.failure)) {
          return std::vector<Pose>{};}
      }
      return path;};
  const double angle_step=2*pi/o.angle_bins;
  const double turn=std::ceil(o.primitive_length/o.minimum_turning_radius/angle_step)*angle_step;
  size_t analytic_cursor=0;
  while (!open.empty() && result.expansions<o.max_expansions) {
    if (std::chrono::duration<double>(Clock::now()-begun).count()>o.max_planning_time) {
      return finish("TIMEOUT");}
    const auto entry=open.top(); open.pop();
    const Node current=nodes[entry.node];
    if (current.g>best[key(current.p,current.direction)]+1e-9) {continue;}
    ++result.expansions;
    if (goalDistance(current.p)<=o.goal_tolerance) {
      result.path=reconstruct(entry.node);
      if (!result.path.empty()) {return finish("SUCCESS");}
    }
    if (result.expansions==1 || result.expansions%o.analytic_interval==0) {
      // StateType requires allocated components, so use the state space allocator.
      auto * a=rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
      auto * b=rs.allocState()->as<ompl::base::SE2StateSpace::StateType>();
      a->setXY(current.p.x,current.p.y);a->setYaw(current.p.yaw);
      std::vector<Pose> best_tail; double best_cost=std::numeric_limits<double>::infinity();
      // Bound expensive swept checks per expansion. Cycle through all valid
      // headings across attempts instead of checking 72 near-identical curves.
      const size_t attempts=std::min<size_t>(10,headings.size());
      for (size_t attempt=0;attempt<attempts;++attempt) {
        const double heading=headings[(analytic_cursor+attempt)%headings.size()];
        if (std::chrono::duration<double>(Clock::now()-begun).count()>o.max_planning_time) {break;}
        b->setXY(goal.x,goal.y);b->setYaw(heading);
        const auto connection=rs.reedsShepp(a,b);Pose p=current.p;std::vector<Pose> tail;
        double cost=0;bool valid=true;int prior_direction=current.direction;
        for (unsigned j=0;j<5 && valid;++j) {
          const double ds=connection.length_[j]*o.minimum_turning_radius;
          if (std::abs(ds)<1e-12) {continue;}
          double curvature=0;
          if (connection.type_[j]==ompl::base::ReedsSheppStateSpace::RS_LEFT) {
            curvature=1/o.minimum_turning_radius;
          } else if (connection.type_[j]==ompl::base::ReedsSheppStateSpace::RS_RIGHT) {
            curvature=-1/o.minimum_turning_radius;}
          const size_t before=tail.size();
          valid=arc(checker,p,ds,curvature,o.validation_step,&tail,&result.failure);
          const int direction=ds<0?-1:1;
          cost+=std::abs(ds)*(ds<0?o.reverse_penalty:1)+
            (direction!=prior_direction?o.change_penalty:0);
          prior_direction=direction;
          for (size_t k=before;k<tail.size();++k) {
            cost+=o.validation_step*o.cost_penalty*checker.softCost(tail[k]);}
        }
        if (valid && goalDistance(p)<1e-6 && cost<best_cost) {
          best_cost=cost;best_tail=std::move(tail);}
      }
      analytic_cursor=(analytic_cursor+attempts)%headings.size();
      rs.freeState(a);rs.freeState(b);
      if (!best_tail.empty()) {
        result.path=reconstruct(entry.node);
        if (!result.path.empty()) {
          result.path.insert(result.path.end(),best_tail.begin(),best_tail.end());
          return finish("SUCCESS");}
      }
    }
    for (int direction:{1,-1}) {for (int steering:{0,1,-1}) {
      const double ds=direction*(steering==0?o.primitive_length:o.minimum_turning_radius*turn);
      const double curvature=steering/o.minimum_turning_radius;
      Pose p=current.p;
      if (!arc(checker,p,ds,curvature,o.validation_step,nullptr,&result.failure)) {
        ++result.rejected_edges;continue;}
      const double g=current.g+std::abs(ds)*(direction<0?o.reverse_penalty:1)*
        (1+o.cost_penalty*checker.softCost(p))+
        (direction!=current.direction?o.change_penalty:0);
      const auto k=key(p,direction);const auto it=best.find(k);
      if (it!=best.end() && it->second<=g) {continue;}
      best[k]=g;const int idx=nodes.size();nodes.push_back({p,g,entry.node,ds,curvature,direction});
      open.push({g+heuristic(p),idx});
    }}
  }
  return finish(open.empty()?"NO_PATH":"EXPANSION_LIMIT");
}
}  // namespace nav2_footprint_planner
