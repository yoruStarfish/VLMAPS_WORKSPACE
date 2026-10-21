#include "nav2_footprint_planner/core.hpp"
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_footprint_planner
{
// All mutable map data are copied under their own mutex. Search never holds the
// costmap update mutex, and the returned path is checked against fresh copies.
class StrictHybridPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_=parent.lock(); if (!node_) {throw std::runtime_error("Planner parent expired");}
    name_=std::move(name);tf_=std::move(tf);costmap_=std::move(costmap);
    auto param=[&](const std::string & key, auto value) {
        nav2_util::declare_parameter_if_not_declared(node_,name_+"."+key,rclcpp::ParameterValue(value));
        return node_->get_parameter(name_+"."+key).get_value<decltype(value)>();};
    options_.search_resolution=param("search_resolution",0.0125);
    options_.angle_bins=positive(param("angle_quantization_bins",72));
    options_.minimum_turning_radius=param("minimum_turning_radius",0.2);
    options_.primitive_length=param("primitive_length",0.05);
    options_.validation_step=param("validation_step",0.01);
    options_.goal_tolerance=param("goal_tolerance",0.025);
    options_.max_planning_time=param("max_planning_time",1.0);
    options_.max_expansions=positive(param("max_expansions",100000));
    options_.analytic_interval=positive(param("analytic_interval",12));
    options_.goal_heading_samples=positive(param("goal_heading_samples",72));
    options_.reverse_penalty=param("reverse_penalty",1.2);
    options_.change_penalty=param("change_penalty",5.0);
    options_.cost_penalty=param("cost_penalty",2.0);
    reuse_enabled_=param("reuse_path_enabled",false);
    reuse_max_age_=param("reuse_max_age",3.0);
    if (!std::isfinite(reuse_max_age_) || reuse_max_age_<=0) {
      throw std::invalid_argument("Invalid reuse cache age");}
    cache_.clear();
    allow_unknown_=param("allow_unknown",false);
    max_age_=param("max_local_costmap_age",1.0);
    if (!std::isfinite(max_age_) || max_age_<=0) {throw std::invalid_argument("Invalid map age");}
    const auto topic=param("local_costmap_topic",std::string("/local_costmap/costmap_raw"));
    // Costmap2DROS has its own spinning thread in planner_server, so local updates
    // continue while a createPlan service/action callback is computing a path.
    subscription_=costmap_->create_subscription<nav2_msgs::msg::Costmap>(topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(local_mutex_);local_=std::move(message);});
    status_=node_->create_publisher<std_msgs::msg::String>(name_+"/status",rclcpp::QoS(1).transient_local());
    debug_=node_->create_publisher<nav_msgs::msg::Path>(name_+"/validated_path",rclcpp::QoS(1).transient_local());
  }
  void activate() override
  {std::lock_guard<std::mutex> lock(plan_mutex_);cache_.clear();status_->on_activate();debug_->on_activate();}
  void deactivate() override
  {std::lock_guard<std::mutex> lock(plan_mutex_);cache_.clear();status_->on_deactivate();debug_->on_deactivate();}
  void cleanup() override
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);cache_.clear();
    subscription_.reset();
    {std::lock_guard<std::mutex> lock(local_mutex_);local_.reset();}
    status_.reset();debug_.reset();costmap_.reset();tf_.reset();node_.reset();
  }
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    // planner_server serializes requests; also protect direct plugin callers.
    std::lock_guard<std::mutex> call_lock(plan_mutex_);
    Result result;ReuseResult reuse;bool reused=false;double cache_age=-1;
    reuse.reason=reuse_enabled_?"NOT_CHECKED":"DISABLED";
    try {
      const auto frame=costmap_->getGlobalFrameID();
      if (start.header.frame_id!=frame || goal.header.frame_id!=frame) {
        throw std::runtime_error("Start and goal must be in global costmap frame");}
      const Pose start_pose{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)};
      const Pose goal_pose{goal.pose.position.x,goal.pose.position.y,0};
      auto checker=snapshot();
      if (reuse_enabled_) {
        cache_age=cache_.age(monotonicNow());
        reuse.reason=cache_.eligibility(goal_pose,frame,monotonicNow(),reuse_max_age_);
        if (reuse.reason=="ELIGIBLE") {
          reuse=reusePath(checker,start_pose,cache_.path(),options_);
          if (!reuse.path.empty()) {
            auto latest=snapshot();
            if (!latest.pathValid(reuse.path,&reuse.failure)) {
              reuse.reason="MAP_CHANGED_CACHED_PATH_INVALID";
            } else if (cache_.eligibility(goal_pose,frame,monotonicNow(),reuse_max_age_)!="ELIGIBLE") {
              reuse.reason="CACHE_EXPIRED";
            } else {
              result.path=std::move(reuse.path);result.status="SUCCESS";reused=true;
            }
            checker=std::move(latest);
          }
        }
      }
      if (!reused) {
        cache_.clear();
        result=search(checker,start_pose,goal_pose,options_);
        if (result.path.empty()) {throw std::runtime_error(result.status);}
        // A fresh plan receives the same final snapshot validation as before.
        auto latest=snapshot();
        if (!latest.pathValid(result.path,&result.failure)) {
          result.status="MAP_CHANGED_PATH_INVALID";throw std::runtime_error(result.status);}
      }
      nav_msgs::msg::Path path;path.header.frame_id=frame;path.header.stamp=node_->now();
      for (const auto & p:result.path) {
        geometry_msgs::msg::PoseStamped q;q.header=path.header;
        q.pose.position.x=p.x;q.pose.position.y=p.y;
        q.pose.orientation.z=std::sin(p.yaw/2);q.pose.orientation.w=std::cos(p.yaw/2);
        path.poses.push_back(q);
      }
      // Never interpolate or smooth after validation: either could cut a corner.
      if (reuse_enabled_) {cache_.remember(result.path,goal_pose,frame,monotonicNow(),reused);}
      report(result,"SUCCESS",reuse,reused,cache_age);debug_->publish(path);return path;
    } catch (const std::exception & e) {
      cache_.clear();
      report(result,e.what(),reuse,reused,cache_age);
      nav_msgs::msg::Path empty;empty.header.frame_id=costmap_->getGlobalFrameID();
      empty.header.stamp=node_->now();debug_->publish(empty);throw;
    }
  }
private:
  static double monotonicNow()
  {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  static unsigned positive(int value)
  {if (value<=0) {throw std::invalid_argument("Search counts must be positive");}return value;}
  CollisionChecker snapshot()
  {
    if (!costmap_->isCurrent()) {throw std::runtime_error("GLOBAL_COSTMAP_NOT_CURRENT");}
    Grid global;global.name="global";
    auto * cm=costmap_->getCostmap();
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cm->getMutex());
      global.width=cm->getSizeInCellsX();global.height=cm->getSizeInCellsY();
      global.resolution=cm->getResolution();global.origin_x=cm->getOriginX();global.origin_y=cm->getOriginY();
      global.costs.assign(cm->getCharMap(),cm->getCharMap()+static_cast<size_t>(global.width)*global.height);
    }
    nav2_msgs::msg::Costmap::ConstSharedPtr message;
    {std::lock_guard<std::mutex> lock(local_mutex_);message=local_;}
    if (!message) {throw std::runtime_error("LOCAL_COSTMAP_MISSING");}
    const double age=(node_->now()-rclcpp::Time(message->header.stamp)).seconds();
    if (age<-.1 || age>max_age_) {throw std::runtime_error("LOCAL_COSTMAP_STALE age="+std::to_string(age));}
    if (std::abs(message->metadata.origin.orientation.x)>1e-8 ||
      std::abs(message->metadata.origin.orientation.y)>1e-8 ||
      std::abs(message->metadata.origin.orientation.z)>1e-8) {
      throw std::runtime_error("ROTATED_LOCAL_GRID_UNSUPPORTED");}
    Grid local;local.name="local";local.partial=true;
    local.width=message->metadata.size_x;local.height=message->metadata.size_y;
    local.resolution=message->metadata.resolution;
    local.origin_x=message->metadata.origin.position.x;local.origin_y=message->metadata.origin.position.y;
    local.costs=message->data;
    if (message->header.frame_id!=costmap_->getGlobalFrameID()) {
      const auto tr=tf_->lookupTransform(message->header.frame_id,costmap_->getGlobalFrameID(),
        tf2::TimePointZero,tf2::durationFromSec(.1));
      // Static transforms have stamp zero and do not expire.
      const rclcpp::Time stamp(tr.header.stamp);
      if (stamp.nanoseconds()!=0 && std::abs((node_->now()-stamp).seconds())>max_age_) {
        throw std::runtime_error("LOCAL_TRANSFORM_STALE");}
      local.from_global={tr.transform.translation.x,tr.transform.translation.y,tf2::getYaw(tr.transform.rotation)};
    }
    std::vector<Point> footprint;
    for (const auto & p:costmap_->getRobotFootprint()) {footprint.push_back({p.x,p.y});}
    return CollisionChecker({std::move(global),std::move(local)},std::move(footprint),allow_unknown_);
  }
  void report(const Result & result,const std::string & state,
    const ReuseResult & reuse,bool reused,double cache_age)
  {
    std::ostringstream s;s<<state<<" expansions="<<result.expansions<<" rejected_edges="<<result.rejected_edges
      <<" search_seconds="<<result.elapsed
      <<" decision="<<(reused?"REUSE":"REPLAN")<<" reuse_reason="<<reuse.reason
      <<" reuse_seconds="<<reuse.elapsed<<" cache_age="<<cache_age;
    if (reused) {
      s<<" nearest_index="<<reuse.nearest_index<<" join_index="<<reuse.join_index
       <<" connection_length="<<reuse.connection_length;
    } else if (!reuse.failure.reason.empty()) {
      s<<" reuse_grid="<<reuse.failure.grid<<" reuse_cell="<<reuse.failure.cell_x<<","<<reuse.failure.cell_y
       <<" reuse_pose="<<reuse.failure.pose.x<<","<<reuse.failure.pose.y<<","<<reuse.failure.pose.yaw;
    }
    if (state!="SUCCESS" && !result.failure.reason.empty()) {
      s<<" reason="<<result.failure.reason<<" grid="<<result.failure.grid
       <<" pose="<<result.failure.pose.x<<","<<result.failure.pose.y<<","<<result.failure.pose.yaw
       <<" cell="<<result.failure.cell_x<<","<<result.failure.cell_y;
    }
    std_msgs::msg::String msg;msg.data=s.str();status_->publish(msg);
    if (state=="SUCCESS") {RCLCPP_INFO(node_->get_logger(),"%s: %s",name_.c_str(),msg.data.c_str());}
    else {RCLCPP_WARN(node_->get_logger(),"%s: %s",name_.c_str(),msg.data.c_str());}
  }
  Options options_;
  PathCache cache_;
  bool reuse_enabled_{false};double reuse_max_age_{3};
  bool allow_unknown_{false};double max_age_{1};std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::mutex local_mutex_,plan_mutex_;
  nav2_msgs::msg::Costmap::ConstSharedPtr local_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_;
};
}  // namespace nav2_footprint_planner
PLUGINLIB_EXPORT_CLASS(nav2_footprint_planner::StrictHybridPlanner,nav2_core::GlobalPlanner)
