#include "nav2_footprint_planner/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace nav2_footprint_planner;
// Portable text fixture: number of grids; each grid's geometry/transform/partial
// flag and raw byte costs; footprint vertices; start/goal; reference path poses.
int main(int argc,char ** argv)
{
  try {
    if (argc<3 || argc>5) {throw std::runtime_error("usage: footprint_replay fixture output_prefix [time_budget_seconds] [reuse]");}
    std::ifstream in(argv[1]);in.exceptions(std::ios::failbit|std::ios::badbit);
    unsigned n;in>>n;std::vector<Grid> grids;
    for (unsigned i=0;i<n;++i) {
      Grid g;in>>g.name>>g.width>>g.height>>g.resolution>>g.origin_x>>g.origin_y
        >>g.from_global.x>>g.from_global.y>>g.from_global.yaw>>g.partial;
      g.costs.resize(static_cast<size_t>(g.width)*g.height);
      for (auto & c:g.costs) {unsigned value;in>>value;if(value>255) {throw std::runtime_error("Invalid cost");}c=value;}
      grids.push_back(std::move(g));
    }
    in>>n;std::vector<Point> footprint(n);for(auto & p:footprint) {in>>p.x>>p.y;}
    Pose start,goal;in>>start.x>>start.y>>start.yaw>>goal.x>>goal.y>>goal.yaw;
    in>>n;std::vector<Pose> reference(n);for(auto & p:reference) {in>>p.x>>p.y>>p.yaw;}
    CollisionChecker checker(std::move(grids),std::move(footprint),false);
    Options options;if(argc>=4) {options.max_planning_time=std::stod(argv[3]);}
    Failure reference_failure;const bool reference_valid=checker.pathValid(reference,&reference_failure);
    Result result;ReuseResult reuse;
    if (argc==5) {
      if (std::string(argv[4])!="reuse") {throw std::runtime_error("Unknown replay mode");}
      reuse=reusePath(checker,start,reference,options);
      result.path=reuse.path;result.failure=reuse.failure;result.elapsed=reuse.elapsed;
      result.status=reuse.path.empty()?"REUSE_REJECTED":"SUCCESS";
    } else {result=search(checker,start,goal,options);}
    Failure failure;const bool checked=checker.pathValid(result.path,&failure);
    std::ofstream csv(std::string(argv[2])+".csv");csv<<std::setprecision(17)<<"x,y,yaw\n";
    for(const auto & p:result.path) {csv<<p.x<<","<<p.y<<","<<p.yaw<<"\n";}
    std::ofstream report(std::string(argv[2])+".json");report<<std::setprecision(12)
      <<"{\"status\":\""<<result.status<<"\",\"expansions\":"<<result.expansions
      <<",\"rejected_edges\":"<<result.rejected_edges<<",\"elapsed\":"<<result.elapsed
      <<",\"poses\":"<<result.path.size()<<",\"validated\":"<<(checked?"true":"false")
      <<",\"reuse_reason\":\""<<reuse.reason<<"\",\"nearest_index\":"<<reuse.nearest_index
      <<",\"join_index\":"<<reuse.join_index<<",\"connection_length\":"<<reuse.connection_length
      <<",\"reference_valid\":"<<(reference_valid?"true":"false")
      <<",\"reference_collision\":\""<<reference_failure.grid<<":"<<reference_failure.cell_x<<","<<reference_failure.cell_y
      <<"\",\"failure\":\""<<result.failure.reason<<"\",\"failure_grid\":\""<<result.failure.grid
      <<"\",\"failure_cell\":["<<result.failure.cell_x<<","<<result.failure.cell_y<<"]}\n";
    std::cout<<result.status<<" expansions="<<result.expansions<<" elapsed="<<result.elapsed
      <<" validated="<<checked<<" reference_valid="<<reference_valid<<"\n";
    return checked?0:2;
  } catch(const std::exception & e) {std::cerr<<e.what()<<"\n";return 1;}
}
