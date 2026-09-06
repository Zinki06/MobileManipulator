#include <iostream>
#include <iomanip>
#include "pick_and_place/grasp_kinematics.hpp"
int main() {
  double x,y,z,floor;std::vector<double> start(4),goal;
  if(!(std::cin>>x>>y>>z>>floor))return 1;
  for(auto &q:start)if(!(std::cin>>q))return 1;
  if(!std::isfinite(z)||!std::isfinite(floor)||z-floor<.015||z-floor>.15)return 2;
  double pitch=-60.0,offset=0.0;
  bool found=false;
  for(double candidate=-60.0;candidate<=-45.0;candidate+=2.5){
    offset=.05-pick_and_place::fingerOffsetZ(candidate*M_PI/180.0);
    if(pick_and_place::solve4DofIK(x,y,z+offset,goal,candidate,false)){
      pitch=candidate;found=true;break;
    }
  }
  if(!found)return 3;
  if(!pick_and_place::fingerPathClear(start,{goal},floor))return 4;
  double min_clearance=10;
  for(int i=0;i<=200;++i){std::vector<double> q(4);for(int j=0;j<4;++j)q[j]=start[j]+(goal[j]-start[j])*i/200.;
    min_clearance=std::min(min_clearance,pick_and_place::fingerFloorClearance(q,floor));}
  if(min_clearance<z-floor+.045)return 5;
  std::cout<<std::setprecision(12)<<"{\"joints\":[";
  for(int j=0;j<4;++j){if(j)std::cout<<",";std::cout<<goal[j];}
  std::cout<<"],\"pitch_degrees\":"<<pitch<<",\"tcp_height_above_surface\":"<<offset<<",\"finger_height_above_surface\":"
    <<pick_and_place::fingerFloorClearance(goal,floor)-(z-floor)
    <<",\"minimum_path_floor_clearance\":"<<min_clearance<<"}";
}
